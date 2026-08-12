// SysID_Motor - Muestreo de datos para System Identification Toolbox (MATLAB)
//
// Prueba CUALQUIERA de los 3 motores por comando, SIN recablear nada -
// reutiliza los mismos 3 buses de encoder y los mismos 3 drivers ya
// permanentemente cableados en el proyecto principal (Fase7):
//   Motor 1: AS5600 #1 -> Wire  (GPIO21/22)     | BTS7960 #1 -> RPWM25/LPWM26
//   Motor 2: AS5600 #2 -> Wire1 (GPIO32/33)     | BTS7960 #2 -> RPWM27/LPWM18
//   Motor 3: AS5600 #3 -> SoftWire (GPIO4/13)   | BTS7960 #3 -> RPWM19/LPWM23
//
// IMPORTANTE: esto ignora a proposito toda la logica de control ya
// construida en Fase7 (HOME, CALVEL, duty calibrado por motor, limites de
// cambio de sentido). Es una caracterizacion en lazo abierto, un motor a
// la vez: se aplica directamente el duty crudo (0-255) que dicta la señal
// PRBS al motor seleccionado, y los otros dos quedan forzados en 0 todo
// el tiempo (no solo al arrancar) por seguridad.
//
// Senal de excitacion: PRBS (secuencia binaria pseudoaleatoria) generada
// con un LFSR maximal de 8 bits (periodo 255), alternando entre DUTY_MIN y
// DUTY_MAX cada CHIP_HOLD_MS, en UN solo sentido (RPWM; LPWM siempre 0,
// sin cambios bruscos de direccion).
//
// Formato de datos por Serial (CSV), listo para leer_datos_sysid.m:
//   t_ms,duty,pos_raw
//
// Comandos por Serial:
//   START1 / START2 / START3 -> inicia una prueba de ~20s en ese motor
//   STOP                     -> aborta de inmediato, detiene el motor activo

#include <Wire.h>
#include <AS5600.h>
#include <ESP32_SoftWire.h>

AS5600 as5600_1(&Wire);
AS5600 as5600_2(&Wire1);
SoftWire i2c3;

#define AS5600_ADDR      0x36
#define AS5600_REG_ANGLE 0x0E // registro con histeresis, igual criterio que Fase7

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define CORE_LEDC_V3 1
#else
  #define CORE_LEDC_V3 0
#endif

const uint32_t PWM_FREQ_HZ  = 20000;
const uint8_t  PWM_RES_BITS = 8;

const uint8_t PIN_RPWM[3] = {25, 27, 19};
const uint8_t PIN_LPWM[3] = {26, 18, 23};

#if !CORE_LEDC_V3
const uint8_t CANAL_RPWM[3] = {0, 2, 4};
const uint8_t CANAL_LPWM[3] = {1, 3, 5};
#endif

// ---------------- Parametros del experimento (ajustables) ----------------
const uint32_t TS_MS             = 10;    // periodo de muestreo: 10 ms (100 Hz)
const uint32_t DURACION_TOTAL_MS = 20000; // ~20 segundos totales de prueba
const uint32_t CHIP_HOLD_MS      = 100;   // cuanto se mantiene cada nivel del PRBS
                                           // antes de pasar al siguiente bit. Si el
                                           // ajuste del modelo sale malo, prueba
                                           // subir esto (motor lento) o bajarlo
                                           // (motor rapido) y repite la prueba.
const uint8_t  DUTY_MIN = 0;              // nivel bajo del PRBS
const uint8_t  DUTY_MAX = 255;            // nivel alto del PRBS (tope maximo pedido)

// ---------------- Generador PRBS: LFSR maximal de 8 bits ----------------
// Polinomio x^8 + x^6 + x^5 + x^4 + 1 (maximal length, periodo 255).
uint8_t lfsr = 0x01; // semilla; nunca debe quedar en 0 (se estancaria)

bool siguienteBitPRBS() {
  uint8_t bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1;
  lfsr = (uint8_t)((lfsr >> 1) | (bit << 7));
  return (lfsr & 1);
}

// ---------------- PWM: los 6 pines (3 drivers) siempre atados ----------------

void configurarPWM() {
  for (uint8_t m = 0; m < 3; m++) {
#if CORE_LEDC_V3
    ledcAttach(PIN_RPWM[m], PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_LPWM[m], PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(CANAL_RPWM[m], PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_RPWM[m], CANAL_RPWM[m]);
    ledcSetup(CANAL_LPWM[m], PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_LPWM[m], CANAL_LPWM[m]);
#endif
  }
}

// Escribe un duty (0 = detenido) en el motor `m`. Un solo sentido (RPWM);
// LPWM siempre en 0, sin cambios bruscos de direccion.
void escribirDuty(uint8_t m, uint8_t duty) {
#if CORE_LEDC_V3
  ledcWrite(PIN_LPWM[m], 0);
  ledcWrite(PIN_RPWM[m], duty);
#else
  ledcWrite(CANAL_LPWM[m], 0);
  ledcWrite(CANAL_RPWM[m], duty);
#endif
}

// Pone los 3 motores en duty 0. Se llama al arrancar y cada vez que
// cambia cual motor esta bajo prueba, para que los otros dos NUNCA
// reciban duty distinto de 0 mientras no les toca.
void detenerTodos() {
  for (uint8_t m = 0; m < 3; m++) escribirDuty(m, 0);
}

// ---------------- Bus #3 (software): lectura manual, igual criterio que Fase7 ----------------

bool leerReg16_bus3(uint8_t reg, uint16_t &valor) {
  i2c3.beginTransmission(AS5600_ADDR);
  i2c3.write(reg);
  if (i2c3.endTransmission(false) != 0) return false;
  size_t n = i2c3.requestFrom((uint8_t)AS5600_ADDR, (size_t)2, true);
  if (n != 2) return false;
  uint8_t hi = i2c3.read();
  uint8_t lo = i2c3.read();
  valor = ((uint16_t)hi << 8) | lo;
  return true;
}

int32_t posicionAcum3   = 0;
int16_t ultimaPosicion3 = 0;
bool    primeraLectura3 = true;

bool actualizarPosicion3() {
  uint16_t raw;
  if (!leerReg16_bus3(AS5600_REG_ANGLE, raw)) return false;
  raw &= 0x0FFF;
  int16_t value = (int16_t)raw;

  if (primeraLectura3) {
    ultimaPosicion3 = value;
    primeraLectura3 = false;
    return true;
  }
  if ((ultimaPosicion3 > 2048) && (value < (ultimaPosicion3 - 2048))) {
    posicionAcum3 = posicionAcum3 + 4096 - ultimaPosicion3 + value;
  } else if ((value > 2048) && (ultimaPosicion3 < (value - 2048))) {
    posicionAcum3 = posicionAcum3 - 4096 - ultimaPosicion3 + value;
  } else {
    posicionAcum3 = posicionAcum3 - ultimaPosicion3 + value;
  }
  ultimaPosicion3 = value;
  return true;
}

void resetPosicion3() {
  uint16_t raw;
  if (leerReg16_bus3(AS5600_REG_ANGLE, raw)) {
    ultimaPosicion3 = (int16_t)(raw & 0x0FFF);
  }
  posicionAcum3 = 0;
}

// ---------------- Lectura de posicion unificada por motor (0, 1 o 2) ----------------

int32_t leerPosicion(uint8_t m) {
  switch (m) {
    case 0: return as5600_1.getCumulativePosition(true);
    case 1: return as5600_2.getCumulativePosition(true);
    case 2: actualizarPosicion3(); return posicionAcum3;
    default: return 0;
  }
}

void resetPosicion(uint8_t m) {
  switch (m) {
    case 0: as5600_1.resetCumulativePosition(0); break;
    case 1: as5600_2.resetCumulativePosition(0); break;
    case 2: resetPosicion3(); break;
  }
}

// ---------------- Programa principal ----------------

bool     corriendo        = false;
int8_t   motorPrueba       = -1; // 0, 1 o 2 mientras corre; -1 si ninguno
uint32_t inicioPrueba      = 0;
uint32_t ultimaMuestra     = 0;
uint32_t ultimoCambioChip  = 0;
uint8_t  dutyActual        = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== SysID_Motor: caracterizacion en lazo abierto (PRBS) ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println(as5600_1.begin() ? "OK: AS5600 #1 (Motor 1) detectado." : "AVISO: AS5600 #1 (Motor 1) no responde - START1 fallara.");

  Wire1.begin(32, 33);
  Wire1.setClock(400000);
  Serial.println(as5600_2.begin() ? "OK: AS5600 #2 (Motor 2) detectado." : "AVISO: AS5600 #2 (Motor 2) no responde - START2 fallara.");

  i2c3.begin(4, 13, 100000);
  uint16_t rawTest;
  Serial.println(leerReg16_bus3(AS5600_REG_ANGLE, rawTest) ? "OK: AS5600 #3 (Motor 3) detectado." : "AVISO: AS5600 #3 (Motor 3) no responde - START3 fallara.");

  configurarPWM();
  detenerTodos();

  Serial.println();
  Serial.print("Duracion: ");
  Serial.print(DURACION_TOTAL_MS / 1000);
  Serial.print("s | Ts: ");
  Serial.print(TS_MS);
  Serial.print("ms | duty PRBS: ");
  Serial.print(DUTY_MIN);
  Serial.print("-");
  Serial.print(DUTY_MAX);
  Serial.print(" | chip: ");
  Serial.print(CHIP_HOLD_MS);
  Serial.println("ms");
  Serial.println("Comandos: START1 / START2 / START3 -> inicia prueba en ese motor | STOP -> aborta");
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    linea.toUpperCase();

    if (!corriendo && (linea == "START1" || linea == "START2" || linea == "START3")) {
      uint8_t m = linea.charAt(5) - '1'; // 'START1' -> 0, 'START2' -> 1, 'START3' -> 2
      motorPrueba = m;
      resetPosicion(m); // arranca la posicion en 0 para esta prueba
      Serial.println("t_ms,duty,pos_raw");
      corriendo = true;
      inicioPrueba = millis();
      ultimaMuestra = inicioPrueba;
      ultimoCambioChip = inicioPrueba;
      dutyActual = siguienteBitPRBS() ? DUTY_MAX : DUTY_MIN;
      detenerTodos();
      escribirDuty(m, dutyActual);
    } else if (corriendo && linea == "STOP") {
      detenerTodos();
      corriendo = false;
      motorPrueba = -1;
      Serial.println("ABORTADO");
    }
  }

  if (!corriendo) return;

  uint32_t ahora = millis();
  uint32_t transcurrido = ahora - inicioPrueba;

  if (transcurrido >= DURACION_TOTAL_MS) {
    detenerTodos();
    corriendo = false;
    motorPrueba = -1;
    Serial.println("FIN");
    return;
  }

  if (ahora - ultimoCambioChip >= CHIP_HOLD_MS) {
    ultimoCambioChip = ahora;
    dutyActual = siguienteBitPRBS() ? DUTY_MAX : DUTY_MIN;
    escribirDuty((uint8_t)motorPrueba, dutyActual);
  }

  if (ahora - ultimaMuestra >= TS_MS) {
    ultimaMuestra = ahora;
    int32_t pos = leerPosicion((uint8_t)motorPrueba);
    Serial.print(transcurrido);
    Serial.print(",");
    Serial.print(dutyActual);
    Serial.print(",");
    Serial.println(pos);
  }
}
