// SysID_Motor - Muestreo de datos para System Identification Toolbox (MATLAB)
//
// Prueba UN motor + UN driver + UN encoder a la vez, conectados como si
// fueran "Motor 1" del proyecto principal:
//   AS5600 bajo prueba  -> Wire (GPIO21 SDA / GPIO22 SCL)
//   BTS7960 bajo prueba -> RPWM GPIO25 / LPWM GPIO26 (LPWM siempre en 0,
//                          la prueba es en UN solo sentido, sin cambios
//                          bruscos de direccion)
//
// IMPORTANTE: esto ignora a proposito toda la logica de control ya
// construida en Fase7 (HOME, CALVEL, duty calibrado por motor, limites de
// cambio de sentido). Es una caracterizacion en lazo abierto: se aplica
// directamente el duty crudo (0-255) que dicta la señal PRBS.
//
// Senal de excitacion: PRBS (secuencia binaria pseudoaleatoria) generada
// con un LFSR maximal de 8 bits (periodo 255), alternando entre DUTY_MIN y
// DUTY_MAX cada CHIP_HOLD_MS. Es la señal estandar para identificacion de
// sistemas lineales (excita un rango de frecuencias, no un solo punto).
//
// Formato de datos por Serial (CSV), listo para leer.m de MATLAB:
//   t_ms,duty,pos_raw
//
// Comandos por Serial:
//   START -> inicia una prueba de ~20s (DURACION_TOTAL_MS)
//   STOP  -> aborta de inmediato, detiene el motor (funciona en cualquier momento)

#include <Wire.h>
#include <AS5600.h>

AS5600 as5600(&Wire);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define CORE_LEDC_V3 1
#else
  #define CORE_LEDC_V3 0
#endif

const uint8_t  PIN_RPWM      = 25;
const uint8_t  PIN_LPWM      = 26;
const uint32_t PWM_FREQ_HZ   = 20000;
const uint8_t  PWM_RES_BITS  = 8;

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

void configurarPWM() {
#if CORE_LEDC_V3
  ledcAttach(PIN_RPWM, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(PIN_LPWM, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_RPWM, 0);
  ledcSetup(1, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_LPWM, 1);
#endif
}

void escribirDuty(uint8_t duty) {
#if CORE_LEDC_V3
  ledcWrite(PIN_RPWM, duty);
  ledcWrite(PIN_LPWM, 0);
#else
  ledcWrite(0, duty);
  ledcWrite(1, 0);
#endif
}

bool     corriendo       = false;
uint32_t inicioPrueba    = 0;
uint32_t ultimaMuestra   = 0;
uint32_t ultimoCambioChip = 0;
uint8_t  dutyActual      = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  bool ok = as5600.begin();
  Serial.println();
  Serial.println(ok ? "OK: AS5600 detectado en 0x36." : "ERROR: AS5600 no responde - revisa cableado antes de continuar.");
  if (!ok) {
    while (true) delay(1000); // no continuar sin sensor conectado
  }

  configurarPWM();
  escribirDuty(0);

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
  Serial.println("Escribe START y Enter para comenzar. STOP aborta en cualquier momento.");
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    linea.toUpperCase();

    if (!corriendo && linea == "START") {
      as5600.resetCumulativePosition(0); // arranca la posicion en 0 para esta prueba
      Serial.println("t_ms,duty,pos_raw");
      corriendo = true;
      inicioPrueba = millis();
      ultimaMuestra = inicioPrueba;
      ultimoCambioChip = inicioPrueba;
      dutyActual = siguienteBitPRBS() ? DUTY_MAX : DUTY_MIN;
      escribirDuty(dutyActual);
    } else if (corriendo && linea == "STOP") {
      escribirDuty(0);
      corriendo = false;
      Serial.println("ABORTADO");
    }
  }

  if (!corriendo) return;

  uint32_t ahora = millis();
  uint32_t transcurrido = ahora - inicioPrueba;

  if (transcurrido >= DURACION_TOTAL_MS) {
    escribirDuty(0);
    corriendo = false;
    Serial.println("FIN");
    return;
  }

  if (ahora - ultimoCambioChip >= CHIP_HOLD_MS) {
    ultimoCambioChip = ahora;
    dutyActual = siguienteBitPRBS() ? DUTY_MAX : DUTY_MIN;
    escribirDuty(dutyActual);
  }

  if (ahora - ultimaMuestra >= TS_MS) {
    ultimaMuestra = ahora;
    int32_t pos = as5600.getCumulativePosition(true);
    Serial.print(transcurrido);
    Serial.print(",");
    Serial.print(dutyActual);
    Serial.print(",");
    Serial.println(pos);
  }
}
