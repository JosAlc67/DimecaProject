// Fase 7 - Integracion final: 3 AS5600 + 3 BTS7960, SIN PID todavia.
// Lee los 3 encoders periodicamente y permite controlar los 3 motores
// de forma independiente por comandos Serial, con la misma proteccion
// de la Fase 6: no se permite invertir sentido sin pasar por parada.
//
// IMPORTANTE: la lectura de los 3 AS5600 es completamente independiente
// de los motores y se puede probar ahora mismo. Los comandos de motor
// no tienen ningun efecto (ni riesgo) mientras los BTS7960 no tengan
// VCC/B+ energizados - las lineas RPWM/LPWM quedan sin potencia detras.
//
// Bus #1: Wire      (GPIO21/22)  - AS5600 #1
// Bus #2: Wire1     (GPIO32/33)  - AS5600 #2
// Bus #3: SoftWire  (GPIO4/13)   - AS5600 #3 (lectura manual de registros)
// BTS7960 #1: RPWM 25 / LPWM 26
// BTS7960 #2: RPWM 27 / LPWM 18
// BTS7960 #3: RPWM 19 / LPWM 23
//
// Comandos Serial (115200, terminar linea con Enter):
//   M1 / M2 / M3  -> selecciona el motor activo (por defecto: 1)
//   F             -> gira el motor activo hacia adelante a baja velocidad
//   R             -> gira el motor activo en reversa a baja velocidad
//   S             -> detiene el motor activo
//   SS            -> PARADA DE EMERGENCIA: detiene los 3 motores ya

#include <Wire.h>
#include <AS5600.h>
#include <ESP32_SoftWire.h>

// ---------------- Encoders ----------------

#define AS5600_ADDR           0x36
#define AS5600_REG_RAW_ANGLE   0x0C
#define AS5600_REG_STATUS      0x0B
#define AS5600_STATUS_MD       0x20

AS5600 as5600_1(&Wire);
AS5600 as5600_2(&Wire1);
SoftWire i2c3;

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

bool leerReg8_bus3(uint8_t reg, uint8_t &valor) {
  i2c3.beginTransmission(AS5600_ADDR);
  i2c3.write(reg);
  if (i2c3.endTransmission(false) != 0) return false;
  size_t n = i2c3.requestFrom((uint8_t)AS5600_ADDR, (size_t)1, true);
  if (n != 1) return false;
  valor = i2c3.read();
  return true;
}

bool leerAS5600_3(uint16_t &raw, bool &imanOk) {
  uint16_t v;
  if (!leerReg16_bus3(AS5600_REG_RAW_ANGLE, v)) return false;
  raw = v & 0x0FFF;
  uint8_t stReg = 0;
  imanOk = leerReg8_bus3(AS5600_REG_STATUS, stReg) && (stReg & AS5600_STATUS_MD);
  return true;
}

void imprimirAS5600(const char *nombre, bool ok, uint16_t raw, bool imanOk) {
  if (!ok) {
    Serial.print(nombre);
    Serial.println(": SIN COMUNICACION I2C");
    return;
  }
  float deg = raw * 360.0f / 4096.0f;
  Serial.print(nombre);
  Serial.print(": ");
  Serial.print(raw);
  Serial.print(" -> ");
  Serial.print(deg, 2);
  Serial.print(" deg");
  if (!imanOk) Serial.print("  (SIN IMAN)");
  Serial.println();
}

// ---------------- Motores (BTS7960) ----------------

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define CORE_LEDC_V3 1
#else
  #define CORE_LEDC_V3 0
#endif

const uint32_t PWM_FREQ_HZ    = 20000;
const uint8_t  PWM_RES_BITS   = 8;
// 60 (~23%) resulto insuficiente para vencer la friccion estatica de los
// motores (no arrancaban). 255 (100%) si arranco, confirmado en pruebas,
// pero no se quiere dejar al maximo como velocidad "baja" por defecto.
// 150 (~59%) es un valor intermedio SIN VALIDAR EN HARDWARE todavia:
// confirma que los 3 motores arrancan con este valor, o ajustalo.
const uint8_t  VELOCIDAD_BAJA = 150;

const uint8_t PIN_RPWM[3] = {25, 27, 19};
const uint8_t PIN_LPWM[3] = {26, 18, 23};

#if !CORE_LEDC_V3
const uint8_t CANAL_RPWM[3] = {0, 2, 4};
const uint8_t CANAL_LPWM[3] = {1, 3, 5};
#endif

enum EstadoMotor { PARADO, ADELANTE, REVERSA };
EstadoMotor estadoMotor[3] = {PARADO, PARADO, PARADO};
uint8_t motorActivo = 0;

void configurarPWM(uint8_t pin, uint8_t canal) {
#if CORE_LEDC_V3
  ledcAttach(pin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(canal, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pin, canal);
#endif
}

void escribirPWM(uint8_t pin, uint8_t canal, uint32_t duty) {
#if CORE_LEDC_V3
  ledcWrite(pin, duty);
#else
  ledcWrite(canal, duty);
#endif
}

uint8_t canalRPWM(uint8_t m) {
#if CORE_LEDC_V3
  return 0;
#else
  return CANAL_RPWM[m];
#endif
}
uint8_t canalLPWM(uint8_t m) {
#if CORE_LEDC_V3
  return 0;
#else
  return CANAL_LPWM[m];
#endif
}

void detenerMotor(uint8_t m) {
  escribirPWM(PIN_RPWM[m], canalRPWM(m), 0);
  escribirPWM(PIN_LPWM[m], canalLPWM(m), 0);
  estadoMotor[m] = PARADO;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": DETENIDO");
}

void girarAdelante(uint8_t m) {
  if (estadoMotor[m] != PARADO) {
    Serial.println("RECHAZADO: detén el motor (S) antes de cambiar de sentido.");
    return;
  }
  escribirPWM(PIN_LPWM[m], canalLPWM(m), 0);
  escribirPWM(PIN_RPWM[m], canalRPWM(m), VELOCIDAD_BAJA);
  estadoMotor[m] = ADELANTE;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": GIRANDO ADELANTE (baja velocidad)");
}

void girarReversa(uint8_t m) {
  if (estadoMotor[m] != PARADO) {
    Serial.println("RECHAZADO: detén el motor (S) antes de cambiar de sentido.");
    return;
  }
  escribirPWM(PIN_RPWM[m], canalRPWM(m), 0);
  escribirPWM(PIN_LPWM[m], canalLPWM(m), VELOCIDAD_BAJA);
  estadoMotor[m] = REVERSA;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": GIRANDO REVERSA (baja velocidad)");
}

void procesarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "M1") { motorActivo = 0; Serial.println("Motor activo: 1"); }
  else if (cmd == "M2") { motorActivo = 1; Serial.println("Motor activo: 2"); }
  else if (cmd == "M3") { motorActivo = 2; Serial.println("Motor activo: 3"); }
  else if (cmd == "F") girarAdelante(motorActivo);
  else if (cmd == "R") girarReversa(motorActivo);
  else if (cmd == "S") detenerMotor(motorActivo);
  else if (cmd == "SS") {
    for (uint8_t i = 0; i < 3; i++) detenerMotor(i);
    Serial.println("PARADA DE EMERGENCIA: los 3 motores detenidos.");
  }
  else if (cmd.length() > 0) {
    Serial.println("Comando no reconocido. Usa: M1 M2 M3 F R S SS");
  }
}

// ---------------- Programa principal ----------------

const uint32_t INTERVALO_LECTURA_MS = 500;
uint32_t ultimaLectura = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Fase 7: integracion final (3 AS5600 + 3 BTS7960, sin PID) ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println(as5600_1.begin() ? "OK: AS5600 #1" : "ERROR: AS5600 #1 no responde al iniciar");

  Wire1.begin(32, 33);
  Wire1.setClock(400000);
  Serial.println(as5600_2.begin() ? "OK: AS5600 #2" : "ERROR: AS5600 #2 no responde al iniciar");

  i2c3.begin(4, 13, 100000);
  uint16_t rawTest;
  bool imanTest;
  Serial.println(leerAS5600_3(rawTest, imanTest) ? "OK: AS5600 #3" : "ERROR: AS5600 #3 no responde al iniciar");

  for (uint8_t i = 0; i < 3; i++) {
#if CORE_LEDC_V3
    configurarPWM(PIN_RPWM[i], 0);
    configurarPWM(PIN_LPWM[i], 0);
#else
    configurarPWM(PIN_RPWM[i], CANAL_RPWM[i]);
    configurarPWM(PIN_LPWM[i], CANAL_LPWM[i]);
#endif
    detenerMotor(i); // arranca siempre detenido
  }

  Serial.println();
  Serial.println("Comandos: M1 M2 M3 (seleccionar motor) | F (adelante) | R (reversa) | S (detener) | SS (parada de emergencia)");
  Serial.println("Motor activo por defecto: 1. Angulos se imprimen cada 500 ms.");
  Serial.println();

  ultimaLectura = millis();
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarComando(linea);
  }

  uint32_t ahora = millis();
  if (ahora - ultimaLectura >= INTERVALO_LECTURA_MS) {
    ultimaLectura = ahora;

    bool conectado1 = as5600_1.isConnected();
    imprimirAS5600("AS5600 #1", conectado1, conectado1 ? as5600_1.rawAngle() : 0,
                    conectado1 ? as5600_1.magnetDetected() : false);

    bool conectado2 = as5600_2.isConnected();
    imprimirAS5600("AS5600 #2", conectado2, conectado2 ? as5600_2.rawAngle() : 0,
                    conectado2 ? as5600_2.magnetDetected() : false);

    uint16_t raw3;
    bool iman3;
    bool conectado3 = leerAS5600_3(raw3, iman3);
    imprimirAS5600("AS5600 #3", conectado3, raw3, iman3);

    Serial.println("---");
  }
}
