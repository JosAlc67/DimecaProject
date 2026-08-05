// Fase 4 - AS5600 #1 (Wire, GPIO21/22) + AS5600 #2 (Wire1, GPIO32/33)
//        + AS5600 #3 (I2C por software con ESP32_SoftWire, GPIO4/13)
//
// El sensor #3 NO usa la libreria AS5600 de RobTillaart: su constructor
// exige un TwoWire*, y la clase SoftWire de ESP32_SoftWire no hereda de
// TwoWire (ninguna libreria de I2C por software para ESP32 lo hace).
// Por eso aqui se leen a mano los mismos registros que usa esa libreria:
//   0x0C/0x0D = RAW ANGLE (12 bits)
//   0x0B      = STATUS, bit 0x20 = iman detectado (MD)
//
// Velocidad del bus software: 100 kHz (conservadora), muy por debajo del
// limite practico de un I2C bit-banged en ESP32.
// No conectar todavia ningun BTS7960.

#include <Wire.h>
#include <AS5600.h>
#include <ESP32_SoftWire.h>

#define AS5600_ADDR           0x36
#define AS5600_REG_RAW_ANGLE  0x0C
#define AS5600_REG_STATUS     0x0B
#define AS5600_STATUS_MD      0x20

AS5600 as5600_1(&Wire);
AS5600 as5600_2(&Wire1);
SoftWire i2c3;

bool ok1 = false;
bool ok2 = false;
bool ok3 = false;

bool leerReg16_bus3(uint8_t reg, uint16_t &valor) {
  i2c3.beginTransmission(AS5600_ADDR);
  i2c3.write(reg);
  if (i2c3.endTransmission(false) != 0) return false;   // repeated start
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

  uint8_t st = 0;
  imanOk = leerReg8_bus3(AS5600_REG_STATUS, st) && (st & AS5600_STATUS_MD);
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

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Fase 4: AS5600 #1 (Wire 21/22) + #2 (Wire1 32/33) + #3 (SoftWire 4/13) ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  ok1 = as5600_1.begin();
  Serial.println(ok1 ? "OK: AS5600 #1 (Wire 21/22)" : "ERROR: AS5600 #1 (Wire 21/22)");

  Wire1.begin(32, 33);
  Wire1.setClock(400000);
  ok2 = as5600_2.begin();
  Serial.println(ok2 ? "OK: AS5600 #2 (Wire1 32/33)" : "ERROR: AS5600 #2 (Wire1 32/33)");

  i2c3.begin(4, 13, 100000);
  uint16_t rawTest;
  bool imanTest;
  ok3 = leerAS5600_3(rawTest, imanTest);
  Serial.println(ok3 ? "OK: AS5600 #3 (SoftWire 4/13, 100 kHz)" : "ERROR: AS5600 #3 (SoftWire 4/13)");

  Serial.println("Prueba cruzada: gira cada iman por separado y confirma que los otros dos no cambian.");
}

void loop() {
  if (ok1) {
    bool conectado = as5600_1.isConnected();
    imprimirAS5600("AS5600 #1", conectado, conectado ? as5600_1.rawAngle() : 0,
                    conectado ? as5600_1.magnetDetected() : false);
  } else {
    imprimirAS5600("AS5600 #1", false, 0, false);
  }

  if (ok2) {
    bool conectado = as5600_2.isConnected();
    imprimirAS5600("AS5600 #2", conectado, conectado ? as5600_2.rawAngle() : 0,
                    conectado ? as5600_2.magnetDetected() : false);
  } else {
    imprimirAS5600("AS5600 #2", false, 0, false);
  }

  uint16_t raw3 = 0;
  bool iman3 = false;
  bool leido3 = leerAS5600_3(raw3, iman3);
  imprimirAS5600("AS5600 #3", leido3, raw3, iman3);

  Serial.println("---");
  delay(200);
}
