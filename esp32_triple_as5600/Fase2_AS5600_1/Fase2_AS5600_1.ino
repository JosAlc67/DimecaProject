// Fase 2 - Prueba minima: SOLO AS5600 #1 sobre el bus I2C hardware (Wire)
// Conexion: AS5600 #1 VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22
// No conectar todavia AS5600 #2, #3 ni ningun BTS7960.

#include <Wire.h>
#include <AS5600.h>

AS5600 as5600(&Wire);

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("=== Fase 2: AS5600 #1 (I2C hardware, Wire, SDA=21 SCL=22) ===");

  if (as5600.begin()) {
    Serial.println("OK: bus I2C #1 iniciado y AS5600 #1 responde en 0x36.");
  } else {
    Serial.println("ERROR: AS5600 #1 no responde en 0x36. Revisa VCC/GND/SDA/SCL.");
  }
}

void loop() {
  if (as5600.isConnected()) {
    uint16_t raw = as5600.rawAngle();
    float deg = raw * 360.0f / 4096.0f;
    bool magnet = as5600.magnetDetected();

    Serial.print("AS5600 #1: raw=");
    Serial.print(raw);
    Serial.print(" -> ");
    Serial.print(deg, 2);
    Serial.print(" deg | iman=");
    Serial.println(magnet ? "OK" : "NO DETECTADO");
  } else {
    Serial.println("AS5600 #1: SIN COMUNICACION I2C");
  }

  delay(200);
}
