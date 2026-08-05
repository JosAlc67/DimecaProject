// Fase 3 - AS5600 #1 (Wire, GPIO21/22) + AS5600 #2 (Wire1, GPIO32/33)
// Ambos son buses I2C hardware independientes; los dos sensores responden
// en 0x36 pero cada uno vive en su propio bus, sin conflicto de direccion.
// GPIO16/17 quedan libres, sin usar en este proyecto.
// No conectar todavia AS5600 #3 ni ningun BTS7960.

#include <Wire.h>
#include <AS5600.h>

AS5600 as5600_1(&Wire);
AS5600 as5600_2(&Wire1);

bool ok1 = false;
bool ok2 = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== Fase 3: AS5600 #1 (Wire 21/22) + AS5600 #2 (Wire1 32/33) ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  ok1 = as5600_1.begin();
  Serial.println(ok1 ? "OK: AS5600 #1 responde en bus Wire (21/22)."
                      : "ERROR: AS5600 #1 no responde en bus Wire (21/22).");

  Wire1.begin(32, 33);
  Wire1.setClock(400000);
  ok2 = as5600_2.begin();
  Serial.println(ok2 ? "OK: AS5600 #2 responde en bus Wire1 (32/33)."
                      : "ERROR: AS5600 #2 no responde en bus Wire1 (32/33).");

  Serial.println("Prueba cruzada: gira solo el iman de #1 y confirma que #2 no cambia; luego al reves.");
}

void imprimirLectura(const char *nombre, AS5600 &sensor, bool inicializado) {
  if (inicializado && sensor.isConnected()) {
    uint16_t raw = sensor.rawAngle();
    float deg = raw * 360.0f / 4096.0f;
    Serial.print(nombre);
    Serial.print(": ");
    Serial.print(raw);
    Serial.print(" -> ");
    Serial.print(deg, 2);
    Serial.print(" deg");
    if (!sensor.magnetDetected()) Serial.print("  (SIN IMAN)");
    Serial.println();
  } else {
    Serial.print(nombre);
    Serial.println(": SIN COMUNICACION I2C");
  }
}

void loop() {
  imprimirLectura("AS5600 #1", as5600_1, ok1);
  imprimirLectura("AS5600 #2", as5600_2, ok2);
  Serial.println("---");
  delay(200);
}
