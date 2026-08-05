// Diagnostico - Escaner I2C crudo, SIN la libreria AS5600 y SIN ESP32_SoftWire.
// Objetivo: ver si el bus responde a nivel de protocolo (ACK/NACK) en los
// dos buses hardware, para descartar o confirmar un problema fisico/electrico
// independiente de cualquier libreria.
//
// Desconecta fisicamente el AS5600 #3 antes de esta prueba (deja solo #1 y #2).
// Alimenta el ESP32 desde cero (USB desconectado unos segundos, no solo reset).

#include <Wire.h>

void escanearBus(const char *nombre, TwoWire &bus, int sda, int scl) {
  Serial.print("--- Escaneando ");
  Serial.print(nombre);
  Serial.print(" (SDA=");
  Serial.print(sda);
  Serial.print(" SCL=");
  Serial.print(scl);
  Serial.println(") ---");

  bus.begin(sda, scl);
  bus.setClock(100000);

  int encontrados = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t error = bus.endTransmission();
    if (error == 0) {
      Serial.print("  Dispositivo encontrado en 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      encontrados++;
    }
  }

  if (encontrados == 0) {
    Serial.println("  NINGUN dispositivo respondio en este bus.");
  } else {
    Serial.print("  Total encontrados: ");
    Serial.println(encontrados);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== Diagnostico: escaner I2C crudo (Wire y Wire1) ===");

  escanearBus("Wire  (bus #1)", Wire, 21, 22);
  escanearBus("Wire1 (bus #2)", Wire1, 32, 33);

  Serial.println("=== Fin del escaneo. Se repite cada 5 segundos. ===");
}

void loop() {
  delay(5000);
  Serial.println();
  escanearBus("Wire  (bus #1)", Wire, 21, 22);
  escanearBus("Wire1 (bus #2)", Wire1, 32, 33);
}
