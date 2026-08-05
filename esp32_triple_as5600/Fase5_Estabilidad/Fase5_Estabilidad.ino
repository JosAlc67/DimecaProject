// Fase 5 - Prueba de estabilidad: lectura continua de los 3 AS5600 durante
// varios minutos, con conteo de errores de comunicacion e iman por sensor,
// y deteccion de perdida/recuperacion de comunicacion.
//
// Bus #1: Wire      (GPIO21/22)  - AS5600 #1 (libreria AS5600)
// Bus #2: Wire1     (GPIO32/33)  - AS5600 #2 (libreria AS5600)
// Bus #3: SoftWire  (GPIO4/13)   - AS5600 #3 (lectura manual de registros)
//
// No conectar todavia ningun BTS7960. No hay cambios de cableado respecto
// a la Fase 4.

#include <Wire.h>
#include <AS5600.h>
#include <ESP32_SoftWire.h>

#define AS5600_ADDR           0x36
#define AS5600_REG_RAW_ANGLE   0x0C
#define AS5600_REG_STATUS      0x0B
#define AS5600_STATUS_MD       0x20

const uint32_t INTERVALO_LECTURA_MS = 50;
const uint32_t INTERVALO_REPORTE_MS = 5000;
const uint8_t  UMBRAL_OFFLINE       = 5;   // fallos consecutivos para declarar "sin respuesta"

AS5600 as5600_1(&Wire);
AS5600 as5600_2(&Wire1);
SoftWire i2c3;

struct Stats {
  const char *nombre;
  uint32_t lecturas        = 0;
  uint32_t erroresComm     = 0;
  uint32_t erroresIman     = 0;
  uint32_t consecutivos    = 0;
  uint32_t maxConsecutivos = 0;
  uint32_t caidas          = 0;
  bool     online          = true;
};

Stats st1{"AS5600 #1 (Wire 21/22)   "};
Stats st2{"AS5600 #2 (Wire1 32/33)  "};
Stats st3{"AS5600 #3 (SoftWire 4/13)"};

uint32_t ultimaLectura = 0;
uint32_t ultimoReporte = 0;

void registrarResultado(Stats &st, bool commOk, bool imanOk) {
  st.lecturas++;

  if (!commOk) {
    st.erroresComm++;
    st.consecutivos++;
    if (st.consecutivos > st.maxConsecutivos) st.maxConsecutivos = st.consecutivos;
    if (st.online && st.consecutivos >= UMBRAL_OFFLINE) {
      st.online = false;
      st.caidas++;
      Serial.print("[EVENTO ");
      Serial.print(millis() / 1000);
      Serial.print("s] ");
      Serial.print(st.nombre);
      Serial.println(": DEJO DE RESPONDER");
    }
    return;
  }

  st.consecutivos = 0;
  if (!st.online) {
    st.online = true;
    Serial.print("[EVENTO ");
    Serial.print(millis() / 1000);
    Serial.print("s] ");
    Serial.print(st.nombre);
    Serial.println(": VOLVIO A RESPONDER");
  }
  if (!imanOk) st.erroresIman++;
}

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

void imprimirLineaReporte(const Stats &st) {
  float tasaError = (st.lecturas == 0) ? 0.0f : (100.0f * st.erroresComm / st.lecturas);
  Serial.print(st.nombre);
  Serial.print(" | estado=");
  Serial.print(st.online ? "ONLINE " : "OFFLINE");
  Serial.print(" | lecturas=");
  Serial.print(st.lecturas);
  Serial.print(" | err_comm=");
  Serial.print(st.erroresComm);
  Serial.print(" (");
  Serial.print(tasaError, 2);
  Serial.print("%)");
  Serial.print(" | err_iman=");
  Serial.print(st.erroresIman);
  Serial.print(" | max_consec=");
  Serial.print(st.maxConsecutivos);
  Serial.print(" | caidas=");
  Serial.println(st.caidas);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Fase 5: prueba de estabilidad (varios minutos) ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println(as5600_1.begin() ? "OK: AS5600 #1 inicial" : "ERROR: AS5600 #1 no responde al iniciar");

  Wire1.begin(32, 33);
  Wire1.setClock(400000);
  Serial.println(as5600_2.begin() ? "OK: AS5600 #2 inicial" : "ERROR: AS5600 #2 no responde al iniciar");

  i2c3.begin(4, 13, 100000);
  uint16_t rawTest;
  bool imanTest;
  Serial.println(leerAS5600_3(rawTest, imanTest) ? "OK: AS5600 #3 inicial" : "ERROR: AS5600 #3 no responde al iniciar");

  Serial.println("Iniciando lectura continua. Reporte cada 5 segundos.");
  Serial.println();

  ultimaLectura = millis();
  ultimoReporte = millis();
}

void loop() {
  uint32_t ahora = millis();

  if (ahora - ultimaLectura >= INTERVALO_LECTURA_MS) {
    ultimaLectura = ahora;

    as5600_1.rawAngle();
    bool comm1 = (as5600_1.lastError() == 0);
    bool iman1 = comm1 && as5600_1.magnetDetected();
    registrarResultado(st1, comm1, iman1);

    as5600_2.rawAngle();
    bool comm2 = (as5600_2.lastError() == 0);
    bool iman2 = comm2 && as5600_2.magnetDetected();
    registrarResultado(st2, comm2, iman2);

    uint16_t raw3;
    bool iman3;
    bool comm3 = leerAS5600_3(raw3, iman3);
    registrarResultado(st3, comm3, iman3);
  }

  if (ahora - ultimoReporte >= INTERVALO_REPORTE_MS) {
    ultimoReporte = ahora;
    Serial.print("--- Reporte a los ");
    Serial.print(ahora / 1000);
    Serial.println(" s ---");
    imprimirLineaReporte(st1);
    imprimirLineaReporte(st2);
    imprimirLineaReporte(st3);
    Serial.println();
  }
}
