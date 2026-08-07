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
//   CAL1/CAL2/CAL3-> calibra el cero de ese encoder en la posicion FISICA
//                    actual del eje (offset queda guardado en flash,
//                    sobrevive a reinicios). Esto NO reduce el error de
//                    +-1.5 grados de linealidad (eso requiere una
//                    referencia externa y se deja para otra sesion) -
//                    solo fija donde esta el 0 grados de cada sensor,
//                    y ese mismo punto pasa a ser tambien "0 vueltas".
//
// SEGUIMIENTO DE VUELTAS COMPLETAS (multi-turn):
// Los ejes pueden dar varias vueltas completas, asi que ademas del angulo
// dentro de la vuelta actual (0-359.99, con el salto de siempre en la
// frontera) se lleva una POSICION ACUMULADA continua que nunca se reinicia
// sola: sigue sumando o restando aunque cruces 360 grados una y otra vez.
// Para #1/#2 se usa getCumulativePosition()/getRevolutions() de la propia
// libreria AS5600 (verificado en su codigo fuente). Para #3 (bus por
// software, sin libreria) se replica a mano el mismo algoritmo exacto.
// Esa posicion acumulada es la que conviene usar como variable de PID mas
// adelante: al no reiniciarse nunca en 0, no sufre el problema de "salto
// enorme de error" que sí tiene el angulo 0-359.99 cerca de la frontera.
//
// IMPORTANTE - limitacion: el conteo de vueltas vive solo en RAM. Si el
// ESP32 se apaga/reinicia, las vueltas se pierden y se vuelve a contar
// desde 0 en el punto donde haya quedado la calibracion guardada en flash
// (el offset de grados SI sobrevive, el conteo de vueltas NO). Si en algun
// momento necesitas que el conteo de vueltas tambien sobreviva un apagado,
// avisame - se puede hacer pero implica guardar en flash periodicamente
// (no en cada lectura, por desgaste de la memoria flash).
//
// IMPORTANTE - requisito de velocidad de actualizacion: el algoritmo de
// conteo de vueltas se rompe si el eje gira mas de media vuelta (180 grados)
// entre dos actualizaciones consecutivas. Por eso la actualizacion de
// posicion ocurre en CADA iteracion de loop() (no solo cada 500ms como la
// impresion por Serial) - mientras el motor no gire absurdamente rapido
// entre dos iteraciones de loop(), esto tiene margen de sobra.

#include <Wire.h>
#include <AS5600.h>
#include <ESP32_SoftWire.h>
#include <Preferences.h>

// ---------------- Encoders ----------------

#define AS5600_ADDR           0x36
#define AS5600_REG_ANGLE        0x0E  // registro CON histeresis; el mismo
                                       // que usa internamente readAngle()
                                       // de la libreria, y por lo tanto el
                                       // mismo que usa getCumulativePosition().
                                       // Se usa aqui para que el sensor #3
                                       // sea consistente con #1/#2.
#define AS5600_REG_STATUS      0x0B
#define AS5600_STATUS_MD       0x20

AS5600 as5600_1(&Wire);
AS5600 as5600_2(&Wire1);
SoftWire i2c3;

// Estado de comunicacion de cada sensor, actualizado en cada iteracion de
// loop() (ver mas abajo). Se declara aqui arriba porque tanto el homing
// como el reporte por Serial lo necesitan.
bool comOk1 = false, comOk2 = false, comOk3 = false;

// ---------------- Calibracion (offset de cero) persistente en NVS ----------------

Preferences prefs;
const char *NVS_NAMESPACE = "as5600cal";

// Offset del sensor #3, aplicado a mano (no hay libreria para el bus software).
// Se guarda/lee como entero de 0 a 4095 (mismo convenio que usa la libreria
// AS5600 internamente para offsets negativos via complemento a 2 en 12 bits).
int16_t offsetRaw3 = 0;

int16_t cargarOffsetRaw(uint8_t indice) {
  char clave[8];
  snprintf(clave, sizeof(clave), "off%u", indice);
  prefs.begin(NVS_NAMESPACE, true); // solo lectura
  int16_t valor = prefs.getShort(clave, 0);
  prefs.end();
  return valor;
}

void guardarOffsetRaw(uint8_t indice, int16_t offsetRaw) {
  char clave[8];
  snprintf(clave, sizeof(clave), "off%u", indice);
  prefs.begin(NVS_NAMESPACE, false); // lectura/escritura
  prefs.putShort(clave, offsetRaw);
  prefs.end();
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

// Lectura del angulo (registro 0x0E, con offset de calibracion aplicado)
// del sensor #3. Es el camino RAPIDO, sin leer el registro de estado, para
// no cargar el bus por software durante la actualizacion de cada iteracion
// de loop(). El estado del iman se revisa aparte, solo al imprimir.
bool leerAnguloConOffset_bus3(uint16_t &raw) {
  uint16_t v;
  if (!leerReg16_bus3(AS5600_REG_ANGLE, v)) return false;
  raw = ((v & 0x0FFF) + offsetRaw3) & 0x0FFF;
  return true;
}

bool leerAS5600_3(uint16_t &raw, bool &imanOk) {
  if (!leerAnguloConOffset_bus3(raw)) return false;
  uint8_t stReg = 0;
  imanOk = leerReg8_bus3(AS5600_REG_STATUS, stReg) && (stReg & AS5600_STATUS_MD);
  return true;
}

// ---------------- Seguimiento de vueltas completas (multi-turn) ----------------
//
// Para el sensor #3 se replica a mano el mismo algoritmo que usa
// AS5600::getCumulativePosition()/getRevolutions() en la libreria de
// RobTillaart (verificado en su codigo fuente): compara la lectura actual
// contra la anterior, y si el salto es mayor a media vuelta (2048 cuentas
// de 4096), asume que se cruzo la frontera 0/4095 y suma o resta una
// vuelta completa segun el sentido.

int32_t posicionAcum3   = 0;
int16_t ultimaPosicion3 = 0;
bool    primeraLectura3 = true;

bool actualizarPosicion3() {
  uint16_t raw;
  if (!leerAnguloConOffset_bus3(raw)) return false;
  int16_t value = (int16_t)raw;

  if (primeraLectura3) {
    ultimaPosicion3 = value;
    primeraLectura3 = false;
    return true;
  }

  if ((ultimaPosicion3 > 2048) && (value < (ultimaPosicion3 - 2048))) {
    posicionAcum3 = posicionAcum3 + 4096 - ultimaPosicion3 + value;      // vuelta completa CW
  } else if ((value > 2048) && (ultimaPosicion3 < (value - 2048))) {
    posicionAcum3 = posicionAcum3 - 4096 - ultimaPosicion3 + value;      // vuelta completa CCW
  } else {
    posicionAcum3 = posicionAcum3 - ultimaPosicion3 + value;
  }
  ultimaPosicion3 = value;
  return true;
}

int32_t revoluciones3() {
  int32_t p = posicionAcum3 >> 12; // dividir por 4096
  if (p < 0) p++;                  // corrige el redondeo de numeros negativos
  return p;
}

// Fija la posicion acumulada actual como punto de partida `nuevaPosicion`
// (normalmente 0), resincronizando tambien la referencia interna para que
// el proximo actualizarPosicion3() no calcule un salto falso.
void resetPosicionAcumulada3(int32_t nuevaPosicion) {
  uint16_t raw;
  leerAnguloConOffset_bus3(raw);
  ultimaPosicion3 = (int16_t)raw;
  posicionAcum3 = nuevaPosicion;
}

void imprimirPosicion(const char *nombre, bool ok, int32_t posicionRaw, int32_t vueltas, bool imanOk) {
  if (!ok) {
    Serial.print(nombre);
    Serial.println(": SIN COMUNICACION I2C");
    return;
  }
  int32_t dentroDeVuelta = posicionRaw % 4096;
  if (dentroDeVuelta < 0) dentroDeVuelta += 4096;
  float anguloDentroDeVuelta = dentroDeVuelta * 360.0f / 4096.0f;
  float anguloAcumulado = posicionRaw * 360.0f / 4096.0f;

  Serial.print(nombre);
  Serial.print(": ");
  Serial.print(anguloDentroDeVuelta, 2);
  Serial.print(" deg (vuelta actual) | vueltas=");
  Serial.print(vueltas);
  Serial.print(" | acumulado=");
  Serial.print(anguloAcumulado, 2);
  Serial.print(" deg");
  if (!imanOk) Serial.print("  (SIN IMAN)");
  Serial.println();
}

// Calibra el sensor `indice` (1, 2 o 3) para que la posicion FISICA actual
// del eje pase a leerse como 0 grados Y como 0 vueltas. offsetRaw =
// (4096 - crudo) & 0x0FFF es el mismo truco de complemento a 2 en 12 bits
// que usa internamente AS5600::setOffset() para offsets negativos
// (verificado en el codigo fuente de la libreria): sumar ese offset y
// enmascarar a 12 bits equivale a restar el valor crudo original, sin
// importar el signo. Tras fijar el offset, se resincroniza el contador de
// vueltas a 0 en ese mismo punto - si no se hiciera esto, el cambio de
// offset se interpretaria como un salto fisico y corromperia el conteo.
void calibrarSensor(uint8_t indice) {
  if (indice == 1) {
    as5600_1.setOffset(0);
    uint16_t crudo = as5600_1.readAngle();
    if (as5600_1.lastError() != 0) {
      Serial.println("ERROR: no se pudo leer AS5600 #1 para calibrar.");
      return;
    }
    int16_t offsetRaw = (4096 - crudo) & 0x0FFF;
    as5600_1.setOffset(offsetRaw * AS5600_RAW_TO_DEGREES);
    as5600_1.resetCumulativePosition(0);
    guardarOffsetRaw(1, offsetRaw);
    Serial.println("AS5600 #1 calibrado: posicion actual = 0.00 deg, 0 vueltas (offset guardado en flash).");
  } else if (indice == 2) {
    as5600_2.setOffset(0);
    uint16_t crudo = as5600_2.readAngle();
    if (as5600_2.lastError() != 0) {
      Serial.println("ERROR: no se pudo leer AS5600 #2 para calibrar.");
      return;
    }
    int16_t offsetRaw = (4096 - crudo) & 0x0FFF;
    as5600_2.setOffset(offsetRaw * AS5600_RAW_TO_DEGREES);
    as5600_2.resetCumulativePosition(0);
    guardarOffsetRaw(2, offsetRaw);
    Serial.println("AS5600 #2 calibrado: posicion actual = 0.00 deg, 0 vueltas (offset guardado en flash).");
  } else if (indice == 3) {
    uint16_t crudo;
    if (!leerReg16_bus3(AS5600_REG_ANGLE, crudo)) {
      Serial.println("ERROR: no se pudo leer AS5600 #3 para calibrar.");
      return;
    }
    crudo &= 0x0FFF;
    offsetRaw3 = (4096 - crudo) & 0x0FFF;
    resetPosicionAcumulada3(0);
    guardarOffsetRaw(3, offsetRaw3);
    Serial.println("AS5600 #3 calibrado: posicion actual = 0.00 deg, 0 vueltas (offset guardado en flash).");
  }
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

// ---------------- HOME (regreso a 0) - lazo cerrado simple, SIN PID ----------------
//
// "Si no estas en 0, muevete hacia 0 a VELOCIDAD_BAJA; detente al llegar."
// No hay ganancias ni ajuste fino de velocidad - es control bang-bang, el
// mas simple que existe. Asume que AS5600 #N esta en el mismo eje que
// Motor N (#1<->#1, #2<->#2, #3<->#3) - SIN VERIFICAR, confirmalo.
//
// Incluye dos protecciones para no empujar indefinidamente en una
// direccion incorrecta:
//  - Si no mejora el error en HOMING_MARGEN_PROGRESO_MS, aborta.
//  - Si se excede HOMING_TIMEOUT_MS en total, aborta.

const float    TOLERANCIA_HOMING_DEG     = 3.0f;   // se considera "en home" dentro de esto
const uint32_t HOMING_TIMEOUT_MS         = 20000;  // aborta si tarda mas de esto en total
const uint32_t HOMING_MARGEN_PROGRESO_MS = 4000;   // debe mejorar el error en este tiempo

// +1: mandar "adelante" (F) incrementa la posicion acumulada de ese motor.
// -1: "adelante" (F) la disminuye (equivale a que "reversa" la incrementa).
// SIN VALIDAR EN HARDWARE: si al mandar HOME el motor se aleja de 0 en vez
// de acercarse (o se aborta por "no se acerca a 0"), invierte el signo del
// motor correspondiente aqui.
int8_t signoHoming[3] = {+1, +1, +1};

bool homingActivo[3]         = {false, false, false};
uint32_t homingInicio[3]     = {0, 0, 0};
uint32_t homingUltimoProgreso[3] = {0, 0, 0};
float homingMejorError[3]    = {0, 0, 0};

// Lee la posicion acumulada (continua, sin wraparound) en grados del
// encoder correspondiente al motor `m` (0, 1 o 2).
// Mapeo fisico confirmado por el usuario: AS5600 #1 esta en el eje del
// Motor 3, y AS5600 #3 esta en el eje del Motor 1 (cruzados). AS5600 #2
// si corresponde al Motor 2.
bool leerAnguloAcumuladoGrados(uint8_t m, float &grados) {
  int32_t posRaw;
  bool ok;
  switch (m) {
    case 0: ok = comOk3; posRaw = posicionAcum3; break;                    // Motor 1 -> AS5600 #3
    case 1: ok = comOk2; posRaw = as5600_2.getCumulativePosition(false); break; // Motor 2 -> AS5600 #2
    case 2: ok = comOk1; posRaw = as5600_1.getCumulativePosition(false); break; // Motor 3 -> AS5600 #1
    default: return false;
  }
  if (!ok) return false;
  grados = posRaw * (360.0f / 4096.0f);
  return true;
}

void detenerHoming(uint8_t m, const char *motivo) {
  homingActivo[m] = false;
  detenerMotor(m);
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.print(": HOME ");
  Serial.println(motivo);
}

void iniciarHoming(uint8_t m) {
  if (estadoMotor[m] != PARADO) {
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.println(": RECHAZADO - detenlo (S) antes de iniciar HOME.");
    return;
  }
  float grados;
  bool ok = leerAnguloAcumuladoGrados(m, grados);
  if (!ok) {
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.println(": RECHAZADO - encoder sin comunicacion, no se puede iniciar HOME.");
    return;
  }
  homingActivo[m] = true;
  homingInicio[m] = millis();
  homingUltimoProgreso[m] = millis();
  homingMejorError[m] = fabs(grados);
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.print(": iniciando HOME desde ");
  Serial.print(grados, 2);
  Serial.println(" deg...");
}

// Se llama en CADA iteracion de loop() para los motores con homing activo.
void actualizarHoming(uint8_t m) {
  if (!homingActivo[m]) return;

  float grados;
  if (!leerAnguloAcumuladoGrados(m, grados)) {
    detenerHoming(m, "ABORTADO - se perdio comunicacion con el encoder.");
    return;
  }

  float errorAbs = fabs(grados);

  if (errorAbs <= TOLERANCIA_HOMING_DEG) {
    homingActivo[m] = false;
    detenerMotor(m);
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.print(": HOME alcanzado, posicion final = ");
    Serial.print(grados, 2);
    Serial.println(" deg.");
    return;
  }

  if (millis() - homingInicio[m] > HOMING_TIMEOUT_MS) {
    detenerHoming(m, "ABORTADO - tiempo limite excedido.");
    return;
  }

  if (errorAbs < homingMejorError[m] - 0.5f) {
    homingMejorError[m] = errorAbs;
    homingUltimoProgreso[m] = millis();
  } else if (millis() - homingUltimoProgreso[m] > HOMING_MARGEN_PROGRESO_MS) {
    detenerHoming(m, "ABORTADO - no se acerca a 0. Revisa signoHoming[] o el acople mecanico.");
    return;
  }

  bool necesitaDisminuir = (grados > 0);
  bool debeIrAdelante = (signoHoming[m] > 0) ? !necesitaDisminuir : necesitaDisminuir;
  EstadoMotor direccionDeseada = debeIrAdelante ? ADELANTE : REVERSA;

  if (estadoMotor[m] != direccionDeseada) {
    if (estadoMotor[m] != PARADO) detenerMotor(m);
    if (debeIrAdelante) girarAdelante(m);
    else girarReversa(m);
  }
}

// Imprime el estado de los 3 encoders bajo demanda (comando STATUS), en vez
// de hacerlo automaticamente cada cierto tiempo - eso generaba demasiado
// ruido visual en el Monitor Serial.
void imprimirEstadoCompleto() {
  bool imanOk1 = comOk1 && as5600_1.magnetDetected();
  bool imanOk2 = comOk2 && as5600_2.magnetDetected();
  uint8_t st3 = 0;
  bool imanOk3 = comOk3 && leerReg8_bus3(AS5600_REG_STATUS, st3) && (st3 & AS5600_STATUS_MD);

  imprimirPosicion("AS5600 #1", comOk1, as5600_1.getCumulativePosition(false), as5600_1.getRevolutions(), imanOk1);
  imprimirPosicion("AS5600 #2", comOk2, as5600_2.getCumulativePosition(false), as5600_2.getRevolutions(), imanOk2);
  imprimirPosicion("AS5600 #3", comOk3, posicionAcum3, revoluciones3(), imanOk3);
  Serial.println("---");
}

void procesarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "M1") { motorActivo = 0; Serial.println("Motor activo: 1"); }
  else if (cmd == "M2") { motorActivo = 1; Serial.println("Motor activo: 2"); }
  else if (cmd == "M3") { motorActivo = 2; Serial.println("Motor activo: 3"); }
  else if (cmd == "F") girarAdelante(motorActivo);
  else if (cmd == "R") girarReversa(motorActivo);
  else if (cmd == "S") { homingActivo[motorActivo] = false; detenerMotor(motorActivo); }
  else if (cmd == "SS") {
    for (uint8_t i = 0; i < 3; i++) { homingActivo[i] = false; detenerMotor(i); }
    Serial.println("PARADA DE EMERGENCIA: los 3 motores detenidos.");
  }
  else if (cmd == "CAL1") calibrarSensor(1);
  else if (cmd == "CAL2") calibrarSensor(2);
  else if (cmd == "CAL3") calibrarSensor(3);
  else if (cmd == "HOME1") iniciarHoming(0);
  else if (cmd == "HOME2") iniciarHoming(1);
  else if (cmd == "HOME3") iniciarHoming(2);
  else if (cmd == "HOME") {
    for (uint8_t i = 0; i < 3; i++) iniciarHoming(i);
  }
  else if (cmd == "STATUS") imprimirEstadoCompleto();
  else if (cmd.length() > 0) {
    Serial.println("Comando no reconocido. Usa: M1 M2 M3 F R S SS CAL1 CAL2 CAL3 HOME1 HOME2 HOME3 HOME STATUS");
  }
}

// ---------------- Programa principal ----------------

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

  // Restaura offsets de calibracion guardados en flash (si existen).
  int16_t off1 = cargarOffsetRaw(1);
  int16_t off2 = cargarOffsetRaw(2);
  offsetRaw3    = cargarOffsetRaw(3);
  as5600_1.setOffset(off1 * AS5600_RAW_TO_DEGREES);
  as5600_2.setOffset(off2 * AS5600_RAW_TO_DEGREES);
  Serial.print("Offsets cargados de flash -> #1: ");
  Serial.print(off1);
  Serial.print(" | #2: ");
  Serial.print(off2);
  Serial.print(" | #3: ");
  Serial.println(offsetRaw3);

  // IMPORTANTE: inicializa la referencia interna de seguimiento de vueltas
  // con la posicion FISICA real actual (no con 0 a secas). Sin este paso,
  // la primera llamada a getCumulativePosition()/actualizarPosicion3()
  // podria interpretar la diferencia entre "0 por defecto" y la posicion
  // real como si fuera una vuelta completa que nunca ocurrio.
  as5600_1.resetCumulativePosition(0);
  as5600_2.resetCumulativePosition(0);
  resetPosicionAcumulada3(0);
  Serial.println("Conteo de vueltas iniciado en 0 para los 3 sensores (vive solo en RAM, no sobrevive un reinicio).");

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
  Serial.println("Calibracion: CAL1 CAL2 CAL3 -> fija 0 deg en la posicion fisica actual de ese eje (persiste en flash)");
  Serial.println("HOME: HOME1 HOME2 HOME3 (un motor) | HOME (los 3 a la vez) -> regreso a 0 en lazo cerrado, sin PID");
  Serial.println("STATUS -> imprime el angulo/vueltas de los 3 encoders (ya no se imprime solo, para no saturar el Monitor Serial)");
  Serial.println("Motor activo por defecto: 1.");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarComando(linea);
  }

  // Actualizacion RAPIDA del conteo de vueltas: se hace en cada iteracion
  // de loop(), no solo cuando toca imprimir, porque si el eje gira mas de
  // media vuelta entre dos actualizaciones el algoritmo pierde la cuenta.
  as5600_1.getCumulativePosition(true);
  comOk1 = (as5600_1.lastError() == 0);

  as5600_2.getCumulativePosition(true);
  comOk2 = (as5600_2.lastError() == 0);

  comOk3 = actualizarPosicion3();

  // Avanza el homing (si esta activo) de cada motor, tambien en cada
  // iteracion de loop() para reaccionar rapido si hay que abortar.
  actualizarHoming(0);
  actualizarHoming(1);
  actualizarHoming(2);
}
