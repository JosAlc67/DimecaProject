// Fase 7 - Integracion final: 3 AS5600 + 3 BTS7960.
// Lee los 3 encoders periodicamente y permite controlar los 3 motores
// de forma independiente por comandos Serial, con la misma proteccion
// de la Fase 6 para el modo manual (F/R/HOME): no se permite invertir
// sentido sin pasar por parada. Ademas incluye PID real por motor
// (comandos PID1/PID2/PID3, ver esa seccion mas abajo) con ganancias
// obtenidas por el usuario en MATLAB a partir de las plantas
// identificadas con SysID_Motor.ino.
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
// PERSISTENCIA: la posicion acumulada de cada motor se guarda en flash
// (NVS) cada vez que ese motor se detiene (detenerMotor()), y ademas cada
// CHECKPOINT_INTERVALO_MS por seguridad si el ESP32 se apagara mientras un
// motor sigue en movimiento. Al arrancar se restaura, asi "0" sigue
// significando el mismo punto fisico entre reinicios/apagados. Esto NO
// funciona si el eje se mueve a mano sin alimentacion: el AS5600 es un
// sensor relativo para el conteo de vueltas, no detecta nada sin corriente.
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

// Posicion acumulada (multi-turn) de cada encoder, persistida para que "0"
// siga significando el mismo punto fisico entre reinicios/apagados. Solo
// es valido si el eje no se movio a mano mientras el ESP32 estaba apagado.
int32_t cargarPosicionAcum(uint8_t indice) {
  char clave[8];
  snprintf(clave, sizeof(clave), "pos%u", indice);
  prefs.begin(NVS_NAMESPACE, true);
  int32_t valor = prefs.getInt(clave, 0);
  prefs.end();
  return valor;
}

void guardarPosicionAcum(uint8_t indice, int32_t valor) {
  char clave[8];
  snprintf(clave, sizeof(clave), "pos%u", indice);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putInt(clave, valor);
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

// Actualizacion RAPIDA del conteo de vueltas de los 3 encoders. Se llama en
// cada iteracion de loop() (no solo cuando toca imprimir), porque si el eje
// gira mas de media vuelta entre dos actualizaciones el algoritmo pierde la
// cuenta. TAMBIEN se llama explicitamente despues de cualquier delay()
// bloqueante (por ejemplo en medirVelocidadGrados()), porque mientras
// loop() esta bloqueado esta actualizacion no ocurre sola.
void actualizarEncoders() {
  as5600_1.getCumulativePosition(true);
  comOk1 = (as5600_1.lastError() == 0);

  as5600_2.getCumulativePosition(true);
  comOk2 = (as5600_2.lastError() == 0);

  comOk3 = actualizarPosicion3();
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
// motores (no arrancaban). 255 (100%) si arranco, confirmado en pruebas.
// Duty minimo aceptado al calibrar velocidades (ver CALVEL mas abajo),
// para no terminar bajando tanto el duty de un motor rapido que deje de
// arrancar.
const uint8_t DUTY_MIN_SEGURO = 90;

// Duty por motor (ya no es un solo valor global): cada motor puede
// necesitar un duty distinto para girar a una velocidad real similar a
// los otros dos, dado que a igual duty no giran igual de rapido. Se
// inicializa en 150 (el valor validado antes de tener CALVEL) y se
// persiste en flash una vez calibrado.
uint8_t velocidadMotor[3] = {150, 150, 150};

int16_t cargarVelocidadMotor(uint8_t indice) {
  char clave[8];
  snprintf(clave, sizeof(clave), "vel%u", indice);
  prefs.begin(NVS_NAMESPACE, true);
  int16_t valor = prefs.getShort(clave, 150); // 150 = valor por defecto si nunca se calibro
  prefs.end();
  return valor;
}

void guardarVelocidadMotor(uint8_t indice, int16_t valor) {
  char clave[8];
  snprintf(clave, sizeof(clave), "vel%u", indice);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putShort(clave, valor);
  prefs.end();
}

const uint8_t PIN_RPWM[3] = {25, 27, 19};
const uint8_t PIN_LPWM[3] = {26, 18, 23};

#if !CORE_LEDC_V3
const uint8_t CANAL_RPWM[3] = {0, 2, 4};
const uint8_t CANAL_LPWM[3] = {1, 3, 5};
#endif

enum EstadoMotor { PARADO, ADELANTE, REVERSA };
EstadoMotor estadoMotor[3] = {PARADO, PARADO, PARADO};
uint8_t motorActivo = 0;

// Declarado aqui (no junto con el resto del PID mas abajo) porque
// girarAdelante()/girarReversa()/iniciarHoming() necesitan consultarlo
// para rechazar comandos manuales mientras el PID de ese motor este activo.
bool pidActivo[3] = {false, false, false};

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

// Posicion acumulada RAW (sin convertir a grados) del encoder emparejado
// con el motor `m` (mapeo confirmado 1-1, 2-2, 3-3).
int32_t leerPosicionAcumRaw(uint8_t m) {
  switch (m) {
    case 0: return as5600_1.getCumulativePosition(false);
    case 1: return as5600_2.getCumulativePosition(false);
    case 2: return posicionAcum3;
    default: return 0;
  }
}

// Guarda en flash la posicion acumulada actual del encoder del motor `m`,
// para poder restaurarla en el proximo arranque (ver setup()).
void guardarPosicionMotor(uint8_t m) {
  guardarPosicionAcum(m + 1, leerPosicionAcumRaw(m));
}

void detenerMotor(uint8_t m) {
  escribirPWM(PIN_RPWM[m], canalRPWM(m), 0);
  escribirPWM(PIN_LPWM[m], canalLPWM(m), 0);
  estadoMotor[m] = PARADO;
  guardarPosicionMotor(m);
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": DETENIDO");
}

void girarAdelante(uint8_t m) {
  if (pidActivo[m]) {
    Serial.println("RECHAZADO: PID activo en este motor, manda S primero.");
    return;
  }
  if (estadoMotor[m] != PARADO) {
    Serial.println("RECHAZADO: detén el motor (S) antes de cambiar de sentido.");
    return;
  }
  escribirPWM(PIN_LPWM[m], canalLPWM(m), 0);
  escribirPWM(PIN_RPWM[m], canalRPWM(m), velocidadMotor[m]);
  estadoMotor[m] = ADELANTE;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": GIRANDO ADELANTE (baja velocidad)");
}

void girarReversa(uint8_t m) {
  if (pidActivo[m]) {
    Serial.println("RECHAZADO: PID activo en este motor, manda S primero.");
    return;
  }
  if (estadoMotor[m] != PARADO) {
    Serial.println("RECHAZADO: detén el motor (S) antes de cambiar de sentido.");
    return;
  }
  escribirPWM(PIN_RPWM[m], canalRPWM(m), 0);
  escribirPWM(PIN_LPWM[m], canalLPWM(m), velocidadMotor[m]);
  estadoMotor[m] = REVERSA;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": GIRANDO REVERSA (baja velocidad)");
}

// ---------------- CALVEL: calibracion de velocidad entre motores ----------------
//
// A igual duty, los 3 motores no giran necesariamente a la misma velocidad
// real (friccion, tolerancias de fabricacion, etc). CALVEL mide con el
// propio encoder cuantos grados/segundo gira cada motor a su duty actual,
// y ajusta el duty de cada uno (de forma proporcional, asumiendo relacion
// aproximadamente lineal duty<->velocidad en este rango) para que los 3
// terminen girando a una velocidad similar a la del motor mas lento -no
// se puede acelerar un motor mas alla de lo que ya da a duty 255, asi que
// siempre se iguala hacia abajo, nunca hacia arriba.
//
// Esto es calibracion de lazo abierto (se mide una vez, se fija un duty
// nuevo y listo) - no es control de velocidad en tiempo real ni PID.

// 3000ms (antes 1500ms): con una ventana corta, el transitorio de arranque
// (fraccion de segundo hasta alcanzar velocidad estable) pesa demasiado en
// la medicion y da resultados poco repetibles entre corridas (~10% de
// variacion observada en hardware con 1.5s). Una ventana mas larga diluye
// ese transitorio y promedia mejor cualquier variacion de corto plazo.
const uint32_t CALVEL_DURACION_MS = 3000;

// Gira el motor `m` hacia adelante durante `duracionMs` y devuelve la
// velocidad medida en grados/segundo (valor absoluto). Requiere que el
// motor este detenido antes de llamar.
//
// Usa un sondeo periodico (actualizarEncoders() cada 20ms) en vez de un
// solo delay() largo: si se usara un delay() bloqueante único con una
// ventana larga, y el motor llegara a girar mas de media vuelta durante
// esa espera, el algoritmo de deteccion de vuelta se confundiria (el
// mismo riesgo que ya se resolvio para loop() en general, aqui reaparece
// si la ventana de medicion es larga y no se refresca seguido).
float medirVelocidadGrados(uint8_t m, uint32_t duracionMs) {
  actualizarEncoders(); // asegura que "antes" sea una lectura fresca
  int32_t antes = leerPosicionAcumRaw(m);
  girarAdelante(m);

  uint32_t inicio = millis();
  while (millis() - inicio < duracionMs) {
    actualizarEncoders();
    delay(20);
  }

  detenerMotor(m);
  int32_t despues = leerPosicionAcumRaw(m);

  float gradosRecorridos = fabs((float)(despues - antes)) * (360.0f / 4096.0f);
  return gradosRecorridos / (duracionMs / 1000.0f);
}

void calibrarVelocidades() {
  for (uint8_t i = 0; i < 3; i++) {
    if (estadoMotor[i] != PARADO) {
      Serial.println("RECHAZADO: los 3 motores deben estar detenidos (S/SS) antes de CALVEL.");
      return;
    }
  }

  Serial.println("Calibrando velocidades... (gira cada motor por turnos, unos segundos)");

  float velocidad[3];
  for (uint8_t i = 0; i < 3; i++) {
    velocidad[i] = medirVelocidadGrados(i, CALVEL_DURACION_MS);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(": duty=");
    Serial.print(velocidadMotor[i]);
    Serial.print(" -> ");
    Serial.print(velocidad[i], 2);
    Serial.println(" deg/s medidos");
  }

  float velocidadObjetivo = velocidad[0];
  for (uint8_t i = 1; i < 3; i++) {
    if (velocidad[i] < velocidadObjetivo) velocidadObjetivo = velocidad[i];
  }

  Serial.print("Velocidad objetivo (la del motor mas lento): ");
  Serial.print(velocidadObjetivo, 2);
  Serial.println(" deg/s");

  for (uint8_t i = 0; i < 3; i++) {
    if (velocidad[i] <= 0.01f) {
      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.println(": ADVERTENCIA - velocidad medida ~0, no se ajusta (revisa el motor).");
      continue;
    }
    float factor = velocidadObjetivo / velocidad[i];
    int16_t nuevaDuty = (int16_t)round(velocidadMotor[i] * factor);
    if (nuevaDuty < DUTY_MIN_SEGURO) {
      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.print(": el duty calculado (");
      Serial.print(nuevaDuty);
      Serial.print(") es menor al minimo seguro (");
      Serial.print(DUTY_MIN_SEGURO);
      Serial.println("), se deja en el minimo - este motor seguira algo mas rapido que los otros.");
      nuevaDuty = DUTY_MIN_SEGURO;
    }
    if (nuevaDuty > 255) nuevaDuty = 255;

    velocidadMotor[i] = (uint8_t)nuevaDuty;
    guardarVelocidadMotor(i + 1, nuevaDuty);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(": duty ajustado a ");
    Serial.print(nuevaDuty);
    Serial.println(" (guardado en flash).");
  }

  Serial.println("Calibracion de velocidad terminada. Puedes correr CALVEL de nuevo para refinar.");
}

// Variante manual: "CALVEL M<1-3> <duty>". En vez de tomar como objetivo
// la velocidad del motor mas lento, mide el motor `motorRef` EXACTAMENTE
// al duty `dutyRef` que se le indique, usa esa velocidad medida como
// objetivo fijo, y ajusta los otros dos motores para acercarse a ella.
// El motor de referencia queda fijado en `dutyRef` (no se recalcula).
void calibrarVelocidadesConReferencia(uint8_t motorRef, uint8_t dutyRef) {
  for (uint8_t i = 0; i < 3; i++) {
    if (estadoMotor[i] != PARADO) {
      Serial.println("RECHAZADO: los 3 motores deben estar detenidos (S/SS) antes de CALVEL.");
      return;
    }
  }

  Serial.print("Calibrando con referencia: Motor ");
  Serial.print(motorRef + 1);
  Serial.print(" fijado a duty ");
  Serial.println(dutyRef);

  velocidadMotor[motorRef] = dutyRef;
  float velocidadObjetivo = medirVelocidadGrados(motorRef, CALVEL_DURACION_MS);
  guardarVelocidadMotor(motorRef + 1, dutyRef);
  Serial.print("Motor ");
  Serial.print(motorRef + 1);
  Serial.print(" (referencia): duty=");
  Serial.print(dutyRef);
  Serial.print(" -> ");
  Serial.print(velocidadObjetivo, 2);
  Serial.println(" deg/s medidos. Esa es la velocidad objetivo para los otros dos.");

  for (uint8_t i = 0; i < 3; i++) {
    if (i == motorRef) continue; // ya quedo fijo, no se recalcula

    float velocidadMedida = medirVelocidadGrados(i, CALVEL_DURACION_MS);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(": duty=");
    Serial.print(velocidadMotor[i]);
    Serial.print(" -> ");
    Serial.print(velocidadMedida, 2);
    Serial.println(" deg/s medidos");

    if (velocidadMedida <= 0.01f) {
      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.println(": ADVERTENCIA - velocidad medida ~0, no se ajusta (revisa el motor).");
      continue;
    }

    float factor = velocidadObjetivo / velocidadMedida;
    int32_t nuevaDuty = (int32_t)round((float)velocidadMotor[i] * factor);

    if (nuevaDuty > 255) {
      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.println(": ADVERTENCIA - ni siquiera a duty 255 alcanza la velocidad de referencia. Se deja en 255 (quedara mas lento que el objetivo).");
      nuevaDuty = 255;
    } else if (nuevaDuty < DUTY_MIN_SEGURO) {
      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.print(": el duty calculado (");
      Serial.print(nuevaDuty);
      Serial.print(") es menor al minimo seguro (");
      Serial.print(DUTY_MIN_SEGURO);
      Serial.println("), se deja en el minimo - este motor seguira algo mas rapido que el objetivo.");
      nuevaDuty = DUTY_MIN_SEGURO;
    }

    velocidadMotor[i] = (uint8_t)nuevaDuty;
    guardarVelocidadMotor(i + 1, (int16_t)nuevaDuty);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(": duty ajustado a ");
    Serial.print(nuevaDuty);
    Serial.println(" (guardado en flash).");
  }

  Serial.println("Calibracion con referencia terminada.");
}

// ---------------- HOME (regreso a 0) - lazo cerrado simple, SIN PID ----------------
//
// Regla fija para los 3 motores, confirmada en hardware: F (adelante)
// RESTA del acumulado y R (reversa) SUMA. Por lo tanto, para volver a 0:
// si el acumulado es NEGATIVO se manda REVERSA (R, que suma y lo acerca
// a 0); si es POSITIVO se manda ADELANTE (F, que resta y lo acerca a 0).
// Se detiene al llegar dentro de la tolerancia. No hay ganancias ni
// ajuste fino de velocidad - es control bang-bang, el mas simple posible.
//
// Mapeo motor->encoder confirmado por el usuario: 1-1, 2-2, 3-3 (sin cruces).
//
// Se mantiene una sola proteccion: si no mejora el error en
// HOMING_MARGEN_PROGRESO_MS, o si se excede HOMING_TIMEOUT_MS en total,
// aborta y detiene el motor - con la regla fija, si esto pasa es señal de
// un problema real (encoder desacoplado, motor sin fuerza, etc.), no de
// un signo mal elegido.

const float    TOLERANCIA_HOMING_DEG     = 3.0f;   // se considera "en home" dentro de esto
const uint32_t HOMING_TIMEOUT_MS         = 20000;  // aborta si tarda mas de esto en total
const uint32_t HOMING_MARGEN_PROGRESO_MS = 4000;   // debe mejorar el error en este tiempo

bool homingActivo[3]         = {false, false, false};
uint32_t homingInicio[3]     = {0, 0, 0};
uint32_t homingUltimoProgreso[3] = {0, 0, 0};
float homingMejorError[3]    = {0, 0, 0};

// Lee la posicion acumulada (continua, sin wraparound) en grados del
// encoder correspondiente al motor `m` (0, 1 o 2).
// Mapeo fisico confirmado por el usuario: cada AS5600 esta emparejado con
// su motor del mismo numero (1-1, 2-2, 3-3), sin cruces.
bool leerAnguloAcumuladoGrados(uint8_t m, float &grados) {
  int32_t posRaw;
  bool ok;
  switch (m) {
    case 0: ok = comOk1; posRaw = as5600_1.getCumulativePosition(false); break; // Motor 1 -> AS5600 #1
    case 1: ok = comOk2; posRaw = as5600_2.getCumulativePosition(false); break; // Motor 2 -> AS5600 #2
    case 2: ok = comOk3; posRaw = posicionAcum3; break;                        // Motor 3 -> AS5600 #3
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
  if (pidActivo[m]) {
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.println(": RECHAZADO - PID activo en este motor, manda S primero.");
    return;
  }
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
    detenerHoming(m, "ABORTADO - no se acerca a 0. Revisa el encoder/acople mecanico.");
    return;
  }

  // Regla fija: negativo -> reversa (R, suma), positivo -> adelante (F, resta).
  bool debeIrAdelante = (grados > 0);
  EstadoMotor direccionDeseada = debeIrAdelante ? ADELANTE : REVERSA;

  if (estadoMotor[m] != direccionDeseada) {
    if (estadoMotor[m] != PARADO) detenerMotor(m);
    if (debeIrAdelante) girarAdelante(m);
    else girarReversa(m);
  }
}

// ---------------- PID real por motor (identificado con System Identification Toolbox) ----------------
//
// Ganancias obtenidas por el usuario en MATLAB a partir de las plantas
// identificadas con SysID_Motor.ino (una funcion de transferencia
// posicion(deg)/duty por motor, ver esa carpeta). A diferencia de HOME
// (bang-bang, velocidad fija, se detiene al llegar), este PID escribe un
// duty CONTINUO y con signo, proporcional al error - no pasa por
// girarAdelante()/girarReversa() ni por la maquina de estados de esas
// funciones, porque un PID bien sintonizado cruza el cero de salida de
// forma suave conforme se acerca al setpoint, sin el "cambio brusco de
// sentido a velocidad fija" que esa maquina de estados esta pensada para
// evitar en el modo manual/bang-bang.
//
// IMPORTANTE sobre el signo de Kp: las 3 ganancias son NEGATIVAS. Esto es
// correcto, no un error de MATLAB: ya se confirmo en hardware (ver HOME)
// que "adelante" (duty positivo) RESTA del acumulado y "reversa" SUMA -
// es decir la planta identificada tiene ganancia negativa. Para que la
// realimentacion sea efectivamente negativa (estabilizante), el
// controlador necesita ganancia negativa tambien. Los signos coinciden
// exactamente con lo que ya se sabia del hardware.
//
// Se dejan anotados los valores de P y PD por si se quieren probar en
// vez del PID completo (bastaria con poner Ki=0 y/o Kd=0 en el arreglo).
//
// --- Motor 1 ---   P: Kp=-54.708699 | PD: Kp=-2.096146, Kd=0.000000
// --- Motor 2 ---   P: Kp=-77.922487 | PD: Kp=-2.120143, Kd=0.000000
// --- Motor 3 ---   P: Kp=-77.537923 | PD: Kp=-2.264696, Kd=0.000000

struct GananciasPID { float Kp; float Ki; float Kd; };

GananciasPID pidMotor[3] = {
  {-2.080548f, -0.420325f, -0.165080f}, // Motor 1 (PID completo)
  {-2.086742f, -0.374856f, -0.150000f}, // Motor 2 (Kd=0 tal cual identificado en MATLAB; se
                                         // agrega -0.15 a mano como punto de partida SIN
                                         // VALIDAR, tomando de referencia el Kd de Motor 1/3
                                         // que tienen Kp/Ki similares. Sin derivativo el motor
                                         // cabeceaba bastante antes de llegar al setpoint - el
                                         // termino derivativo es justo el que amortigua eso.
  {-2.249454f, -0.441603f, -0.179295f}  // Motor 3 (PID completo)
};

const uint32_t PID_INTERVALO_MS = 20;    // ciclo de control: 50 Hz
const float    PID_SALIDA_MAX   = 255.0f; // mismas unidades (duty 0-255) que uso la identificacion

// Piso de friccion estatica: en Fase 6 se confirmo que duty=60 (~23%) NO
// alcanza a arrancar estos motores, y duty=150 (~59%) si. Sin este piso, al
// achicarse el error el PID pide una salida chica que nunca mueve el motor
// - se queda "pegado" esperando a que el integral se infle solo, que es
// justo el error lento y persistente que se vio en las pruebas. VALOR SIN
// VALIDAR EN HARDWARE TODAVIA (punto de partida entre 60 y 150): si el
// motor sigue sin arrancar cerca del setpoint, subelo; si pica/vibra
// demasiado, bajalo.
const uint8_t  PID_DUTY_MIN_FRICCION = 80;

// Dentro de esta banda alrededor del setpoint se considera "llegado": el
// AS5600 ya tiene ~1.5 deg de error de linealidad propio, asi que perseguir
// un error menor que eso es perseguir ruido del sensor, no error real - y
// con el piso de friccion de arriba, insistir ahi solo logra que el motor
// pique sin asentarse nunca.
const float    PID_ZONA_MUERTA_DEG = 1.5f;

float    pidSetpoint[3]      = {0, 0, 0}; // grados, en la misma escala continua "acumulado"
float    pidIntegral[3]      = {0, 0, 0};
float    pidErrorAnterior[3] = {0, 0, 0};
uint32_t pidUltimoTiempo[3]  = {0, 0, 0};

// Escribe un duty CON SIGNO en el motor `m`: positivo = adelante (RPWM),
// negativo = reversa (LPWM), 0 = detenido. Bypassa girarAdelante()/
// girarReversa() a proposito (ver nota arriba), pero si actualiza
// estadoMotor[] para que STATUS y el resto del sistema lo reporten bien.
void escribirComandoMotor(uint8_t m, float u) {
  float magnitud = fabs(u);
  if (magnitud > PID_SALIDA_MAX) magnitud = PID_SALIDA_MAX;
  uint8_t duty = (uint8_t)magnitud;

  if (u > 0.0f) {
    escribirPWM(PIN_LPWM[m], canalLPWM(m), 0);
    escribirPWM(PIN_RPWM[m], canalRPWM(m), duty);
    estadoMotor[m] = ADELANTE;
  } else if (u < 0.0f) {
    escribirPWM(PIN_RPWM[m], canalRPWM(m), 0);
    escribirPWM(PIN_LPWM[m], canalLPWM(m), duty);
    estadoMotor[m] = REVERSA;
  } else {
    escribirPWM(PIN_RPWM[m], canalRPWM(m), 0);
    escribirPWM(PIN_LPWM[m], canalLPWM(m), 0);
    estadoMotor[m] = PARADO;
  }
}

void iniciarPID(uint8_t m, float setpointDeg) {
  if (homingActivo[m]) {
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.println(": RECHAZADO - HOME activo en este motor, manda S primero.");
    return;
  }
  if (estadoMotor[m] != PARADO) {
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.println(": RECHAZADO - detenlo (S) antes de iniciar PID.");
    return;
  }
  float actual;
  if (!leerAnguloAcumuladoGrados(m, actual)) {
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.println(": RECHAZADO - encoder sin comunicacion, no se puede iniciar PID.");
    return;
  }
  pidSetpoint[m]      = setpointDeg;
  pidIntegral[m]      = 0.0f;
  pidErrorAnterior[m] = setpointDeg - actual;
  pidUltimoTiempo[m]  = millis();
  pidActivo[m]        = true;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.print(": PID activo, setpoint=");
  Serial.print(setpointDeg, 2);
  Serial.print(" deg (actual=");
  Serial.print(actual, 2);
  Serial.println(" deg)");
}

void detenerPID(uint8_t m) {
  pidActivo[m] = false;
  detenerMotor(m);
}

// Se llama en CADA iteracion de loop() para los motores con PID activo.
// Internamente se autolimita a PID_INTERVALO_MS (no necesita otro control
// de tiempo por fuera).
void actualizarPID(uint8_t m) {
  if (!pidActivo[m]) return;

  uint32_t ahora = millis();
  if (ahora - pidUltimoTiempo[m] < PID_INTERVALO_MS) return;
  float dt = (ahora - pidUltimoTiempo[m]) / 1000.0f;
  pidUltimoTiempo[m] = ahora;

  float actual;
  if (!leerAnguloAcumuladoGrados(m, actual)) {
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.println(": PID ABORTADO - se perdio comunicacion con el encoder.");
    detenerPID(m);
    return;
  }

  float error = pidSetpoint[m] - actual;

  if (fabs(error) <= PID_ZONA_MUERTA_DEG) {
    // Ya llegamos (dentro del margen de ruido del sensor): no seguir
    // forzando el motor contra su propia friccion estatica por una
    // fraccion de grado que ademas no se puede medir con certeza.
    pidErrorAnterior[m] = error;
    escribirComandoMotor(m, 0.0f);
    return;
  }

  float integralTentativa = pidIntegral[m] + error * dt;
  float derivada = (dt > 0.0f) ? (error - pidErrorAnterior[m]) / dt : 0.0f;

  GananciasPID &g = pidMotor[m];
  float salida = g.Kp * error + g.Ki * integralTentativa + g.Kd * derivada;

  // Anti-windup simple: si la salida ya esta saturada, no sigue acumulando
  // el termino integral (evita que el integrador se "infle" sin limite
  // mientras el motor de todos modos no puede ir mas rapido).
  if (salida > PID_SALIDA_MAX || salida < -PID_SALIDA_MAX) {
    salida = constrain(salida, -PID_SALIDA_MAX, PID_SALIDA_MAX);
  } else {
    pidIntegral[m] = integralTentativa;
  }

  // Piso de friccion: si el PID pide movimiento pero por debajo del duty
  // que vence la friccion estatica, subirlo a ese piso (conservando el
  // signo) - si no, el motor nunca arranca cerca del setpoint.
  if (salida > 0.0f && salida < (float)PID_DUTY_MIN_FRICCION) {
    salida = (float)PID_DUTY_MIN_FRICCION;
  } else if (salida < 0.0f && salida > -(float)PID_DUTY_MIN_FRICCION) {
    salida = -(float)PID_DUTY_MIN_FRICCION;
  }

  pidErrorAnterior[m] = error;
  escribirComandoMotor(m, salida);
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

  for (uint8_t m = 0; m < 3; m++) {
    if (!pidActivo[m]) continue;
    float actual;
    leerAnguloAcumuladoGrados(m, actual);
    Serial.print("Motor ");
    Serial.print(m + 1);
    Serial.print(": PID activo | setpoint=");
    Serial.print(pidSetpoint[m], 2);
    Serial.print(" deg | actual=");
    Serial.print(actual, 2);
    Serial.print(" deg | error=");
    Serial.print(pidSetpoint[m] - actual, 2);
    Serial.println(" deg");
  }
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
  else if (cmd == "S") { homingActivo[motorActivo] = false; pidActivo[motorActivo] = false; detenerMotor(motorActivo); }
  else if (cmd == "SS") {
    for (uint8_t i = 0; i < 3; i++) { homingActivo[i] = false; pidActivo[i] = false; detenerMotor(i); }
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
  else if (cmd == "CALVEL") calibrarVelocidades();
  else if (cmd.startsWith("CALVEL ")) {
    // Formato: CALVEL M<1-3> <duty 0-255>. Ejemplo: CALVEL M3 225
    String resto = cmd.substring(7);
    resto.trim();
    int espacio = resto.indexOf(' ');
    bool formatoValido = false;
    if (espacio > 0) {
      String parteMotor = resto.substring(0, espacio);
      String parteDuty = resto.substring(espacio + 1);
      parteMotor.trim();
      parteDuty.trim();
      if (parteMotor.length() == 2 && parteMotor.charAt(0) == 'M') {
        int numMotor = parteMotor.substring(1).toInt();
        int duty = parteDuty.toInt();
        if (numMotor >= 1 && numMotor <= 3 && duty >= 0 && duty <= 255) {
          formatoValido = true;
          calibrarVelocidadesConReferencia((uint8_t)(numMotor - 1), (uint8_t)duty);
        }
      }
    }
    if (!formatoValido) {
      Serial.println("Formato invalido. Usa: CALVEL M<1-3> <duty 0-255>. Ejemplo: CALVEL M3 225");
    }
  }
  else if (cmd.startsWith("PID1 ")) iniciarPID(0, cmd.substring(5).toFloat());
  else if (cmd.startsWith("PID2 ")) iniciarPID(1, cmd.substring(5).toFloat());
  else if (cmd.startsWith("PID3 ")) iniciarPID(2, cmd.substring(5).toFloat());
  else if (cmd.length() > 0) {
    Serial.println("Comando no reconocido. Usa: M1 M2 M3 F R S SS CAL1 CAL2 CAL3 HOME1 HOME2 HOME3 HOME STATUS CALVEL CALVEL M<1-3> <duty> PID1/2/3 <setpoint_deg>");
  }
}

// ---------------- Programa principal ----------------

const uint32_t CHECKPOINT_INTERVALO_MS = 60000; // respaldo de posicion cada 60s
uint32_t ultimoCheckpoint = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Fase 7: integracion final (3 AS5600 + 3 BTS7960 + PID identificado) ===");

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

  // Restaura la posicion acumulada guardada en flash (si existe) en vez de
  // reiniciar siempre en 0, para que "0" siga siendo el mismo punto fisico
  // entre reinicios/apagados. Esto asume que el eje no se movio a mano
  // mientras el ESP32 estaba apagado (el AS5600 es un sensor relativo para
  // el conteo de vueltas, no puede detectar movimiento sin alimentacion).
  // resetCumulativePosition()/resetPosicionAcumulada3() tambien resincronizan
  // la referencia interna con la posicion FISICA real actual, evitando que
  // la primera lectura interprete un salto falso.
  int32_t pos1 = cargarPosicionAcum(1);
  int32_t pos2 = cargarPosicionAcum(2);
  int32_t pos3 = cargarPosicionAcum(3);
  as5600_1.resetCumulativePosition(pos1);
  as5600_2.resetCumulativePosition(pos2);
  resetPosicionAcumulada3(pos3);
  Serial.print("Posicion restaurada de flash (cuentas RAW) -> #1: ");
  Serial.print(pos1);
  Serial.print(" | #2: ");
  Serial.print(pos2);
  Serial.print(" | #3: ");
  Serial.println(pos3);

  // Restaura el duty por motor calibrado con CALVEL (150 por defecto si
  // nunca se calibro).
  velocidadMotor[0] = (uint8_t)cargarVelocidadMotor(1);
  velocidadMotor[1] = (uint8_t)cargarVelocidadMotor(2);
  velocidadMotor[2] = (uint8_t)cargarVelocidadMotor(3);
  Serial.print("Duty por motor cargado de flash -> #1: ");
  Serial.print(velocidadMotor[0]);
  Serial.print(" | #2: ");
  Serial.print(velocidadMotor[1]);
  Serial.print(" | #3: ");
  Serial.println(velocidadMotor[2]);

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
  Serial.println("CALVEL -> mide y empareja la velocidad real de los 3 motores contra el mas lento (requiere los 3 detenidos, persiste en flash)");
  Serial.println("CALVEL M<1-3> <duty> -> igual, pero fija el motor indicado a ese duty y usa su velocidad como objetivo. Ej: CALVEL M3 225");
  Serial.println("PID1/PID2/PID3 <setpoint_deg> -> activa el PID real (ganancias identificadas) en ese motor, con ese setpoint. Ej: PID2 45.0");
  Serial.println("Motor activo por defecto: 1.");
  Serial.println();

  ultimoCheckpoint = millis();
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarComando(linea);
  }

  actualizarEncoders();

  // Avanza el homing (si esta activo) de cada motor, tambien en cada
  // iteracion de loop() para reaccionar rapido si hay que abortar.
  actualizarHoming(0);
  actualizarHoming(1);
  actualizarHoming(2);

  // Avanza el PID (si esta activo) de cada motor. actualizarPID() se
  // autolimita a PID_INTERVALO_MS, es seguro llamarla en cada iteracion.
  actualizarPID(0);
  actualizarPID(1);
  actualizarPID(2);

  // Respaldo periodico en flash, por si el ESP32 pierde alimentacion
  // mientras un motor sigue en movimiento (detenerMotor() ya guarda al
  // detenerse, esto es solo la red de seguridad para ese caso puntual).
  uint32_t ahora = millis();
  if (ahora - ultimoCheckpoint >= CHECKPOINT_INTERVALO_MS) {
    ultimoCheckpoint = ahora;
    for (uint8_t i = 0; i < 3; i++) guardarPosicionMotor(i);
  }
}
