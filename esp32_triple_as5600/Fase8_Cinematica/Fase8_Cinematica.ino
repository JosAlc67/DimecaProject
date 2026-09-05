// Fase 8 - Cinematica: parte de Fase 7 (3 AS5600 + 3 BTS7960 + PID real
// por motor) y agrega una capa de traduccion arriba: el comando
// BEND <theta_deg> <phi_deg> convierte un doblez deseado del tentaculo
// COMPLETO (34cm, 21 segmentos pasivos, cuerpo conico) en los 3 setpoints
// de PID correspondientes - modelo PCC, 3 tendones a 120 grados que corren
// por dentro de los 21 segmentos y anclan solo en la punta, ver la seccion
// "Cinematica" mas abajo. Ademas incluye TURN <theta_deg> <phi_deg>
// [vel_deg_s], que gira la direccion del doblez manteniendo theta fijo -
// a diferencia de BEND (que salta directo al setpoint final de posicion
// de cada motor), TURN controla VELOCIDAD en lazo abierto (feedforward),
// dando un giro conico continuo y suave en vez de reposicionamientos
// discretos - pensado para no soltar algo que el tentaculo tenga agarrado
// al cambiar de direccion. Todo lo demas (encoders, motores, HOME, PID
// individual) es identico a Fase 7.
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

int32_t  posicionAcum3        = 0;
int16_t  ultimaPosicion3      = 0;
bool     primeraLectura3      = true;
uint32_t tiempoUltimaLectura3 = 0;

// Cota de velocidad fisicamente plausible del eje, con margen holgado
// sobre lo medido en CALVEL/PID a duty maximo (unas decenas de deg/s).
// Se usa SOLO para filtrar lecturas corruptas del bus #3: a diferencia
// de #1/#2 (I2C por hardware), el #3 es bit-banged por software, sin CRC
// ni filtro de glitches, y mucho mas sensible al ruido electrico que
// meten los BTS7960 al conmutar PWM cerca del cable. Una sola lectura
// corrupta que "parezca" un salto de mas de media vuelta se tomaba antes
// como una vuelta completa real (+-4096 cuentas, +-360 grados) de golpe,
// corrompiendo el acumulado permanentemente - eso es lo que producia el
// desfase de decenas de grados visto en las pruebas con Motor 3, peor
// mientras mas rapido/largo giraba (mas lecturas por I2C software en
// ese tramo = mas chance de que alguna saliera corrupta).
const float VELOCIDAD_MAX_PLAUSIBLE_DEG_S = 150.0f;

// La libreria oficial de RobTillaart (usada para #1/#2), ante un fallo I2C,
// NO reporta error de inmediato: reutiliza internamente el ultimo angulo
// bueno leido y sigue - un solo tropiezo transitorio del bus no interrumpe
// nada. Replicamos ese mismo criterio de tolerancia aqui: solo despues de
// varios fallos SEGUIDOS se reporta "sin comunicacion" de verdad. Sin esto,
// un solo glitch transitorio del bus #3 (mas propenso a esto por ser
// bit-banged por software) abortaba de golpe un PID o HOME activo en Motor
// 3, aunque el sensor siguiera funcionando bien el ciclo siguiente.
uint8_t fallosConsecutivos3 = 0;
const uint8_t MAX_FALLOS_CONSECUTIVOS_BUS3 = 5;

bool actualizarPosicion3() {
  uint16_t raw;
  if (!leerAnguloConOffset_bus3(raw)) {
    fallosConsecutivos3++;
    return (fallosConsecutivos3 < MAX_FALLOS_CONSECUTIVOS_BUS3);
  }
  fallosConsecutivos3 = 0;
  int16_t value = (int16_t)raw;

  uint32_t ahora = micros();

  if (primeraLectura3) {
    ultimaPosicion3 = value;
    tiempoUltimaLectura3 = ahora;
    primeraLectura3 = false;
    return true;
  }

  int32_t deltaBruto;
  if ((ultimaPosicion3 > 2048) && (value < (ultimaPosicion3 - 2048))) {
    deltaBruto = 4096 - ultimaPosicion3 + value;      // vuelta completa CW
  } else if ((value > 2048) && (ultimaPosicion3 < (value - 2048))) {
    deltaBruto = -4096 - ultimaPosicion3 + value;     // vuelta completa CCW
  } else {
    deltaBruto = value - ultimaPosicion3;
  }

  float dtSeg = (ahora - tiempoUltimaLectura3) / 1000000.0f;
  float velocidadImplicada = fabs((float)deltaBruto) * (360.0f / 4096.0f) / max(dtSeg, 0.0001f);

  if (velocidadImplicada > VELOCIDAD_MAX_PLAUSIBLE_DEG_S) {
    // Lectura descartada: el salto implica una velocidad imposible para
    // este motor, casi seguro ruido del bus. No se toca ultimaPosicion3
    // ni posicionAcum3 (se reintenta con la siguiente lectura); comOk3
    // sigue en true porque la transaccion I2C en si no fallo.
    return true;
  }

  posicionAcum3 += deltaBruto;
  ultimaPosicion3 = value;
  tiempoUltimaLectura3 = ahora;
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
  tiempoUltimaLectura3 = micros();
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

// ---------------- Velocidad estimada en vivo (para STATUS) ----------------
//
// Velocidad actual (deg/s, con signo) de cada motor, para mostrar en
// STATUS. Se recalcula sola en el fondo cada VELOCIDAD_ESTIMA_INTERVALO_MS
// viendo cuanto se movio el acumulado de cada encoder en ese intervalo -
// no importa si el motor esta en manual, HOME o PID, cualquier movimiento
// real se refleja. Se pasa por un filtro exponencial (EMA) para no
// mostrar puro ruido de cuantizacion del encoder (12 bits) de golpe.
const uint32_t VELOCIDAD_ESTIMA_INTERVALO_MS = 200;
const float    VELOCIDAD_EMA_ALFA            = 0.3f; // 0-1: mas alto sigue mas rapido pero mas ruidoso

int32_t  velPosAnterior[3]        = {0, 0, 0};
uint32_t velTiempoAnterior        = 0;
float    velocidadEstimadaDegS[3] = {0.0f, 0.0f, 0.0f};

void actualizarVelocidadEstimada() {
  uint32_t ahora = millis();
  if (velTiempoAnterior != 0 && (ahora - velTiempoAnterior) < VELOCIDAD_ESTIMA_INTERVALO_MS) return;

  if (velTiempoAnterior != 0) {
    float dtSeg = (ahora - velTiempoAnterior) / 1000.0f;
    for (uint8_t m = 0; m < 3; m++) {
      int32_t posActual = leerPosicionAcumRaw(m);
      float instantanea = (posActual - velPosAnterior[m]) * (360.0f / 4096.0f) / dtSeg;
      velocidadEstimadaDegS[m] = VELOCIDAD_EMA_ALFA * instantanea + (1.0f - VELOCIDAD_EMA_ALFA) * velocidadEstimadaDegS[m];
      velPosAnterior[m] = posActual;
    }
  } else {
    for (uint8_t m = 0; m < 3; m++) velPosAnterior[m] = leerPosicionAcumRaw(m);
  }
  velTiempoAnterior = ahora;
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

// Fija el duty de un motor a mano (sin medir nada) y lo persiste en
// flash, igual que CALVEL pero sin el paso de medicion automatica - para
// cuando el ajuste de velocidad se calcula aparte a partir de datos ya
// tomados (ver comando SETVEL<1-3> mas abajo).
void fijarVelocidadMotor(uint8_t m, uint8_t duty) {
  velocidadMotor[m] = duty;
  guardarVelocidadMotor(m + 1, duty);
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.print(": duty fijado a mano en ");
  Serial.print(duty);
  Serial.println(" (guardado en flash).");
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

// Cerca del setpoint se baja la velocidad: a velocidadMotor[m] (tipico
// ~150) el motor sigue girando un poco por inercia despues de cortar el
// PWM, y ese "coast" era buena parte del error final constante que se
// veia en HOME. Con menos velocidad al final, menos inercia, menos
// sobrepaso, y el resultado queda mas cerca de 0.
const float    HOME_ZONA_LENTA_DEG       = 15.0f;
const uint8_t  HOME_DUTY_LENTO           = DUTY_MIN_SEGURO; // 90: ya confirmado como piso seguro

bool homingActivo[3]         = {false, false, false};
uint32_t homingInicio[3]     = {0, 0, 0};
uint32_t homingUltimoProgreso[3] = {0, 0, 0};
float homingMejorError[3]    = {0, 0, 0};
uint8_t homingDutyActual[3]  = {0, 0, 0};

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

// Escribe PWM directo con sentido y duty elegidos, para HOME. A
// diferencia de girarAdelante()/girarReversa() (que siempre usan
// velocidadMotor[m] fijo), este permite bajar la velocidad cerca del
// setpoint sin pasar por el guard de "debe estar PARADO" de esas
// funciones - HOME ya controla el flujo por su cuenta.
void homingMover(uint8_t m, bool adelante, uint8_t duty) {
  if (adelante) {
    escribirPWM(PIN_LPWM[m], canalLPWM(m), 0);
    escribirPWM(PIN_RPWM[m], canalRPWM(m), duty);
    estadoMotor[m] = ADELANTE;
  } else {
    escribirPWM(PIN_RPWM[m], canalRPWM(m), 0);
    escribirPWM(PIN_LPWM[m], canalLPWM(m), duty);
    estadoMotor[m] = REVERSA;
  }
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
  homingDutyActual[m] = 0;
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
  uint8_t dutyDeseado = (errorAbs <= HOME_ZONA_LENTA_DEG) ? HOME_DUTY_LENTO : velocidadMotor[m];

  if (estadoMotor[m] != direccionDeseada) {
    if (estadoMotor[m] != PARADO) detenerMotor(m);
    homingMover(m, debeIrAdelante, dutyDeseado);
    homingDutyActual[m] = dutyDeseado;
  } else if (dutyDeseado != homingDutyActual[m]) {
    // Mismo sentido, solo se entra/sale de la zona lenta - no hace falta
    // detenerse primero para esto.
    homingMover(m, debeIrAdelante, dutyDeseado);
    homingDutyActual[m] = dutyDeseado;
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
  // El guard de "debe estar PARADO" tiene sentido para bang-bang (F/R/HOME:
  // cambiar de sentido de golpe a velocidad fija es riesgoso), pero no para
  // PID - redirigir un lazo cerrado que ya esta corriendo (pidActivo[m])
  // hacia un setpoint nuevo es normal y seguro, el PID hace la transicion
  // solo. Sin esta excepcion, BEND no podia retargetear un motor que
  // todavia estuviera convergiendo del BEND anterior.
  if (estadoMotor[m] != PARADO && !pidActivo[m]) {
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

// ---------------- Cinematica (PCC: 3 cables a 120 grados) ----------------
//
// Traduce un doblez deseado del segmento - theta (magnitud, 0 = recto) y
// phi (direccion, acimut 0-360) - a los 3 setpoints de motor que ya
// entiende iniciarPID()/PIDn. Es pura matematica, no toca ningun motor
// directamente: reusa el 100% del PID ya construido y sintonizado.
//
// Modelo PCC estandar para N cables a un radio r_cable del eje central del
// segmento, cable i en la posicion angular phi_i:
//   deltaL_i = -theta_rad * r_cable * cos(phi - phi_i)
// El cable del lado hacia el que se dobla (phi cerca de phi_i) se acorta
// (deltaL negativo -> hay que recoger); el del lado opuesto se alarga
// (deltaL positivo -> hay que soltar). Coincide con que los cables solo
// jalan (no empujan) - por eso los 3 motores deben estar activos siempre,
// ninguno puede quedar "libre".
//
// IMPORTANTE: el tentaculo NO es un cilindro de un solo segmento - es un
// cuerpo conico de 21 segmentos pasivos (34 cm de largo total), y los 3
// tendones corren por dentro de todos ellos anclando solo en la punta (el
// segmento mas chico). O sea los 3 motores doblan el tentaculo COMPLETO
// como un solo arco de curvatura constante (PCC), no solo el primer
// segmento - los 21 segmentos son pasivos, no tienen motor propio.
//
// Como el radio del cable cambia a lo largo del cono (32mm en la base, ya
// no es constante), la integral de deltaL a lo largo del arco para un cono
// de conicidad LINEAL con curvatura constante se reduce, tras cancelarse
// el largo total, a la MISMA formula de arriba pero usando el radio
// PROMEDIO entre la base y la punta en vez de un radio de un solo punto:
//   r_cable_efectivo = (r_base + r_punta) / 2 = (32 + 4.5) / 2 = 18.25 mm
//
// Constantes medidas por el usuario en el hardware real:
const float R_CABLE_MM        = 18.25f; // radio efectivo (promedio base/punta del cono, ver arriba)
const float R_CARRETE_MM      = 12.0f;  // radio de carrete, igual en los 3 motores
const float L_MAX_RECOGER_MM  = 110.0f; // 11 cm: maximo cable que se puede recoger con seguridad
const float L_MAX_SOLTAR_MM   = 200.0f; // 20 cm: maximo cable que se puede soltar con seguridad

// Posicion angular de cada cable (grados), asumiendo Motor 1/2/3 a 0/120/240
// - coincide con el mapeo motor<->encoder 1-1,2-2,3-3 ya confirmado. Si la
// disposicion fisica real de los cables no es exactamente esta, este es el
// unico lugar que hay que corregir.
const float PHI_CABLE_DEG[3] = {0.0f, 120.0f, 240.0f};

// Ultimo (theta, phi) comandado - no es una lectura de sensor, es la
// referencia que el propio sistema de cinematica esta pidiendo en este
// momento. La usa TURN para saber desde donde arrancar el giro sin que el
// usuario tenga que repetir el theta actual cada vez.
float thetaActualCmd = 0.0f;
float phiActualCmd   = 0.0f;

// ---------------- TURN: giro por control de VELOCIDAD (feedforward) ----------------
//
// BEND salta directo al setpoint final de cada motor. La primera version
// de TURN reescribia el setpoint de POSICION del PID cada 50ms en pasos
// chicos, pero el PID de posicion tiene piso de friccion y zona muerta
// pensados para asentarse en un punto FIJO, no para perseguir una
// referencia en movimiento continuo - cada paso chico disparaba un
// "tiron" nuevo del piso de friccion, sintiendose brusco/tropero.
//
// Esta version controla velocidad en lazo abierto (feedforward) en vez de
// posicion: se deriva la formula de PCC respecto a phi para saber a que
// velocidad angular debe girar cada motor en cada instante, se convierte
// esa velocidad a duty con el factor deg/s-por-duty ya obtenido en CALVEL,
// y se escribe ese duty directo (bypasseando el PID de posicion mientras
// dura el giro). El resultado: mientras phi barre, cada motor acelera y
// frena de forma continua segun sin(phi - phi_i) - los que deben soltar
// cable y los que deben recogerlo cambian de forma suave, no a saltos: un
// "tira y afloja" continuo entre los 3 en vez de reposicionamientos
// discretos. Al llegar al destino, se apaga el lazo abierto y se entrega
// el control al PID de posicion (mismo camino que BEND) para asentar con
// precision en el punto final.

// deg/s reales por unidad de duty, derivado de los resultados de CALVEL
// (los 3 motores quedaron calibrados a ~55.22 deg/s en su duty corregido:
// M1=139, M2=122, M3=135). Aproximacion lineal - ignora la no linealidad
// de la friccion estatica en duties muy bajos, valida solo lejos de esa
// zona (por eso el piso GIRO_DUTY_MIN_FRICCION de mas abajo).
const float K_DUTY_A_VELOCIDAD[3] = {55.22f / 139.0f, 55.22f / 122.0f, 55.22f / 135.0f};

bool     giroActivo        = false;
float    giroThetaFijo     = 0.0f;
float    giroPhiDestino    = 0.0f;
float    giroVelocidadDegS = 0.0f;
uint32_t giroUltimoTiempo  = 0;

const uint32_t GIRO_INTERVALO_MS            = 20;   // igual que el PID (50 Hz) - duty se siente continuo
const float    GIRO_VELOCIDAD_DEG_S_DEFAULT = 15.0f; // velocidad angular por defecto, sin validar en hardware
const float    GIRO_TOLERANCIA_DEG          = 1.0f;
const float    GIRO_DUTY_MIN_FRICCION       = 70.0f; // piso de arranque, mismo criterio que PID_DUTY_MIN_FRICCION

// Ganancia de la correccion contra la posicion real del encoder (ver
// actualizarGiro). Negativa por el mismo motivo que las ganancias del PID
// normal (duty positivo resta del acumulado). Deliberadamente mas chica en
// magnitud que el Kp del PID (~-2.08 a -2.25): aqui es solo un ajuste
// fino sobre el feedforward, no el control principal - una ganancia tan
// fuerte como la del PID pelearia contra el feedforward en vez de solo
// corregirlo. Sin validar en hardware todavia.
const float    GIRO_KP_CORRECCION           = -0.5f;

// Diferencia angular con signo, en (-180, 180], tomando siempre el camino
// mas corto de "desde" a "hasta" - evita que un giro de 350 grados se haga
// dando toda la vuelta larga quedando del lado equivocado.
float diferenciaAngularCorta(float desde, float hasta) {
  float diff = fmodf(hasta - desde + 180.0f, 360.0f);
  if (diff < 0) diff += 360.0f;
  return diff - 180.0f;
}

// Calcula los 3 setpoints de motor (grados) para un doblez (theta, phi)
// dados, dejando el resultado en setpoints[3]. Aplica los limites fisicos
// de cable (L_MAX_RECOGER_MM / L_MAX_SOLTAR_MM) por seguridad - si theta
// pide mas cable del que hay margen, se recorta ahi (no se aborta).
void calcularSetpointsCinematica(float theta_deg, float phi_deg, float setpoints[3]) {
  float theta_rad = theta_deg * (PI / 180.0f);
  float phi_rad = phi_deg * (PI / 180.0f);

  for (uint8_t i = 0; i < 3; i++) {
    float phi_i_rad = PHI_CABLE_DEG[i] * (PI / 180.0f);
    float deltaL_mm = -theta_rad * R_CABLE_MM * cosf(phi_rad - phi_i_rad);

    if (deltaL_mm < -L_MAX_RECOGER_MM) deltaL_mm = -L_MAX_RECOGER_MM;
    if (deltaL_mm > L_MAX_SOLTAR_MM) deltaL_mm = L_MAX_SOLTAR_MM;

    setpoints[i] = (deltaL_mm / R_CARRETE_MM) * (180.0f / PI);
  }
}

// Comando BEND <theta_deg> <phi_deg>: calcula los setpoints y activa el PID
// de los 3 motores a la vez. Requiere los 3 motores detenidos (mismo
// criterio que CALVEL) - si alguno esta en HOME o PID activo, iniciarPID()
// ya rechaza individualmente con su propio mensaje.
void iniciarCinematica(float theta_deg, float phi_deg) {
  giroActivo = false; // BEND es un salto directo al setpoint final, cancela cualquier TURN en curso
  thetaActualCmd = theta_deg;
  phiActualCmd   = phi_deg;

  float setpoints[3];
  calcularSetpointsCinematica(theta_deg, phi_deg, setpoints);

  Serial.print("BEND theta=");
  Serial.print(theta_deg, 2);
  Serial.print(" deg | phi=");
  Serial.print(phi_deg, 2);
  Serial.print(" deg -> setpoints: M1=");
  Serial.print(setpoints[0], 2);
  Serial.print(" | M2=");
  Serial.print(setpoints[1], 2);
  Serial.print(" | M3=");
  Serial.println(setpoints[2], 2);

  for (uint8_t m = 0; m < 3; m++) iniciarPID(m, setpoints[m]);
}

// Comando TURN <theta_deg> <phi_destino_deg> [velocidad_deg_s]: gira hacia
// phi_destino_deg mantieniendo theta_deg fijo durante todo el giro (igual
// convencion de argumentos que BEND: primero cuanto cerrar, despues hacia
// donde). Apaga el PID de posicion de los 3 motores mientras dura el giro
// - lo controla actualizarGiro() por duty directo (ver arriba el porque).
void iniciarGiro(float thetaFijoDeg, float phiDestinoDeg, float velocidadDegS) {
  if (homingActivo[0] || homingActivo[1] || homingActivo[2]) {
    Serial.println("TURN: RECHAZADO - hay un HOME activo, manda S primero.");
    return;
  }

  giroThetaFijo     = thetaFijoDeg;
  giroPhiDestino    = phiDestinoDeg;
  giroVelocidadDegS = (velocidadDegS > 0.0f) ? velocidadDegS : GIRO_VELOCIDAD_DEG_S_DEFAULT;
  giroUltimoTiempo  = millis();
  giroActivo        = true;
  thetaActualCmd    = thetaFijoDeg;

  // El PID de posicion se apaga mientras dura el giro - actualizarGiro()
  // escribe duty directo (feedforward de velocidad), no setpoints de
  // posicion. Se reactiva solo al llegar al destino, para asentar fino.
  for (uint8_t m = 0; m < 3; m++) pidActivo[m] = false;

  Serial.print("TURN: girando de phi=");
  Serial.print(phiActualCmd, 2);
  Serial.print(" a phi=");
  Serial.print(giroPhiDestino, 2);
  Serial.print(" deg, manteniendo theta=");
  Serial.print(giroThetaFijo, 2);
  Serial.print(" deg, a ");
  Serial.print(giroVelocidadDegS, 1);
  Serial.println(" deg/s.");
}

// Se llama en cada iteracion de loop(). Mientras el giro esta activo,
// calcula la velocidad angular que necesita cada motor en este instante
// (derivada de la formula PCC respecto a phi) y escribe el duty
// correspondiente directo - sin pasar por el PID de posicion. Al llegar al
// destino, entrega el control al PID (mismo camino que BEND) para asentar
// con precision en el punto final.
void actualizarGiro() {
  if (!giroActivo) return;

  uint32_t ahora = millis();
  if (ahora - giroUltimoTiempo < GIRO_INTERVALO_MS) return;
  float dtSeg = (ahora - giroUltimoTiempo) / 1000.0f;
  giroUltimoTiempo = ahora;

  float restante = diferenciaAngularCorta(phiActualCmd, giroPhiDestino);

  if (fabs(restante) <= GIRO_TOLERANCIA_DEG) {
    giroActivo = false;
    phiActualCmd = giroPhiDestino;

    // iniciarPID() rechaza si estadoMotor[m] != PARADO y pidActivo[m] es
    // falso - y aqui SIEMPRE es asi, porque escribirComandoMotor() (usado
    // durante todo el giro) deja estadoMotor[m] en ADELANTE/REVERSA, no en
    // PARADO. Sin este detenerMotor() previo, iniciarPID() se rechazaba en
    // silencio para los 3 motores, y como nada mas escribia duty despues,
    // los motores se quedaban girando indefinidamente al ultimo duty del
    // giro - eso era la "fuga sin control" justo tras este mensaje.
    for (uint8_t m = 0; m < 3; m++) detenerMotor(m);

    float setpoints[3];
    calcularSetpointsCinematica(giroThetaFijo, phiActualCmd, setpoints);
    for (uint8_t m = 0; m < 3; m++) iniciarPID(m, setpoints[m]);
    Serial.println("TURN: giro completado, PID tomando el control para asentar.");
    return;
  }

  float sentido       = (restante > 0.0f) ? 1.0f : -1.0f;
  float thetaRad       = giroThetaFijo * (PI / 180.0f);
  float dPhiDt_rad     = sentido * giroVelocidadDegS * (PI / 180.0f); // rad/s
  float phiActualRad   = phiActualCmd * (PI / 180.0f);

  // Setpoints "de referencia" para este instante (mismo phi que lleva el
  // software) - se usan SOLO para la correccion de abajo, no se le pasan
  // a ningun PID mientras el giro esta activo.
  float setpointsRef[3];
  calcularSetpointsCinematica(giroThetaFijo, phiActualCmd, setpointsRef);

  for (uint8_t m = 0; m < 3; m++) {
    float phi_i_rad = PHI_CABLE_DEG[m] * (PI / 180.0f);
    // d(deltaL)/dt = theta_rad * r_cable * sin(phi - phi_i) * dphi/dt
    float dL_dt = thetaRad * R_CABLE_MM * sinf(phiActualRad - phi_i_rad) * dPhiDt_rad;
    // grados de motor por segundo:
    float dThetaMotor_dt = (dL_dt / R_CARRETE_MM) * (180.0f / PI);
    // Signo: en esta planta (confirmado en hardware, mismo criterio que
    // usa el PID normal) duty positivo (adelante) RESTA del acumulado y
    // duty negativo (reversa) SUMA - por eso el feedforward va con signo
    // negativo aqui; sin este signo el motor gira al reves de lo que
    // phiActualCmd asume, y el error crece sin freno en vez de cerrarse
    // (la causa del "sube sin limite" en pruebas con theta/phi grandes).
    float dutyFF = -dThetaMotor_dt / K_DUTY_A_VELOCIDAD[m];

    // Correccion contra la posicion REAL del encoder (no solo el phi que
    // lleva el software) - sin esto, cualquier imprecision del factor
    // duty->velocidad (peor a duty alto, que es justo donde mas se nota
    // con theta/phi grandes) se acumularia sin freno durante todo el giro.
    float actual;
    float dutyCorreccion = 0.0f;
    if (leerAnguloAcumuladoGrados(m, actual)) {
      float error = setpointsRef[m] - actual;
      dutyCorreccion = GIRO_KP_CORRECCION * error; // mismo signo negativo que el PID normal
    }

    float duty = dutyFF + dutyCorreccion;
    if (duty > 0.0f && duty < GIRO_DUTY_MIN_FRICCION) duty = GIRO_DUTY_MIN_FRICCION;
    else if (duty < 0.0f && duty > -GIRO_DUTY_MIN_FRICCION) duty = -GIRO_DUTY_MIN_FRICCION;
    duty = constrain(duty, -255.0f, 255.0f);

    escribirComandoMotor(m, duty);
  }

  phiActualCmd += sentido * giroVelocidadDegS * dtSeg;
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

  Serial.print("Velocidad actual (deg/s) -> M1: ");
  Serial.print(velocidadEstimadaDegS[0], 2);
  Serial.print(" | M2: ");
  Serial.print(velocidadEstimadaDegS[1], 2);
  Serial.print(" | M3: ");
  Serial.println(velocidadEstimadaDegS[2], 2);

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
  else if (cmd == "S") { homingActivo[motorActivo] = false; pidActivo[motorActivo] = false; giroActivo = false; detenerMotor(motorActivo); }
  else if (cmd == "SS") {
    giroActivo = false;
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
  else if (cmd.startsWith("SETVEL1 ")) fijarVelocidadMotor(0, (uint8_t)constrain(cmd.substring(8).toInt(), 0, 255));
  else if (cmd.startsWith("SETVEL2 ")) fijarVelocidadMotor(1, (uint8_t)constrain(cmd.substring(8).toInt(), 0, 255));
  else if (cmd.startsWith("SETVEL3 ")) fijarVelocidadMotor(2, (uint8_t)constrain(cmd.substring(8).toInt(), 0, 255));
  else if (cmd.startsWith("BEND ")) {
    // Formato: BEND <theta_deg> <phi_deg>. Ejemplo: BEND 15 90
    String resto = cmd.substring(5);
    resto.trim();
    int espacio = resto.indexOf(' ');
    if (espacio > 0) {
      float theta = resto.substring(0, espacio).toFloat();
      float phi = resto.substring(espacio + 1).toFloat();
      iniciarCinematica(theta, phi);
    } else {
      Serial.println("Formato invalido. Usa: BEND <theta_deg> <phi_deg>. Ejemplo: BEND 15 90");
    }
  }
  else if (cmd.startsWith("TURN ")) {
    // Formato: TURN <theta_deg> <phi_destino_deg> [velocidad_deg_s].
    // Ejemplo: TURN 45 270   o   TURN 45 270 15
    String resto = cmd.substring(5);
    resto.trim();
    int esp1 = resto.indexOf(' ');
    if (esp1 > 0) {
      float thetaDeg = resto.substring(0, esp1).toFloat();
      String resto2 = resto.substring(esp1 + 1);
      resto2.trim();
      int esp2 = resto2.indexOf(' ');
      float phiDestino, velocidad = 0.0f;
      if (esp2 > 0) {
        phiDestino = resto2.substring(0, esp2).toFloat();
        velocidad = resto2.substring(esp2 + 1).toFloat();
      } else {
        phiDestino = resto2.toFloat();
      }
      iniciarGiro(thetaDeg, phiDestino, velocidad);
    } else {
      Serial.println("Formato invalido. Usa: TURN <theta_deg> <phi_deg> [vel_deg_s]. Ejemplo: TURN 45 270 15");
    }
  }
  else if (cmd.length() > 0) {
    Serial.println("Comando no reconocido. Usa: M1 M2 M3 F R S SS CAL1 CAL2 CAL3 HOME1 HOME2 HOME3 HOME STATUS CALVEL CALVEL M<1-3> <duty> SETVEL1/2/3 <duty> PID1/2/3 <setpoint_deg> BEND <theta_deg> <phi_deg> TURN <theta_deg> <phi_deg> [vel_deg_s]");
  }
}

// ---------------- Programa principal ----------------

const uint32_t CHECKPOINT_INTERVALO_MS = 60000; // respaldo de posicion cada 60s
uint32_t ultimoCheckpoint = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Fase 8: cinematica (PCC, 3 cables a 120 deg) sobre PID identificado ===");

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
  Serial.println("STATUS -> imprime el angulo/vueltas y la velocidad actual (deg/s) de los 3 encoders (ya no se imprime solo, para no saturar el Monitor Serial)");
  Serial.println("CALVEL -> mide y empareja la velocidad real de los 3 motores contra el mas lento (requiere los 3 detenidos, persiste en flash)");
  Serial.println("CALVEL M<1-3> <duty> -> igual, pero fija el motor indicado a ese duty y usa su velocidad como objetivo. Ej: CALVEL M3 225");
  Serial.println("SETVEL1/SETVEL2/SETVEL3 <duty> -> fija a mano el duty de ese motor (sin medir), persiste en flash. Ej: SETVEL2 122");
  Serial.println("PID1/PID2/PID3 <setpoint_deg> -> activa el PID real (ganancias identificadas) en ese motor, con ese setpoint. Ej: PID2 45.0");
  Serial.println("BEND <theta_deg> <phi_deg> -> dobla el segmento theta grados hacia la direccion phi (modelo PCC, 3 cables a 120), activa el PID de los 3 motores. Ej: BEND 15 90");
  Serial.println("TURN <theta_deg> <phi_deg> [vel_deg_s] -> gira la direccion del doblez a phi_deg manteniendo theta_deg fijo durante todo el giro (control de velocidad, no de posicion - un tiron continuo suave, no pasa por apertura total). Ej: TURN 45 270 15");
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
  actualizarVelocidadEstimada();

  // Avanza el homing (si esta activo) de cada motor, tambien en cada
  // iteracion de loop() para reaccionar rapido si hay que abortar.
  actualizarHoming(0);
  actualizarHoming(1);
  actualizarHoming(2);

  // Avanza el giro TURN (si esta activo) antes que el PID, para que el
  // setpoint recalculado en este ciclo ya este listo cuando el PID actue.
  actualizarGiro();

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
