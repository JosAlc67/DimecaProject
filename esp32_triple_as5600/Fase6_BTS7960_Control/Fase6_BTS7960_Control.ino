// Fase 6 - Control basico de 3 BTS7960 por comandos Serial, SIN cambios
// bruscos de sentido: para invertir el sentido de un motor, primero hay
// que detenerlo (comando S) - el codigo rechaza el cambio directo.
//
// R_EN y L_EN de cada BTS7960 NO se controlan por GPIO: quedan puenteados
// fijos a su riel logico segun el propio diseño del usuario.
//
// Compatible con core arduino-esp32 2.x y 3.x: la API de ledc cambio por
// completo entre versiones (ledcSetup/ledcAttachPin vs ledcAttach), asi
// que se selecciona la correcta en tiempo de compilacion con el macro
// oficial ESP_ARDUINO_VERSION_VAL (definido en esp_arduino_version.h,
// incluido automaticamente por Arduino.h).
//
// Comandos por Serial (Monitor Serial, 115200, terminar linea con Enter):
//   M1 / M2 / M3  -> selecciona el motor activo (por defecto: 1)
//   F             -> gira el motor activo hacia adelante a baja velocidad
//                    (solo permitido si el motor activo esta detenido)
//   R             -> gira el motor activo en reversa a baja velocidad
//                    (solo permitido si el motor activo esta detenido)
//   S             -> detiene el motor activo
//   SS            -> PARADA DE EMERGENCIA: detiene los 3 motores ya

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define CORE_LEDC_V3 1
#else
  #define CORE_LEDC_V3 0
#endif

const uint32_t PWM_FREQ_HZ   = 20000;  // 20 kHz: por encima del oido humano
const uint8_t  PWM_RES_BITS  = 8;      // 0-255
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
EstadoMotor estado[3] = {PARADO, PARADO, PARADO};

uint8_t motorActivo = 0; // indice 0..2 (motor 1..3)

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

void detenerMotor(uint8_t m) {
  escribirPWM(PIN_RPWM[m], CANAL_RPWM_SEGURO(m), 0);
  escribirPWM(PIN_LPWM[m], CANAL_LPWM_SEGURO(m), 0);
  estado[m] = PARADO;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": DETENIDO");
}

// Helpers para no repetir el #if en cada llamada (en core 3.x el canal no se usa)
uint8_t CANAL_RPWM_SEGURO(uint8_t m) {
#if CORE_LEDC_V3
  return 0;
#else
  return CANAL_RPWM[m];
#endif
}
uint8_t CANAL_LPWM_SEGURO(uint8_t m) {
#if CORE_LEDC_V3
  return 0;
#else
  return CANAL_LPWM[m];
#endif
}

void girarAdelante(uint8_t m) {
  if (estado[m] != PARADO) {
    Serial.println("RECHAZADO: detén el motor (S) antes de cambiar de sentido.");
    return;
  }
  escribirPWM(PIN_LPWM[m], CANAL_LPWM_SEGURO(m), 0);
  escribirPWM(PIN_RPWM[m], CANAL_RPWM_SEGURO(m), VELOCIDAD_BAJA);
  estado[m] = ADELANTE;
  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(": GIRANDO ADELANTE (baja velocidad)");
}

void girarReversa(uint8_t m) {
  if (estado[m] != PARADO) {
    Serial.println("RECHAZADO: detén el motor (S) antes de cambiar de sentido.");
    return;
  }
  escribirPWM(PIN_RPWM[m], CANAL_RPWM_SEGURO(m), 0);
  escribirPWM(PIN_LPWM[m], CANAL_LPWM_SEGURO(m), VELOCIDAD_BAJA);
  estado[m] = REVERSA;
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

void setup() {
  Serial.begin(115200);
  delay(500);

  for (uint8_t i = 0; i < 3; i++) {
#if CORE_LEDC_V3
    configurarPWM(PIN_RPWM[i], 0);
    configurarPWM(PIN_LPWM[i], 0);
#else
    configurarPWM(PIN_RPWM[i], CANAL_RPWM[i]);
    configurarPWM(PIN_LPWM[i], CANAL_LPWM[i]);
#endif
    detenerMotor(i); // arranca siempre detenido, nunca en movimiento
  }

  Serial.println();
  Serial.println("=== Fase 6: control basico BTS7960 (sin motores/potencia todavia) ===");
  Serial.println("Comandos: M1 M2 M3 (seleccionar motor) | F (adelante) | R (reversa) | S (detener) | SS (parada de emergencia)");
  Serial.println("Motor activo por defecto: 1");
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    procesarComando(linea);
  }
}
