/*
  Control de temperatura para Arduino Leonardo + FreeRTOS
  HW base: https://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl

  Implementa dos controladores:
  1) PID clásico (con anti-windup y derivada filtrada)
  2) Control moderno tipo LQI (realimentación de estado + integrador)

  Librerías necesarias:
  - Arduino_FreeRTOS (https://github.com/feilipu/Arduino_FreeRTOS_Library)

  Comandos por Serial (115200 baudios):
  - MODE PID
  - MODE MODERN
  - SP <valor_en_C>
*/

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <math.h>

// -------------------- Configuración de hardware --------------------
static const uint8_t PIN_HEATER_PWM = 3;   // PWM al transistor/MOSFET del calefactor
static const uint8_t PIN_TEMP_ADC   = A0;  // NTC en divisor

// -------------------- Configuración de muestreo --------------------
static const float TS = 0.2f;  // [s] período de control
static const TickType_t TASK_PERIOD_TICKS = pdMS_TO_TICKS((uint16_t)(TS * 1000.0f));

// -------------------- Parámetros del sensor (NTC Beta) --------------------
// Ajustar según tu HW real si es necesario
static const float R_FIXED = 10000.0f; // resistor fijo del divisor [ohm]
static const float R0 = 10000.0f;      // resistencia NTC a T0 [ohm]
static const float T0_K = 298.15f;     // 25°C en Kelvin
static const float BETA = 3950.0f;     // constante beta típica

// -------------------- Límites y setpoint --------------------
static const float U_MIN = 0.0f;    // 0% potencia
static const float U_MAX = 100.0f;  // 100% potencia
static const float DEFAULT_SP = 45.0f;

// -------------------- Selección de controlador --------------------
enum ControlMode : uint8_t {
  CONTROL_PID = 0,
  CONTROL_MODERN = 1,
};

struct SharedState {
  float tempC;
  float tempFiltC;
  float setpointC;
  float controlPct;
  float ambientC;
  ControlMode mode;
};

static SharedState gState = {25.0f, 25.0f, DEFAULT_SP, 0.0f, 25.0f, CONTROL_PID};
static SemaphoreHandle_t gStateMutex;

// -------------------- PID --------------------
struct PIDController {
  float kp;
  float ki;
  float kd;
  float tau_d;      // filtro de derivada
  float integ;
  float d_prev;
  float e_prev;
  float u_prev;
};

static PIDController gPID = {
  .kp = 5.0f,
  .ki = 0.18f,
  .kd = 10.0f,
  .tau_d = 0.25f,
  .integ = 0.0f,
  .d_prev = 0.0f,
  .e_prev = 0.0f,
  .u_prev = 0.0f,
};

// -------------------- Control moderno (LQI simplificado) --------------------
// Modelo térmico discreto de primer orden:
// x[k+1] = a*x[k] + b*u[k], donde x es desviación de temperatura respecto a ambiente
// Diseño tipo LQI implementado como:
// xi[k+1] = xi[k] + (r - y)*Ts
// u = -Kx*x + Ki*xi
struct ModernController {
  float a;   // dinámica discreta
  float b;   // ganancia discreta por % de potencia
  float kx;  // ganancia estado
  float ki;  // ganancia integrador
  float xi;
};

static ModernController gModern = {
  .a = 0.985f,
  .b = 0.09f,
  .kx = 1.2f,
  .ki = 0.35f,
  .xi = 0.0f,
};

static float clampf(const float v, const float lo, const float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static float adcToTempC(const int adc) {
  const float adcSafe = (adc <= 0) ? 1.0f : ((adc >= 1023) ? 1022.0f : (float)adc);

  // Divisor: Vout = Vcc * (R_ntc / (R_fixed + R_ntc))
  const float rNtc = R_FIXED * (adcSafe / (1023.0f - adcSafe));

  // Ecuación Beta
  const float invT = (1.0f / T0_K) + (1.0f / BETA) * logf(rNtc / R0);
  const float tempK = 1.0f / invT;
  return tempK - 273.15f;
}

static float runPID(PIDController &c, const float sp, const float y) {
  const float e = sp - y;

  // Integrador con anti-windup por clamping condicional
  const float integCandidate = c.integ + c.ki * TS * e;

  // Derivada filtrada
  const float de = (e - c.e_prev) / TS;
  const float alpha = c.tau_d / (c.tau_d + TS);
  const float d = alpha * c.d_prev + (1.0f - alpha) * de;

  float uUnsat = c.kp * e + integCandidate + c.kd * d;
  float uSat = clampf(uUnsat, U_MIN, U_MAX);

  // Si no saturo, o si la integración ayuda a salir de saturación, aceptamos integrador
  const bool satHigh = (uUnsat > U_MAX);
  const bool satLow = (uUnsat < U_MIN);
  const bool integrate = (!satHigh && !satLow) || (satHigh && e < 0.0f) || (satLow && e > 0.0f);

  if (integrate) {
    c.integ = integCandidate;
    uUnsat = c.kp * e + c.integ + c.kd * d;
    uSat = clampf(uUnsat, U_MIN, U_MAX);
  }

  c.e_prev = e;
  c.d_prev = d;
  c.u_prev = uSat;
  return uSat;
}

static float runModern(ModernController &c, const float sp, const float y, const float ambient) {
  const float x = y - ambient;          // estado medido (desviación térmica)
  const float r = sp - ambient;         // referencia equivalente
  const float e = r - x;

  // Integrador del error
  c.xi += e * TS;

  // Ley de control LQI simplificada
  const float uRaw = -c.kx * x + c.ki * c.xi;
  float uSat = clampf(uRaw, U_MIN, U_MAX);

  // Anti-windup simple: rollback parcial si saturó
  if (uRaw != uSat) {
    c.xi -= 0.5f * e * TS;
  }

  return uSat;
}

static void setHeaterPct(const float pct) {
  const float p = clampf(pct, U_MIN, U_MAX);
  const uint8_t pwm = (uint8_t)roundf((p / 100.0f) * 255.0f);
  analogWrite(PIN_HEATER_PWM, pwm);
}

static void taskSensor(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  const float alpha = 0.20f;

  for (;;) {
    const int adc = analogRead(PIN_TEMP_ADC);
    const float t = adcToTempC(adc);

    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gState.tempC = t;
    gState.tempFiltC = alpha * t + (1.0f - alpha) * gState.tempFiltC;
    xSemaphoreGive(gStateMutex);

    vTaskDelayUntil(&xLastWakeTime, TASK_PERIOD_TICKS);
  }
}

static void taskControl(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    float sp, y, ambient;
    ControlMode mode;

    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    sp = gState.setpointC;
    y = gState.tempFiltC;
    ambient = gState.ambientC;
    mode = gState.mode;
    xSemaphoreGive(gStateMutex);

    float u = 0.0f;
    if (mode == CONTROL_PID) {
      u = runPID(gPID, sp, y);
    } else {
      u = runModern(gModern, sp, y, ambient);
    }

    setHeaterPct(u);

    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gState.controlPct = u;
    xSemaphoreGive(gStateMutex);

    vTaskDelayUntil(&xLastWakeTime, TASK_PERIOD_TICKS);
  }
}

static void parseSerialCommand(const String &cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "MODE PID") {
    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gState.mode = CONTROL_PID;
    gPID.integ = 0.0f;
    gPID.d_prev = 0.0f;
    gPID.e_prev = 0.0f;
    xSemaphoreGive(gStateMutex);
    Serial.println(F("OK MODE PID"));
    return;
  }

  if (cmd == "MODE MODERN") {
    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gState.mode = CONTROL_MODERN;
    gModern.xi = 0.0f;
    xSemaphoreGive(gStateMutex);
    Serial.println(F("OK MODE MODERN"));
    return;
  }

  if (cmd.startsWith("SP ")) {
    const float newSp = cmd.substring(3).toFloat();
    const float safeSp = clampf(newSp, 20.0f, 85.0f);

    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gState.setpointC = safeSp;
    xSemaphoreGive(gStateMutex);

    Serial.print(F("OK SP "));
    Serial.println(safeSp, 2);
    return;
  }

  Serial.println(F("ERR CMD"));
}

static void taskSerial(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  String line;

  for (;;) {
    while (Serial.available() > 0) {
      const char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (line.length() > 0) {
          parseSerialCommand(line);
          line = "";
        }
      } else {
        line += c;
      }
    }

    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    const float t = gState.tempC;
    const float tf = gState.tempFiltC;
    const float sp = gState.setpointC;
    const float u = gState.controlPct;
    const ControlMode m = gState.mode;
    xSemaphoreGive(gStateMutex);

    Serial.print(F("MODE="));
    Serial.print((m == CONTROL_PID) ? F("PID") : F("MODERN"));
    Serial.print(F(", T="));
    Serial.print(t, 2);
    Serial.print(F(", TF="));
    Serial.print(tf, 2);
    Serial.print(F(", SP="));
    Serial.print(sp, 2);
    Serial.print(F(", U="));
    Serial.println(u, 1);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

void setup() {
  pinMode(PIN_HEATER_PWM, OUTPUT);
  analogWrite(PIN_HEATER_PWM, 0);

  pinMode(PIN_TEMP_ADC, INPUT);

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  gStateMutex = xSemaphoreCreateMutex();
  if (gStateMutex == NULL) {
    // Si falla el mutex, dejamos heater apagado
    while (true) {
      analogWrite(PIN_HEATER_PWM, 0);
      delay(100);
    }
  }

  // Estima ambiente en el arranque
  const int adc = analogRead(PIN_TEMP_ADC);
  const float t0 = adcToTempC(adc);
  xSemaphoreTake(gStateMutex, portMAX_DELAY);
  gState.tempC = t0;
  gState.tempFiltC = t0;
  gState.ambientC = t0;
  xSemaphoreGive(gStateMutex);

  xTaskCreate(taskSensor,  "Sensor",  192, NULL, 3, NULL);
  xTaskCreate(taskControl, "Control", 256, NULL, 2, NULL);
  xTaskCreate(taskSerial,  "Serial",  256, NULL, 1, NULL);
}

void loop() {
  // No se usa con FreeRTOS
}
