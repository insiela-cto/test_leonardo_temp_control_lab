/*
  Control de temperatura para Arduino Leonardo + FreeRTOS
  HW base: https://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl

  Implementa dos controladores:
  1) PID clásico (con anti-windup y derivada filtrada)
  2) Control moderno basado en observador + realimentación de estado + integrador (LQI-like)

  Librerías necesarias:
  - Arduino_FreeRTOS (https://github.com/feilipu/Arduino_FreeRTOS_Library)

  Comandos por Serial (115200 baudios):
  - MODE PID
  - MODE MODERN
  - SP <valor_en_C>
  - PID <kp> <ki> <kd>
  - MODERN <kx> <ki>
*/

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// -------------------- Configuración de hardware --------------------
static const uint8_t PIN_HEATER_PWM = 3;   // PWM al transistor/MOSFET del calefactor
static const uint8_t PIN_TEMP_ADC   = A0;  // NTC en divisor

// -------------------- Configuración de muestreo --------------------
static const float TS = 0.2f;  // [s] período de control
static const TickType_t TASK_PERIOD_TICKS = pdMS_TO_TICKS((uint16_t)(TS * 1000.0f));

// -------------------- Parámetros del sensor (NTC Beta) --------------------
static const float R_FIXED = 10000.0f; // resistor fijo del divisor [ohm]
static const float R0 = 10000.0f;      // resistencia NTC a T0 [ohm]
static const float T0_K = 298.15f;     // 25°C en Kelvin
static const float BETA = 3950.0f;     // constante beta típica

// -------------------- Límites y setpoint --------------------
static const float U_MIN = 0.0f;    // 0% potencia
static const float U_MAX = 100.0f;  // 100% potencia
static const float DEFAULT_SP = 45.0f;

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
  float tau_d;
  float integ;
  float d_prev;
  float e_prev;
};

static PIDController gPID = {
  .kp = 5.0f,
  .ki = 0.18f,
  .kd = 10.0f,
  .tau_d = 0.25f,
  .integ = 0.0f,
  .d_prev = 0.0f,
  .e_prev = 0.0f,
};

// -------------------- Control moderno (LQI-like) --------------------
// Modelo térmico: x[k+1] = a*x[k] + b*u[k], y[k] = x[k] + ambiente
// x_hat se estima con un observador discreto simple.
struct ModernController {
  float a;   // dinámica discreta del proceso térmico
  float b;   // ganancia discreta por % de potencia
  float l;   // ganancia del observador
  float kx;  // realimentación de estado
  float ki;  // ganancia del integrador de error
  float x_hat;
  float xi;
  float u_prev;
};

static ModernController gModern = {
  .a = 0.985f,
  .b = 0.09f,
  .l = 0.25f,
  .kx = 1.10f,
  .ki = 0.30f,
  .x_hat = 0.0f,
  .xi = 0.0f,
  .u_prev = 0.0f,
};

static float clampf(const float v, const float lo, const float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static float adcToTempC(const int adc) {
  const float adcSafe = (adc <= 0) ? 1.0f : ((adc >= 1023) ? 1022.0f : (float)adc);
  const float rNtc = R_FIXED * (adcSafe / (1023.0f - adcSafe));
  const float invT = (1.0f / T0_K) + (1.0f / BETA) * logf(rNtc / R0);
  const float tempK = 1.0f / invT;
  return tempK - 273.15f;
}

static float runPID(PIDController &c, const float sp, const float y) {
  const float e = sp - y;
  const float integCandidate = c.integ + c.ki * TS * e;

  const float de = (e - c.e_prev) / TS;
  const float alpha = c.tau_d / (c.tau_d + TS);
  const float d = alpha * c.d_prev + (1.0f - alpha) * de;

  float uUnsat = c.kp * e + integCandidate + c.kd * d;
  float uSat = clampf(uUnsat, U_MIN, U_MAX);

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
  return uSat;
}

static float runModern(ModernController &c, const float sp, const float y, const float ambient) {
  const float yState = y - ambient;

  // Observador de estado (usa modelo + corrección por error de medición)
  c.x_hat = c.a * c.x_hat + c.b * c.u_prev + c.l * (yState - c.x_hat);

  const float r = sp - ambient;
  const float e = r - c.x_hat;
  c.xi += e * TS;

  const float uRaw = -c.kx * c.x_hat + c.ki * c.xi;
  const float uSat = clampf(uRaw, U_MIN, U_MAX);

  // anti-windup por back-calculation simple
  c.xi += 0.05f * (uSat - uRaw);

  c.u_prev = uSat;
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

    float u = (mode == CONTROL_PID) ? runPID(gPID, sp, y) : runModern(gModern, sp, y, ambient);

    setHeaterPct(u);

    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gState.controlPct = u;
    xSemaphoreGive(gStateMutex);

    vTaskDelayUntil(&xLastWakeTime, TASK_PERIOD_TICKS);
  }
}

static void resetControllers() {
  gPID.integ = 0.0f;
  gPID.d_prev = 0.0f;
  gPID.e_prev = 0.0f;

  gModern.xi = 0.0f;
  gModern.x_hat = 0.0f;
  gModern.u_prev = 0.0f;
}

static void parseSerialCommand(char *line) {
  // tokenización en sitio
  char *token = strtok(line, " \t");
  if (token == NULL) {
    return;
  }

  if (strcmp(token, "MODE") == 0) {
    char *arg = strtok(NULL, " \t");
    if (arg != NULL && strcmp(arg, "PID") == 0) {
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      gState.mode = CONTROL_PID;
      resetControllers();
      xSemaphoreGive(gStateMutex);
      Serial.println(F("OK MODE PID"));
      return;
    }
    if (arg != NULL && strcmp(arg, "MODERN") == 0) {
      xSemaphoreTake(gStateMutex, portMAX_DELAY);
      gState.mode = CONTROL_MODERN;
      resetControllers();
      xSemaphoreGive(gStateMutex);
      Serial.println(F("OK MODE MODERN"));
      return;
    }
    Serial.println(F("ERR MODE"));
    return;
  }

  if (strcmp(token, "SP") == 0) {
    char *arg = strtok(NULL, " \t");
    if (arg == NULL) {
      Serial.println(F("ERR SP"));
      return;
    }
    const float newSp = atof(arg);
    const float safeSp = clampf(newSp, 20.0f, 85.0f);

    xSemaphoreTake(gStateMutex, portMAX_DELAY);
    gState.setpointC = safeSp;
    xSemaphoreGive(gStateMutex);

    Serial.print(F("OK SP "));
    Serial.println(safeSp, 2);
    return;
  }

  if (strcmp(token, "PID") == 0) {
    char *a = strtok(NULL, " \t");
    char *b = strtok(NULL, " \t");
    char *c = strtok(NULL, " \t");
    if (a == NULL || b == NULL || c == NULL) {
      Serial.println(F("ERR PID"));
      return;
    }

    gPID.kp = atof(a);
    gPID.ki = atof(b);
    gPID.kd = atof(c);
    resetControllers();

    Serial.print(F("OK PID "));
    Serial.print(gPID.kp, 3);
    Serial.print(F(" "));
    Serial.print(gPID.ki, 3);
    Serial.print(F(" "));
    Serial.println(gPID.kd, 3);
    return;
  }

  if (strcmp(token, "MODERN") == 0) {
    char *a = strtok(NULL, " \t");
    char *b = strtok(NULL, " \t");
    if (a == NULL || b == NULL) {
      Serial.println(F("ERR MODERN"));
      return;
    }

    gModern.kx = atof(a);
    gModern.ki = atof(b);
    resetControllers();

    Serial.print(F("OK MODERN "));
    Serial.print(gModern.kx, 3);
    Serial.print(F(" "));
    Serial.println(gModern.ki, 3);
    return;
  }

  Serial.println(F("ERR CMD"));
}

static void taskSerial(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  static char line[64];
  static uint8_t idx = 0;

  for (;;) {
    while (Serial.available() > 0) {
      const char ch = (char)Serial.read();

      if (ch == '\r' || ch == '\n') {
        if (idx > 0) {
          line[idx] = '\0';

          // pasa a mayúsculas ASCII simple
          for (uint8_t i = 0; i < idx; ++i) {
            if (line[i] >= 'a' && line[i] <= 'z') {
              line[i] = (char)(line[i] - ('a' - 'A'));
            }
          }

          parseSerialCommand(line);
          idx = 0;
        }
      } else if (idx < (sizeof(line) - 1)) {
        line[idx++] = ch;
      } else {
        idx = 0;
        Serial.println(F("ERR LONG"));
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
    while (true) {
      analogWrite(PIN_HEATER_PWM, 0);
      delay(100);
    }
  }

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
