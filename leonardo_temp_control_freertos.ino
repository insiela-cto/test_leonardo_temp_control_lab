/*
  Firmware base orientado a requerimientos para ESP32 + FreeRTOS (iteración 1)
  ---------------------------------------------------------------------------
  Objetivo: alinear la base de firmware con el propósito del proyecto:
  - ESP32 dual core
  - FreeRTOS
  - FSM central reactiva por eventos explícitos
  - Servicio Biológico Permanente (SBP) desacoplado de interacción
  - HAL como única capa que toca hardware

  Alcance de esta iteración:
  - Ciclo autónomo 8h luz / 16h oscuridad
  - Burbujeo mínimo permanente
  - Modos experienciales por FSM
  - Interacción por botón + serial

  Pendiente (siguientes iteraciones):
  - WiFi AP/STA + webserver async
  - NVS persistencia
  - NTP y zona horaria
  - mantenimiento/recordatorios completos
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// -----------------------------------------------------------------------------
// 1) HAL (única capa que opera hardware)
// -----------------------------------------------------------------------------
namespace Hal {
static constexpr int PIN_LIGHT_PWM = 18;   // Luz blanca principal
static constexpr int PIN_BUBBLE_PWM = 19;  // Motobomba aire (driver PWM)
static constexpr int PIN_TEMP_ADC = 34;    // ADC1 (entrada analógica)
static constexpr int PIN_BUTTON = 27;      // Botón físico

static constexpr int LEDC_FREQ_HZ = 5000;
static constexpr int LEDC_RES_BITS = 8;
static constexpr int CH_LIGHT = 0;
static constexpr int CH_BUBBLE = 1;

// Compatibilidad LEDC:
// - Core ESP32 2.x: ledcSetup + ledcAttachPin + ledcWrite(canal,duty)
// - Core ESP32 3.x: ledcAttach(pin,freq,res) + ledcWrite(pin,duty)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static constexpr bool USE_LEDC_V3_API = true;
#else
static constexpr bool USE_LEDC_V3_API = false;
#endif

static float clampf(const float v, const float lo, const float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void init() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  if (USE_LEDC_V3_API) {
    ledcAttach(PIN_LIGHT_PWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttach(PIN_BUBBLE_PWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcWrite(PIN_LIGHT_PWM, 0);
    ledcWrite(PIN_BUBBLE_PWM, 0);
  } else {
    ledcSetup(CH_LIGHT, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_LIGHT_PWM, CH_LIGHT);

    ledcSetup(CH_BUBBLE, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_BUBBLE_PWM, CH_BUBBLE);

    ledcWrite(CH_LIGHT, 0);
    ledcWrite(CH_BUBBLE, 0);
  }
}

void writeLightPct(const float pct) {
  const float p = clampf(pct, 0.0f, 100.0f);
  const uint8_t duty = static_cast<uint8_t>(roundf((p / 100.0f) * 255.0f));
  if (USE_LEDC_V3_API) {
    ledcWrite(PIN_LIGHT_PWM, duty);
  } else {
    ledcWrite(CH_LIGHT, duty);
  }
}

void writeBubblePct(const float pct) {
  const float p = clampf(pct, 0.0f, 100.0f);
  const uint8_t duty = static_cast<uint8_t>(roundf((p / 100.0f) * 255.0f));
  if (USE_LEDC_V3_API) {
    ledcWrite(PIN_BUBBLE_PWM, duty);
  } else {
    ledcWrite(CH_BUBBLE, duty);
  }
}

bool readButtonPressed() {
  return digitalRead(PIN_BUTTON) == LOW;
}

// NTC Beta (ajustable)
static constexpr float R_FIXED = 10000.0f;
static constexpr float R0 = 10000.0f;
static constexpr float T0_K = 298.15f;
static constexpr float BETA = 3950.0f;

float readTempC() {
  // ESP32 ADC1 default width 12 bits (0..4095)
  const int adcRaw = analogRead(PIN_TEMP_ADC);
  const float adc = (adcRaw <= 0) ? 1.0f : ((adcRaw >= 4095) ? 4094.0f : static_cast<float>(adcRaw));

  // Divisor: Vout = Vcc * Rntc / (R_FIXED + Rntc)
  const float rNtc = R_FIXED * (adc / (4095.0f - adc));
  const float invT = (1.0f / T0_K) + (1.0f / BETA) * logf(rNtc / R0);
  const float tempK = 1.0f / invT;
  return tempK - 273.15f;
}
}  // namespace Hal

// -----------------------------------------------------------------------------
// 2) Modelo FSM + eventos explícitos
// -----------------------------------------------------------------------------
enum SystemState : uint8_t {
  STATE_INIT = 0,
  STATE_UNCONFIGURED,
  STATE_STANDBY,
  STATE_USER_LIGHT_ON,
  STATE_MEDITATION_1,
  STATE_MEDITATION_2,
  STATE_ACTIVE_PAUSE,
  STATE_ERROR,
};

enum EventType : uint8_t {
  EV_BOOT = 0,
  EV_WIFI_MISSING,
  EV_BUTTON_SHORT,
  EV_BUTTON_LONG,
  EV_MEDITATION_1_REQ,
  EV_MEDITATION_2_REQ,
  EV_ACTIVE_PAUSE_REQ,
  EV_ACTIVE_PAUSE_TIMEOUT,
  EV_LIGHT_FORCE_OFF,
  EV_ERROR_RECOVERABLE,
  EV_ERROR_CRITICAL,
};

struct Event {
  EventType type;
  uint32_t data;
};

struct Config {
  float dayLightPct;
  float minBubblePct;
  uint16_t dayMinutes;    // 8h en modo autónomo
  uint16_t nightMinutes;  // 16h en modo autónomo
  bool activePauseEnabled;
  uint16_t activePauseEveryMinutes;
  uint16_t activePauseDurationSec;
};

struct Runtime {
  SystemState state;
  float tempC;
  float tempFiltC;
  bool isDay;
  bool lightForcedOff;

  uint32_t cycleElapsedSec;
  uint32_t pausePeriodElapsedSec;
  uint32_t pauseElapsedSec;

  float sbpLightPct;
  float sbpBubblePct;
  float outLightPct;
  float outBubblePct;
};

static Config gCfg = {
    85.0f,
    20.0f,
    static_cast<uint16_t>(8u * 60u),
    static_cast<uint16_t>(16u * 60u),
    true,
    60u,
    120u,
};

static Runtime gRt = {
    STATE_INIT,
    25.0f,
    25.0f,
    true,
    false,
    0u,
    0u,
    0u,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
};

static SemaphoreHandle_t gMutex = nullptr;
static QueueHandle_t gEventQueue = nullptr;

static bool postEvent(const EventType type, const uint32_t data = 0u) {
  const Event e{type, data};
  return xQueueSend(gEventQueue, &e, 0) == pdTRUE;
}

// -----------------------------------------------------------------------------
// 3) SBP y composición de salidas
// -----------------------------------------------------------------------------
static void sbpComputeBase() {
  xSemaphoreTake(gMutex, portMAX_DELAY);

  // Servicio biológico mínimo permanente
  gRt.sbpBubblePct = gCfg.minBubblePct;
  gRt.sbpLightPct = (gRt.isDay && !gRt.lightForcedOff) ? gCfg.dayLightPct : 0.0f;

  xSemaphoreGive(gMutex);
}

static void composeAndApplyOutputs() {
  float light;
  float bubble;
  SystemState st;

  xSemaphoreTake(gMutex, portMAX_DELAY);
  light = gRt.sbpLightPct;
  bubble = gRt.sbpBubblePct;
  st = gRt.state;

  if (st == STATE_STANDBY) {
    light = 0.0f;
  } else if (st == STATE_MEDITATION_1) {
    light = 35.0f;
  } else if (st == STATE_MEDITATION_2) {
    light = 15.0f;
  } else if (st == STATE_ACTIVE_PAUSE) {
    light = 100.0f;
    bubble = 40.0f;
  }

  gRt.outLightPct = light;
  gRt.outBubblePct = bubble;
  xSemaphoreGive(gMutex);

  Hal::writeLightPct(light);
  Hal::writeBubblePct(bubble);
}

// -----------------------------------------------------------------------------
// 4) FSM central
// -----------------------------------------------------------------------------
static void setState(const SystemState s) {
  xSemaphoreTake(gMutex, portMAX_DELAY);
  gRt.state = s;
  if (s != STATE_ACTIVE_PAUSE) {
    gRt.pauseElapsedSec = 0u;
  }
  xSemaphoreGive(gMutex);
}

static void processEvent(const Event &e) {
  xSemaphoreTake(gMutex, portMAX_DELAY);
  const SystemState current = gRt.state;
  xSemaphoreGive(gMutex);

  switch (current) {
    case STATE_INIT:
      if (e.type == EV_BOOT) {
        setState(STATE_UNCONFIGURED);
      }
      break;

    case STATE_UNCONFIGURED:
      if (e.type == EV_WIFI_MISSING) {
        setState(STATE_STANDBY);
      }
      break;

    case STATE_STANDBY:
      if (e.type == EV_BUTTON_SHORT) {
        setState(STATE_USER_LIGHT_ON);
      } else if (e.type == EV_MEDITATION_1_REQ) {
        setState(STATE_MEDITATION_1);
      } else if (e.type == EV_MEDITATION_2_REQ) {
        setState(STATE_MEDITATION_2);
      }
      break;

    case STATE_USER_LIGHT_ON:
      if (e.type == EV_BUTTON_SHORT) {
        setState(STATE_MEDITATION_1);
      } else if (e.type == EV_ACTIVE_PAUSE_REQ) {
        setState(STATE_ACTIVE_PAUSE);
      } else if (e.type == EV_LIGHT_FORCE_OFF) {
        xSemaphoreTake(gMutex, portMAX_DELAY);
        gRt.lightForcedOff = true;
        xSemaphoreGive(gMutex);
      }
      break;

    case STATE_MEDITATION_1:
      if (e.type == EV_BUTTON_SHORT) {
        setState(STATE_MEDITATION_2);
      } else if (e.type == EV_BUTTON_LONG) {
        setState(STATE_USER_LIGHT_ON);
      }
      break;

    case STATE_MEDITATION_2:
      if (e.type == EV_BUTTON_SHORT || e.type == EV_BUTTON_LONG) {
        setState(STATE_USER_LIGHT_ON);
      }
      break;

    case STATE_ACTIVE_PAUSE:
      if (e.type == EV_ACTIVE_PAUSE_TIMEOUT || e.type == EV_BUTTON_LONG) {
        setState(STATE_USER_LIGHT_ON);
      }
      break;

    case STATE_ERROR:
      if (e.type == EV_ERROR_RECOVERABLE) {
        setState(STATE_STANDBY);
      }
      break;

    default:
      setState(STATE_ERROR);
      break;
  }

  if (e.type == EV_ERROR_CRITICAL) {
    setState(STATE_ERROR);
  }
}

// -----------------------------------------------------------------------------
// 5) Tareas FreeRTOS
// -----------------------------------------------------------------------------
static void taskFsm(void *pvParameters) {
  (void)pvParameters;
  Event e{};

  for (;;) {
    if (xQueueReceive(gEventQueue, &e, portMAX_DELAY) == pdTRUE) {
      processEvent(e);
      sbpComputeBase();
      composeAndApplyOutputs();
    }
  }
}

static void taskSensor(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();
  constexpr float alpha = 0.2f;

  for (;;) {
    const float t = Hal::readTempC();

    xSemaphoreTake(gMutex, portMAX_DELAY);
    gRt.tempC = t;
    gRt.tempFiltC = alpha * t + (1.0f - alpha) * gRt.tempFiltC;
    xSemaphoreGive(gMutex);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200));
  }
}

static void taskScheduler(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    xSemaphoreTake(gMutex, portMAX_DELAY);

    gRt.cycleElapsedSec += 1u;
    gRt.pausePeriodElapsedSec += 1u;

    const uint32_t daySec = static_cast<uint32_t>(gCfg.dayMinutes) * 60u;
    const uint32_t cycleSec = static_cast<uint32_t>(gCfg.dayMinutes + gCfg.nightMinutes) * 60u;

    if (gRt.cycleElapsedSec >= cycleSec) {
      gRt.cycleElapsedSec = 0u;
    }
    gRt.isDay = (gRt.cycleElapsedSec < daySec);

    if (gCfg.activePauseEnabled &&
        gRt.pausePeriodElapsedSec >= static_cast<uint32_t>(gCfg.activePauseEveryMinutes) * 60u) {
      gRt.pausePeriodElapsedSec = 0u;
      postEvent(EV_ACTIVE_PAUSE_REQ);
    }

    if (gRt.state == STATE_ACTIVE_PAUSE) {
      gRt.pauseElapsedSec += 1u;
      if (gRt.pauseElapsedSec >= gCfg.activePauseDurationSec) {
        postEvent(EV_ACTIVE_PAUSE_TIMEOUT);
      }
    }

    xSemaphoreGive(gMutex);

    sbpComputeBase();
    composeAndApplyOutputs();

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
  }
}

static void taskButton(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();

  bool prevPressed = false;
  uint16_t ticksPressed = 0u;

  for (;;) {
    const bool pressed = Hal::readButtonPressed();

    if (pressed && ticksPressed < 5000u) {
      ticksPressed++;
    }

    if (prevPressed && !pressed) {
      if (ticksPressed >= 30u) {
        postEvent(EV_BUTTON_LONG);   // 3.0 s (100 ms period)
      } else if (ticksPressed >= 1u) {
        postEvent(EV_BUTTON_SHORT);  // 100..2900 ms
      }
      ticksPressed = 0u;
    }

    prevPressed = pressed;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
  }
}

static void printTelemetry() {
  xSemaphoreTake(gMutex, portMAX_DELAY);
  const Runtime r = gRt;
  xSemaphoreGive(gMutex);

  Serial.print(F("STATE="));
  Serial.print(static_cast<int>(r.state));
  Serial.print(F(",T="));
  Serial.print(r.tempC, 2);
  Serial.print(F(",TF="));
  Serial.print(r.tempFiltC, 2);
  Serial.print(F(",DAY="));
  Serial.print(r.isDay ? F("1") : F("0"));
  Serial.print(F(",L="));
  Serial.print(r.outLightPct, 1);
  Serial.print(F(",B="));
  Serial.println(r.outBubblePct, 1);
}

static void parseSerialLine(char *line) {
  char *tok = strtok(line, " \t");
  if (tok == nullptr) {
    return;
  }

  if (strcmp(tok, "MODE") == 0) {
    char *arg = strtok(nullptr, " \t");
    if (arg != nullptr && strcmp(arg, "STANDBY") == 0) {
      postEvent(EV_WIFI_MISSING);
      Serial.println(F("OK MODE STANDBY"));
      return;
    }
    if (arg != nullptr && strcmp(arg, "M1") == 0) {
      postEvent(EV_MEDITATION_1_REQ);
      Serial.println(F("OK MODE M1"));
      return;
    }
    if (arg != nullptr && strcmp(arg, "M2") == 0) {
      postEvent(EV_MEDITATION_2_REQ);
      Serial.println(F("OK MODE M2"));
      return;
    }

    Serial.println(F("ERR MODE"));
    return;
  }

  if (strcmp(tok, "PAUSE") == 0) {
    postEvent(EV_ACTIVE_PAUSE_REQ);
    Serial.println(F("OK PAUSE"));
    return;
  }

  if (strcmp(tok, "LIGHTOFF") == 0) {
    postEvent(EV_LIGHT_FORCE_OFF);
    Serial.println(F("OK LIGHTOFF"));
    return;
  }

  if (strcmp(tok, "DAYPWM") == 0) {
    char *arg = strtok(nullptr, " \t");
    if (arg == nullptr) {
      Serial.println(F("ERR DAYPWM"));
      return;
    }

    const float v = atof(arg);
    xSemaphoreTake(gMutex, portMAX_DELAY);
    gCfg.dayLightPct = Hal::clampf(v, 5.0f, 100.0f);
    xSemaphoreGive(gMutex);

    Serial.print(F("OK DAYPWM "));
    Serial.println(gCfg.dayLightPct, 1);
    return;
  }

  Serial.println(F("ERR CMD"));
}

static void taskSerial(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();

  static char line[64];
  static uint8_t idx = 0;

  for (;;) {
    while (Serial.available() > 0) {
      const char ch = static_cast<char>(Serial.read());
      if (ch == '\r' || ch == '\n') {
        if (idx > 0) {
          line[idx] = '\0';

          for (uint8_t i = 0; i < idx; ++i) {
            if (line[i] >= 'a' && line[i] <= 'z') {
              line[i] = static_cast<char>(line[i] - ('a' - 'A'));
            }
          }

          parseSerialLine(line);
          idx = 0;
        }
      } else if (idx < sizeof(line) - 1u) {
        line[idx++] = ch;
      } else {
        idx = 0;
        Serial.println(F("ERR LONG"));
      }
    }

    printTelemetry();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
  }
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Hal::init();

  Serial.begin(115200);
  Serial.println(F("BOOT: ESP32 firmware init"));
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000u) {
    // espera corta opcional de puerto serial
  }

  gMutex = xSemaphoreCreateMutex();
  gEventQueue = xQueueCreate(24, sizeof(Event));

  if (gMutex == nullptr || gEventQueue == nullptr) {
    // Estado seguro: luz off + burbujeo mínimo
    Hal::writeLightPct(0.0f);
    Hal::writeBubblePct(20.0f);
    for (;;) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  const float tInit = Hal::readTempC();
  xSemaphoreTake(gMutex, portMAX_DELAY);
  gRt.tempC = tInit;
  gRt.tempFiltC = tInit;
  xSemaphoreGive(gMutex);

  // Tareas (núcleo no fijado en esta iteración)
  xTaskCreate(taskFsm, "fsm", 4096, nullptr, 3, nullptr);
  xTaskCreate(taskSensor, "sensor", 3072, nullptr, 2, nullptr);
  xTaskCreate(taskScheduler, "scheduler", 3072, nullptr, 2, nullptr);
  xTaskCreate(taskButton, "button", 2048, nullptr, 2, nullptr);
  xTaskCreate(taskSerial, "serial", 4096, nullptr, 1, nullptr);

  postEvent(EV_BOOT);
  postEvent(EV_WIFI_MISSING);  // RF-04: sin Internet -> STANDBY
}

void loop() {
  // No usado. La ejecución ocurre en tareas FreeRTOS.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
