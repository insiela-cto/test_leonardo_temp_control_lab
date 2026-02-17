# Firmware base (ESP32 + FreeRTOS) orientado a requerimientos

Este proyecto quedó ajustado al **propósito original para ESP32**.

## Qué implementa esta iteración

- Arquitectura por capas:
  - **HAL**: única capa que toca hardware.
  - **Servicios base**: sensor + scheduler temporal autónomo.
  - **FSM central**: autoridad única de transición de modos.
  - **Composición de salidas**: SBP + overrides autorizados.
- **Servicio Biológico Permanente (SBP)**:
  - Burbujeo mínimo siempre activo.
  - Ciclo autónomo 8h luz / 16h oscuridad.
- Modos de operación en FSM:
  - `INIT`
  - `UNCONFIGURED`
  - `STANDBY`
  - `USER_LIGHT_ON`
  - `MEDITATION_1`
  - `MEDITATION_2`
  - `ACTIVE_PAUSE`
  - `ERROR`

Archivo principal:
- `leonardo_temp_control_freertos.ino`

> El nombre del archivo se conserva por continuidad del repositorio, pero el contenido está orientado a ESP32.

## Mapeo de hardware actual (ESP32)

- Luz blanca PWM: GPIO `18` (LEDC canal 0)
- Burbujeo PWM: GPIO `19` (LEDC canal 1)
- NTC (ADC): GPIO `34` (ADC1)
- Botón físico: GPIO `27` (INPUT_PULLUP)

## Comandos seriales (115200)

- `MODE STANDBY`
- `MODE M1`
- `MODE M2`
- `PAUSE`
- `LIGHTOFF`
- `DAYPWM <5..100>`

Telemetría cada segundo:
- Estado
- Temperatura instantánea y filtrada
- Día/Noche
- PWM de luz y burbujeo

## Requisitos de compilación

- Core Arduino para ESP32 instalado.
- Board example: `esp32:esp32:esp32`

Compilación (referencia):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

## Pendiente en siguientes iteraciones

- WiFi AP/STA + mDNS + webserver asíncrono.
- Persistencia NVS (sin escrituras periódicas de tiempo).
- Sincronización NTP cada 15 min + fallback relativo.
- Mantenimiento semanal/trimestral y reset físico.
- Seguridad/autenticación de escritura en web.
- OTA con validación y rollback.
