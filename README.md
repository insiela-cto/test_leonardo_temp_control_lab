# Control de temperatura en Arduino Leonardo con FreeRTOS

Implementación basada en el hardware del laboratorio de control de temperatura de APMonitor:
https://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl

## Incluye dos estrategias de control

1. **PID clásico**
   - Proporcional + integral + derivativo.
   - Anti-windup por integración condicional.
   - Derivada filtrada.

2. **Control moderno (LQI simplificado)**
   - Modelo térmico discreto de primer orden.
   - Realimentación de estado (temperatura respecto al ambiente).
   - Integrador del error para eliminar offset en régimen.

## Archivo principal

- `leonardo_temp_control_freertos.ino`

## Requisitos

- Placa: **Arduino Leonardo**
- Librería: **Arduino_FreeRTOS**
- Sensor NTC en divisor resistivo (A0)
- Etapa de potencia para calefactor por PWM (D3)

## Comandos seriales

A 115200 baudios:

- `MODE PID` → cambia a controlador PID.
- `MODE MODERN` → cambia a controlador moderno LQI.
- `SP <valor>` → ajusta setpoint en °C (20 a 85).

## Notas de ajuste

- Ajusta `R_FIXED`, `R0`, `BETA` si tu NTC es diferente.
- Ajusta ganancias:
  - PID: `kp`, `ki`, `kd`.
  - Moderno: `a`, `b`, `kx`, `ki`.
- Si el calentamiento es lento o inestable, reduce ganancias antes de aumentar potencia.
