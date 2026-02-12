# Control de temperatura en Arduino Leonardo con FreeRTOS

Implementación basada en el hardware del laboratorio de control de temperatura de APMonitor:
https://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl

## Incluye dos estrategias de control

1. **PID clásico**
   - Proporcional + integral + derivativo.
   - Anti-windup por integración condicional.
   - Derivada filtrada.

2. **Control moderno (LQI-like con observador)**
   - Modelo térmico discreto de primer orden.
   - Observador para estimar estado térmico interno.
   - Realimentación de estado + integrador de error.
   - Anti-windup por back-calculation.

## Archivo principal

- `leonardo_temp_control_freertos.ino`

## Requisitos

- Placa: **Arduino Leonardo**
- Librería: **Arduino_FreeRTOS** (feilipu)
- Sensor NTC en divisor resistivo (A0)
- Etapa de potencia para calefactor por PWM (D3)

## Comandos seriales

A 115200 baudios:

- `MODE PID` → cambia a controlador PID.
- `MODE MODERN` → cambia a controlador moderno.
- `SP <valor>` → ajusta setpoint en °C (20 a 85).
- `PID <kp> <ki> <kd>` → cambia ganancias PID.
- `MODERN <kx> <ki>` → cambia ganancias del controlador moderno.

## Prueba de compilación con arduino-cli

### En Windows (ruta que indicaste)

Si tu `arduino-cli.exe` está en:

`C:\arduino-cli_1.4.1_Windows_64bit`

Ejemplo en PowerShell:

```powershell
cd <ruta_del_repo>
& "C:\arduino-cli_1.4.1_Windows_64bit\arduino-cli.exe" core update-index
& "C:\arduino-cli_1.4.1_Windows_64bit\arduino-cli.exe" core install arduino:avr
& "C:\arduino-cli_1.4.1_Windows_64bit\arduino-cli.exe" lib install "Arduino_FreeRTOS"
& "C:\arduino-cli_1.4.1_Windows_64bit\arduino-cli.exe" compile --fqbn arduino:avr:leonardo .
```

## Notas de ajuste

- Ajusta `R_FIXED`, `R0`, `BETA` si tu NTC es diferente.
- Ganancias iniciales sugeridas:
  - PID: `kp=5.0`, `ki=0.18`, `kd=10.0`
  - Moderno: `kx=1.10`, `ki=0.30`
- Si aparece oscilación, baja primero `ki` y luego `kp`/`kx`.
