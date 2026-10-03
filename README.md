# V1 Flight Computer

An STM32-based rocket flight computer designed by Anish Agarwal and Aidan Gonzales. This repository contains the KiCad schematics and PCB layout, custom component libraries, and firmware developed in STM32CubeIDE.

The board brings together flight sensing, GPS, LoRa telemetry, microSD logging, main and drogue deployment channels, and a servo output for airbrakes.

## Hardware

The PCB uses four copper layers and an STM32F446RE microcontroller.

| Component | Purpose | Firmware interface |
| --- | --- | --- |
| BMI088 | Accelerometer and gyroscope | I2C1 |
| BMP388 | Pressure, temperature, and barometric altitude | I2C1 |
| LIS3MDL | Magnetometer | I2C1 |
| u-blox NEO-M9N | GPS | USART6 |
| E22-900MM22S | LoRa telemetry | SPI1 |
| W25Q128JVS | 128-Mbit external flash | SPI2 |
| microSD | CSV data logging | SPI3 |

The design also includes battery-voltage and deployment-continuity sensing, a buzzer, a heartbeat LED, and USB-C. The active firmware uses the BMI088; a BMI330 driver is also included in the source tree.

## Firmware

The application is written in C using the STM32 HAL and FatFs. Most of the flight logic is in [`main.c`](V1_STM32/Core/Src/main.c), with separate drivers for the sensors, GPS, radio, SD card, and flash.

The main loop handles:

- Sensor readings and flight-state updates on a nominal 50 ms interval.
- Ground-relative barometric altitude, filtered altitude, and estimated vertical velocity.
- Launch and apogee detection, followed by drogue and main deployment states.
- GPS stream processing and LoRa packet transmission, scheduled at 200 ms intervals.
- Buffered CSV logging to the SD card.
- Servo airbrake control using predicted apogee and a PD controller, enabled seven seconds after launch.

Flight thresholds, deployment settings, and servo limits are defined in `main.c`. The airbrake target and controller gains are inside `Task_Airbrakes()`.

### Data logging

On startup, the firmware selects a numbered log file such as `LOG_0.CSV` or `LOG_1.CSV`. Logged fields include altitude, pressure, temperature, flight state, airbrake output, filtered altitude and velocity, magnetometer readings, and GPS data.

Battery and continuity columns are present, but their periodic sampling task is currently commented out. Data rows also include a Unix timestamp that is missing from the startup CSV header.

The flash driver and a binary logging routine are included, but the current main loop uses the SD logging path and does not call the flash-log flush routine.

## Repository layout

| Path | Contents |
| --- | --- |
| `V1_Hardware/` | KiCad project, schematics, PCB layout, footprints, and 3D models |
| `V1_STM32/` | STM32CubeIDE project and CubeMX configuration |
| `V1_STM32/Core/Src/` | Main application and device drivers |
| `V1_STM32/Core/Inc/` | Headers, telemetry packet definitions, and buffer utilities |
| `V1_STM32/FATFS/` | FatFs configuration and SD disk interface |
| `V1_STM32/Drivers/` | STM32 HAL and CMSIS files |
| `LRA_Parts/` | Component symbols, footprints, and models |

## Opening the project

### Hardware

1. Open `V1_Hardware/V1.kicad_pro` in KiCad. The checked-in design files were saved with KiCad 10.0.
2. In **Preferences → Configure Paths**, set `LRA_PARTS` to the repository's `LRA_Parts` directory.
3. Check the project symbol-library table. Some entries still use absolute Windows paths and need to be pointed to your local copies. Entries that point to a directory should be updated to the corresponding `.kicad_sym` file.

The root schematic is `V1.kicad_sch`. Separate sheets cover power, deployment circuitry, USB-C, memory, GPS/telemetry, and sensors.

### Firmware

1. In STM32CubeIDE, choose **File → Import → General → Existing Projects into Workspace** and select `V1_STM32`.
2. Build the imported project. The repository includes the HAL, CMSIS, and FatFs sources.
3. Use an ST-LINK connection over SWD to program and debug the board. Check the probe settings in the included `V1 Debug.launch` configuration for your setup.

`V1.ioc` contains the CubeMX configuration and records STM32CubeMX 6.14.1 with STM32CubeF4 firmware package V1.28.3. Check the existing application code before regenerating peripheral initialization code.

## Development notes

Some earlier bring-up routines and alternate logging paths remain in the source. Follow the active calls in `main.c` when checking current behavior; a few older comments describe previous pin assignments or hardware revisions.

The `LANDED` state currently follows the main-deployment timer rather than a separate touchdown detector.
