# DroneCAN-to-NMEA Bridge
This project aims to translate DroneCAN messages into standard NMEA 0183 sentences and distribute it simultaneously over WiFi and a wired serial connection. The ARK GPS module also has magnetometer, barometer, and IMU data that may be useful for later phases of this project.

## Hardware
- ESP32
- ARK GPS
- TJA1050 CAN Bus Transceiver
- TTL to RS485 converter
- Standard Horizon GX1600 VHF Radio
- iPad (WiFi only)
- Raspberry Pi running OpenPlotter

## Pin Assignments

| Signal | GPIO |
|---|---|
| CAN TX → TJA1050 | GPIO 5 |
| CAN RX ← TJA1050 | GPIO 4 |
| UART2 TX → RS485 DI | GPIO 17 |
| UART2 RX ← RS485 RO | GPIO 16 |
| RS485 DE/RE enable (active HIGH = transmit) | GPIO 18 |
| Status LED | GPIO 2 (onboard) |

## Build & Flash

This project uses **PlatformIO** with the Arduino framework for ESP32 (`esp32doit-devkit-v1` target). libcanard is fetched automatically on first build.

Build and flash with PlatformIO (`pio run -t upload`) or use the VS Code PlatformIO extension.

## ESP32
ESP32 DEVKIT V1 - DOIT (30 GPIO version)
![ESP32](images/esp32.png)

## ARK GPS
![ARK GPS](images/arkgps.png)
- 5V power
- STM32F412CEU6 MCU
- Ublox M9N GPS
- Bosch BMM150 Magnetometer
- Bosch BMP388 Barometer
- Invensense ICM-42688-P 6-Axis IMU

https://arkelectron.com/product/ark-gps/
https://github.com/ARK-Electronics/ARK_GPS/tree/main
https://docs.px4.io/main/en/dronecan/ark_gps

## TJA1050 CAN Bus Transceiver
![TJA1050](images/TJA1050.jpg)
https://www.amazon.com/dp/B0BLGCSMR1?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_1

## Standard Horizon GX1600 VHF Radio
![GX1600 VHF Radio](images/gx1600.png)
![VHF GPS IN](images/vhfgpsin.png)

## DroneCAN
https://dronecan.github.io/
https://github.com/dronecan/pydronecan
https://github.com/dronecan/libcanard