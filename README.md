# tayutech
repo for collaboration within tayutech activities

detailed pinout of the sensor connectivity: 
# Air Quality Monitor Pinout Documentation

This document describes the wiring/pinout for the air-quality monitor based on:

- Raspberry Pi Pico / Pico 2 W
- Alphasense NO2-B43F + ISB
- Alphasense OX-B431 + ISB
- ADS1115 ADC
- DHT11 temperature/humidity sensor
- CM1109 CO₂ sensor
- PMS7003 Plantower particulate sensor

---

## 1. System Overview

The system measures:

| Sensor | Measurement |
|---|---|
| NO2-B43F + ISB | Nitrogen dioxide, NO₂ |
| OX-B431 + ISB | Oxidising gases, used for O₃ estimation |
| ADS1115 | Analog voltage reading from ISB boards |
| DHT11 | Temperature and relative humidity |
| CM1109 | CO₂ |
| PMS7003 | PM1.0, PM2.5, PM10 |

---

## 2. Power Distribution

### 5 V / VSYS Rail

Use the Pico `VSYS` pin for sensors that require 5 V power.

| Device | Pin | Connects To |
|---|---|---|
| Pico / Pico 2 W | VSYS / physical pin 39 | 5 V power rail |
| NO₂ ISB board | VCC | VSYS |
| OX ISB board | VCC | VSYS |
| PMS7003 | VCC1 | VSYS |
| PMS7003 | VCC2 | VSYS |
| CM1109 | VCC | VSYS / 5 V |

> **Important:** The Alphasense ISB boards should be powered from `VSYS`, not `3V3`, because the ISB supply range is above 3.3 V.

---

### 3.3 V Rail

Use the Pico `3V3 OUT` pin for 3.3 V modules.

| Device | Pin | Connects To |
|---|---|---|
| Pico / Pico 2 W | 3V3 OUT / physical pin 36 | 3.3 V rail |
| ADS1115 | VDD | 3V3 |
| DHT11 | VCC | 3V3 |

---

### Common Ground

All grounds must be connected together.

| Device | Pin | Connects To |
|---|---|---|
| Pico / Pico 2 W | GND | Common GND |
| ADS1115 | GND | Common GND |
| NO₂ ISB board | GND / `-` | Common GND |
| OX ISB board | GND / `-` | Common GND |
| PMS7003 | GND1 | Common GND |
| PMS7003 | GND2 | Common GND |
| CM1109 | GND | Common GND |
| DHT11 | GND | Common GND |

> **Breadboard warning:** Many breadboards have split power rails. Check continuity and bridge the GND rails if required.

---

## 3. Pico Pin Usage Summary

| Pico GPIO | Physical Pin | Connected Device | Function |
|---:|---:|---|---|
| GP0 | Pin 1 | CM1109 RX | UART0 TX |
| GP1 | Pin 2 | CM1109 TX | UART0 RX |
| GP4 | Pin 6 | PMS7003 RX | UART1 TX, optional |
| GP5 | Pin 7 | PMS7003 TX | UART1 RX |
| GP8 | Pin 11 | ADS1115 SDA | I2C SDA |
| GP9 | Pin 12 | ADS1115 SCL | I2C SCL |
| GP16 | Pin 21 | DHT11 DATA | Digital input |
| 3V3 OUT | Pin 36 | ADS1115 + DHT11 power | 3.3 V |
| VSYS | Pin 39 | ISB boards + PMS7003 + CM1109 power | ~5 V from USB |
| GND | Multiple | All modules | Common ground |

---

## 4. ADS1115 to Pico

The ADS1115 reads the analog outputs from the two Alphasense ISB boards.

| ADS1115 Pin | Connects To | Function |
|---|---|---|
| VDD | Pico 3V3 | ADS1115 power |
| GND | Common GND | Ground |
| SDA | Pico GP8 / physical pin 11 | I2C SDA |
| SCL | Pico GP9 / physical pin 12 | I2C SCL |
| ADDR | GND or floating | I2C address `0x48` |
| ALRT | Not connected | Not used |

Code configuration:

```cpp
Wire.setSDA(8);
Wire.setSCL(9);
ads.begin(0x48);
