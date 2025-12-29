[!CAUTION]
Do not use. it will not work and is work in progress

# Itho I2C ESPHome Custom Component

This ESPHome custom component allows you to control Itho ventilation units (CVE, HRU series) via I2C using an ESP32.

## Overview

This component is a replacement for the [ithowifi](https://github.com/arjenhiemstra/ithowifi) project, allowing you to use ESPHome instead of the custom firmware. It provides:

- Direct I2C communication with Itho ventilation units
- Fan speed control (0-100% or preset speeds)
- Temperature and humidity monitoring (if supported by your device)
- Device type detection
- Status queries
- Integration with Home Assistant via ESPHome API

## Supported Devices

Based on the original ithowifi project, this should support:

- CVE ECO 2
- CVE-S OPTIMA
- CVE Silent
- HRU 200 ECO
- HRU 250-300
- HRU 350
- HRU ECO-fan
- DemandFlow
- AutoTemp
- And other Itho devices using I2C protocol

## Hardware Requirements

### I2C Connection to Itho Unit

The hardware connects directly to your Itho unit's I2C bus. Pin configuration depends on your hardware:

#### CVE Hardware (Original ithowifi board):
- **SDA:** GPIO21
- **SCL:** GPIO22
- **Status LED:** GPIO13

#### NON-CVE Hardware:
- **SDA:** GPIO27
- **SCL:** GPIO26

**Important:** The I2C bus operates at **100kHz**. Pull-up resistors are typically not needed as they're usually present on the Itho mainboard.