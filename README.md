> [!IMPORTANT]
> This project will not work anymore since esphome 2026.2.0. Due to nt using the device anymore no more updates will be done
> 

# Itho I2C ESPHome Custom Component

> [!WARNING]
> **Work in Progress - Use at Your Own Risk**
>
> This project is currently under active development and testing. Features may not work as expected, and things may break. Only use this if you're comfortable troubleshooting and understand the risks involved.

> [!IMPORTANT]
> **Limited Device Compatibility**
>
> This component has **only been tested with the Itho CVE ECO 2** ventilation unit. While the code is based on the original ithowifi project that supports multiple devices, compatibility with other Itho models has not been verified. Your device may work, or it may not.

## Overview

This ESPHome custom component allows you to control Itho ventilation units via I2C using an ESP32. It's a replacement for the [ithowifi](https://github.com/arjenhiemstra/ithowifi) project, using ESPHome instead of custom firmware.

### Current Status

✅ **Working Features (CVE ECO 2):**
- Remote command support (Low, Medium, High, Auto modes)
- Fan speed monitoring (RPM and percentage)
- Temperature and humidity sensors
- CO2 sensor (if device has CO2 capability)
- Status queries every 30 seconds
- Device type detection
- Selected mode sensor (numeric and text)
- Virtual remote join/leave
- Home Assistant integration via ESPHome API

⚠️ **Known Limitations:**
- Only tested on CVE ECO 2 units
- PWM fan speed control may revert to auto on some units
- Some sensor indices may vary by device model
- Timer commands not yet implemented
- Cook modes not yet implemented

🔧 **Under Development:**
- Support for other CVE/HRU models
- Additional remote commands
- PWM speed persistence improvements

## Supported Devices

**Confirmed Working:**
- ✅ **CVE ECO 2** - Fully tested and working

**Potentially Compatible (Untested):**
- ⚠️ CVE-S OPTIMA
- ⚠️ CVE Silent
- ⚠️ HRU 200 ECO
- ⚠️ HRU 250-300
- ⚠️ HRU 350
- ⚠️ HRU ECO-fan
- ⚠️ DemandFlow
- ⚠️ AutoTemp

*These devices use similar I2C protocols but have not been tested. Sensor indices and available commands may differ.*

## Hardware Requirements

### ESP32 Board
https://www.nrgwatch.nl/product/itho-cve-rft-wifi-add-on/

### I2C Connection to Itho Unit

The ESP32 connects directly to your Itho unit's I2C bus.

#### Pin Configuration

**CVE Hardware (Original ithowifi board):**
- **SDA:** GPIO21
- **SCL:** GPIO22
- **Status LED:** GPIO13 (optional)

**NON-CVE Hardware:**
- **SDA:** GPIO27
- **SCL:** GPIO26

**Important Notes:**
- I2C bus operates at **100kHz**
- Pull-up resistors typically not needed (present on Itho mainboard)
- Use proper level shifting if your ESP32 runs at 3.3V and Itho expects 5V (usually not required)

### Wiring

Connect the ESP32 to the Itho unit's service connector:

```
Itho Unit          ESP32
---------          -----
I2C SDA   ------>  GPIO21 (or GPIO27)
I2C SCL   ------>  GPIO22 (or GPIO26)
GND       ------>  GND
5V        ------>  5V (power for ESP32)
```

**Safety Warning:** Ensure proper electrical isolation. Do not hot-plug connections. Power off the Itho unit before making connections.

## Usage

### First Time Setup

1. **Upload firmware** to your ESP32
2. **Join the virtual remote** - Press the "Join Remote" button in Home Assistant
3. **Wait 5-10 seconds** for the unit to register the remote
4. **Test commands** - Try pressing "High Mode" to verify it works

### Mode Control

The unit can be controlled in several ways:

**Remote Commands (Recommended):**
- **Auto** - Automatic humidity-based control
- **Low** - Low speed preset (mode 2)
- **Medium** - Medium speed preset (mode 3)
- **High** - High speed preset (mode 4)

**PWM Control (May revert to auto):**
```yaml
# Set specific fan speed 0-100%
- lambda: |-
    id(itho_device).set_speed(50);  // 50%
```

### Monitoring

The component provides real-time monitoring:

- **Temperature** - Room temperature in °C
- **Humidity** - Room humidity in %
- **CO2** - CO2 concentration in ppm (if device has CO2 sensor)
- **Fan Speed RPM** - Actual fan speed
- **Fan Setpoint** - Target fan speed
- **Selected Mode** - Current operating mode (numeric)
- **Selected Mode Text** - Current mode as text (e.g., "High", "Auto")

### Mode Values

| Numeric | Text | Description |
|---------|------|-------------|
| 0 | Standby | Minimal ventilation |
| 1 | Away | Away mode (very low) |
| 2 | Low | Low speed preset |
| 3 | Medium | Medium speed preset |
| 4 | High | High speed preset |
| 5 | Auto | Automatic control |
| 6-8 | Timer1-3 | Timer modes |
| 9 | AutoNight | Night mode |

## Troubleshooting

### Remote Commands Don't Work

**Solution:** You must join the virtual remote first!

1. Press the "Join Remote" button
2. Wait 5-10 seconds
3. Try the remote command again

### Sensors Show "Unavailable"

- Wait 30 seconds for the first status query
- Check I2C wiring (SDA/SCL)
- Verify I2C bus configuration (100kHz)
- Check ESP32 logs for errors

### Wrong Sensor Values

Different Itho models may use different sensor indices. Check logs to see which indices contain what data, then adjust your configuration.

For CVE ECO 2, the working indices are:
- Index 1: Fan setpoint (RPM)
- Index 2: Fan speed (RPM)
- Index 4: Selected mode
- Index 5: Startup counter
- Index 6: Total operation (hours)
- Index 7: Highest CO2 concentration (ppm) - peak value
- Index 8: CO2 concentration (ppm) - current value
- Index 9: Valve position
- Index 10: Humidity (%)
- Index 11: Temperature (°C)

**Note:** Other Itho models may use different indices. Check your logs after uploading firmware to verify.

### PWM Speed Reverts to Auto

This is expected behavior on some units. The Itho unit's internal controller may override manual PWM commands. Use the preset remote commands (Low/Medium/High) instead.


## Contributing

This project is in early development. Contributions are welcome, especially:

- Testing on other Itho device models
- Sensor index mapping for different devices
- Additional command implementations
- Bug fixes and improvements

**If you test this on a different device:**
1. Document your device model
2. Share sensor indices and working commands
3. Submit an issue or pull request with your findings

## Credits

This project is based on the excellent [ithowifi](https://github.com/arjenhiemstra/ithowifi) project by arjenhiemstra and contributors. The I2C protocol implementation and command structures are derived from that project.

## License

This project inherits the license from the original ithowifi project.

## Disclaimer

**USE AT YOUR OWN RISK**

This is an unofficial third-party component. The authors are not responsible for:
- Damage to your Itho ventilation unit
- Damage to your ESP32 or other hardware
- Loss of warranty on your equipment
- Any other consequences of using this software

This software is provided "as is" without warranty of any kind. Only use this if you understand the risks and are comfortable troubleshooting issues.

**Not affiliated with Itho Daalderop BV.**

## Support

For issues, questions, or contributions:
- Open an issue on GitHub
- Check existing documentation files
- Review the original ithowifi project for protocol details

**Remember:** This is a work in progress. Be patient, report issues constructively, and contribute if you can!
