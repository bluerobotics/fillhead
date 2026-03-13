<div align="center">

<img src="assets/icon.png" alt="Fillhead" width="150">

# Fillhead

**Material Injection System Control**

[![release](https://img.shields.io/github/v/release/bluerobotics/fillhead?style=flat-square)](https://github.com/bluerobotics/fillhead/releases/latest)
[![build](https://img.shields.io/github/actions/workflow/status/bluerobotics/fillhead/release.yml?style=flat-square)](https://github.com/bluerobotics/fillhead/actions)
[![downloads](https://img.shields.io/github/downloads/bluerobotics/fillhead/total?style=flat-square)](https://github.com/bluerobotics/fillhead/releases)
[![license: MIT](https://img.shields.io/badge/license-MIT-blue.svg?style=flat-square)](LICENSE)

[View Changelog](CHANGELOG.md) • [Download Latest Release](https://github.com/bluerobotics/fillhead/releases/latest)

</div>

---

## Overview

Firmware for the Fillhead material injection system, built on the Teknic ClearCore platform. This firmware provides precision control for material injection operations with dual-motor injector control, pinch valve management, temperature control, and vacuum system monitoring.

**Designed to work with:** [BR Equipment Control App](https://github.com/bluerobotics/br-equipment-control-app) - A Python/Tkinter GUI for controlling and monitoring the Fillhead and other Blue Robotics manufacturing equipment.

## Features

- **Dual injector motors** - Synchronized control of two ganged motors for material injection
- **Pinch valve control** - Two independent motorized pinch valves (vacuum and injection sides)
- **Temperature control** - PID-based heater control with thermocouple feedback
- **Vacuum system** - Pressure monitoring and control with transducer feedback
- **UDP/Ethernet communication** - Network-based control with device discovery
- **USB Serial fallback** - Direct USB communication support
- **Command-based control** - Simple text-based command protocol
- **Real-time telemetry** - Position, temperature, pressure, torque, and status reporting
- **Homing routines** - Automatic homing for injector and pinch valves (tubed/untubed modes)
- **Leak testing** - Automated vacuum leak detection

---

## Building

### Requirements

- **Atmel Studio 7** (Windows) or compatible ARM GCC toolchain
- **Teknic ClearCore libraries** (included in `lib/` folder)
  - `libClearCore`
  - `LwIP` (Lightweight IP stack)

### Build Instructions

1. Open `fillhead.atsln` in Atmel Studio
2. Ensure libraries are properly referenced:
   - `lib/libClearCore/ClearCore.cppproj`
   - `lib/LwIP/LwIP.cppproj`
3. Select build configuration (Debug or Release)
4. Build the solution (F7)

### Output Files

- `Debug/fillhead.bin` - Binary firmware image
- `Debug/fillhead.uf2` - UF2 format for bootloader flashing

### Troubleshooting

- **Microchip Studio instead of Atmel Studio**  
  The generated Makefiles use `PACK_BASE` and `TOOLCHAIN_BASE`. If your install is under a different path (e.g. Microchip Studio), either:
  - Open the solution in the IDE and do **Clean Solution** then **Rebuild Solution** so the IDE regenerates the Makefiles with your paths, or
  - Before building from the command line, set:
    - `PACK_BASE=C:\Program Files (x86)\Microchip\Studio\7.0\Packs`
    - `TOOLCHAIN_BASE=C:\Program Files (x86)\Microchip\Studio\7.0\toolchain`
- **`arm_math.h: No such file or directory`**  
  Install the **ARM CMSIS 4.5.0** (and 5.4.0) pack from the IDE’s Pack Manager (or equivalent) so `arm_math.h` is available under your packs folder.

## Flashing

### Via BR Equipment Control App (Recommended for Updates)

The easiest way to update firmware on an already-running device is through the [BR Equipment Control App](https://github.com/bluerobotics/br-equipment-control-app):

1. Open the Firmware Manager in the app
2. Select your device
3. Choose the firmware file or download the latest release
4. Click "Update Firmware" - the app handles the entire flashing process automatically

**Note:** Firmware flashing is only supported over USB connections. For initial flashing of a new device, use the bootloader method below.

### Via Bootloader (For Initial Flashing)

For initial flashing of a new device or when the app is not available:

1. Put the ClearCore into bootloader mode (hold button during power-on)
2. Copy `fillhead.uf2` to the mounted bootloader drive
3. The device will automatically reboot with new firmware

### Via Atmel Studio

1. Connect ClearCore via USB
2. Select "Custom Programming Tool" in project settings
3. Build and program (F5)

---

## Communication Protocol

The firmware uses a simple text-based command protocol over UDP or USB serial:

- Commands: `inject <volume> ml`, `feed <distance> mm`, `home_injector`, `set_heater <temp> C`
- Status: `DONE: <command>`, `ERROR: <message>`, `INFO: <message>`
- Telemetry: Periodic status updates with position, temperature, pressure, torque

See the [BR Equipment Control App](https://github.com/bluerobotics/br-equipment-control-app) for full protocol documentation and command reference.

---

## Hardware Configuration

### Motors
- **Injector A**: Primary injector motor (M0)
- **Injector B**: Secondary injector motor (M1) - ganged with A
- **Vacuum Valve**: Pinch valve motor for vacuum side (M2)
- **Injection Valve**: Pinch valve motor for injection side (M3)

### Sensors
- **Thermocouple**: Analog input (A12) for temperature monitoring
- **Vacuum Transducer**: Analog input (A11) for pressure monitoring

### Outputs
- **Heater Relay**: Digital output (IO1) for heater control
- **Vacuum Pump Relay**: Digital output (IO0) for vacuum pump control
- **Vacuum Solenoid Relay**: Digital output (IO5) for solenoid valve control

## Configuration

Key parameters can be configured in `inc/config.h`:

- **Motor parameters**: Steps per mm, velocities, accelerations
- **Temperature control**: PID gains, setpoints, sensor calibration
- **Vacuum control**: Pressure ranges, targets, ramp timeouts
- **Injection parameters**: Piston diameters, speeds, feed rates
- **Homing parameters**: Velocities, accelerations, torque limits for tubed/untubed modes
- **Network settings**: UDP port, packet sizes, telemetry interval

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

Copyright (c) 2025 Blue Robotics

## Contributing

For issues, feature requests, or contributions, please open an issue or pull request on GitHub.

---

<div align="center">

⭐ **Star us on GitHub if you found this useful!**

Made with 💙 by the Blue Robotics team and contributors worldwide

---

<img src="assets/logo.png" alt="Blue Robotics" width="300">

**[bluerobotics.com](https://bluerobotics.com)** | Manufacturing Equipment Control

</div>

