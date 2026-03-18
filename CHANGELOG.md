# Changelog

All notable changes to the Fillhead firmware will be documented in this file.

## [0.1.0] - 2026-03-18

Full architecture alignment: Fillhead is now a strict superset of Pressboi. Every Pressboi feature, command, and subsystem has been ported into Fillhead alongside its existing injection, valve, heater, and vacuum functionality.

### State Machine
- Added `STATE_RESETTING` for non-blocking motor re-enable after reset
- Added `STATE_RECOVERED` for watchdog recovery detection (motors disabled until explicit reset)
- Replaced blocking `clearErrors()` with phased non-blocking reset sequence
- Added fault grace period (`m_faultGracePeriodEnd`) to suppress spurious faults after recovery
- Upgraded watchdog recovery to use `RSTC->RCAUSE.reg` for proper reset-cause detection
- Added `clearWatchdogRecovery()` to restore normal operation from recovered state
- Moved watchdog feed into `performSafetyCheck()` at loop start
- Added RECOVERED and ERROR state command filtering (only reset, discover, dump_error_log allowed)

### Motor / Injector Controller
- Ported `moveAbsolute()` and `moveIncremental()` for general-purpose position moves with force limits
- Ported `setRetract()` and `retract()` for retract position (saved to NVM)
- Added full `MoveState` state machine alongside existing `FeedState` for injection
- Added `pauseOperation()`, `resumeOperation()`, `cancelOperation()` for general moves
- Added `finalizeAndResetActiveMove()` and `fullyResetActiveMove()`
- Added non-blocking `EnableState` state machine (`ENABLE_IDLE`, `ENABLE_WAITING`, `ENABLE_COMPLETE`, `ENABLE_TIMEOUT`)
- Added `handleCommand()` dispatcher matching Pressboi's pattern

### Force Sensing
- Ported motor torque force mode (`m_force_mode` defaults to `"motor_torque"`)
- Added `setForceMode()`, `getForceMode()`, `setForceCalibrationOffset()`, `setForceCalibrationScale()`
- Added `checkForceSensorStatus()` for motor torque validity
- Added per-move force limit with configurable action: retract, hold, skip, abort
- Added `handleLimitReached()` for force limit enforcement during moves
- Ported joule integration with 50Hz sampling (`updateJoules()`)
- Ported machine strain polynomial compensation (5-coefficient)
- Added `evaluateMachineStrainForceFromDeflection()` and `estimateMachineDeflectionFromForce()`
- Added `setMachineStrainCoeffs()` (saved to NVM)

### Hall Sensor Homing
- Replaced torque-based homing with dual hall sensor homing (sensor PN: 326161-0053)
- Added per-axis gantry squaring (each axis stops independently on sensor trigger)
- Added homing phases: rapid approach, backoff, slow approach, final backoff, set zero
- Preserved machine home / cartridge home dual-home concept
- Added homing timeout (`MAX_HOMING_DURATION_MS` = 100s)
- Added per-axis motor control: `stopAxis()`, `isAxisMoving()`, `isHomeSensorTriggered()`

### NVM Support
- Added full NVM layout with magic number validation (`0x464C4831` = "FLH1")
- Stored settings: polarity, force mode, torque offset/scale, home-on-boot, retract position, press threshold, machine strain coefficients
- Added `CMD_DUMP_NVM` with hex/ASCII output and field interpretation
- Added `CMD_RESET_NVM` to restore factory defaults
- All setters write to NVM immediately

### Polarity & Press Threshold
- Added polarity system (`"normal"` / `"inverted"`) saved to NVM
- Added `setPolarity()` / `getPolarity()` with direction inversion on moves
- Added `setPressThreshold()` / `getPressThreshold()` saved to NVM
- Added startpoint recording when press threshold is crossed

### Home on Boot
- Added `m_home_on_boot` flag (from NVM, default true)
- Added delayed auto-home 2s after boot when enabled
- Added `setHomeOnBoot()` / `getHomeOnBoot()`

### Commands
- Created standalone `commands.h` / `commands.cpp` (moved `parseCommand()` out of `CommsController`)
- Switched from fixed-width padded command strings to Pressboi's minimal format (single trailing space)
- Added all Pressboi commands: `home`, `move_abs`, `move_inc`, `set_retract`, `retract`, `pause`, `resume`, `cancel`
- Added system commands: `reset`, `enable`, `disable`, `test_watchdog`, `reboot_bootloader`
- Added NVM commands: `dump_nvm`, `reset_nvm`, `dump_error_log`
- Added force commands: `set_force_mode`, `set_force_offset`, `set_force_zero`, `set_force_scale`, `set_strain_cal`
- Added config commands: `set_polarity`, `home_on_boot`, `set_press_threshold`
- Renamed `clear_errors` to `reset` for Pressboi consistency
- Removed `test_command` (dead code)

### Events System
- Created `inc/events.h` and `src/events.cpp` (new files)
- Defined status prefixes: `FILLHEAD_INFO`, `FILLHEAD_START`, `FILLHEAD_DONE`, `FILLHEAD_ERROR`, `FILLHEAD_RECOVERY`
- Defined telemetry prefix: `FILLHEAD_TELEM`
- Added `Event` enum, `sendEvent()`, `sendEventString()`, `sendEventMulti()`
- Moved prefixes out of `definition/responses.h` into `events.h` as single source of truth

### Communications
- Removed `parseCommand()` and `getCommandParams()` from `CommsController`
- Updated discovery response to include `PORT` and `FW` fields
- Added USB vs network detection (skip `setGuiIp()` for localhost)
- Added 500ms telemetry stabilization delay after GUI discovery

### Telemetry
- Added all Pressboi telemetry fields: force (load cell, motor torque), energy (joules), position (current, retract, target, endpoint, startpoint), press threshold, torque average
- Added home sensor state fields (`home_sensor_m0`, `home_sensor_m1`)
- Added `force_mode`, `polarity`, `force_limit`, `force_source`, `force_adc_raw`
- Added `gui_var` bindings on all telemetry fields
- Changed `main_state` from integer to string type

### Configuration
- Added `FIRMWARE_VERSION "0.1.0"` to `config.h`
- Added hall sensor pin definitions (`HOME_SENSOR_M0` on DI7, `HOME_SENSOR_M1` on DI6)
- Added force sensor constants (scale, offset, min/max, timeout)
- Added machine strain compensation coefficients (5-term polynomial)
- Added move defaults (torque %, velocity, acceleration) for general-purpose moves
- Added `RETRACT_DEFAULT_SPEED_MMS`
- Removed `POST_ABORT_DELAY_MS` (replaced by non-blocking reset)

### Definition Folder
- Merged all Pressboi commands into `commands.json` with descriptions, help text, enums, and defaults
- Updated `telemetry.json` with all Pressboi + Fillhead fields and `gui_var` bindings
- Updated `events.json` schema alignment
- Added `energy_warning` and `endpoint_warning` to `warnings.json`
- Added `device_name` field to `firmware_config.json`
- Created `views.json` with Fillhead operator view definition
- Regenerated `command_parser.cpp` / `command_parser.h` from updated commands
- Regenerated `telemetry.cpp` / `telemetry.h` from updated telemetry
- Regenerated `commands.h` / `responses.h` from updated definitions

### Simulator
- Added Pressboi command simulation: `home`, `move_abs`, `move_inc`, `retract`, `set_retract`, `pause`, `resume`, `cancel`
- Added force/torque simulation with random force generation during moves
- Added position tracking for absolute and incremental moves
- Added joule simulation
- Added home sensor state tracking
- Fixed event prefixes to use `FILLHEAD_DONE:`, `FILLHEAD_ERROR:`, etc.
- Fixed inconsistent command string matching

### GUI
- Added force display (motor torque with force mode indicator)
- Added position display (absolute position in mm)
- Added retract position display
- Added energy (joules) display
- Added endpoint/startpoint display
- Added home sensor indicators (M0, M1)
- Added polarity indicator
- Created `operator_view.py` with PASS/FAIL display, job/serial fields, and cycle time
- Ported `draw_vertical_text`, `make_force_tracer`, `make_unit_stripper` helpers from Pressboi

### Build / CI
- Created `.github/workflows/release.yml` adapted from Pressboi (binary names: `fillhead.bin` / `fillhead.uf2`)
- Updated `.gitignore` to include `!Debug/*.uf2`, `!Debug/*.bin`, `!Debug/*.elf`, `!Debug/*.map`, `!Debug/Makefile`, `!Debug/makedep.mk`, `__pycache__/`, `*.pyc`

---

## [1.0.0] - 2025-11-17

### Added
- **Initial release**: Fillhead injection system firmware
- **Dual injector motors**: Synchronized control of two ganged injector motors for material injection
- **Pinch valve control**: Two independent pinch valves (vacuum and injection sides) with motorized control
- **Heater control**: PID temperature control with thermocouple feedback and relay output
- **Vacuum system**: Pressure monitoring and control with transducer feedback
- **UDP/Ethernet communication**: Network-based control with device discovery
- **USB Serial support**: Direct USB communication fallback
- **Command-based control**: Simple text-based command protocol for all operations
- **Real-time telemetry**: Position, temperature, pressure, torque, and status reporting
- **Homing routines**: Automatic homing for injector and pinch valves (tubed/untubed modes)
- **Injection control**: Volume-based injection with configurable speeds and piston diameter support
- **Leak testing**: Automated vacuum leak detection with configurable thresholds
- **Error handling**: Comprehensive error reporting and recovery mechanisms

### Features
- **Injector system**: Dual motor synchronized control with configurable pitch and steps/mm
- **Pinch valves**: Motorized pinch valves with separate homing routines for tubed/untubed operation
- **Temperature control**: PID-based heater control with configurable setpoints and gains
- **Vacuum control**: Pressure monitoring with configurable targets and ramp timeouts
- **Material handling**: Support for stator and rotor injection modes with different piston diameters
- **Feed operations**: Configurable feed speeds and accelerations for material handling

### Hardware Support
- Built on Teknic ClearCore platform
- ClearPath motor drivers (M0-M3 connectors)
- Analog inputs for thermocouple and vacuum transducer
- Digital I/O for relay control (heater, vacuum pump, solenoid valve)
- Ethernet connectivity for network communication
- USB serial for direct communication
