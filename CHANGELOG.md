# Changelog

All notable changes to the Fillhead firmware will be documented in this file.

## [0.2.0] - 2026-03-18

### Pinch Valve Homing
- Replaced torque-based homing with hall sensor homing (open-side sensor, negative direction); states: CHECK_SENSOR → RAPID_APPROACH → BACKOFF → SLOW_APPROACH → SET_ZERO
- Handles already-on-sensor edge case by skipping to backoff
- Added per-valve `home_on_boot` NVM flags (default enabled); removed all tubed/untubed homing variants and torque-based logic
- Added `home_sensor` field to pinch valve telemetry

### Safety
- Light curtain on IO4 checked every loop under watchdog; beam break aborts all motion → `STATE_ERROR`, blocks reset while obstructed
- Added `lc` telemetry field (1=tripped, 0=clear) and compile-time `LIGHT_CURTAIN_ENABLED` toggle

### Configuration
- Light curtain pins/config: `PIN_LIGHT_CURTAIN` (IO4), `LIGHT_CURTAIN_ACTIVE_STATE`, `LIGHT_CURTAIN_FILTER_MS`
- Hall sensor pins: `HOME_SENSOR_M2` (DI8, vacuum valve), `HOME_SENSOR_M3` (A9, injection valve)
- Pinch homing constants: `PINCH_HOMING_STROKE_MM`, `PINCH_HOMING_RAPID_VEL_MMS`, `PINCH_HOMING_SLOW_VEL_MMS`, `PINCH_HOMING_ACCEL_MMSS`, `PINCH_HOMING_BACKOFF_MM`
- NVM slots: `NVM_SLOT_INJ_VALVE_HOME_ON_BOOT`, `NVM_SLOT_VAC_VALVE_HOME_ON_BOOT`; removed old `PINCH_HOMING_TUBED_*`/`PINCH_HOMING_UNTUBED_*` constants
- Replaced vacuum transducer with XP5-080-01 (compound, -101–100 kPa, 1-5V); updated `VAC_PRESSURE_MIN` → -14.65, `VAC_PRESSURE_MAX` → 14.50 PSIG

### Commands
- Added `injection_valve_home_on_boot` / `vacuum_valve_home_on_boot` (NVM, true/false); removed `*_home_tubed` / `*_home_untubed` variants
- Updated `commands.json` schema for GUI/simulator alignment

### Bug Fixes
- Fixed valve auto-homing not firing when press `home_on_boot` was disabled (delay timer now starts when any homing is pending)
- Fixed simulator valve state after homing (`'Open'` instead of `'Homed'`)

### GUI
- Created `definition/gui.py` standalone panel module with `create_gui_components()` entry point
- Sections: press (force w/ source, position current→target, retract, homed/polarity/motor, energy, endpoint/startpoint), injector (state, home positions, volumes), valves (compact row: state/position/torque bar), vacuum & heater (state w/ ON/OFF color, pressure, temp w/ setpoint)
- Force vs Position graph: live-updating canvas with auto-scaling and clear button
- Helpers: `make_homed_tracer`, `make_state_tracer`, `make_on_off_tracer`, `make_force_tracer`, `make_heater_value_tracer`, `make_vacuum_value_tracer`, `make_unit_stripper`
- Widget factories: `create_torque_widget` (vertical progress bar), `create_device_frame` (bordered card w/ header, state, IP)
- `get_gui_variable_names()` declares all required tkinter variables for telemetry binding

### Simulator
- Updated valve homing simulation: removed tubed/untubed, added `home_on_boot` handling, added `inj_valve_home_sensor`/`vac_valve_home_sensor` to telemetry

---

## [0.1.0] - 2026-03-18

Full architecture alignment: Fillhead is now a strict superset of Pressboi. Every Pressboi feature, command, and subsystem has been ported alongside existing injection, valve, heater, and vacuum functionality.

### State Machine
- Added `STATE_RESETTING` (non-blocking motor re-enable) and `STATE_RECOVERED` (watchdog recovery, motors disabled until reset)
- Replaced blocking `clearErrors()` with phased non-blocking reset; added fault grace period (`m_faultGracePeriodEnd`)
- Watchdog recovery uses `RSTC->RCAUSE.reg` for reset-cause detection; added `clearWatchdogRecovery()`
- Moved watchdog feed into `performSafetyCheck()`; RECOVERED/ERROR states only allow reset, discover, dump_error_log

### Motor / Injector Controller
- Ported `moveAbsolute()`, `moveIncremental()`, `setRetract()`, `retract()` for general-purpose position moves with force limits
- Added full `MoveState` state machine alongside `FeedState`; added `pauseOperation()`, `resumeOperation()`, `cancelOperation()`
- Added `finalizeAndResetActiveMove()`, `fullyResetActiveMove()`, non-blocking `EnableState` state machine, `handleCommand()` dispatcher

### Force Sensing
- Ported motor torque force mode (default `"motor_torque"`); added `setForceMode()`, `getForceMode()`, `setForceCalibrationOffset()`, `setForceCalibrationScale()`, `checkForceSensorStatus()`
- Per-move force limit with configurable action (retract/hold/skip/abort); `handleLimitReached()` enforcement
- Joule integration at 50Hz (`updateJoules()`); 5-coefficient machine strain polynomial (`evaluateMachineStrainForceFromDeflection()`, `estimateMachineDeflectionFromForce()`, `setMachineStrainCoeffs()` → NVM)

### Hall Sensor Homing
- Replaced torque-based with dual hall sensor homing (PN: 326161-0053); per-axis gantry squaring (independent sensor trigger)
- Phases: rapid approach → backoff → slow approach → final backoff → set zero; 100s timeout; preserved machine/cartridge dual-home
- Per-axis motor control: `stopAxis()`, `isAxisMoving()`, `isHomeSensorTriggered()`

### NVM Support
- Full NVM layout with magic number (`0x464C4831` = "FLH1"); stores polarity, force mode, torque offset/scale, home-on-boot, retract, press threshold, strain coefficients
- Added `CMD_DUMP_NVM` (hex/ASCII + field interpretation) and `CMD_RESET_NVM`; all setters write immediately

### Polarity & Press Threshold
- Polarity system (`"normal"`/`"inverted"`) saved to NVM with direction inversion on moves
- Press threshold saved to NVM; startpoint recorded when threshold crossed

### Home on Boot
- `m_home_on_boot` from NVM (default true); delayed auto-home 2s after boot; `setHomeOnBoot()`/`getHomeOnBoot()`

### Commands
- Created standalone `commands.h`/`commands.cpp`; switched from fixed-width padded to minimal format (single trailing space)
- Added all Pressboi commands: `home`, `move_abs`, `move_inc`, `set_retract`, `retract`, `pause`, `resume`, `cancel`
- System: `reset`, `enable`, `disable`, `test_watchdog`, `reboot_bootloader`; NVM: `dump_nvm`, `reset_nvm`, `dump_error_log`
- Force: `set_force_mode`, `set_force_offset`, `set_force_zero`, `set_force_scale`, `set_strain_cal`; config: `set_polarity`, `home_on_boot`, `set_press_threshold`
- Renamed `clear_errors` → `reset`; removed `test_command`

### Events System
- Created `inc/events.h` / `src/events.cpp`; prefixes: `FILLHEAD_INFO`, `FILLHEAD_START`, `FILLHEAD_DONE`, `FILLHEAD_ERROR`, `FILLHEAD_RECOVERY`, `FILLHEAD_TELEM`
- Added `Event` enum, `sendEvent()`, `sendEventString()`, `sendEventMulti()`; moved prefixes from `responses.h` to `events.h`

### Communications
- Removed `parseCommand()`/`getCommandParams()` from `CommsController`; discovery now includes `PORT`/`FW` fields
- USB vs network detection (skip `setGuiIp()` for localhost); 500ms telemetry stabilization delay after discovery

### Telemetry
- Added all Pressboi fields: force (load cell, motor torque), energy, position (current/retract/target/endpoint/startpoint), press threshold, torque average
- Added `home_sensor_m0`/`m1`, `force_mode`, `polarity`, `force_limit`, `force_source`, `force_adc_raw`; `gui_var` bindings on all fields; `main_state` now string type

### Configuration
- `FIRMWARE_VERSION "0.1.0"`; hall sensor pins (`HOME_SENSOR_M0` DI7, `HOME_SENSOR_M1` DI6); force sensor constants (scale/offset/min/max/timeout)
- Machine strain coefficients (5-term polynomial); move defaults (torque%/velocity/acceleration); `RETRACT_DEFAULT_SPEED_MMS`; removed `POST_ABORT_DELAY_MS`

### Definition Folder
- Merged all Pressboi commands into `commands.json` with descriptions, help, enums, defaults; updated `telemetry.json` with all fields + `gui_var` bindings
- Updated `events.json`; added `energy_warning`/`endpoint_warning` to `warnings.json`; added `device_name` to `firmware_config.json`
- Created `views.json`; regenerated all parser/header files from updated definitions

### Simulator
- Added Pressboi command simulation (home, move_abs, move_inc, retract, set_retract, pause, resume, cancel)
- Added force/torque/joule/position/home-sensor simulation; fixed event prefixes and command string matching

### GUI
- Added force, position, retract, energy, endpoint/startpoint displays and home sensor indicators (M0/M1) with polarity indicator
- Created `operator_view.py` (PASS/FAIL, job/serial, cycle time); ported `draw_vertical_text`, `make_force_tracer`, `make_unit_stripper` from Pressboi

### Build / CI
- Created `.github/workflows/release.yml` (binary: `fillhead.bin`/`fillhead.uf2`); updated `.gitignore` for debug artifacts and Python cache

---

## [1.0.0] - 2025-11-17

Initial release of Fillhead injection system firmware.

- Dual injector motors: synchronized control for material injection with configurable pitch and steps/mm
- Pinch valve control: two independent motorized valves (vacuum/injection) with tubed/untubed homing
- Heater control: PID temperature control with thermocouple feedback and relay output
- Vacuum system: pressure monitoring/control with transducer feedback, configurable targets and ramp timeouts
- UDP/Ethernet + USB Serial communication with device discovery and text-based command protocol
- Real-time telemetry: position, temperature, pressure, torque, and status reporting
- Injection control: volume-based with configurable speeds, piston diameter support, stator/rotor modes
- Leak testing: automated vacuum leak detection with configurable thresholds
- Error handling: comprehensive reporting and recovery
- Built on Teknic ClearCore (ClearPath drivers M0-M3, analog/digital I/O, Ethernet + USB)
