---
name: Fillhead Commissioning Plan
overview: Systematic commissioning and testing plan for a first-time power-on of the Fillhead material injection system, covering safety verification, individual subsystem checkout, calibration, and integrated function testing.
todos:
  - id: phase1
    content: "Pre-power checks: verify all wiring, connectors, sensors, light curtain, Ethernet/USB"
    status: pending
  - id: phase2
    content: "Initial power-up: discover device, check telemetry, reset NVM if needed, enable motors"
    status: pending
  - id: phase3
    content: "Safety systems: test light curtain interlock, watchdog recovery, and abort"
    status: pending
  - id: phase4
    content: "Subsystem tests: injector homing/jogging, both pinch valves, heater PID, vacuum pump, force sensor"
    status: pending
  - id: phase5
    content: "Calibration: tare force sensor, set cartridge ratio, retract position, polarity, home-on-boot"
    status: pending
  - id: phase6
    content: "Integrated tests: vacuum leak test, dry injection cycle, full sequence with cartridge"
    status: pending
  - id: phase7
    content: "Final checks: dump NVM, error log, power cycle, document configuration"
    status: pending
isProject: false
---

# Fillhead First Power-On Commissioning Plan

All commands below are sent via the BR Equipment Control App (UDP) or USB serial terminal. Telemetry is visible in the app at 100 ms intervals.

---

## Phase 1 -- Pre-Power Checks (no power)

- Verify all motor connectors are seated (M0/M1 injectors, M2 injection valve, M3 vacuum valve).
- Verify sensor wiring: thermocouple (A12), vacuum transducer (A11), injection valve hall sensor (DI8), injector home sensors (DI7/M0, DI6/M1), vacuum valve hall sensor (A9).
- Verify relay wiring: heater (IO1), vacuum pump (IO0), vacuum solenoid (IO5).
- Verify light curtain wiring (IO4) -- confirm it is connected and unobstructed.
- Verify Ethernet cable or USB cable is connected to host PC.
- Verify load cell Rugeduino (Arduino Uno) is connected to COM-0 (TTL serial).
- Ensure no cartridge is loaded and nothing is in the travel path of the injectors or valves.

---

## Phase 2 -- Initial Power-Up and Communication

1. Apply power to the ClearCore.
2. Open the BR Equipment Control App and confirm `discover_device` succeeds -- the app should find the fillhead on port 8888.
3. Check the telemetry stream is active -- verify `fillhead_state` reports. On first boot, expect `STATE_STANDBY` (or `STATE_RECOVERED` if a watchdog reset occurred).
4. Run `dump_nvm` to inspect current NVM contents. If this is a fresh board, run `reset_nvm` to load factory defaults.
5. Send `enable`. On a normal first boot the system is already in `STATE_STANDBY` and will respond "System already enabled." If the system is in `STATE_DISABLED`, it will transition to `STATE_STANDBY` and enable all motors.

---

## Phase 3 -- Safety Systems

### 3a. Light Curtain

1. With the system in STANDBY, break the light curtain beam by hand.
2. Confirm the system transitions to `STATE_ERROR` and reports "Light curtain tripped."
3. Clear the obstruction. Send `reset`. Confirm return to `STATE_STANDBY`.
4. While the beam is broken, send `reset` again -- confirm it is **rejected** (cannot reset while tripped).

### 3b. Watchdog

1. Send `test_watchdog`. This enters an infinite loop, triggering the 256 ms watchdog timeout and causing a system reset.
2. Confirm the system reboots and enters `STATE_RECOVERED`.
3. Send `reset`. Confirm return to `STATE_STANDBY`.

### 3c. Abort

1. Start a slow jog: `jog_move 50` (50 mm forward at default 1 mm/s).
2. Send `abort`. Confirm all motion stops and the system returns to STANDBY.

---

## Phase 4 -- Individual Subsystem Testing

### 4a. Injector Motors (M0 + M1)

1. Send `machine_home` (or `home`). Watch telemetry for homing phases (RAPID_APPROACH, BACKOFF, SLOW_APPROACH, FINAL_BACKOFF, SET_ZERO). Confirm both DI7 and DI6 hall sensors trigger and position zeros out.
2. Send `jog_move 10` -- confirm injector moves 10 mm forward. Watch `fillhead_injector_position` in telemetry.
3. Send `jog_move -10` -- confirm it returns.
4. Send `move_abs 50 5 500 skip` -- move to 50 mm at 5 mm/s, 500 kg force limit (skip action). Confirm arrival.
5. Send `move_abs 0 5 500 skip` -- return to zero.

### 4b. Injection Valve (M2)

1. Send `injection_valve_home`. Confirm homing completes using DI8 hall sensor, state goes to `open`.
2. Send `injection_valve_close`. Confirm valve state transitions to `closed`.
3. Send `injection_valve_open`. Confirm valve state transitions to `open`.
4. Send `injection_valve_jog 2` then `injection_valve_jog -2` to verify manual control.

### 4c. Vacuum Valve (M3)

1. Send `vacuum_valve_home`. Confirm homing completes using A9 hall sensor, state goes to `open`.
2. Send `vacuum_valve_close`. Confirm state transitions to `closed`.
3. Send `vacuum_valve_open`. Confirm state transitions to `open`.
4. Send `vacuum_valve_jog 2` then `vacuum_valve_jog -2` to verify manual control.

### 4d. Heater

1. Check telemetry for thermocouple temperature -- should read near ambient (e.g., 20-25 C).
2. Send `heater_on 40` (low setpoint of 40 C for safety during commissioning).
3. Monitor temperature rise in telemetry. Confirm it climbs toward 40 C and stabilizes (PID active).
4. Send `heater_off`. Confirm heater relay turns off and temperature begins to drop.

### 4e. Vacuum System

1. Check telemetry for vacuum transducer pressure -- should read near 0 PSIG (atmospheric).
2. Send `vacuum_on -5` (gentle -5 PSIG target for initial test).
3. Confirm pump relay (IO0) and solenoid relay (IO5) activate and pressure drops in telemetry.
4. Send `vacuum_off`. Confirm relays de-energize and pressure returns toward atmospheric.

### 4f. Force Sensor / Load Cell

1. Check telemetry for `fillhead_load_cell_kg` -- should read near 0 with nothing loaded.
2. If force mode is `load_cell`: send `set_force_zero` to tare the sensor.
3. Apply a known weight or press by hand and confirm the reading changes proportionally.
4. If force mode is `motor_torque`: verify `fillhead_motor_torque_pct` reads near zero at rest.

---

## Phase 5 -- Calibration

1. **Force sensor tare**: With no load, send `set_force_zero`.
2. **Vacuum offset**: If the transducer reads non-zero at atmosphere, note the offset. The firmware has `VACUUM_PSIG_OFFSET` for compensation.
3. **Cartridge ratio**: Confirm `set_cartridge_ml_per_mm` is set correctly for your cartridge (default 5.2732 ml/mm). Adjust if using a different cartridge size, e.g. `set_cartridge_ml_per_mm 5.2732`.
4. **Retract position**: Send `set_retract 10 25` to set a 10 mm retract position at 25 mm/s (adjust as needed for your setup).
5. **Polarity**: Send `set_polarity normal` (or `inverted` if the machine is mounted upside-down).
6. **Home-on-boot**: Once homing is verified, optionally enable with `home_on_boot true`, `injection_valve_home_on_boot true`, `vacuum_valve_home_on_boot true`.

---

## Phase 6 -- Integrated Function Test

### 6a. Vacuum Leak Test (no cartridge, cap the port)

1. Cap the vacuum port so the system is sealed.
2. Send `vacuum_leak_test`. Default: pull down to -14 PSIG, settle 2 s, monitor for 0.1 PSIG rise over 10 s.
3. Confirm `passed` response. If `failed`, check fittings and seals.

### 6b. Dry Injection Cycle (no material)

1. Home all axes: `machine_home`, `injection_valve_home`, `vacuum_valve_home`.
2. Open injection valve: `injection_valve_open`.
3. Send `inject 1 0.5` (dispense 1 mL at 0.5 mL/s). Confirm injector moves the correct distance (1 mL / 5.2732 ml/mm ~ 0.19 mm).
4. Confirm `fillhead_injector_dispensed_ml` in telemetry reads approximately 1.0 mL.
5. Close injection valve: `injection_valve_close`.

### 6c. Full Sequence (with cartridge, if available)

1. Load a cartridge.
2. Send `cartridge_home` -- injector contacts the cartridge plunger using torque sensing.
3. Open injection valve, run `vacuum_on`, wait for pulldown, then `inject` a small volume.
4. Send `vacuum_off`, close injection valve.
5. Verify dispensed volume matches expected output.

---

## Phase 7 -- Final Checks

- Send `dump_nvm` and verify all saved parameters match your intended configuration.
- Send `dump_error_log` and confirm no unexpected errors in the log.
- Confirm telemetry is streaming steadily at 100 ms with no dropouts.
- Power cycle the board and confirm it comes back cleanly (auto-homes if configured, reaches STANDBY).
- Document any parameter values that were changed from defaults.

---

## Quick Reference: Expected Telemetry at Idle


| Parameter       | Expected Value               |
| --------------- | ---------------------------- |
| Main state      | STANDBY                      |
| Injector state  | standby                      |
| Injection valve | open (if homed) or not_homed |
| Vacuum valve    | open (if homed) or not_homed |
| Temperature     | Ambient (~20-25 C)           |
| Pressure        | ~0 PSIG                      |
| Load cell       | ~0 kg                        |
| Position        | 0.00 mm (if homed)           |


