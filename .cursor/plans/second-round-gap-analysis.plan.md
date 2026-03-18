---
name: Second-Round Gap Analysis
overview: Second-round gap analysis found 1 CRITICAL gap (ForceSensor class never ported -- Fillhead has same load cell as Pressboi), 1 MAJOR design gap (inject command needs rework with force limiting and cartridge ratio), 1 bug, and several cascading broken code paths.
todos:
  - id: port-force-sensor-class
    content: "CRITICAL: Copy force_sensor.h and force_sensor.cpp from Pressboi to Fillhead. Add ForceSensor m_forceSensor as public member on Fillhead class. Call m_forceSensor.setup() during init at WD_BREADCRUMB_SETUP_FORCE. Call m_forceSensor.update() every main loop iteration at WD_BREADCRUMB_FORCE_UPDATE."
    status: done
  - id: wire-force-sensor-to-injector
    content: "Give Injector access to ForceSensor. Pressboi uses m_controller->m_forceSensor (public member on Pressboi, accessed via back-pointer). Replicate same pattern: Fillhead owns the ForceSensor, Injector accesses it via m_controller->m_forceSensor."
    status: done
  - id: fix-check-force-sensor-status
    content: Replace checkForceSensorStatus() stub in injector_controller.cpp. Currently returns 'Load cell not available on Fillhead' for all load_cell calls. Must call m_controller->m_forceSensor.isConnected(), validate getForce() against FORCE_SENSOR_MIN_KG (-10) and FORCE_SENSOR_MAX_LIMIT_KG (1440), matching Pressboi motor_controller.cpp line 1922.
    status: done
  - id: fix-force-limit-move-active
    content: Uncomment force limit check in updateState() STATE_MOVING/MOVE_ACTIVE (injector_controller.cpp ~line 687). Currently has TODO placeholder. Wire to m_controller->m_forceSensor.getForce(), check against m_active_op_force_limit_kg, call handleLimitReached() on breach. Match Pressboi motor_controller.cpp ~line 708.
    status: done
  - id: fix-move-abs-pre-check
    content: In moveAbsolute() and moveIncremental(), the load_cell pre-move force check currently calls the stubbed checkForceSensorStatus() which rejects all load_cell moves. Once stub is fixed (fix-check-force-sensor-status), also add pre-move force-vs-target check using m_controller->m_forceSensor.getForce() for hold action, matching Pressboi ~line 1213.
    status: done
  - id: fix-update-joules
    content: In updateJoules() (injector_controller.cpp ~line 2010), replace 'float raw_force_sample = 0.0f' with m_controller->m_forceSensor.getForce() when in load_cell mode. For motor_torque mode, use torque-derived force estimate. Joules are currently always zero.
    status: done
  - id: fix-cmd-set-force-zero
    content: "In fillhead.cpp dispatchCommand() CMD_SET_FORCE_ZERO (~line 703): remove error 'No load cell on Fillhead'. Implement Pressboi logic: capture current force from m_forceSensor.getForce(), calculate new offset = old_offset - current_force, call m_forceSensor.setOffset(new_offset), save to NVM."
    status: done
  - id: fix-cmd-set-force-offset
    content: "In fillhead.cpp CMD_SET_FORCE_OFFSET (~line 667): remove 'No load cell on Fillhead' error for load_cell mode. Wire to m_forceSensor.setOffset(offset) matching Pressboi ~line 587."
    status: done
  - id: fix-cmd-set-force-scale
    content: "In fillhead.cpp CMD_SET_FORCE_SCALE (~line 685): remove 'No load cell on Fillhead' error for load_cell mode. Wire to m_forceSensor.setScale(scale) matching Pressboi ~line 611."
    status: done
  - id: fix-telemetry-force-fields
    content: In getTelemetryString(), wire force_load_cell to m_controller->m_forceSensor.getForce() and force_adc_raw to m_controller->m_forceSensor.getRawValue(). Currently both report 0.
    status: done
  - id: fix-nvm-slots-0-1
    content: NVM slots 0-1 will be used automatically by ForceSensor::loadCalibrationFromNVM() once force_sensor.cpp is ported. No extra work needed beyond the port -- just verify NVM slot 0 = load cell offset, slot 1 = load cell scale, matching Pressboi.
    status: done
  - id: change-default-force-mode
    content: Change default force mode from motor_torque to load_cell. In injector_controller.cpp constructor (~line 205), change the NVM-absent default from 'motor_torque' to 'load_cell'. Fillhead has the same load cell hardware as Pressboi.
    status: done
  - id: rework-inject-command
    content: "Merge inject_stator and inject_rotor into single 'inject' command. Remove CMD_INJECT_STATOR and CMD_INJECT_ROTOR from commands.h. Add CMD_INJECT. Rewrite initiateInjectMove() to use stored m_cartridge_ml_per_mm instead of piston diameter params. New signature: inject <volume_ml> [speed_ml_s] [force_limit_kg] [force_action]."
    status: done
  - id: add-set-cartridge-ratio
    content: Add set_cartridge_ratio <ml_per_mm> command. Save to NVM slot 18 (byte offset 0x48). Load in Injector::setup() with default ~4.86 ml/mm (from current stator piston constants). Add m_cartridge_ml_per_mm member to Injector. Add CMD_SET_CARTRIDGE_RATIO to commands.h. Add case to dispatchCommand().
    status: done
  - id: add-force-limit-to-injection
    content: "Add full force limit system to STATE_FEEDING in updateState(). Currently only has checkTorqueLimit() (raw torque abort). Add: force_limit_kg check using m_controller->m_forceSensor.getForce(), configurable force_action (retract/hold/skip/abort), handleLimitReached() call, joule integration. This detects cartridge bottom. Match the STATE_MOVING force limit pattern."
    status: done
  - id: fix-press-report-pressboi-ref
    content: "Fix press_report.html line 1200: change 'pressboi.set_strain_cal' to 'fillhead.set_strain_cal'. Only actionable pressboi reference in the codebase."
    status: done
  - id: fix-reset-nvm-range
    content: Extend CMD_RESET_NVM in fillhead.cpp to clear slots 0-18 (currently only 0-15). Slot 16 = injection valve home-on-boot, slot 17 = vacuum valve home-on-boot, slot 18 = cartridge ratio.
    status: done
  - id: update-commands-json
    content: "Update definition/commands.json: remove inject_stator and inject_rotor entries. Add inject command with params (volume, speed, force_limit, force_action). Add set_cartridge_ratio command with ml_per_mm param."
    status: done
  - id: update-simulator
    content: "Update definition/simulator.py: remove inject_stator/inject_rotor handlers. Add inject handler using stored cartridge ratio. Add set_cartridge_ratio handler."
    status: done
  - id: update-gui
    content: "Update definition/gui.py: replace inject_stator/inject_rotor buttons with single inject button. Add set_cartridge_ratio UI element."
    status: done
  - id: remove-piston-constants
    content: Remove or deprecate STATOR_PISTON_A_DIAMETER_MM, STATOR_PISTON_B_DIAMETER_MM, ROTOR_PISTON_A_DIAMETER_MM, ROTOR_PISTON_B_DIAMETER_MM from config.h. Keep only DEFAULT_CARTRIDGE_ML_PER_MM as the compile-time default.
    status: done
isProject: false
---

# Second-Round Gap Analysis: Pressboi vs Fillhead

**Context:** The first-round alignment (see `fillhead_architecture_alignment.plan.md`) ported 20 major features from Pressboi into Fillhead. This second pass found gaps the first round missed.

---

## CRITICAL: Finding 0 -- ForceSensor class never ported

The entire `ForceSensor` class (`inc/force_sensor.h` + `src/force_sensor.cpp`) exists in Pressboi but does not exist in Fillhead. Fillhead has the same load cell hardware (HX711 via Rugeduino on COM-0), but the first-round port incorrectly assumed Fillhead had no load cell.

This cascades into 7 broken code paths:

- `checkForceSensorStatus()` -- stub returns "Load cell not available on Fillhead"
- `updateJoules()` -- hardcoded `raw_force_sample = 0.0f`, joules always zero
- `CMD_SET_FORCE_ZERO` -- returns error instead of zeroing
- `CMD_SET_FORCE_OFFSET` / `CMD_SET_FORCE_SCALE` in load_cell mode -- returns error
- `updateState()` MOVE_ACTIVE force limit -- commented out with TODO
- `moveAbsolute()` / `moveIncremental()` -- rejects all load_cell moves
- `getTelemetryString()` -- `force_load_cell` and `force_adc_raw` report 0
- NVM slots 0-1 unused (should store load cell offset/scale)

**ForceSensor access pattern:** Match Pressboi -- public `ForceSensor m_forceSensor` on `Fillhead`, Injector accesses via `m_controller->m_forceSensor`.

**Default force mode:** Change from `"motor_torque"` to `"load_cell"` (same hardware as Pressboi).

---

## MAJOR: Finding 1 -- Inject command rework + force limiting

### 1a. Merge inject_stator / inject_rotor into single `inject`

Two commands that differ only in hardcoded piston diameters:

```cpp
case CMD_INJECT_STATOR:
    initiateInjectMove(args, STATOR_PISTON_A_DIAMETER_MM, STATOR_PISTON_B_DIAMETER_MM, ...);
case CMD_INJECT_ROTOR:
    initiateInjectMove(args, ROTOR_PISTON_A_DIAMETER_MM, ROTOR_PISTON_B_DIAMETER_MM, ...);
```

Replace with single `inject` using a persistent `ml_per_mm` ratio:

```
inject <volume_ml> [speed_ml_s] [force_limit_kg] [force_action]
```

### 1b. Add `set_cartridge_ratio <ml_per_mm>`

- Saves to NVM slot 18 (byte offset 0x48)
- Used by `inject` to convert mL to mm: `distance_mm = volume_ml / ml_per_mm`
- Default ~4.86 ml/mm (from current stator piston constants)
- Replaces `STATOR_PISTON_*` and `ROTOR_PISTON_*` compile-time constants

### 1c. Add full force limit system to STATE_FEEDING

Currently `STATE_FEEDING` only has `checkTorqueLimit()` (raw torque abort). `STATE_MOVING` has the full Pressboi system (force_limit_kg, force_action, handleLimitReached, joule integration). Injection needs the same -- this is how bottom-of-cartridge is detected.

---

## Finding 2 -- BUG: Hardcoded `pressboi` in press_report.html

`definition/reports/templates/press_report.html` line 1200:

```javascript
const command = `pressboi.set_strain_cal ${c4} ${c3} ${c2} ${c1} ${c0}`;
```

Should be `fillhead.set_strain_cal`.

---

## Finding 3 -- `reset_nvm` does not clear extended slots

`CMD_RESET_NVM` only resets slots 0-15. Must also reset slots 16 (injection valve home-on-boot), 17 (vacuum valve home-on-boot), and 18 (cartridge ratio).

---

## Findings 4-12 -- Intentional / Clean (no action needed)


| #   | Finding                                               | Category    |
| --- | ----------------------------------------------------- | ----------- |
| 4   | NVM magic numbers differ (PBR1 vs FLH1)               | INTENTIONAL |
| 5   | Default torque limit differs (80 vs 20)               | INTENTIONAL |
| 6   | Config constant naming prefixed for multi-motor       | INTENTIONAL |
| 7   | Telemetry API (struct vs string)                      | INTENTIONAL |
| 8   | Events superset (3 vs 1)                              | INTENTIONAL |
| 9   | Simulator command naming (bare vs prefixed)           | INTENTIONAL |
| 10  | dispatchCommand fully covered (all 25 Pressboi cases) | CLEAN       |
| 11  | NVM slots 3-15 aligned                                | CLEAN       |
| 12  | Telemetry help text cosmetics                         | TRIVIAL     |


