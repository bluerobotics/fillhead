# Fix: Firmware Command Argument Handling

## Problem Summary

The `commands.json` spec declares that `heater_on`, `vacuum_on`, and `vacuum_leak_test` accept optional inline arguments, and the BR equipment control app displays these params to the user. However, the firmware ignores these arguments entirely. Users expect `fillhead.vacuum_on -1` or `fillhead.heater_on 60` to work, but they don't.

## Root Cause

Three layers of disconnect:

1. **`getCommandParams()` in `src/commands.cpp`** — These three commands are NOT listed in the switch statement, so `args` is always `NULL` even when the user provides arguments.
2. **`handleCommand()` dispatch** — The handler functions are called without passing `args` (e.g., `vacuumOn()` instead of `vacuumOn(args)`).
3. **Handler functions** — `vacuumOn()`, `heaterOn()`, and `leakTest()` don't accept or parse arguments at all.

Additionally, `vacuum_on` sets state to `VACUUM_ON` which is a terminal state in `updateState()` — no pressure monitoring occurs, so even with a target, the pump would run forever.

## Commands to Fix

### 1. `vacuum_on [target_psi]` (optional arg)

**Spec from commands.json:**
```json
"vacuum_on": {
    "params": [
        { "parameter": "target", "unit": "psi", "type": "float", "optional": true }
    ]
}
```

**Expected behavior:**
- `fillhead.vacuum_on` (no arg): Turn pump + solenoid valve on indefinitely (current behavior, keep as-is). State = `VACUUM_ON`.
- `fillhead.vacuum_on -1` (with target): Turn pump + solenoid valve on. Monitor pressure. When target is reached, turn pump off but leave solenoid valve open (or close it — check with user intent). Report done. Must have a timeout using `m_rampTimeoutSec`.

**Files to modify:**
- `src/commands.cpp` — Add `CMD_VACUUM_ON` to `getCommandParams()` switch.
- `src/vacuum_controller.cpp`:
  - Change `handleCommand`: pass `args` to `vacuumOn(args)`.
  - Modify `vacuumOn(const char* args)` to parse optional target. If target provided, set `m_targetPsig` and enter a new state (e.g., `VACUUM_RAMP_TO_TARGET`) or reuse `VACUUM_PULLDOWN` with a flag.
  - In `updateState()`, handle the new state: monitor pressure, turn off pump when target reached, report done.
- `inc/vacuum_controller.h` — Update `vacuumOn()` signature. Possibly add a new enum state if needed.

**Design choice — new state vs. reusing PULLDOWN:**
Recommend adding a new state `VACUUM_RAMP` to differentiate from leak test pulldown. When target is reached in `VACUUM_RAMP`:
- Turn pump off (`PIN_VACUUM_RELAY.State(false)`)
- Turn solenoid valve off (`PIN_VACUUM_VALVE_RELAY.State(false)`)
- Set state to `VACUUM_OFF`
- Report `STATUS_PREFIX_DONE` with message like `"vacuum_on complete. Target reached."`

If timeout expires, report `STATUS_PREFIX_ERROR` and reset.

### 2. `heater_on [setpoint_C]` (optional arg)

**Spec from commands.json:**
```json
"heater_on": {
    "params": [
        { "parameter": "setpoint", "unit": "C", "type": "float", "optional": true }
    ]
}
```

**Expected behavior:**
- `fillhead.heater_on` (no arg): Activate PID with current/default setpoint (current behavior).
- `fillhead.heater_on 60` (with setpoint): Set `m_pid_setpoint = 60.0` first, THEN activate PID. Convenience shorthand for `set_heater_setpoint 60` + `heater_on`.

**Files to modify:**
- `src/commands.cpp` — Add `CMD_HEATER_ON` to `getCommandParams()` switch.
- `src/heater_controller.cpp`:
  - Change `handleCommand`: pass `args` to `heaterOn(args)`.
  - Modify `heaterOn(const char* args)` to parse optional setpoint. If provided, validate (same 20-200C range as `setSetpoint`) and set `m_pid_setpoint` before activating PID. If invalid, report error and don't activate.
- `inc/heater_controller.h` — Update `heaterOn()` signature.

### 3. `vacuum_leak_test [delta_psi] [duration_s]` (optional args)

**Spec from commands.json:**
```json
"vacuum_leak_test": {
    "params": [
        { "parameter": "delta", "unit": "psi", "type": "float", "optional": true },
        { "parameter": "duration", "unit": "s", "type": "float", "optional": true }
    ]
}
```

**Expected behavior:**
- `fillhead.vacuum_leak_test` (no args): Use pre-set `m_leakTestDeltaPsig` and `m_leakTestDurationSec` (current behavior).
- `fillhead.vacuum_leak_test 0.5 10` (with args): Override delta and duration for this test only. Don't persist to NVM.
- `fillhead.vacuum_leak_test 0.5` (one arg): Override delta only, use pre-set duration.

**Files to modify:**
- `src/commands.cpp` — Add `CMD_VACUUM_LEAK_TEST` to `getCommandParams()` switch.
- `src/vacuum_controller.cpp`:
  - Change `handleCommand`: pass `args` to `leakTest(args)`.
  - Modify `leakTest(const char* args)` to parse optional delta and duration. Use `sscanf(args, "%f %f", &delta, &duration)` pattern. Only override values that were successfully parsed. Validate ranges (same as `setLeakDelta` / `setLeakDuration`).
- `inc/vacuum_controller.h` — Update `leakTest()` signature.

## Implementation Notes

### Parsing Pattern for Optional Args

Since all three commands have **optional** params, the pattern should be:
```cpp
void vacuumOn(const char* args) {
    if (args != NULL && args[0] != '\0') {
        float target = atof(args);
        // validate and use target
    } else {
        // no-arg behavior (existing logic)
    }
}
```

For `vacuum_leak_test` with two optional params:
```cpp
void leakTest(const char* args) {
    float delta = m_leakTestDeltaPsig;   // defaults
    float duration = m_leakTestDurationSec;
    if (args != NULL && args[0] != '\0') {
        int parsed = sscanf(args, "%f %f", &delta, &duration);
        // parsed == 1: only delta overridden
        // parsed == 2: both overridden
        // validate ranges...
    }
    // proceed with leak test using delta and duration
}
```

### commands.cpp — getCommandParams additions

Add these three cases to the switch in `getCommandParams()`:
```cpp
case CMD_VACUUM_ON:           return cmdStr + strlen(CMD_STR_VACUUM_ON);
case CMD_HEATER_ON:           return cmdStr + strlen(CMD_STR_HEATER_ON);
case CMD_VACUUM_LEAK_TEST:    return cmdStr + strlen(CMD_STR_VACUUM_LEAK_TEST);
```

**IMPORTANT:** Since these params are optional, also check that `CMD_STR_VACUUM_ON`, `CMD_STR_HEATER_ON`, and `CMD_STR_VACUUM_LEAK_TEST` have trailing spaces in their `#define` strings. The code generator adds trailing spaces only for commands with REQUIRED params (`has_required_params` check on line 84 of generator.py). Since all params here are optional, the CMD_STR defines likely do NOT have trailing spaces. You'll need to add trailing spaces to these defines in `inc/commands.h`:
```cpp
#define CMD_STR_VACUUM_ON       "vacuum_on "
#define CMD_STR_HEATER_ON       "heater_on "
#define CMD_STR_VACUUM_LEAK_TEST "vacuum_leak_test "
```
Without the trailing space, `getCommandParams` would return a pointer to `'\0'` (empty string) when no args are given, which is fine for the `args[0] != '\0'` check pattern. But if there IS no trailing space, then `parseCommand` uses `strncmp` with `strlen(CMD_STR_...)` which would fail to match `"vacuum_on -1"` because `"vacuum_on"` (9 chars) wouldn't match the first 9 chars if the define is `"vacuum_on"` without a space... actually, wait — `strncmp("vacuum_on -1", "vacuum_on", 9)` DOES match because it only compares 9 chars. So parsing works fine WITHOUT trailing space.

The trailing space issue only affects `getCommandParams`: without a trailing space, `cmdStr + strlen("vacuum_on")` points to `" -1"` (with leading space). The handler would need to handle the leading space. `atof(" -1")` handles leading whitespace fine, and `sscanf` also skips whitespace. So actually, it works either way. BUT for consistency with the code generator's intent and other commands, add trailing spaces to these three defines.

### Vacuum State Machine Addition

In `inc/vacuum_controller.h`, add to the state enum:
```cpp
enum VacuumState {
    VACUUM_OFF,
    VACUUM_ON,
    VACUUM_RAMP,        // <-- NEW: pumping to a target, then auto-off
    VACUUM_PULLDOWN,
    VACUUM_SETTLING,
    VACUUM_LEAK_TESTING,
    VACUUM_ERROR
};
```

In `updateState()`, add handling for `VACUUM_RAMP`:
```cpp
if (m_state == VACUUM_RAMP) {
    if (m_vacuumPressurePsig <= m_targetPsig) {
        resetState();
        reportEvent(STATUS_PREFIX_DONE, "vacuum_on complete. Target pressure reached.");
    } else if (elapsed_sec > m_rampTimeoutSec) {
        m_state = VACUUM_ERROR;
        PIN_VACUUM_RELAY.State(false);
        PIN_VACUUM_VALVE_RELAY.State(false);
        reportEvent(STATUS_PREFIX_ERROR, "vacuum_on FAILED: Did not reach target pressure in time.");
    }
}
```

Update `isBusy()` to include `VACUUM_RAMP`.

Update `getState()` to return `"Ramping"` for `VACUUM_RAMP`.

Update `getTelemetryString()` — no change needed, `m_state` integer will naturally include the new state.

## Files to Read Before Starting

1. `inc/vacuum_controller.h` — state enum, method signatures
2. `src/vacuum_controller.cpp` — full implementation
3. `inc/heater_controller.h` — method signatures
4. `src/heater_controller.cpp` — full implementation
5. `src/commands.cpp` — `getCommandParams()` switch
6. `inc/commands.h` — CMD_STR defines to check trailing spaces

## Testing Checklist

- [ ] `fillhead.vacuum_on` — pump runs indefinitely (unchanged behavior)
- [ ] `fillhead.vacuum_on -1` — pump runs until -1 PSI reached, then auto-off with DONE message
- [ ] `fillhead.vacuum_on -1` with timeout — ERROR message when target not reached in time
- [ ] `fillhead.heater_on` — PID activates with existing setpoint (unchanged behavior)
- [ ] `fillhead.heater_on 60` — setpoint changes to 60C and PID activates
- [ ] `fillhead.heater_on 999` — error, PID does NOT activate
- [ ] `fillhead.vacuum_leak_test` — uses stored delta/duration (unchanged behavior)
- [ ] `fillhead.vacuum_leak_test 0.5 10` — overrides delta=0.5, duration=10 for this test
- [ ] `fillhead.vacuum_leak_test 0.5` — overrides delta=0.5, uses stored duration
