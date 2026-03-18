# Audit Plan: Optional Command Arguments Implementation

## What Was Implemented

Three firmware commands (`vacuum_on`, `heater_on`, `vacuum_leak_test`) were updated to accept optional inline arguments, per the `commands.json` spec. Previously, the firmware ignored all arguments for these commands.

The original implementation plan is at `.cursor/plans/fix_command_args_plan.md`.

## Files Modified

| File | Change Summary |
|------|---------------|
| `src/commands.cpp` | Added 3 cases to `getCommandParams()` switch |
| `inc/vacuum_controller.h` | Added `VACUUM_RAMP` enum state; changed `vacuumOn()` and `leakTest()` signatures to accept `const char* args` |
| `src/vacuum_controller.cpp` | Rewrote `vacuumOn()`, `leakTest()`, added `VACUUM_RAMP` handling in `updateState()`, updated `getState()` |
| `inc/heater_controller.h` | Changed `heaterOn()` signature to accept `const char* args` |
| `src/heater_controller.cpp` | Rewrote `heaterOn()` with optional setpoint parsing; updated `handleCommand()` dispatch |

**NOT modified (deliberately):** `inc/commands.h` — the CMD_STR defines were kept without trailing spaces. See "Design Decision" below.

## Audit Checklist

### 1. Command Parsing Layer (`src/commands.cpp`, `inc/commands.h`)

- [x] **Verify `getCommandParams()` returns correct pointers for these 3 commands.** Lines 115-117 add all three cases. CMD_STR defines have NO trailing space. For input `"vacuum_on -1"`, `cmdStr + 9` yields `" -1"` (leading space). `atof(" -1")` returns -1.0 and `sscanf(" -1", "%f", ...)` returns 1 — both correctly skip leading whitespace per C standard. **PASS.**
- [x] **Verify `getCommandParams()` returns correct pointers for the NO-ARG case.** For input `"vacuum_on"`, `cmdStr + 9` points to `'\0'`. All handlers check `args != NULL && args[0] != '\0'` which evaluates false → takes no-arg path. **PASS.**
- [x] **Verify `parseCommand()` still works for both cases.** `strncmp("vacuum_on", "vacuum_on", 9)` matches. `strncmp("vacuum_on -1", "vacuum_on", 9)` matches (first 9 chars identical). Line 53: `CMD_STR_VACUUM_LEAK_TEST` checked before `CMD_STR_VACUUM_ON` (line 54), which is before `CMD_STR_VACUUM_OFF` (line 55). `strncmp("vacuum_off", "vacuum_on", 9)` compares `'f'` vs `'n'` at index 7 — no collision. **PASS.**
- [x] **Check `heater_on` vs `heater_off` collision.** `strncmp("heater_off", "heater_on", 9)` — index 7 is `'f'` vs `'n'`, no collision. `CMD_STR_HEATER_ON` (line 45) is checked before `CMD_STR_HEATER_OFF` (line 46). **PASS.**

### 2. `vacuumOn(const char* args)` — `src/vacuum_controller.cpp:70-91`

- [x] **No-arg path preserved.** When `args[0] == '\0'`, the else branch at line 85 sets `VACUUM_ON`, turns on pump + valve, reports START. Identical to original behavior. **PASS.**
- [x] **With-arg path: target validation.** Range `target <= 0.0f && target > -15.0f` is consistent with `setTarget()` at line 202. Accepts 0.0 (atmospheric), which would cause immediate RAMP success. **PASS — consistent with existing validation. Edge case noted below.**
- [x] **With-arg path: `atof("0")` returns 0.0.** Passes `<= 0` check, enters VACUUM_RAMP. Ramp immediately succeeds since ambient pressure is ~0. **LOW RISK — harmless but worth noting.**
- [x] **With-arg path: `atof("abc")` returns 0.0.** Same as 0.0 — enters VACUUM_RAMP, immediately succeeds. **LOW RISK — commands come from controlled app, not arbitrary user text.**
- [x] **State set to `VACUUM_RAMP` with timestamp.** Line 79: `m_stateStartTimeMs = Milliseconds()` called before `updateState()` runs. **PASS.**
- [x] **`m_targetPsig` is overwritten.** ⚠️ Line 74 sets `m_targetPsig = target`. This is shared with `VACUUM_PULLDOWN` (leak test). A `vacuum_on -5` followed by `vacuum_leak_test` (no args) would pull down to -5 instead of the previous configured target. **MEDIUM RISK — acceptable as-is since `set_vacuum_target` has the same effect, but users should be aware. Consider documenting this behavior.**

### 3. `VACUUM_RAMP` State Machine — `src/vacuum_controller.cpp:144-153`

- [x] **Pressure comparison direction.** `m_vacuumPressurePsig <= m_targetPsig` — for target=-5.0, current=-3.0: `-3.0 <= -5.0` is FALSE (still ramping). For current=-5.1: `-5.1 <= -5.0` is TRUE (target reached). Correct for pulling vacuum. **PASS.**
- [x] **Timeout path.** VACUUM_RAMP timeout (lines 149-152) correctly turns off BOTH pump and valve. **PASS.** However, the pre-existing VACUUM_PULLDOWN timeout (lines 162-165) only turns off pump — valve is left open on error. ⚠️ **PRE-EXISTING BUG in VACUUM_PULLDOWN timeout — valve not closed. See Section 10.**
- [x] **Success path calls `resetState()`.** `resetState()` at line 194 sets `VACUUM_OFF`, turns off pump and valve. Then `reportEvent` sends DONE. State is OFF before telemetry is sent. **PASS.**
- [x] **`VACUUM_RAMP` is NOT in the early-return check.** Line 138 returns early for `VACUUM_OFF`, `VACUUM_ON`, `VACUUM_ERROR` only. `VACUUM_RAMP` falls through to the ramp check. **PASS.**
- [x] **`isBusy()` covers `VACUUM_RAMP`.** Line 279: `m_state != VACUUM_OFF && m_state != VACUUM_ERROR`. VACUUM_RAMP returns true. **PASS.**
- [x] **`getState()` has a case for `VACUUM_RAMP`.** Line 285: returns `"Ramping"`. **PASS.**
- [x] **Telemetry integer value.** 🔴 **CONFIRMED BREAKING CHANGE.** `VACUUM_RAMP` is enum value 1, shifting all subsequent values. The app's `definition/telemetry.json` has a `map` for `vac_st` that uses integer keys: `{"0": "off", "1": "pulldown", "2": "settling", "3": "leak testing", "4": "on", "5": "error"}`. With the new enum, `"1"` is now VACUUM_RAMP (not pulldown), `"5"` is now VACUUM_ON (not error), and `"6"` (VACUUM_ERROR) has no mapping at all. **ACTION REQUIRED — see Section 8.**

### 4. `leakTest(const char* args)` — `src/vacuum_controller.cpp:102-134`

- [x] **No-arg path preserved.** When `args[0] == '\0'`, `delta` and `duration` keep their current member variable values. Lines 125-126 write the same values back. No regression. **PASS.**
- [x] **One-arg path.** `sscanf(args, "%f %f", ...)` with `" 0.5"` returns 1. Only delta overridden. Duration keeps current value. **PASS.**
- [x] **Two-arg path.** `sscanf(args, "%f %f", ...)` with `" 0.5 10"` returns 2. Both overridden. **PASS.**
- [x] **Validation ranges match existing setters.** Delta: `> 0.0f && < 5.0f` matches `setLeakDelta` (line 226). Duration: `>= 1.0f && <= 120.0f` matches `setLeakDuration` (line 238). **PASS.**
- [x] **Overrides persist in member variables.** ⚠️ Lines 125-126 write overrides to `m_leakTestDeltaPsig` and `m_leakTestDurationSec`. A subsequent `vacuum_leak_test` (no args) reuses the overridden values. Effectively acts as `set_leak_test_delta` + `set_leak_test_duration_s` + `vacuum_leak_test`. No NVM persistence. **LOW RISK — acceptable behavior, but inline args are not truly temporary. Consider documenting.**
- [x] **`sscanf` with garbage input.** `sscanf(" abc", "%f %f", ...)` returns 0. Neither condition met, defaults used, test proceeds normally. **PASS.**
- [x] **Early return on validation failure.** Invalid delta → error + return (line 113). Invalid duration → error + return (line 120). Test does NOT start. **PASS.**

### 5. `heaterOn(const char* args)` — `src/heater_controller.cpp:61-87`

- [x] **No-arg path preserved.** When `args[0] == '\0'`, skips the setpoint block (line 62 condition is false). Falls through to PID activation check at line 74. If inactive → activates with current setpoint. If active → reports "ignored" at line 85. **PASS.**
- [x] **With-arg path: setpoint validation.** Range `> 20.0f && < 200.0f` matches `setSetpoint` (line 113). If invalid, reports error and returns WITHOUT activating PID (line 70). **PASS.**
- [x] **With-arg, PID already active.** Sets `m_pid_setpoint` at line 65, then reaches `else if (args != NULL && args[0] != '\0')` at line 80, reports "Setpoint updated to X. PID remains active." PID reads `m_pid_setpoint` each update cycle, so change is immediate. Good UX improvement. **PASS.**
- [x] **`atof("0")` returns 0.0.** Fails `> 20.0f` check, reports error, does NOT activate. **PASS.**
- [x] **`atof("abc")` returns 0.0.** Same — fails validation, reports error. **PASS.**
- [x] **`handleCommand()` passes `args`.** Line 44: `heaterOn(args)`. The `args` parameter comes from `getCommandParams()` at line 496 of `fillhead.cpp` → passed through `m_heater.handleCommand(command_enum, args)` at line 640. **PASS.**

### 6. `handleCommand()` Busy Guard — `src/vacuum_controller.cpp:48-54`

- [x] **The busy guard checks `m_state != VACUUM_OFF && m_state != VACUUM_ON`.** With VACUUM_RAMP, `VACUUM_RAMP != VACUUM_OFF` and `VACUUM_RAMP != VACUUM_ON` → guard blocks. Correct — can't start a new operation while ramping. **PASS.**
- [x] **`vacuum_off` while in `VACUUM_RAMP`.** Guard only blocks `CMD_VACUUM_ON` and `CMD_VACUUM_LEAK_TEST`. `CMD_VACUUM_OFF` passes through → `vacuumOff()` → `resetState()`. Correctly allows canceling a ramp. **PASS.**

### 7. Design Decision: No Trailing Spaces in CMD_STR Defines

The original plan recommended adding trailing spaces to `CMD_STR_VACUUM_ON`, `CMD_STR_HEATER_ON`, and `CMD_STR_VACUUM_LEAK_TEST`. The implementation did NOT do this. Reason: adding trailing spaces would break `parseCommand()` for the no-arg case. For example, `strncmp("vacuum_on", "vacuum_on ", 10)` fails because the 10th char of the input is `'\0'` but the 10th char of the define is `' '`.

- [x] **Verify this reasoning is correct.** `strncmp("vacuum_on", "vacuum_on", 9)` → match. **PASS.**
- [x] **Verify with args.** `strncmp("vacuum_on -1", "vacuum_on", 9)` → match (first 9 chars identical). **PASS.**
- [x] **Verify `getCommandParams` return.** `cmdStr + 9` points to `" -1"`. `atof(" -1")` returns -1.0. **PASS.**
- [x] **Verify no-arg `getCommandParams` return.** `cmdStr + 9` points to `'\0'`. `args[0] != '\0'` is false → no-arg path. **PASS.** Design decision is sound.

### 8. Enum Ordering Impact (CRITICAL)

- [x] **`VACUUM_RAMP` was inserted at position 1 in the enum.** 🔴 **CONFIRMED BREAKING CHANGE.** Investigated the BR equipment control app: `definition/telemetry.json` in this repo defines `vacuum_state` with `firmware_key: "vac_st"` and an integer-keyed `map`: `{"0": "off", "1": "pulldown", "2": "settling", "3": "leak testing", "4": "on", "5": "error"}`. The app's dynamic telemetry parser (`parse_dynamic_telemetry` in `network.py`) uses this map to convert the integer to a display string.

  **Current app mapping vs. new firmware values:**
  | Firmware sends | App displays | Actual state |
  |---|---|---|
  | 0 | "off" | VACUUM_OFF ✅ |
  | 1 | "pulldown" | VACUUM_RAMP ❌ |
  | 2 | "settling" | VACUUM_PULLDOWN ❌ |
  | 3 | "leak testing" | VACUUM_SETTLING ❌ |
  | 4 | "on" | VACUUM_LEAK_TESTING ❌ |
  | 5 | "error" | VACUUM_ON ❌ |
  | 6 | *(no mapping)* | VACUUM_ERROR ❌ |

  **Fix options (pick one):**
  **(A) Move `VACUUM_RAMP` to position 5 in the enum** (before `VACUUM_ERROR`), preserving all existing integer values. Then update `telemetry.json` to add `"5": "ramping"` and shift `"5": "error"` to `"6": "error"`.
  **(B) Keep current enum order and update `telemetry.json`** to match the new mapping: `{"0": "off", "1": "ramping", "2": "pulldown", "3": "settling", "4": "leak testing", "5": "on", "6": "error"}`.
  
  **Recommendation: Option (A)** — moving the new state to the end minimizes risk. Only the new state and ERROR change. Existing scripts or data logs referencing states 0-4 remain correct.

### 9. Cross-Cutting Concerns

- [x] **`STATUS_MESSAGE_BUFFER_SIZE` is large enough.** Line 46 in `config.h`: `#define STATUS_MESSAGE_BUFFER_SIZE 256`. Longest new message is ~80 chars. **PASS.**
- [x] **No memory leaks or dynamic allocation.** All buffers are stack-allocated (`char msg[STATUS_MESSAGE_BUFFER_SIZE]`). No `new`/`malloc`. **PASS.**
- [x] **Thread safety.** ClearCore firmware runs a single main loop (`while(true) { fillhead.loop(); }`). The only ISR is the watchdog handler, which only disables motors and sets flags — it does not modify vacuum or heater controller state. No race conditions. **PASS.**
- [x] **`VACUUM_ON` state is still terminal.** `VACUUM_ON` in early-return check at line 138. Pump runs until `vacuum_off`. No change from original. **PASS.**

### 10. Pre-Existing Bug: VACUUM_PULLDOWN Timeout Leaves Valve Open

⚠️ Discovered during audit — not introduced by the optional args changes, but worth fixing.

In `updateState()` at lines 162-165, the `VACUUM_PULLDOWN` timeout path only turns off the pump relay:
```cpp
} else if (elapsed_sec > m_rampTimeoutSec) {
    m_state = VACUUM_ERROR;
    PIN_VACUUM_RELAY.State(false);       // Pump off
    // PIN_VACUUM_VALVE_RELAY NOT set!   // Valve left OPEN
    reportEvent(STATUS_PREFIX_ERROR, "LEAK_TEST FAILED: ...");
}
```

Compare with `VACUUM_RAMP` timeout (lines 149-152) which correctly closes both:
```cpp
m_state = VACUUM_ERROR;
PIN_VACUUM_RELAY.State(false);
PIN_VACUUM_VALVE_RELAY.State(false);  // Valve also closed
```

And the `VACUUM_PULLDOWN` *success* path (lines 157-158) which also correctly closes both.

**Impact:** If a leak test pulldown times out, the solenoid valve remains energized (open) while the system is in VACUUM_ERROR. It stays open until either `vacuum_off` or `reset` is sent (both call `resetState()` which closes it). The valve coil draws power unnecessarily and the vacuum path remains open.

**Recommended fix:** Add `PIN_VACUUM_VALVE_RELAY.State(false);` to the PULLDOWN timeout path.

## Audit Results Summary

**All 31 checklist items verified.** Core command parsing, argument handling, state machine logic, and busy guards are correct. Three issues need attention before merging:

### Must Fix Before Merge

1. 🔴 **HIGH — Enum reordering breaks telemetry integer mapping** (Section 8). `VACUUM_RAMP` inserted at position 1 shifts all existing enum values. The app's `telemetry.json` map is now wrong — every vacuum state except OFF will display the wrong name. **Fix: move `VACUUM_RAMP` to position 5 (before `VACUUM_ERROR`) in the enum, and update `telemetry.json` to add `"5": "ramping"` and move error to `"6"`.**

### Should Fix (Pre-Existing)

2. ⚠️ **MEDIUM — VACUUM_PULLDOWN timeout doesn't close valve** (Section 10). Pre-existing bug, not caused by these changes. The solenoid valve is left open on pulldown timeout. **Fix: add `PIN_VACUUM_VALVE_RELAY.State(false);` to the PULLDOWN timeout path at line 164.**

### Accept / Document

3. ⚠️ **MEDIUM — `m_targetPsig` shared between RAMP and PULLDOWN** (Section 2). A `vacuum_on -5` followed by `vacuum_leak_test` (no args) would pull down to -5 instead of the previous default. Acceptable since `set_vacuum_target` has the same effect, but should be documented.
4. ⚠️ **LOW — Leak test overrides persist in memory** (Section 4). Inline args to `vacuum_leak_test` change the member variables permanently for the session. Acceptable — effectively a convenience shorthand for set + run.
5. ⚠️ **LOW — `atof` returns 0.0 for non-numeric input** (Sections 2, 5). Mitigated by validation ranges rejecting 0.0 in the heater case. For vacuum, 0.0 passes validation and would cause an immediate RAMP success. Commands come from a controlled app, so risk is minimal.
