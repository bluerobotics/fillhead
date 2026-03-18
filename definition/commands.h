/**
 * @file commands.h
 * @brief Defines the command interface for the Fillhead controller.
 * @details Fillhead = Pressboi + pinch valves + heater + vacuum + injection.
 * This header contains ALL Pressboi commands plus Fillhead-specific extensions.
 */
#pragma once

//==================================================================================================
// Command Strings (Host → Device)
//==================================================================================================

/**
 * @name General System Commands
 * @{
 */
#define CMD_STR_DISCOVER_DEVICE             "DISCOVER_DEVICE"
#define CMD_STR_RESET                       "reset"
#define CMD_STR_ENABLE                      "enable"
#define CMD_STR_DISABLE                     "disable"
#define CMD_STR_ABORT                       "abort"
#define CMD_STR_TEST_WATCHDOG               "test_watchdog"
#define CMD_STR_REBOOT_BOOTLOADER           "reboot_bootloader"
#define CMD_STR_DUMP_NVM                    "dump_nvm"
#define CMD_STR_RESET_NVM                   "reset_nvm"
#define CMD_STR_DUMP_ERROR_LOG              "dump_error_log"
/** @} */

/**
 * @name Force/Calibration Commands (from Pressboi)
 * @{
 */
#define CMD_STR_SET_FORCE_MODE              "set_force_mode "
#define CMD_STR_SET_FORCE_OFFSET            "set_force_offset "
#define CMD_STR_SET_FORCE_ZERO              "set_force_zero"
#define CMD_STR_SET_FORCE_SCALE             "set_force_scale "
#define CMD_STR_SET_STRAIN_CAL              "set_strain_cal "
#define CMD_STR_SET_POLARITY                "set_polarity "
#define CMD_STR_HOME_ON_BOOT                "home_on_boot "
#define CMD_STR_SET_PRESS_THRESHOLD         "set_press_threshold "
/** @} */

/**
 * @name General Motion Commands (from Pressboi)
 * @{
 */
#define CMD_STR_HOME                        "home"
#define CMD_STR_MOVE_ABS                    "move_abs "
#define CMD_STR_MOVE_INC                    "move_inc "
#define CMD_STR_SET_RETRACT                 "set_retract "
#define CMD_STR_RETRACT                     "retract"
#define CMD_STR_PAUSE                       "pause"
#define CMD_STR_RESUME                      "resume"
#define CMD_STR_CANCEL                      "cancel"
/** @} */

/**
 * @name Fillhead Injection Commands
 * @{
 */
#define CMD_STR_INJECT_STATOR               "inject_stator "
#define CMD_STR_INJECT_ROTOR                "inject_rotor "
#define CMD_STR_JOG_MOVE                    "jog_move "
#define CMD_STR_MACHINE_HOME                "machine_home"
#define CMD_STR_MACHINE_HOME_MOVE           "machine_home_move"
#define CMD_STR_CARTRIDGE_HOME              "cartridge_home"
#define CMD_STR_CARTRIDGE_HOME_MOVE         "cartridge_home_move"
#define CMD_STR_MOVE_TO_CARTRIDGE_HOME      "move_to_cartridge_home"
#define CMD_STR_MOVE_TO_CARTRIDGE_RETRACT   "move_to_cartridge_retract "
#define CMD_STR_PAUSE_INJECTION             "pause_injection"
#define CMD_STR_RESUME_INJECTION            "resume_injection"
#define CMD_STR_CANCEL_INJECTION            "cancel_injection"
/** @} */

/**
 * @name Valve Commands (Fillhead-specific)
 * @{
 */
#define CMD_STR_INJECTION_VALVE_HOME_ON_BOOT "injection_valve_home_on_boot "
#define CMD_STR_INJECTION_VALVE_HOME        "injection_valve_home"
#define CMD_STR_INJECTION_VALVE_OPEN        "injection_valve_open"
#define CMD_STR_INJECTION_VALVE_CLOSE       "injection_valve_close"
#define CMD_STR_INJECTION_VALVE_JOG         "injection_valve_jog "
#define CMD_STR_VACUUM_VALVE_HOME_ON_BOOT   "vacuum_valve_home_on_boot "
#define CMD_STR_VACUUM_VALVE_HOME           "vacuum_valve_home"
#define CMD_STR_VACUUM_VALVE_OPEN           "vacuum_valve_open"
#define CMD_STR_VACUUM_VALVE_CLOSE          "vacuum_valve_close"
#define CMD_STR_VACUUM_VALVE_JOG            "vacuum_valve_jog "
/** @} */

/**
 * @name Heater Commands (Fillhead-specific)
 * @{
 */
#define CMD_STR_HEATER_ON                   "heater_on"
#define CMD_STR_HEATER_OFF                  "heater_off"
#define CMD_STR_SET_HEATER_GAINS            "set_heater_gains "
#define CMD_STR_SET_HEATER_SETPOINT         "set_heater_setpoint "
/** @} */

/**
 * @name Vacuum Commands (Fillhead-specific)
 * @{
 */
#define CMD_STR_VACUUM_ON                   "vacuum_on"
#define CMD_STR_VACUUM_OFF                  "vacuum_off"
#define CMD_STR_VACUUM_LEAK_TEST            "vacuum_leak_test"
#define CMD_STR_SET_VACUUM_TARGET           "set_vacuum_target "
#define CMD_STR_SET_VACUUM_TIMEOUT_S        "set_vacuum_timeout_s "
#define CMD_STR_SET_LEAK_TEST_DELTA         "set_leak_test_delta "
#define CMD_STR_SET_LEAK_TEST_DURATION_S    "set_leak_test_duration_s "
/** @} */

//==================================================================================================
// Command Enum
//==================================================================================================

typedef enum {
    CMD_UNKNOWN,

    // General System Commands
    CMD_DISCOVER_DEVICE,
    CMD_RESET,
    CMD_ENABLE,
    CMD_DISABLE,
    CMD_ABORT,
    CMD_TEST_WATCHDOG,
    CMD_REBOOT_BOOTLOADER,
    CMD_DUMP_NVM,
    CMD_RESET_NVM,
    CMD_DUMP_ERROR_LOG,

    // Force/Calibration Commands (from Pressboi)
    CMD_SET_FORCE_MODE,
    CMD_SET_FORCE_OFFSET,
    CMD_SET_FORCE_ZERO,
    CMD_SET_FORCE_SCALE,
    CMD_SET_STRAIN_CAL,
    CMD_SET_POLARITY,
    CMD_HOME_ON_BOOT,
    CMD_SET_PRESS_THRESHOLD,

    // General Motion Commands (from Pressboi)
    CMD_HOME,
    CMD_MOVE_ABS,
    CMD_MOVE_INC,
    CMD_SET_RETRACT,
    CMD_RETRACT,
    CMD_PAUSE,
    CMD_RESUME,
    CMD_CANCEL,

    // Fillhead Injection Commands
    CMD_INJECT_STATOR,
    CMD_INJECT_ROTOR,
    CMD_JOG_MOVE,
    CMD_MACHINE_HOME,
    CMD_MACHINE_HOME_MOVE,
    CMD_CARTRIDGE_HOME,
    CMD_CARTRIDGE_HOME_MOVE,
    CMD_MOVE_TO_CARTRIDGE_HOME,
    CMD_MOVE_TO_CARTRIDGE_RETRACT,
    CMD_PAUSE_INJECTION,
    CMD_RESUME_INJECTION,
    CMD_CANCEL_INJECTION,

    // Valve Commands
    CMD_INJECTION_VALVE_HOME_ON_BOOT,
    CMD_INJECTION_VALVE_HOME,
    CMD_INJECTION_VALVE_OPEN,
    CMD_INJECTION_VALVE_CLOSE,
    CMD_INJECTION_VALVE_JOG,
    CMD_VACUUM_VALVE_HOME_ON_BOOT,
    CMD_VACUUM_VALVE_HOME,
    CMD_VACUUM_VALVE_OPEN,
    CMD_VACUUM_VALVE_CLOSE,
    CMD_VACUUM_VALVE_JOG,

    // Heater Commands
    CMD_HEATER_ON,
    CMD_HEATER_OFF,
    CMD_SET_HEATER_GAINS,
    CMD_SET_HEATER_SETPOINT,

    // Vacuum Commands
    CMD_VACUUM_ON,
    CMD_VACUUM_OFF,
    CMD_VACUUM_LEAK_TEST,
    CMD_SET_VACUUM_TARGET,
    CMD_SET_VACUUM_TIMEOUT_S,
    CMD_SET_LEAK_TEST_DELTA,
    CMD_SET_LEAK_TEST_DURATION_S
} Command;

//==================================================================================================
// Command Parser Functions
//==================================================================================================

Command parseCommand(const char* cmdStr);
const char* getCommandParams(const char* cmdStr, Command cmd);
