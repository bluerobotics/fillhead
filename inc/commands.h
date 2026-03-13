/**
 * @file commands.h
 * @brief Defines the command interface for the Fillhead controller.
 * @details AUTO-GENERATED FILE - DO NOT EDIT MANUALLY
 * Generated from commands.json on 2025-10-22 18:05:29
 * 
 * This header file defines all commands that can be sent TO the Fillhead device.
 * For response message formats, see responses.h
 * To modify commands, edit commands.json and regenerate this file.
 */
 #pragma once

 //==================================================================================================
 // Command Strings (Host → Device)
 //==================================================================================================
 
 /**
  * @name General System Commands
  * @{
  */
 #define CMD_STR_ENABLE                              "enable                        " ///< Command to enable all motors.
 #define CMD_STR_DISABLE                             "disable                       " ///< Command to disable all motors.
 #define CMD_STR_DISCOVER_DEVICE                     "discover_device               " ///< Generic command for any device to respond to.
 #define CMD_STR_ABORT                               "abort                         " ///< Command to halt all ongoing operations.
 #define CMD_STR_CLEAR_ERRORS                        "clear_errors                  " ///< Command to clear any existing fault states.
 /** @} */
 
 /**
  * @name Motion Commands
  * @{
  */
 #define CMD_STR_INJECT_STATOR                       "inject_stator                 " ///< Command to dispense a specific volume using the stator (5:1) configuration.
 #define CMD_STR_INJECT_ROTOR                        "inject_rotor                  " ///< Command to dispense a specific volume using the rotor (1:1) configuration.
 #define CMD_STR_JOG_MOVE                            "jog_move                      " ///< Command to jog the injector motors by a relative distance.
 #define CMD_STR_MACHINE_HOME                        "machine_home                  " ///< Command to home the main machine axis.
 #define CMD_STR_MACHINE_HOME_MOVE                   "machine_home_move             " ///< Command to move to machine home position.
 #define CMD_STR_CARTRIDGE_HOME                      "cartridge_home                " ///< Command to home the injector against the cartridge.
 #define CMD_STR_CARTRIDGE_HOME_MOVE                 "cartridge_home_move           " ///< Command to move to cartridge home position.
 #define CMD_STR_MOVE_TO_CARTRIDGE_HOME              "move_to_cartridge_home        " ///< Command to move the injector to the cartridge home position.
 #define CMD_STR_MOVE_TO_CARTRIDGE_RETRACT           "move_to_cartridge_retract     " ///< Command to retract the injector a specified distance from cartridge home.
 #define CMD_STR_PAUSE_INJECTION                     "pause_injection               " ///< Command to pause an ongoing injection.
 #define CMD_STR_RESUME_INJECTION                    "resume_injection              " ///< Command to resume a paused injection.
 #define CMD_STR_CANCEL_INJECTION                    "cancel_injection              " ///< Command to cancel an ongoing injection.
 /** @} */
 
 /**
  * @name Valve Commands
  * @{
  */
 #define CMD_STR_INJECTION_VALVE_HOME                "injection_valve_home          " ///< Command to home the injection valve.
 #define CMD_STR_INJECTION_VALVE_HOME_UNTUBED        "injection_valve_home_untubed  " ///< Command to home the injection valve (untubed).
 #define CMD_STR_INJECTION_VALVE_HOME_TUBED          "injection_valve_home_tubed    " ///< Command to home the injection valve (tubed).
 #define CMD_STR_INJECTION_VALVE_OPEN                "injection_valve_open          " ///< Command to open the injection valve.
 #define CMD_STR_INJECTION_VALVE_CLOSE               "injection_valve_close         " ///< Command to close the injection valve.
 #define CMD_STR_INJECTION_VALVE_JOG                 "injection_valve_jog           " ///< Command to jog the injection valve motor.
 #define CMD_STR_VACUUM_VALVE_HOME                   "vacuum_valve_home             " ///< Command to home the vacuum valve.
 #define CMD_STR_VACUUM_VALVE_HOME_UNTUBED           "vacuum_valve_home_untubed     " ///< Command to home the vacuum valve (untubed).
 #define CMD_STR_VACUUM_VALVE_HOME_TUBED             "vacuum_valve_home_tubed       " ///< Command to home the vacuum valve (tubed).
 #define CMD_STR_VACUUM_VALVE_OPEN                   "vacuum_valve_open             " ///< Command to open the vacuum valve.
 #define CMD_STR_VACUUM_VALVE_CLOSE                  "vacuum_valve_close            " ///< Command to close the vacuum valve.
 #define CMD_STR_VACUUM_VALVE_JOG                    "vacuum_valve_jog              " ///< Command to jog the vacuum valve motor.
 /** @} */
 
 /**
  * @name Heater Commands
  * @{
  */
 #define CMD_STR_HEATER_ON                           "heater_on                     " ///< Command to turn the heater on.
 #define CMD_STR_HEATER_OFF                          "heater_off                    " ///< Command to turn the heater off.
 #define CMD_STR_SET_HEATER_GAINS                    "set_heater_gains              " ///< Command to set heater PID gains.
 #define CMD_STR_SET_HEATER_SETPOINT                  "set_heater_setpoint           " ///< Command to set heater temperature setpoint.
 /** @} */
 
 /**
  * @name Vacuum Commands
  * @{
  */
 #define CMD_STR_VACUUM_ON                           "vacuum_on                     " ///< Command to turn the vacuum pump on.
 #define CMD_STR_VACUUM_OFF                          "vacuum_off                    " ///< Command to turn the vacuum pump off.
 #define CMD_STR_VACUUM_LEAK_TEST                    "vacuum_leak_test              " ///< Command to initiate a vacuum leak test.
 #define CMD_STR_SET_VACUUM_TARGET                   "set_vacuum_target             " ///< Command to set vacuum target pressure.
 #define CMD_STR_SET_VACUUM_TIMEOUT_S                "set_vacuum_timeout_s          " ///< Command to set vacuum timeout (seconds).
 #define CMD_STR_SET_LEAK_TEST_DELTA                 "set_leak_test_delta           " ///< Command to set leak test delta.
 #define CMD_STR_SET_LEAK_TEST_DURATION_S            "set_leak_test_duration_s      " ///< Command to set leak test duration (seconds).
 /** @} */
 
 //==================================================================================================
 // Command Enum
 //==================================================================================================
 
 /**
  * @enum Command
  * @brief Enumerates all possible commands that can be processed by the Fillhead.
  * @details This enum provides a type-safe way to handle incoming commands.
  */
 typedef enum {
	 CMD_UNKNOWN,                        ///< Represents an unrecognized or invalid command.
 
	 // General System Commands
	 CMD_ENABLE,                             ///< @see CMD_STR_ENABLE
	 CMD_DISABLE,                            ///< @see CMD_STR_DISABLE
	 CMD_DISCOVER_DEVICE,                    ///< @see CMD_STR_DISCOVER_DEVICE
	 CMD_ABORT,                              ///< @see CMD_STR_ABORT
	 CMD_CLEAR_ERRORS,                       ///< @see CMD_STR_CLEAR_ERRORS

	 // Motion Commands
	 CMD_INJECT_STATOR,                      ///< @see CMD_STR_INJECT_STATOR
	 CMD_INJECT_ROTOR,                       ///< @see CMD_STR_INJECT_ROTOR
	 CMD_JOG_MOVE,                           ///< @see CMD_STR_JOG_MOVE
	 CMD_MACHINE_HOME,                       ///< @see CMD_STR_MACHINE_HOME
	 CMD_MACHINE_HOME_MOVE,                  ///< @see CMD_STR_MACHINE_HOME_MOVE
	 CMD_CARTRIDGE_HOME,                     ///< @see CMD_STR_CARTRIDGE_HOME
	 CMD_CARTRIDGE_HOME_MOVE,                ///< @see CMD_STR_CARTRIDGE_HOME_MOVE
	 CMD_MOVE_TO_CARTRIDGE_HOME,             ///< @see CMD_STR_MOVE_TO_CARTRIDGE_HOME
	 CMD_MOVE_TO_CARTRIDGE_RETRACT,          ///< @see CMD_STR_MOVE_TO_CARTRIDGE_RETRACT
	 CMD_PAUSE_INJECTION,                    ///< @see CMD_STR_PAUSE_INJECTION
	 CMD_RESUME_INJECTION,                   ///< @see CMD_STR_RESUME_INJECTION
	 CMD_CANCEL_INJECTION,                   ///< @see CMD_STR_CANCEL_INJECTION

	 // Valve Commands
	 CMD_INJECTION_VALVE_HOME,               ///< @see CMD_STR_INJECTION_VALVE_HOME
	 CMD_INJECTION_VALVE_HOME_UNTUBED,       ///< @see CMD_STR_INJECTION_VALVE_HOME_UNTUBED
	 CMD_INJECTION_VALVE_HOME_TUBED,         ///< @see CMD_STR_INJECTION_VALVE_HOME_TUBED
	 CMD_INJECTION_VALVE_OPEN,               ///< @see CMD_STR_INJECTION_VALVE_OPEN
	 CMD_INJECTION_VALVE_CLOSE,              ///< @see CMD_STR_INJECTION_VALVE_CLOSE
	 CMD_INJECTION_VALVE_JOG,                ///< @see CMD_STR_INJECTION_VALVE_JOG
	 CMD_VACUUM_VALVE_HOME,                  ///< @see CMD_STR_VACUUM_VALVE_HOME
	 CMD_VACUUM_VALVE_HOME_UNTUBED,          ///< @see CMD_STR_VACUUM_VALVE_HOME_UNTUBED
	 CMD_VACUUM_VALVE_HOME_TUBED,            ///< @see CMD_STR_VACUUM_VALVE_HOME_TUBED
	 CMD_VACUUM_VALVE_OPEN,                  ///< @see CMD_STR_VACUUM_VALVE_OPEN
	 CMD_VACUUM_VALVE_CLOSE,                 ///< @see CMD_STR_VACUUM_VALVE_CLOSE
	 CMD_VACUUM_VALVE_JOG,                   ///< @see CMD_STR_VACUUM_VALVE_JOG

	 // Heater Commands
	 CMD_HEATER_ON,                          ///< @see CMD_STR_HEATER_ON
	 CMD_HEATER_OFF,                         ///< @see CMD_STR_HEATER_OFF
	 CMD_SET_HEATER_GAINS,                    ///< @see CMD_STR_SET_HEATER_GAINS
	 CMD_SET_HEATER_SETPOINT,                 ///< @see CMD_STR_SET_HEATER_SETPOINT

	 // Vacuum Commands
	 CMD_VACUUM_ON,                          ///< @see CMD_STR_VACUUM_ON
	 CMD_VACUUM_OFF,                         ///< @see CMD_STR_VACUUM_OFF
	 CMD_VACUUM_LEAK_TEST,                   ///< @see CMD_STR_VACUUM_LEAK_TEST
	 CMD_SET_VACUUM_TARGET,                  ///< @see CMD_STR_SET_VACUUM_TARGET
	 CMD_SET_VACUUM_TIMEOUT_S,               ///< @see CMD_STR_SET_VACUUM_TIMEOUT_S
	 CMD_SET_LEAK_TEST_DELTA,                ///< @see CMD_STR_SET_LEAK_TEST_DELTA
	 CMD_SET_LEAK_TEST_DURATION_S            ///< @see CMD_STR_SET_LEAK_TEST_DURATION_S
 } Command;
 