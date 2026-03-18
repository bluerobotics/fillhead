/**
 * @file config.h
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Central configuration file for the Fillhead firmware.
 *
 * @details This file consolidates all compile-time constants, hardware pin definitions,
 * and default operational parameters for the entire Fillhead system. By centralizing these
 * values, it simplifies tuning and maintenance, and ensures consistency across all
 * controller modules. The parameters are organized into logical sections for clarity.
 */
#pragma once
#include "ClearCore.h"

//==================================================================================================
// Device Identity & Network Configuration
//==================================================================================================
/**
 * @name Device Identity
 * @{
 */
#define DEVICE_NAME_UPPER               "FILLHEAD" ///< Device name in uppercase for message prefixes and logs.
#define DEVICE_NAME_LOWER               "fillhead" ///< Device name in lowercase for identifiers.
/** @} */

/**
 * @name Network and Communication Settings
 * @{
 */
#define LOCAL_PORT                      8888      ///< The UDP port this device listens on for incoming commands.
#define CLIENT_PORT                     6272      ///< The UDP port the GUI client listens on.
#define MAX_PACKET_LENGTH               1024      ///< Maximum size in bytes for a single UDP packet. Must be large enough for the longest telemetry string.
#define RX_QUEUE_SIZE                   32        ///< Number of incoming messages that can be buffered before processing.
#define TX_QUEUE_SIZE                   32        ///< Number of outgoing messages that can be buffered before sending.
#define MAX_MESSAGE_LENGTH              MAX_PACKET_LENGTH ///< Maximum size of a single message in the Rx/Tx queues.
#define TELEMETRY_INTERVAL_MS			100       ///< How often (in milliseconds) telemetry data is published to the GUI.
/** @} */

//==================================================================================================
// System Behavior
//==================================================================================================
/**
 * @name General System Behavior
 * @{
 */
#define STATUS_MESSAGE_BUFFER_SIZE      256       ///< Standard buffer size for composing status and error messages.
#define FIRMWARE_VERSION                "0.5.0"
/** @} */

//==================================================================================================
// Watchdog Timer Configuration
//==================================================================================================
/**
 * @name Watchdog Timer Configuration
 * @{
 */
#define WATCHDOG_ENABLED                    true      ///< Enable/disable watchdog timer. When enabled, system must feed it regularly or motors will be disabled.
#define WATCHDOG_TIMEOUT_MS                 256       ///< Watchdog timeout period in milliseconds. System will reset if not fed within this time.
#define WATCHDOG_RECOVERY_FLAG              0xDEADBEEF ///< Magic number written to backup register to indicate watchdog recovery.

// Breadcrumb codes to identify where the watchdog timeout occurred.
// These mirror the Pressboi definitions so diagnostic tools can interpret both devices consistently.
#define WD_BREADCRUMB_SAFETY_CHECK          0x01      ///< Watchdog timeout in safety check
#define WD_BREADCRUMB_COMMS_UPDATE          0x02      ///< Watchdog timeout in communications update
#define WD_BREADCRUMB_RX_DEQUEUE            0x03      ///< Watchdog timeout in RX message dequeue
#define WD_BREADCRUMB_UPDATE_STATE          0x04      ///< Watchdog timeout in state machine update
#define WD_BREADCRUMB_FORCE_UPDATE          0x05      ///< Watchdog timeout in force/vacuum/heater update
#define WD_BREADCRUMB_MOTOR_UPDATE          0x06      ///< Watchdog timeout in motor update
#define WD_BREADCRUMB_TELEMETRY             0x07      ///< Watchdog timeout in telemetry publishing
#define WD_BREADCRUMB_UDP_PROCESS           0x08      ///< Watchdog timeout in UDP packet processing
#define WD_BREADCRUMB_USB_PROCESS           0x09      ///< Watchdog timeout in USB serial processing
#define WD_BREADCRUMB_TX_QUEUE              0x0A      ///< Watchdog timeout in TX queue processing
#define WD_BREADCRUMB_UDP_SEND              0x0B      ///< Watchdog timeout in UDP packet send
#define WD_BREADCRUMB_NETWORK_REFRESH       0x0C      ///< Watchdog timeout in network packet refresh
#define WD_BREADCRUMB_USB_SEND              0x0D      ///< Watchdog timeout in USB send operation
#define WD_BREADCRUMB_USB_RECONNECT         0x0E      ///< Watchdog timeout in USB reconnection handling
#define WD_BREADCRUMB_USB_RECOVERY          0x0F      ///< Watchdog timeout in USB recovery operation
#define WD_BREADCRUMB_REPORT_EVENT          0x10      ///< Watchdog timeout in reportEvent (start)
#define WD_BREADCRUMB_ENQUEUE_TX            0x11      ///< Watchdog timeout in enqueueTx
#define WD_BREADCRUMB_MOTOR_IS_FAULT        0x12      ///< Watchdog timeout in motor fault check
#define WD_BREADCRUMB_MOTOR_STATE_SWITCH    0x13      ///< Watchdog timeout in state machine switch
#define WD_BREADCRUMB_PROCESS_TX_QUEUE      0x14      ///< Watchdog timeout in processTxQueue start
#define WD_BREADCRUMB_TX_QUEUE_DEQUEUE      0x15      ///< Watchdog timeout in TX queue dequeue
#define WD_BREADCRUMB_TX_QUEUE_UDP          0x16      ///< Watchdog timeout in TX queue UDP send
#define WD_BREADCRUMB_TX_QUEUE_USB          0x17      ///< Watchdog timeout in TX queue USB send
#define WD_BREADCRUMB_DISPATCH_CMD          0x18      ///< Watchdog timeout in dispatchCommand
#define WD_BREADCRUMB_PARSE_CMD             0x19      ///< Watchdog timeout in parseCommand
#define WD_BREADCRUMB_MOTOR_FAULT_REPORT    0x1A      ///< Watchdog timeout in motor fault report
#define WD_BREADCRUMB_STATE_BUSY_CHECK      0x1B      ///< Watchdog timeout in motor busy check
#define WD_BREADCRUMB_UDP_PACKET_READ       0x1C      ///< Watchdog timeout in UDP packet read
#define WD_BREADCRUMB_RX_ENQUEUE            0x1D      ///< Watchdog timeout in RX enqueue
#define WD_BREADCRUMB_USB_AVAILABLE         0x1E      ///< Watchdog timeout checking USB available
#define WD_BREADCRUMB_USB_READ              0x1F      ///< Watchdog timeout in USB read
#define WD_BREADCRUMB_NETWORK_INPUT         0x20      ///< Watchdog timeout in network low_level_input
#define WD_BREADCRUMB_LWIP_INPUT            0x21      ///< Watchdog timeout in ethernetif_input (lwIP)
#define WD_BREADCRUMB_LWIP_TIMEOUT          0x22      ///< Watchdog timeout in sys_check_timeouts (lwIP)
#define WD_BREADCRUMB_SETUP                 0xFE      ///< Watchdog timeout during setup (before loop)
#define WD_BREADCRUMB_SETUP_MOTOR_MODE      0xF0      ///< Watchdog timeout in motor mode setup
#define WD_BREADCRUMB_SETUP_COMMS           0xF1      ///< Watchdog timeout in comms.setup()
#define WD_BREADCRUMB_SETUP_MOTOR           0xF2      ///< Watchdog timeout in motor setup
#define WD_BREADCRUMB_SETUP_FORCE           0xF3      ///< Watchdog timeout in vacuum/heater setup
#define WD_BREADCRUMB_SETUP_WD_RECOVERY     0xF4      ///< Watchdog timeout in handleWatchdogRecovery()
#define WD_BREADCRUMB_SETUP_WD_INIT         0xF5      ///< Watchdog timeout in initializeWatchdog()
#define WD_BREADCRUMB_SETUP_USB             0xF6      ///< Watchdog timeout in setupUsbSerial()
#define WD_BREADCRUMB_SETUP_ETHERNET        0xF7      ///< Watchdog timeout in setupEthernet()
#define WD_BREADCRUMB_SETUP_DHCP            0xF8      ///< Watchdog timeout in DHCP
#define WD_BREADCRUMB_SETUP_LINK_WAIT       0xF9      ///< Watchdog timeout waiting for ethernet link
#define WD_BREADCRUMB_UNKNOWN               0xFF      ///< Watchdog timeout in unknown location
/** @} */

//==================================================================================================
// System Parameters & Conversions
//==================================================================================================
/**
 * @name Core System Parameters and Unit Conversions
 * @{
 */
#define INJECTOR_PITCH_MM_PER_REV       5.0f      ///< Linear travel (in mm) of the injector plunger for one full motor revolution.
#define PINCH_PITCH_MM_PER_REV          2.0f      ///< Linear travel (in mm) of the pinch valve actuator for one full motor revolution.
#define PULSES_PER_REV                  800       ///< Number of step pulses required for one full motor revolution (dependent on microstepping settings).
#define STEPS_PER_MM_INJECTOR           (PULSES_PER_REV / INJECTOR_PITCH_MM_PER_REV)   ///< Calculated steps per millimeter for the main injector drive.
#define STEPS_PER_MM_PINCH              (PULSES_PER_REV / PINCH_PITCH_MM_PER_REV)      ///< Calculated steps per millimeter for the pinch valve mechanism.
#define MAX_HOMING_DURATION_MS          100000    ///< Maximum time (in milliseconds) a homing operation is allowed to run before timing out.
/** @} */

//==================================================================================================
// Hardware Pin Definitions
//==================================================================================================
/**
 * @name Hardware Pin and Connector Assignments
 * @{
 */
// --- Motors ---
#define MOTOR_INJECTOR_A                ConnectorM0 ///< Primary injector motor.
#define MOTOR_INJECTOR_B                ConnectorM1 ///< Secondary, ganged injector motor.
#define MOTOR_VACUUM_VALVE              ConnectorM3 ///< Motor for the pinch valve on the vacuum side.
#define MOTOR_INJECTION_VALVE           ConnectorM2 ///< Motor for the pinch valve on the injection side.

// --- Analog Sensors ---
#define PIN_THERMOCOUPLE                ConnectorA12 ///< Analog input for the heater thermocouple.
// Sensor PN: XP5-080-01
#define PIN_VACUUM_TRANSDUCER           ConnectorA11 ///< Analog input for the vacuum pressure transducer.

// --- Digital Outputs (Relays) ---
#define PIN_HEATER_RELAY                ConnectorIO1 ///< Digital output to control the heater relay.
#define PIN_VACUUM_RELAY                ConnectorIO0 ///< Digital output to control the vacuum pump relay.
#define PIN_VACUUM_VALVE_RELAY          ConnectorIO5 ///< Digital output to control the vacuum solenoid valve relay.

// --- Safety Inputs ---
#define PIN_LIGHT_CURTAIN               ConnectorIO4 ///< Digital input for the light curtain safety sensor on IO4.
#define LIGHT_CURTAIN_ACTIVE_STATE      false        ///< Pin state when beam is broken (false = active LOW, typical NPN normally-closed).
#define LIGHT_CURTAIN_FILTER_MS         2            ///< Debounce filter length in milliseconds for the light curtain input.
#define LIGHT_CURTAIN_ENABLED           true         ///< Enable/disable light curtain safety check. Set false if sensor is not installed.

// --- Home Sensors (Hall Effect) ---
// Sensor PN: 326161-0053
#define HOME_SENSOR_M0                  ConnectorDI7 ///< Motor A (M0) home sensor on DI7.
#define HOME_SENSOR_M1                  ConnectorDI6 ///< Motor B (M1) home sensor on DI6.
#define HOME_SENSOR_M2                  ConnectorDI8 ///< Vacuum pinch valve home sensor on DI8 (used by injection valve on M2 after port swap).
#define HOME_SENSOR_M3                  ConnectorA9  ///< Injection pinch valve home sensor on A9 (used by vacuum valve on M3 after port swap).
#define HOME_SENSOR_ACTIVE_STATE        true         ///< true = sensor outputs HIGH when triggered (active high).
#define HOME_SENSOR_FILTER_MS           2            ///< Debounce filter length in milliseconds for home sensors.
/** @} */

//==================================================================================================
// Sensor & Control Parameters
//==================================================================================================
/**
 * @name Sensor and Control Loop Parameters
 * @{
 */
#define SENSOR_SAMPLE_INTERVAL_MS       100       ///< How often (in ms) to sample temperature and vacuum sensors.
#define EWMA_ALPHA_SENSORS              0.5f      ///< Smoothing factor (alpha) for the Exponentially Weighted Moving Average filter on sensor readings.
#define EWMA_ALPHA_TORQUE               0.2f      ///< Smoothing factor (alpha) for the EWMA filter on motor torque readings.

/**
 * @name Heater Control Defaults
 * @{
 */
#define TC_V_REF                        10.0f     ///< ADC reference voltage (10V for ClearCore analog inputs).
#define TC_V_OFFSET                     1.25f     ///< Thermocouple sensor's output voltage at 0 degrees Celsius.
#define TC_GAIN                         200.0f    ///< Conversion factor from volts to degrees Celsius for the thermocouple sensor.
#define PID_UPDATE_INTERVAL_MS          100       ///< How often (in ms) the PID calculation for the heater is performed.
#define PID_PWM_PERIOD_MS               1000      ///< Time window (in ms) for the time-proportioned heater relay control (software PWM).
#define DEFAULT_HEATER_SETPOINT_C       70.0f     ///< Default target temperature in Celsius.
#define DEFAULT_HEATER_KP               60.0f     ///< Default Proportional (P) gain for the PID controller.
#define DEFAULT_HEATER_KI               2.5f      ///< Default Integral (I) gain for the PID controller.
#define DEFAULT_HEATER_KD               40.0f     ///< Default Derivative (D) gain for the PID controller.
/** @} */

/**
 * @name Vacuum System Defaults
 * @{
 */
#define VAC_V_OUT_MIN                   1.0f      ///< Sensor output voltage at minimum pressure (-14.65 PSIG).
#define VAC_V_OUT_MAX                   5.0f      ///< Sensor output voltage at maximum pressure (14.50 PSIG).
#define VAC_PRESSURE_MIN                -14.65f   ///< Minimum pressure in PSIG (-101 kPa).
#define VAC_PRESSURE_MAX                14.50f    ///< Maximum pressure in PSIG (100 kPa).
#define VACUUM_PSIG_OFFSET              0.0f      ///< A calibration offset to be added to the raw vacuum sensor reading.
#define DEFAULT_VACUUM_TARGET_PSIG      -14.0f    ///< Default target pressure in PSIG for vacuum operations.
#define DEFAULT_VACUUM_RAMP_TIMEOUT_MS  30000     ///< Default time (in ms) allowed to reach the target pressure before a timeout error.
#define DEFAULT_LEAK_TEST_DELTA_PSIG    0.1f      ///< Default maximum allowed pressure drop during a leak test.
#define DEFAULT_LEAK_TEST_DURATION_MS   10000     ///< Default duration (in ms) for a leak test.
#define VACUUM_SETTLE_TIME_S            2.0f      ///< Time (in s) to let pressure stabilize before starting a leak test measurement.
/** @} */
/** @} */

//==================================================================================================
// Force Sensor Configuration
//==================================================================================================
/**
 * @name Force Sensor Configuration
 * @{
 */
#define FORCE_SENSOR_ENABLED                true      ///< Enable/disable force sensor functionality.
#define FORCE_SENSOR_SCALE_FACTOR           -0.00023076f  ///< Default scale factor: kg = raw_adc × scale.
#define FORCE_SENSOR_OFFSET_KG              6.5f     ///< Default offset (kg) for calibration.

/** Machine strain compensation coefficients (force vs. distance polynomial) */
#define MACHINE_STRAIN_COEFF_X4            -4617.6757f     ///< x^4 coefficient
#define MACHINE_STRAIN_COEFF_X3             6910.4430f     ///< x^3 coefficient
#define MACHINE_STRAIN_COEFF_X2            -1585.3214f     ///< x^2 coefficient
#define MACHINE_STRAIN_COEFF_X1             320.1528f      ///< x coefficient
#define MACHINE_STRAIN_COEFF_C              -0.2376f       ///< Constant term
#define MACHINE_STRAIN_MAX_DEFLECTION_MM     5.0f          ///< Max expected machine flex deflection
#define MACHINE_STRAIN_CONTACT_FORCE_KG      3.0f          ///< Force threshold to declare contact
#define RETRACT_DEFAULT_SPEED_MMS           25.0f          ///< Default retract speed when none specified
#define FORCE_SENSOR_MIN_KG                 -10.0f    ///< Minimum valid force reading (kg).
#define FORCE_SENSOR_MAX_KG                 1200.0f   ///< Maximum expected force (kg).
#define FORCE_SENSOR_MAX_SAFETY_FACTOR      1.2f      ///< Safety factor for maximum force.
#define FORCE_SENSOR_MAX_LIMIT_KG           (FORCE_SENSOR_MAX_KG * FORCE_SENSOR_MAX_SAFETY_FACTOR) ///< Calculated max limit.
#define FORCE_SENSOR_TIMEOUT_MS             1000      ///< Time (ms) without readings before disconnected.
/** @} */

//==================================================================================================
// Motion & Operation Defaults
//==================================================================================================
/**
 * @name Motion and Operation Parameters
 * @{
 */

/**
 * @name Motor Setup Defaults
 * @{
 */
#define TORQUE_HLFB_AT_POSITION		-9999.0f  ///< Special value from ClearCore HLFB when a move is complete and the motor is at position.
#define MOTOR_DEFAULT_VEL_MAX_MMS           156.25f   ///< Default maximum velocity for motors in mm/s.
#define MOTOR_DEFAULT_ACCEL_MAX_MMSS        625.0f    ///< Default maximum acceleration for motors in mm/s^2.
#define MOTOR_DEFAULT_VEL_MAX_SPS           (int)(MOTOR_DEFAULT_VEL_MAX_MMS * STEPS_PER_MM_INJECTOR) ///< Default max velocity in steps/sec, derived from mm/s.
#define MOTOR_DEFAULT_ACCEL_MAX_SPS2        (int)(MOTOR_DEFAULT_ACCEL_MAX_MMSS * STEPS_PER_MM_INJECTOR) ///< Default max acceleration in steps/sec^2, derived from mm/s^2.
#define PINCH_DEFAULT_VEL_MAX_SPS           (int)(MOTOR_DEFAULT_VEL_MAX_MMS * STEPS_PER_MM_PINCH)      ///< Default max velocity for pinch valves in steps/sec.
#define PINCH_DEFAULT_ACCEL_MAX_SPS2        (int)(MOTOR_DEFAULT_ACCEL_MAX_MMSS * STEPS_PER_MM_PINCH)   ///< Default max acceleration for pinch valves in steps/sec^2.
/** @} */

/**
 * @name Timeouts
 * @{
 */
#define MOVE_START_TIMEOUT_MS           (250)     ///< Time (in ms) to wait for a move to start before flagging a motor error.
/** @} */

/**
 * @name Initialization Defaults
 * @{
 */
#define DEFAULT_INJECTOR_TORQUE_LIMIT       20.0f     ///< Default torque limit (%) for general injector motor operations.
#define DEFAULT_INJECTOR_TORQUE_OFFSET      -2.4f     ///< Default torque offset (%) to account for sensor bias or no-load friction.
/** @} */

/**
 * @name Injector Homing Defaults
 * @{
 */
#define INJECTOR_HOMING_STROKE_MM           500.0f    ///< Maximum travel distance (mm) during a homing sequence.
#define INJECTOR_HOMING_RAPID_VEL_MMS       5.0f      ///< Velocity (mm/s) for the initial high-speed search for the hard stop.
#define INJECTOR_HOMING_TOUCH_VEL_MMS       1.0f      ///< Velocity (mm/s) for the final, slow-speed precise touch-off.
#define INJECTOR_HOMING_BACKOFF_VEL_MMS     1.0f      ///< Velocity (mm/s) for backing off the hard stop.
#define INJECTOR_HOMING_ACCEL_MMSS          100.0f    ///< Acceleration (mm/s^2) for all homing moves.
#define INJECTOR_HOMING_SEARCH_TORQUE_PERCENT 10.0f   ///< Torque limit (%) used to detect the hard stop.
#define INJECTOR_HOMING_BACKOFF_TORQUE_PERCENT 40.0f  ///< Higher torque limit (%) for the back-off move to prevent stalling.
#define INJECTOR_HOMING_BACKOFF_MM          1.0f      ///< Distance (mm) to back off from the hard stop.
/** @} */

/**
 * @name Pinch Valve Homing (Hall Sensor)
 * @details Pinch valves home toward the OPEN end of their stroke where a hall effect
 * sensor is mounted. The valve moves in the negative (open) direction to find the
 * sensor, backs off, then re-approaches slowly for precision.
 * @{
 */
#define PINCH_HOMING_STROKE_MM                  50.0f  ///< Max travel (mm) during rapid search toward the home sensor.
#define PINCH_HOMING_RAPID_VEL_MMS              5.0f   ///< Velocity (mm/s) for the initial rapid search toward the sensor.
#define PINCH_HOMING_SLOW_VEL_MMS               1.0f   ///< Velocity (mm/s) for the slow precision approach after backoff.
#define PINCH_HOMING_ACCEL_MMSS                 100.0f ///< Acceleration (mm/s^2) for all homing moves.
#define PINCH_HOMING_BACKOFF_MM                 2.0f   ///< Distance (mm) to back off from the sensor before the slow approach.

#define NVM_SLOT_INJ_VALVE_HOME_ON_BOOT         (16 * 4) ///< NVM byte offset for injection valve home-on-boot flag.
#define NVM_SLOT_VAC_VALVE_HOME_ON_BOOT         (17 * 4) ///< NVM byte offset for vacuum valve home-on-boot flag.
#define NVM_SLOT_CARTRIDGE_ML_PER_MM            (18 * 4) ///< NVM byte offset for cartridge ml_per_mm (stored as int * 10000).
/** @} */

/**
 * @name Pinch Valve Operation Defaults
 * @{
 */
#define PINCH_VALVE_PINCH_TORQUE_PERCENT    10.0f     ///< Target torque (%) for closing the valve during normal operation.
#define PINCH_VALVE_PINCH_VEL_MMS           2.0f      ///< Speed (mm/s) for the closing (pinching) move.
#define PINCH_VALVE_OPEN_VEL_MMS            10.0f     ///< Speed (mm/s) for the opening move (returning to the homed position).
#define PINCH_VALVE_OPEN_ACCEL_MMSS         50.0f     ///< Acceleration (mm/s^2) for the opening move.
/** @} */

/**
 * @name Jogging Defaults
 * @{
 */
#define JOG_DEFAULT_TORQUE_PERCENT          30        ///< Default torque limit (%) for jog moves.
#define JOG_DEFAULT_VEL_MMS                 1.0f      ///< Default velocity (mm/s) for jog moves.
#define JOG_DEFAULT_ACCEL_MMSS              10.0f     ///< Default acceleration (mm/s^2) for jog moves.
#define PINCH_JOG_DEFAULT_VEL_MMS           5.0f      ///< Default velocity (mm/s) for pinch valve jog moves.
#define PINCH_JOG_DEFAULT_ACCEL_MMSS        25.0f     ///< Default acceleration (mm/s^2) for pinch valve jog moves.
/** @} */

/**
 * @name Injection Defaults
 * @{
 */
#define INJECT_DEFAULT_SPEED_MLS            0.5f      ///< Default injection speed in ml/s.
#define DEFAULT_CARTRIDGE_ML_PER_MM         5.2732f   ///< Default cartridge ratio: mL dispensed per mm piston travel (from legacy 75+33mm stator pistons).
/** @} */

/**
 * @name Feed Defaults
 * @{
 */
#define FEED_DEFAULT_TORQUE_PERCENT         30        ///< Default torque limit (%) for feed/inject moves.
#define FEED_DEFAULT_VELOCITY_MMS           6.25f     ///< Default velocity (mm/s) for feed/inject moves.
#define FEED_DEFAULT_ACCEL_MMSS             62.5f     ///< Default acceleration (mm/s^2) for feed/inject moves.
#define FEED_DEFAULT_VELOCITY_SPS           (int)(FEED_DEFAULT_VELOCITY_MMS * STEPS_PER_MM_INJECTOR) ///< Default feed velocity in steps/sec.
#define FEED_DEFAULT_ACCEL_SPS2             (int)(FEED_DEFAULT_ACCEL_MMSS * STEPS_PER_MM_INJECTOR)   ///< Default feed acceleration in steps/sec^2.
#define INJECT_DEFAULT_VELOCITY_MMS         0.625f    ///< Fallback velocity if a command provides an invalid value.
#define INJECT_DEFAULT_VELOCITY_SPS         (int)(INJECT_DEFAULT_VELOCITY_MMS * STEPS_PER_MM_INJECTOR) ///< Fallback velocity in steps/sec.
/** @} */

/**
 * @name Move Defaults (Pressboi-compatible)
 * @{
 */
#define MOVE_DEFAULT_TORQUE_PERCENT         30        ///< Default torque limit (%) for general moves.
#define MOVE_DEFAULT_VELOCITY_MMS           6.25f     ///< Default velocity (mm/s) for general moves.
#define MOVE_DEFAULT_ACCEL_MMSS             62.5f     ///< Default acceleration (mm/s^2) for general moves.
#define MOVE_DEFAULT_VELOCITY_SPS           (int)(MOVE_DEFAULT_VELOCITY_MMS * STEPS_PER_MM_INJECTOR) ///< Default velocity in steps/sec.
#define MOVE_DEFAULT_ACCEL_SPS2             (int)(MOVE_DEFAULT_ACCEL_MMSS * STEPS_PER_MM_INJECTOR)   ///< Default acceleration in steps/sec^2.
/** @} */
/** @} */

