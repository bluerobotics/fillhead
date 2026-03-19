/**
 * @file injector_controller.h
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Defines the controller for the dual-motor material injector system.
 *
 * @details This file defines the `Injector` class, which is responsible for managing
 * the two ganged motors that drive the material dispensing mechanism. It contains
 * the complex state machines required for homing (both to the machine's physical
 * zero and to a material cartridge), precision injection moves, general-purpose
 * absolute/incremental moves, and manual jogging.
 * The various state enums that govern the injector's behavior are also defined here.
 *
 * This header is a superset of both the original Fillhead injector controller and
 * the Pressboi MotorController, combining injection-specific functionality with
 * general-purpose press/move capabilities.
 */
#pragma once

#include "config.h"
#include "comms_controller.h"
#include "commands.h"

class Fillhead; // Forward declaration

/**
 * @enum HomingState
 * @brief Defines the top-level active homing operation for the injector motors.
 */
enum HomingState : uint8_t {
	HOMING_NONE,        ///< No homing operation is active.
	HOMING_MACHINE,     ///< Homing to the machine's physical zero point (fully retracted).
	HOMING_CARTRIDGE    ///< Homing to the front of the material cartridge.
};

/**
 * @enum HomingPhase
 * @brief Defines the detailed sub-state or "phase" within an active homing operation.
 * @note This is a legacy global definition and may differ from the private one inside the `Injector` class.
 */
enum HomingPhase : uint8_t {
	HOMING_PHASE_IDLE_GLOBAL,   ///< Homing is not active.
	HOMING_PHASE_STARTING_MOVE, ///< Initial delay to confirm motor movement has begun.
	HOMING_PHASE_RAPID_MOVE,    ///< High-speed move towards the hard stop.
	HOMING_PHASE_BACK_OFF,      ///< Moving away from the hard stop after initial contact.
	HOMING_PHASE_TOUCH_OFF,     ///< Slow-speed move to precisely locate the hard stop.
	HOMING_PHASE_RETRACT,       ///< Moving to a final, safe position after homing is complete.
	HOMING_PHASE_COMPLETE,      ///< The homing sequence finished successfully.
	HOMING_PHASE_ERROR_GLOBAL   ///< The homing sequence failed.
};

/**
 * @enum FeedState
 * @brief Defines the state of an injection or material feed operation.
 * @details This enum tracks the progress of a dispensing action, from starting
 * to pausing, resuming, and completing.
 */
enum FeedState : uint8_t {
	FEED_NONE,                  ///< No feed operation is active or defined.
	FEED_STANDBY,               ///< Ready to start a feed operation.
	FEED_INJECT_STARTING,       ///< An injection move has been commanded and is starting.
	FEED_INJECT_ACTIVE,         ///< An injection move is currently in progress.
	FEED_INJECT_PAUSED,         ///< An active injection has been paused by the user.
	FEED_INJECT_RESUMING,       ///< A paused injection is resuming.
	FEED_MOVING_TO_HOME,        ///< Moving to the previously established cartridge home position.
	FEED_MOVING_TO_RETRACT,     ///< Moving to a retracted position relative to the cartridge home.
	FEED_INJECTION_CANCELLED,   ///< The injection was cancelled by the user.
	FEED_INJECTION_COMPLETED    ///< The injection finished successfully.
};

/**
 * @enum MoveState
 * @brief Defines the state of a general move operation (move_abs / move_inc).
 * @details This enum tracks the progress of a non-injection move, from starting
 * to pausing, resuming, and completing. Ported from Pressboi MotorController.
 */
enum MoveState : uint8_t {
	MOVE_NONE,                  ///< No move operation is active or defined.
	MOVE_STANDBY,               ///< Ready to start a move operation.
	MOVE_STARTING,              ///< A move has been commanded and is starting.
	MOVE_ACTIVE,                ///< A move is currently in progress.
	MOVE_PAUSED,                ///< An active move has been paused by the user.
	MOVE_RESUMING,              ///< A paused move is resuming.
	MOVE_TO_HOME,               ///< Moving to the previously established start position.
	MOVE_TO_RETRACT,            ///< Moving to a retracted position relative to the start.
	MOVE_CANCELLED,             ///< The move was cancelled by the user.
	MOVE_COMPLETED              ///< The move finished successfully.
};

/**
 * @enum EnableState
 * @brief Defines the state of a non-blocking motor enable operation.
 * @details Used to track motor enable status without blocking the main loop.
 * Ported from Pressboi MotorController.
 */
enum EnableState : uint8_t {
	ENABLE_IDLE,                ///< No enable operation in progress.
	ENABLE_WAITING,             ///< Waiting for motors to report enabled status.
	ENABLE_COMPLETE,            ///< Motors have successfully enabled.
	ENABLE_TIMEOUT              ///< Motors did not enable within timeout period.
};


/**
 * @class Injector
 * @brief Manages the dual-motor injector system for dispensing material and general moves.
 *
 * @details This class is the core of the material dispensing functionality and also supports
 * general-purpose absolute/incremental moves. It orchestrates the two ganged motors (M0 and M1)
 * to perform complex, multi-stage operations.
 * Key responsibilities include:
 * - A hierarchical state machine for managing overall state (e.g., STANDBY, HOMING, FEEDING, MOVING).
 * - Nested state machines for detailed processes like the multi-phase homing sequence.
 * - Torque-based sensing for detecting hard stops during homing and stall conditions.
 * - Volume-based control of injections, converting mL to motor steps.
 * - Distance-based control of general moves (absolute/incremental).
 * - Force sensing with configurable modes (motor torque or load cell).
 * - Gantry-squaring homing with independent axis hall-effect sensors.
 * - Handling user commands for jogging, homing, injecting, moving, and pausing/resuming operations.
 * - Reporting detailed telemetry on position, torque, force, and operational status.
 */
class Injector {
public:
    // ── Construction & Lifecycle ──────────────────────────────────────────

    /**
     * @brief Constructs a new Injector object.
     * @param motorA Pointer to the primary ClearCore motor driver (M0).
     * @param motorB Pointer to the secondary (ganged) ClearCore motor driver (M1).
     * @param controller Pointer to the main `Fillhead` controller, used for reporting events.
     */
    Injector(MotorDriver* motorA, MotorDriver* motorB, Fillhead* controller);

    /**
     * @brief Initializes the injector motors and their configurations.
     * @details Configures motor settings such as max velocity and acceleration, and enables
     * the motor drivers. This should be called once at startup.
     */
    void setup();

    /**
     * @brief Updates the internal state machines for the injector.
     * @details This is the heart of the injector's non-blocking operation. It should be
     * called repeatedly in the main application loop to advance the active state machines
     * for homing, feeding, moving, and jogging operations.
     */
    void updateState();

    /**
     * @brief Handles user commands related to the injector.
     * @param cmd The `Command` enum representing the command to be executed.
     * @param args A C-style string containing any arguments for the command (e.g., distance, volume).
     */
    void handleCommand(Command cmd, const char* args);

    /**
     * @brief Gets the telemetry string for the injector system.
     * @return A constant character pointer to a formatted string containing key-value pairs
     *         of injector telemetry data (position, torque, homing status, etc.).
     */
    const char* getTelemetryString();

    // ── Enable / Disable ─────────────────────────────────────────────────

    /**
     * @brief Enables the injector motors and sets their default parameters (non-blocking).
     * @details Requests motor enable and starts the enable state machine.
     * Call updateEnableState() to check progress.
     */
    void enable();

    /**
     * @brief Disables the injector motors.
     */
    void disable();

    /**
     * @brief Updates the non-blocking enable state machine.
     * @return The current EnableState after the update.
     */
    EnableState updateEnableState();

    /**
     * @brief Checks if the enable operation has completed (success or timeout).
     * @return true if enable is complete or timed out, false if still waiting.
     */
    bool isEnableComplete() const;

    /**
     * @brief Gets the current enable state.
     * @return The current EnableState value.
     */
    EnableState getEnableState() const { return m_enableState; }

    // ── Motion Control ───────────────────────────────────────────────────

    /**
     * @brief Aborts any ongoing injector motion by commanding a deceleration stop.
     */
    void abortMove();

    /**
     * @brief Resets the injector's state machines and error conditions to their default idle state.
     */
    void reset();

    /**
     * @brief Pauses an active general move operation (for move_abs / move_inc).
     */
    void pauseGeneralMove();

    /**
     * @brief Resumes a paused general move operation (for move_abs / move_inc).
     */
    void resumeGeneralMove();

    /**
     * @brief Cancels an active general move operation (for move_abs / move_inc).
     */
    void cancelGeneralMove();

    // ── Status Queries ───────────────────────────────────────────────────

    /**
     * @brief Checks if either of the injector motors is in a hardware fault state.
     * @return `true` if a motor is in fault, `false` otherwise.
     */
    bool isInFault() const;

    /**
     * @brief Checks if the injector is busy with any operation.
     * @return `true` if the injector's main state is not `STATE_STANDBY`.
     */
    bool isBusy() const;

    /**
     * @brief Gets the current high-level state of the injector as a human-readable string.
     * @return A `const char*` representing the current state (e.g., "Homing", "Feeding", "Moving").
     */
    const char* getState() const;

    // ── Force / Calibration ──────────────────────────────────────────────

    /**
     * @brief Sets the force sensing mode and saves to NVM.
     * @param mode "motor_torque" or "load_cell"
     * @return true if successful, false if invalid mode
     */
    bool setForceMode(const char* mode);

    /**
     * @brief Gets the current force sensing mode.
     * @return The current force mode string
     */
    const char* getForceMode() const;

    /**
     * @brief Sets the calibration offset for the current force mode.
     * @param offset Offset value (kg for load_cell, torque% intercept for motor_torque)
     */
    void setForceCalibrationOffset(float offset);

    /**
     * @brief Sets the calibration scale for the current force mode.
     * @param scale Scale/slope value (kg/raw for load_cell, torque%/kg for motor_torque)
     */
    void setForceCalibrationScale(float scale);

    /**
     * @brief Gets the calibration offset for the current force mode.
     * @return Current offset value
     */
    float getForceCalibrationOffset() const;

    /**
     * @brief Gets the calibration scale for the current force mode.
     * @return Current scale value
     */
    float getForceCalibrationScale() const;

    // ── Polarity ─────────────────────────────────────────────────────────

    /**
     * @brief Sets the coordinate system polarity and saves to NVM.
     * @param polarity "normal" or "inverted"
     * @return true if successful, false if invalid polarity
     */
    bool setPolarity(const char* polarity);

    /**
     * @brief Gets the current coordinate system polarity.
     * @return The current polarity string ("normal" or "inverted")
     */
    const char* getPolarity() const;

    // ── Home on Boot ─────────────────────────────────────────────────────

    /**
     * @brief Sets the home on boot setting and saves to NVM.
     * @param enabled "true" or "false"
     * @return true if successful, false if invalid parameter
     */
    bool setHomeOnBoot(const char* enabled);

    /**
     * @brief Gets the current home on boot setting.
     * @return true if home on boot is enabled, false otherwise
     */
    bool getHomeOnBoot() const;

    // ── Press Threshold ──────────────────────────────────────────────────

    /**
     * @brief Sets the press force threshold and saves to NVM.
     * @param threshold_kg Threshold force in kg (0.1 to 50.0)
     * @return true if successful, false if out of range
     */
    bool setPressThreshold(float threshold_kg);

    /**
     * @brief Gets the current press force threshold.
     * @return Press threshold in kg
     */
    float getPressThreshold() const { return m_press_threshold_kg; }

    /**
     * @brief Gets the position where press threshold was crossed.
     * @return Startpoint position in mm
     */
    float getPressStartpoint() const { return m_press_startpoint_mm; }

    // ── Hall-Effect Home Sensors ─────────────────────────────────────────

    /**
     * @brief Gets the current state of Motor A (M0) home sensor.
     * @return true if sensor is triggered (active), false otherwise
     */
    bool getHomeSensorStateM0() const;

    /**
     * @brief Gets the current state of Motor B (M1) home sensor.
     * @return true if sensor is triggered (active), false otherwise
     */
    bool getHomeSensorStateM1() const;

    // ── Machine Strain Compensation ──────────────────────────────────────

    /**
     * @brief Sets machine strain compensation coefficients and saves to NVM.
     * @param x4 Coefficient for x^4 term
     * @param x3 Coefficient for x^3 term
     * @param x2 Coefficient for x^2 term
     * @param x1 Coefficient for x^1 term
     * @param c  Constant term
     */
    void setMachineStrainCoeffs(float x4, float x3, float x2, float x1, float c);

private:
    // ── Private Helper Methods ───────────────────────────────────────────
    /** @name Private Helper Methods @{ */

    void startMove(long steps, int velSps, int accelSps2);
    void startMoveAxis(int axis, long steps, int velSps, int accelSps2);
    void stopAxis(int axis);
    bool isMoving();
    bool isAxisMoving(int axis);
    float getSmoothedTorque(MotorDriver *motor, float *smoothedValue, bool *firstRead);
    bool checkTorqueLimit();
    bool checkForceSensorStatus(const char** errorMsg);
    void handleLimitReached(const char* limit_type, float limit_value);
    void handleFeedLimitReached(const char* limit_type, float limit_value);
    void finalizeAndResetActiveDispenseOperation(bool success);
    void fullyResetActiveDispenseOperation();
    void finalizeAndResetActiveMove(bool success);
    void fullyResetActiveMove();
    void updateJoules();
    void reportEvent(const char* statusType, const char* message);

    void setupHomeSensors();
    bool isHomeSensorTriggered(int axis);
    bool getHomeSensorState(int axis);

    float evaluateMachineStrainForceFromDeflection(float deflection_mm) const;
    float estimateMachineDeflectionFromForce(float force_kg) const;

    /** @} */

    // ── Private Command Handlers ─────────────────────────────────────────
    /** @name Private Command Handlers @{ */

    void home();
    void cartridgeHome();
    void moveToCartridgeHome();
    void moveToCartridgeRetract(const char* args);
    void initiateInjectMove(const char* args);
    void setCartridgeMlPerMm(const char* args);
    void pauseOperation();
    void resumeOperation();
    void cancelOperation();
    void setTorqueOffset(const char* args);

    void moveAbsolute(const char* args);
    void moveIncremental(const char* args);
    void setRetract(const char* args);
    void retract(const char* args);

    /** @} */

    // ── Core References ──────────────────────────────────────────────────

    Fillhead* m_controller;      ///< Pointer to the main `Fillhead` controller for event reporting.
    MotorDriver* m_motorA;       ///< Pointer to the primary motor driver (M0).
    MotorDriver* m_motorB;       ///< Pointer to the secondary, ganged motor driver (M1).

    // ── Top-Level State Machine ──────────────────────────────────────────

    /**
     * @enum State
     * @brief Defines the top-level operational state of the injector.
     */
    typedef enum {
        STATE_STANDBY,      ///< Injector is idle and ready for commands.
        STATE_HOMING,       ///< Injector is performing a homing sequence.
        STATE_FEEDING,      ///< Injector is performing an injection or retract move.
        STATE_MOVING,       ///< Injector is performing a general move_abs/move_inc operation.
        STATE_MOTOR_FAULT   ///< An injector motor has entered a hardware fault state.
    } State;
    State m_state; ///< The current top-level state of the injector.

    HomingState m_homingState; ///< The type of homing being performed (Machine vs. Cartridge).

    // ── Homing Phase State Machine (Gantry-Squaring) ─────────────────────

    /**
     * @enum HomingPhase
     * @brief Defines the detailed sub-states for the gantry-squaring homing sequence.
     * @details This state machine supports independent axis control with hall effect sensors.
     * Each axis can stop independently when its sensor triggers, enabling gantry squaring.
     */
    typedef enum {
        HOMING_PHASE_IDLE,              ///< Homing is not active.
        RAPID_APPROACH_START,           ///< Starting rapid approach move toward home sensors.
        RAPID_APPROACH_WAIT_TO_START,   ///< Waiting for rapid move to begin.
        RAPID_APPROACH_MOVING,          ///< Executing rapid move, monitoring sensors independently.
        BACKOFF_START,                  ///< Starting backoff move away from sensors.
        BACKOFF_WAIT_TO_START,          ///< Waiting for backoff move to begin.
        BACKOFF_MOVING,                 ///< Executing backoff move.
        SLOW_APPROACH_START,            ///< Starting slow approach for precision homing.
        SLOW_APPROACH_WAIT_TO_START,    ///< Waiting for slow move to begin.
        SLOW_APPROACH_MOVING,           ///< Executing slow move, monitoring sensors independently.
        FINAL_BACKOFF_START,            ///< Starting final backoff to offset position.
        FINAL_BACKOFF_WAIT_TO_START,    ///< Waiting for final backoff to begin.
        FINAL_BACKOFF_MOVING,           ///< Executing final backoff move.
        SET_ZERO,                       ///< Setting the final position as the logical zero.
        HOMING_PHASE_ERROR              ///< Homing sequence failed.
    } HomingPhase;
    HomingPhase m_homingPhase; ///< The current phase of an active homing sequence.

    // ── Feed & Move Sub-State Machines ───────────────────────────────────

    FeedState m_feedState;             ///< The current state of an injection or feed operation.
    MoveState m_moveState;             ///< The current state of a general move operation.

    // ── Homing Flags ─────────────────────────────────────────────────────

    bool m_homingMachineDone;          ///< Flag indicating if machine homing has been successfully completed.
    bool m_homingCartridgeDone;        ///< Flag indicating if cartridge homing has been successfully completed.
    uint32_t m_homingStartTime;        ///< Timestamp (ms) when the homing sequence started, used for timeout.

    // ── Gantry Squaring Homing Variables ─────────────────────────────────
    /** @name Gantry Squaring Homing Variables @{ */

    bool m_homeSensorsInitialized;     ///< Flag indicating home sensors have been configured.
    bool m_axisAHomeSensorTriggered;   ///< M0 home sensor has been triggered in current phase.
    bool m_axisBHomeSensorTriggered;   ///< M1 home sensor has been triggered in current phase.
    bool m_axisAStopped;               ///< M0 has been stopped in current homing phase.
    bool m_axisBStopped;               ///< M1 has been stopped in current homing phase.

    /** @} */

    // ── Non-Blocking Enable State ────────────────────────────────────────
    /** @name Non-Blocking Enable State Variables @{ */

    EnableState m_enableState;         ///< Current state of the non-blocking enable operation.
    uint32_t m_enableStartTime;        ///< Timestamp (ms) when enable was requested.

    /** @} */

    // ── Motor / Enable State ─────────────────────────────────────────────

    bool m_isEnabled;                  ///< Flag indicating if motors are currently enabled.
    bool m_retractDone;                ///< Flag indicating if retract position has been set.
    bool m_pausedMessageSent;          ///< Flag to prevent spamming "paused" messages.

    // ── Torque & Force Sensing ───────────────────────────────────────────

    float m_torqueLimit;               ///< Current torque limit (%) for detecting hard stops or stalls.
    float m_torqueOffset;              ///< User-configurable offset (%) for torque readings to account for bias.
    float m_smoothedTorqueValue0, m_smoothedTorqueValue1; ///< Smoothed torque values for each motor.
    bool m_firstTorqueReading0, m_firstTorqueReading1;   ///< Flags for initializing the torque smoothing EWMA filter.
    char m_force_mode[16];             ///< Persistent force sensing mode: "motor_torque" or "load_cell" (stored in NVM).
    float m_motor_torque_offset;       ///< Motor torque equation offset (stored in NVM, default 1.04).
    float m_motor_torque_scale;        ///< Motor torque equation scale (stored in NVM, default 0.0335).

    // ── Polarity & Boot Settings ─────────────────────────────────────────

    char m_polarity[16];               ///< Coordinate system polarity: "normal" or "inverted" (stored in NVM).
    bool m_home_on_boot;               ///< Home on boot setting: true = auto-home, false = skip (stored in NVM).

    // ── Reference Positions ──────────────────────────────────────────────

    int32_t m_machineHomeReferenceSteps;     ///< Stored step count for machine home position.
    int32_t m_cartridgeHomeReferenceSteps;   ///< Stored step count for cartridge home position.
    int32_t m_retractReferenceSteps;         ///< Stored step count for retract position.
    float m_retract_position_mm;             ///< Retract position in mm (stored in NVM, default 0.0).
    float m_press_threshold_kg;              ///< Force threshold (kg) for energy/startpoint recording (stored in NVM).

    // ── Cumulative Tracking ──────────────────────────────────────────────

    float m_cumulative_dispensed_ml;   ///< Cumulative dispensed volume (mL) since the last cartridge home.
    float m_cumulative_distance_mm;    ///< Cumulative distance traveled (mm) since the last home.

    // ── Feed (Injection) Default Parameters ──────────────────────────────

    float m_cartridge_ml_per_mm;       ///< NVM-persisted cartridge ratio (mL per mm of piston travel).
    int m_feedDefaultTorquePercent;    ///< Default torque (%) for feed moves.
    long m_feedDefaultVelocitySPS;     ///< Default velocity (steps/sec) for feed moves.
    long m_feedDefaultAccelSPS2;       ///< Default acceleration (steps/sec^2) for feed moves.

    // ── General Move Default Parameters ──────────────────────────────────

    int m_moveDefaultTorquePercent;    ///< Default torque (%) for general moves.
    long m_moveDefaultVelocitySPS;     ///< Default velocity (steps/sec) for general moves.
    long m_moveDefaultAccelSPS2;       ///< Default acceleration (steps/sec^2) for general moves.

    // ── Homing Parameters ────────────────────────────────────────────────

    long m_homingDistanceSteps;        ///< Max travel distance (steps) for a homing move.
    long m_homingBackoffSteps;         ///< Backoff distance (steps) for a homing move.
    int m_homingRapidSps;              ///< Rapid speed (steps/sec) for a homing move.
    int m_homingTouchSps;              ///< Slow touch-off speed (steps/sec) for a homing move.
    int m_homingBackoffSps;            ///< Backoff speed (steps/sec) for a homing move.
    int m_homingAccelSps2;             ///< Acceleration (steps/sec^2) for homing moves.

    // ── Command Tracking ─────────────────────────────────────────────────

    const char* m_activeFeedCommand;   ///< Stores the original feed command string for logging upon completion.
    const char* m_activeMoveCommand;   ///< Stores the original move command string for logging upon completion.
    const char* m_originalMoveCommand; ///< Stores the very first command before any retract substitution.

    // ── Active Dispense Operation Variables (Injection) ──────────────────
    /** @name Active Dispense Operation Variables @{ */

    float m_active_op_target_ml;            ///< Target dispense volume in mL for the current operation.
    float m_active_op_total_dispensed_ml;   ///< Total volume dispensed so far in the current operation.
    float m_last_completed_dispense_ml;     ///< The volume of the last successful dispense operation.
    float m_active_op_steps_per_ml;         ///< Conversion factor from steps to mL for the current operation.
    long m_active_op_total_target_steps;    ///< Target dispense distance in steps for the current operation.
    long m_active_op_remaining_steps;       ///< Remaining steps in the current operation (used for pause/resume).
    long m_active_op_segment_initial_axis_steps; ///< Start position for a move segment (used for pause/resume).
    long m_active_op_initial_axis_steps;    ///< Start position for the entire operation.
    int m_active_op_velocity_sps;           ///< Velocity (steps/sec) for the current operation.
    int m_active_op_accel_sps2;             ///< Acceleration (steps/sec^2) for the current operation.
    int m_active_op_torque_percent;         ///< Torque limit (%) for the current operation.
    float m_active_op_feed_force_limit_kg;  ///< Force limit (kg) for injection (STATE_FEEDING), 0 = disabled.
    char m_active_op_feed_force_action[16]; ///< Action on feed force limit: "retract", "hold", "skip", "abort".
    uint32_t m_feedStartTime;               ///< Timestamp (ms) when a feed operation started.

    /** @} */

    // ── Active Move Operation Variables (General) ────────────────────────
    /** @name Active Move Operation Variables @{ */

    float m_active_op_force_limit_kg;       ///< Target force limit (kg) for force-based moves.
    char m_active_op_force_action[16];      ///< Action to take when force limit reached: "retract", "hold", "skip".
    char m_active_op_force_mode[16];        ///< Limit mode: "force" (load cell) or "torque" (motor torque).
    float m_active_op_total_distance_mm;    ///< Total distance traveled (mm) in current operation.
    float m_last_completed_distance_mm;     ///< Distance (mm) of the last completed move operation.
    long m_active_op_target_position_steps; ///< Absolute target position in steps for the current operation.
    uint32_t m_moveStartTime;               ///< Timestamp (ms) when a move operation started.
    double m_joules;                        ///< Energy expended (Joules) during current move, integrated at 50Hz.
    float m_endpoint_mm;                    ///< Actual position (mm) where last move ended (force trigger or completion).
    float m_press_startpoint_mm;            ///< Position (mm) where force threshold was crossed (press started).
    double m_prev_position_mm;              ///< Previous position (mm) for joule integration calculations.

    /** @} */

    // ── Machine Strain Compensation ──────────────────────────────────────
    /** @name Machine Strain Compensation Variables @{ */

    double m_machineStrainBaselinePosMm;    ///< Absolute position (mm) that defines zero deflection for strain compensation.
    double m_prevMachineDeflectionMm;       ///< Estimated machine flex deflection (mm) at previous sample.
    double m_prevTotalDeflectionMm;         ///< Cumulative machine deflection at previous sample.
    double m_machineEnergyJ;                ///< Accumulated machine-flex energy (Joules) during current move.
    bool m_machineStrainContactActive;      ///< Indicates whether force threshold has been reached and flex compensation is active.
    bool m_jouleIntegrationActive;          ///< Flag indicating whether joule integration is currently active.
    bool m_forceLimitTriggered;             ///< Tracks whether the current move has hit the force limit.
    float m_machineStrainCoeffs[5];         ///< Machine strain compensation coefficients [x^4, x^3, x^2, x, constant].
    float m_prevForceKg;                    ///< Previous force sample (kg) used for joule integration.
    bool m_prevForceValid;                  ///< Indicates whether previous force sample is valid.

    /** @} */

    // ── Retract & Misc ───────────────────────────────────────────────────

    float m_retractSpeedMms;                ///< Stored retract speed to use when not overridden.

    // ── Telemetry Buffer ─────────────────────────────────────────────────

    char m_telemetryBuffer[256]; ///< Buffer for the formatted telemetry string.
};
