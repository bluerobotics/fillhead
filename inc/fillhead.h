/**
 * @file fillhead.h
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Defines the master controller for the entire Fillhead system.
 *
 * @details This header file defines the `Fillhead` class, which serves as the central
 * orchestrator for all Fillhead operations. It owns and manages all specialized
 * sub-controllers (Injector, PinchValves, Heater, Vacuum) and is responsible for
 * the main application state machine, command dispatch, and telemetry reporting.
 * The top-level system state enums are also defined here.
 */
#pragma once

#include "config.h"
#include "responses.h"
#include "comms_controller.h"
#include "injector_controller.h"
#include "pinch_valve_controller.h"
#include "heater_controller.h"
#include "vacuum_controller.h"

/**
 * @enum MainState
 * @brief Defines the top-level operational state of the entire Fillhead system.
 * @details This enum represents the high-level status of the machine. The state is
 * determined by the combined status of all sub-controllers.
 */
enum MainState : uint8_t {
	STATE_STANDBY,       ///< System is idle, initialized, and ready to accept commands.
	STATE_BUSY,          ///< A non-error operation (e.g., homing, injecting) is in progress.
	STATE_ERROR,         ///< A fault has occurred, typically a motor fault. Requires `CLEAR_ERRORS` to recover.
	STATE_DISABLED,      ///< System is disabled; motors will not move. Requires `ENABLE` to recover.
	STATE_CLEARING_ERRORS///< A special state to manage the non-blocking error recovery process.
};

/**
 * @enum ErrorState
 * @brief Defines various specific error conditions the system can encounter.
 * @details This enum is intended for more granular error reporting in the future, though
 * it is not fully implemented yet.
 */
enum ErrorState : uint8_t {
	ERROR_NONE,                   ///< No error.
	ERROR_MANUAL_ABORT,           ///< Operation aborted by user command.
	ERROR_TORQUE_ABORT,           ///< Operation aborted due to exceeding torque limits.
	ERROR_MOTION_EXCEEDED_ABORT,  ///< Operation aborted because motion limits were exceeded.
	ERROR_NO_CARTRIDGE_HOME,      ///< A required cartridge home position is not set.
	ERROR_NO_MACHINE_HOME,        ///< A required machine home position is not set.
	ERROR_HOMING_TIMEOUT,         ///< Homing operation took too long to complete.
	ERROR_HOMING_NO_TORQUE_RAPID, ///< Homing failed to detect torque during the rapid move.
	ERROR_HOMING_NO_TORQUE_TOUCH, ///< Homing failed to detect torque during the touch-off move.
	ERROR_INVALID_INJECTION,      ///< An injection command was invalid.
	ERROR_NOT_HOMED,              ///< An operation required homing, but the system is not homed.
	ERROR_INVALID_PARAMETERS,     ///< A command was received with invalid parameters.
	ERROR_MOTORS_DISABLED         ///< An operation was blocked because motors are disabled.
};

/**
 * @class Fillhead
 * @brief The master controller for the Fillhead system.
 * @details This class acts as the "brain" of the Fillhead. It follows the Singleton
 * pattern (instantiated once globally) and is responsible for the entire application
 * lifecycle, from setup to the continuous execution of the main loop. It owns all
 * sub-controller objects and delegates commands to them.
 */
class Fillhead {
public:
    /**
     * @brief Constructs the Fillhead master controller.
     * @details The constructor initializes all member variables and uses a member
     * initializer list to instantiate all the specialized sub-controllers, ensuring
     * they are ready before `setup()` is called.
     */
    Fillhead();

    /**
     * @brief Initializes all hardware and sub-controllers for the entire system.
     * @details This method should be called once at startup. It sequentially calls the
     * `setup()` method for each component in the correct order to ensure proper
     * hardware and software initialization.
     */
    void setup();

    /**
     * @brief The main execution loop for the Fillhead system.
     * @details This function is called continuously from `main()`. It orchestrates all
     * real-time operations in a non-blocking manner, including processing communication
     * queues, dispatching commands, updating state machines, and managing periodic tasks.
     */
    void loop();

    /**
     * @brief Public interface for sub-controllers to send status messages.
     * @details This wrapper method allows owned objects (like `Injector` or `HeaterController`)
     * to send status messages (e.g., INFO, DONE, ERROR) through the `CommsController`
     * without needing a direct pointer to it, reducing coupling.
     * @param statusType The prefix for the message (e.g., "INFO: "). See `config.h`.
     * @param message The content of the message to send.
     */
    void reportEvent(const char* statusType, const char* message);

private:
    /**
     * @brief Updates the main system state and the state machines of all sub-controllers.
     * @details This function is called once per loop. It aggregates the status of all
     * sub-controllers (e.g., checking for faults or busy states) to determine the
     * overall system `MainState`. It then calls the `updateState()` method for each
     * sub-controller to advance their individual state machines.
     */
    void updateState();

    /**
     * @brief Performs safety checks and, when enabled, participates in watchdog feeding.
     * @details Currently a placeholder for future safety logic. Mirrors the role of
     *          Pressboi's performSafetyCheck() so watchdog integration remains similar.
     */
    void performSafetyCheck();

	/**
	 * @brief Master command handler; dispatches incoming commands to the correct sub-system.
	 * @details This function acts as a switchboard. It takes a message from the
	 * communications queue, uses the `CommsController` to parse it into a `Command` enum,
	 * and then calls the appropriate handler function or delegates the command to the
	 * relevant sub-controller.
	 * @param msg The incoming message object containing the command to be executed.
	 */
    void dispatchCommand(const Message& msg);

	/**
	 * @brief Aggregates telemetry data from all sub-controllers and sends it as a single packet.
	 * @details This function is called periodically. It polls each sub-controller for its
	 * latest telemetry data, assembles it into a single formatted string, and enqueues
	 * it for transmission by the `CommsController`.
	 */
    void publishTelemetry();

    // --- System-Level Command Handlers ---
    /**
     * @brief Enables all motors and places the system in a ready state.
     */
    void enable();

    /**
     * @brief Disables all motors and stops all operations.
     */
    void disable();

    /**
     * @brief Halts all motion immediately and resets the system state to standby.
     */
    void abort();

    /**
     * @brief Resets any error states, clears motor faults, and returns the system to standby.
     */
    void clearErrors();

    /**
     * @brief Resets all sub-controllers to their idle states and sets the main state to STANDBY.
     */
    void standby();

#if WATCHDOG_ENABLED
    /**
     * @brief Checks for watchdog recovery after motor setup.
     * @details Detects if the system recovered from a watchdog timeout and moves to a safe state.
     */
    void handleWatchdogRecovery();

    /**
     * @brief Initializes the watchdog timer with early warning interrupt.
     */
    void initializeWatchdog();

    /**
     * @brief Feeds (clears/resets) the watchdog timer.
     * @details This must be called regularly from the main loop to prevent watchdog reset.
     */
    void feedWatchdog();
#endif

    // --- Component Ownership ---
    CommsController  m_comms;           ///< Manages all network and serial communication.
    Injector         m_injector;        ///< Manages the dual-motor dispensing system.
    PinchValve       m_injectorValve;   ///< Manages the motorized injection pinch valve.
    PinchValve       m_vacuumValve;     ///< Manages the motorized vacuum pinch valve.
    HeaterController m_heater;          ///< Manages the heater PID control loop.
    VacuumController m_vacuum;          ///< Manages the vacuum pump and pressure monitoring.

    // Main system state machine
    MainState m_mainState;              ///< The current high-level state of the Fillhead system.

    // Timers for periodic tasks
    uint32_t m_lastTelemetryTime;       ///< Timestamp of the last telemetry transmission.
    uint32_t m_lastSensorSampleTime;    ///< Timestamp of the last sensor poll.
};