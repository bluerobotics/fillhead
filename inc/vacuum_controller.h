/**
 * @file vacuum_controller.h
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Defines the controller for the vacuum system.
 *
 * @details This file defines the `VacuumController` class, which is responsible for
 * managing the vacuum pump, monitoring pressure, and executing automated sequences
 * like leak tests. It includes a state machine to handle the various stages of
 * vacuum operation.
 */
#pragma once

#include "config.h"
#include "comms_controller.h"
#include "commands.h"

class Fillhead; // Forward declaration

/**
 * @enum VacuumState
 * @brief Defines the operational state of the vacuum system.
 */
enum VacuumState : uint8_t {
	VACUUM_OFF,             ///< 0 — The vacuum pump and valve are off. The system is idle.
	VACUUM_PULLDOWN,        ///< 1 — The pump is on, pulling down to the target pressure for a leak test.
	VACUUM_SETTLING,        ///< 2 — The pump is off, allowing pressure to stabilize before a leak test.
	VACUUM_LEAK_TESTING,    ///< 3 — The system is holding vacuum and monitoring for pressure changes to detect leaks.
	VACUUM_ON,              ///< 4 — The pump is on and will run continuously until a `VACUUM_OFF` command is received.
	VACUUM_RAMP,            ///< 5 — The pump is on, ramping to a user-specified target pressure, then auto-off.
	VACUUM_ERROR            ///< 6 — The system failed to reach target pressure or failed a leak test.
};

/**
 * @class VacuumController
 * @brief Manages and controls the vacuum system of the fillhead.
 *
 * @details This class encapsulates all functionality related to the vacuum system.
 * It manages the vacuum pump relay, reads the pressure transducer, and implements
 * a state machine for operations like pulling down to a target pressure, holding
 * that pressure, and performing automated leak tests.
 */
class VacuumController {
	public:
	/**
	 * @brief Constructs a new VacuumController object.
	 * @param controller A pointer to the main `Fillhead` controller, used for reporting events.
	 */
	VacuumController(Fillhead* controller);

	/**
	 * @brief Initializes the hardware for the vacuum controller.
	 * @details This method configures the GPIO pin for the vacuum pump relay as an output
	 * and the analog input pin for the pressure transducer. It should be called once at startup.
	 */
	void setup();

	/**
	 * @brief Reads and updates the current vacuum pressure reading.
	 * @details This method should be called periodically. It reads the raw analog voltage
	 * from the pressure transducer, converts it to PSIG based on calibration values
	 * in `config.h`, and applies an Exponentially Weighted Moving Average (EWMA) filter
	 * to smooth the reading.
	 */
	void updateVacuum();

	/**
	 * @brief Manages the state machine for vacuum control.
	 * @details This method should be called periodically in the main loop. It is the core
	 * of the vacuum system's operation, transitioning between states like `VACUUM_PULLDOWN`
	 * and `VACUUM_LEAK_TESTING` based on sensor readings and timers.
	 */
	void updateState();

	/**
	 * @brief Handles user commands related to the vacuum system.
	 * @param cmd The `Command` enum representing the command to be executed.
	 * @param args A C-style string containing any arguments for the command (e.g., target pressure).
	 */
	void handleCommand(Command cmd, const char* args);

	/**
	 * @brief Gets the telemetry string for the vacuum system.
	 * @return A constant character pointer to a formatted string containing key-value pairs
	 *         of vacuum system telemetry data (e.g., pressure, state, target).
	 */
	const char* getTelemetryString();

	/**
	 * @brief Resets the state of the vacuum controller to its initial, idle state.
	 * @details This will turn off the vacuum pump and reset any ongoing operations like
	 * a leak test.
	 */
	void resetState();

	/**
	 * @brief Checks if the vacuum controller is busy with an operation.
	 * @return `true` if the controller is in any state other than `VACUUM_OFF` or `VACUUM_ERROR`.
	 */
	bool isBusy() const;

	/**
	 * @brief Gets the current state of the vacuum system as a human-readable string.
	 * @return A `const char*` representing the current state (e.g., "VACUUM_PULLDOWN").
	 */
	const char* getState() const;

	private:
	/**
	 * @brief Formats and sends a status message via the main `Fillhead` controller.
	 * @param statusType The prefix for the message (e.g., "INFO: ").
	 * @param message The content of the message to send.
	 */
	void reportEvent(const char* statusType, const char* message);
	
	Fillhead* m_controller;             ///< Pointer to the main `Fillhead` controller for sending messages.
	VacuumState m_state;                ///< The current operational state of the vacuum system.
	float m_vacuumPressurePsig;         ///< The raw vacuum pressure reading from the sensor in PSIG.
	float m_smoothedVacuumPsig;         ///< The smoothed (EWMA) vacuum pressure in PSIG.
	bool m_firstVacuumReading;          ///< Flag to handle the first reading for the smoothing filter.
	float m_targetPsig;                 ///< The target vacuum pressure for pulldown and hold states in PSIG.
	float m_rampTimeoutSec;             ///< The timeout in seconds for reaching the target pressure during pulldown.
	float m_leakTestDeltaPsig;          ///< The maximum allowed pressure increase (less negative) during a leak test in PSIG.
	float m_leakTestDurationSec;        ///< The duration of the leak test in seconds.
	uint32_t m_stateStartTimeMs;        ///< Timestamp (in milliseconds) when the current state was entered.
	float m_leakTestStartPressure;      ///< The pressure recorded at the beginning of a leak test.
	char m_telemetryBuffer[256];        ///< A buffer to store the formatted telemetry string.

	/**
	 * @brief Activates the vacuum pump relay, optionally ramping to a target pressure.
	 * @param args Optional target pressure in PSIG. If NULL/empty, pump runs indefinitely.
	 */
	void vacuumOn(const char* args);

	/**
	 * @brief Deactivates the vacuum pump relay.
	 */
	void vacuumOff();

	/**
	 * @brief Initiates a leak test sequence by transitioning to the `VACUUM_PULLDOWN` state.
	 * @param args Optional delta and duration overrides (e.g., "0.5 10").
	 */
	void leakTest(const char* args);

	/**
	 * @brief Sets the target vacuum pressure from a string argument.
	 * @param args A C-style string containing the new target pressure in PSIG.
	 */
	void setTarget(const char* args);

	/**
	 * @brief Sets the pulldown timeout from a string argument.
	 * @param args A C-style string containing the new timeout in seconds.
	 */
	void setTimeout(const char* args);

	/**
	 * @brief Sets the leak test delta pressure from a string argument.
	 * @param args A C-style string containing the new maximum allowed pressure change in PSIG.
	 */
	void setLeakDelta(const char* args);

	/**
	 * @brief Sets the leak test duration from a string argument.
	 * @param args A C-style string containing the new duration in seconds.
	 */
	void setLeakDuration(const char* args);
};
