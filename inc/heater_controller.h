/**
 * @file heater_controller.h
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Defines the controller for the material heating system.
 *
 * @details This file defines the `HeaterController` class, which is responsible for
 * maintaining the material at a target temperature using a PID control loop.
 * It manages the heater relay, reads temperature from a thermocouple, and handles
 * all related user commands.
 */
#pragma once

#include "config.h"
#include "comms_controller.h"
#include "commands.h"

class Fillhead; // Forward declaration

/**
 * @enum HeaterState
 * @brief Defines the operational state of the heater.
 */
enum HeaterState : uint8_t {
	HEATER_OFF,         ///< Heater is off and the PID loop is inactive.
	HEATER_PID_ACTIVE   ///< Heater is on and being actively controlled by the PID loop.
};

/**
 * @class HeaterController
 * @brief Manages the heating element of the fillhead using a PID controller.
 *
 * @details This class encapsulates all functionality related to temperature control.
 * It handles the low-level hardware interface for the heater relay and thermocouple,
 * implements a PID control loop to maintain a precise setpoint, and exposes a
 * command interface for turning the heater on/off and tuning its parameters.
 */
class HeaterController {
	public:
	/**
	 * @brief Constructs a new HeaterController object.
	 * @param controller A pointer to the main `Fillhead` controller, used to send
	 *                   status messages via the `reportEvent` method.
	 */
	HeaterController(Fillhead* controller);

	/**
	 * @brief Initializes the hardware for the heater controller.
	 * @details This method configures the GPIO pin for the heater relay as an output
	 * and the analog input pin for the thermocouple. It should be called once at startup.
	 */
	void setup();

	/**
	 * @brief Reads the current temperature from the thermocouple.
	 * @details This method should be called periodically. It reads the raw analog
	 * voltage from the thermocouple, converts it to degrees Celsius based on the
	 * calibration values in `config.h`, and applies an Exponentially Weighted Moving
	 * Average (EWMA) filter to smooth the reading.
	 */
	void updateTemperature();

	/**
	 * @brief Manages the state machine and executes the PID control loop for the heater.
	 * @details This method is the core of the heater's operation and should be called
	 * repeatedly in the main loop. When the heater state is `HEATER_PID_ACTIVE`,
	 * it periodically runs the PID calculation and uses the output to control the
	 * heater relay via a time-proportioned (software PWM) algorithm.
	 */
	void updateState();

	/**
	 * @brief Handles user commands related to the heater.
	 * @param cmd The `Command` enum representing the command to be executed.
	 * @param args A C-style string containing any arguments required for the command
	 *             (e.g., PID gains, setpoint temperature).
	 */
	void handleCommand(Command cmd, const char* args);

	/**
	 * @brief Gets the telemetry string for the heater system.
	 * @return A constant character pointer to a string containing key-value pairs
	 *         of heater telemetry data, including temperature, setpoint, and state.
	 */
	const char* getTelemetryString();

	/**
	 * @brief Gets the current state of the heater as a human-readable string.
	 * @return A `const char*` representing the current state (e.g., "HEATER_OFF").
	 */
	const char* getState() const;

private:
	Fillhead* m_controller;             ///< Pointer to the main Fillhead controller for sending messages.
	HeaterState m_heaterState;          ///< The current operational state of the heater (ON/OFF).
	float m_temperatureCelsius;         ///< The raw temperature reading from the thermocouple in Celsius.
	float m_smoothedTemperatureCelsius; ///< The smoothed (EWMA) temperature in Celsius.
	bool m_firstTempReading;            ///< Flag to handle the first reading for the smoothing filter.
	float m_pid_setpoint;               ///< The target temperature for the PID controller in Celsius.
	float m_pid_kp;                     ///< The Proportional (P) gain for the PID controller.
	float m_pid_ki;                     ///< The Integral (I) gain for the PID controller.
	float m_pid_kd;                     ///< The Derivative (D) gain for the PID controller.
	float m_pid_integral;               ///< The integral term accumulator for the PID controller.
	float m_pid_last_error;             ///< The previous error value used for the derivative term calculation.
	uint32_t m_pid_last_time;           ///< Timestamp of the last PID update, used to calculate delta time.
	float m_pid_output;                 ///< The output of the PID calculation, representing a duty cycle (0-100).
	float m_fanThresholdCelsius;        ///< Temperature threshold (C) above which the cooling fan activates.
	bool m_fanActive;                   ///< Current state of the cooling fan output.
	char m_telemetryBuffer[256];        ///< A buffer to store the formatted telemetry string.

	/**
	 * @brief Updates the cooling fan output based on the current temperature.
	 * @details Uses Schmitt hysteresis: fan turns on when temperature >= threshold,
	 * and turns off only when temperature falls below threshold - COOLING_FAN_HYSTERESIS_C (1°C).
	 * Runs independently of heater PID state.
	 */
	void updateCoolingFan();

	/**
	 * @brief Sets the cooling fan activation threshold from a string argument.
	 * @param args A C-style string containing the new threshold in Celsius.
	 */
	void setFanThreshold(const char* args);

	/**
	 * @brief Resets the PID controller's state variables.
	 * @details This is called when the heater is turned on or when the PID gains
	 * are changed to prevent integral windup and ensure a clean start for the controller.
	 */
	void resetPID();

	/**
	 * @brief Formats and sends a status message via the main Fillhead controller.
	 * @param statusType The prefix for the message (e.g., "INFO: "). See `config.h`.
	 * @param message The content of the message to send.
	 */
	void reportEvent(const char* statusType, const char* message);

	/**
	 * @brief Turns the heater on and enables the PID control loop.
	 * @param args Optional setpoint in Celsius. If NULL/empty, uses current setpoint.
	 */
	void heaterOn(const char* args);

	/**
	 * @brief Turns the heater off, disables the PID control loop, and resets PID variables.
	 */
	void heaterOff();

	/**
	 * @brief Sets the PID gains from a string argument.
	 * @param args A C-style string containing the new Kp, Ki, and Kd values,
	 *             which are parsed from the string.
	 */
	void setGains(const char* args);

	/**
	 * @brief Sets the PID setpoint (target temperature) from a string argument.
	 * @param args A C-style string containing the new setpoint in Celsius,
	 *             which is parsed from the string.
	 */
	void setSetpoint(const char* args);
};
