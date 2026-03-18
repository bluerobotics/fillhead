/**
 * @file pinch_valve_controller.h
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Defines the controller for a single motorized pinch valve.
 *
 * @details This file defines the `PinchValve` class, which encapsulates the logic for
 * controlling one of the motorized pinch valves. It includes a detailed state machine
 * for managing homing (via hall effect sensor), opening, closing, and jogging operations.
 * The class also supports NVM-backed home-on-boot configuration and provides a telemetry
 * interface.
 */
#pragma once

#include "config.h"
#include "comms_controller.h"
#include "ClearCore.h"
#include "commands.h"
#include "NvmManager.h"

class Fillhead; // Forward declaration

/**
 * @enum PinchValveState
 * @brief Defines the main operational states of a pinch valve.
 */
enum PinchValveState {
	VALVE_NOT_HOMED,        ///< The valve has not been homed and its position is unknown. This is the default state on boot.
	VALVE_CLOSED,           ///< The valve is stationary in the fully closed position.
	VALVE_OPEN,             ///< The valve is stationary in the fully open position (after a successful home).
	VALVE_HALTED,           ///< The valve was stopped mid-move by an abort command and is in a known, intermediate position. It is still homed.
	VALVE_MOVING,           ///< The valve is actively moving between the open and closed positions.
	VALVE_HOMING,           ///< The valve is currently executing its homing sequence.
	VALVE_JOGGING,          ///< The valve is performing a manual jog move.
	VALVE_RESETTING,        ///< The valve is waiting for motion to stop before completing a reset.
	VALVE_ERROR,            ///< An error has occurred (e.g., timeout, motor fault, sensor not found).
};

/**
 * @class PinchValve
 * @brief Controls a single motorized pinch valve using hall effect sensor homing.
 *
 * @details This class encapsulates all logic for a single pinch valve actuator.
 * It manages a multi-level state machine to handle complex, non-blocking
 * operations like the multi-phase homing sequence. Homing moves the valve
 * toward the OPEN end of its stroke where a hall sensor is mounted. The valve
 * provides a simple public interface (`open()`, `close()`, `home()`) while
 * abstracting away the underlying motor control and state transitions.
 */
class PinchValve {
public:
	/**
	 * @brief Constructs a new PinchValve object.
	 * @param name A string identifier for the valve (e.g., "inj_valve"), used for logging.
	 * @param motor A pointer to the ClearCore `MotorDriver` object that this class will control.
	 * @param homeSensor A reference to the ClearCore connector for this valve's hall effect home sensor.
	 * @param nvmSlot The NVM byte offset for this valve's home-on-boot setting.
	 * @param controller A pointer to the main `Fillhead` controller, used for reporting events.
	 */
	PinchValve(const char* name, MotorDriver* motor, DigitalIn& homeSensor, int nvmSlot, Fillhead* controller);

	/**
	 * @brief Initializes the pinch valve motor, home sensor, and NVM settings.
	 * @details Configures the motor's operational parameters, sets up the home sensor
	 * digital input with filtering, loads the home-on-boot setting from NVM,
	 * and enables the motor driver. This should be called once at startup.
	 */
	void setup();

	/**
	 * @brief Updates the internal state machine for the pinch valve.
	 * @details This method should be called repeatedly in the main application loop. It is
	 * the core of the valve's non-blocking operation, advancing the state machines
	 * for homing, opening, closing, and jogging.
	 */
	void updateState();

	/**
	 * @brief Handles user commands specific to this pinch valve.
	 * @param cmd The `Command` enum representing the command to execute.
	 * @param args A C-style string containing any arguments for the command (e.g., jog distance).
	 */
	void handleCommand(Command cmd, const char* args);

	/**
	 * @brief Initiates the hall sensor homing sequence for the valve.
	 * @details The valve moves toward the OPEN end of its stroke to find the hall sensor,
	 * backs off, re-approaches slowly for precision, then sets the position as zero.
	 */
	void home();

	/**
	 * @brief Commands the valve to move to the fully open position (its homed position).
	 */
	void open();

	/**
	 * @brief Commands the valve to move to the fully closed (pinched) position.
	 */
	void close();

	/**
	 * @brief Commands the valve to perform a manual jog move.
	 * @param args A C-style string containing the distance and speed for the jog.
	 */
	void jog(const char* args);

	void enable();
	void disable();
	void abort();
	void reset();

	bool isHomed() const;
	bool isOpen() const;
	const char* getTelemetryString();
	bool isInFault() const;
	bool isBusy() const;
	const char* getState() const;

	/**
	 * @brief Gets the NVM-backed home-on-boot setting.
	 * @return `true` if the valve should auto-home on boot.
	 */
	bool getHomeOnBoot() const;

	/**
	 * @brief Sets the home-on-boot flag and persists it to NVM.
	 * @param enabled "true" or "false" as a C-string.
	 * @return `true` if the value was valid and saved.
	 */
	bool setHomeOnBoot(const char* enabled);

	/**
	 * @brief Reads the current state of the hall effect home sensor.
	 * @return `true` if the sensor is in its active (triggered) state.
	 */
	bool isHomeSensorTriggered() const;

private:
	typedef enum {
		MOVE_TYPE_NONE,
		MOVE_TYPE_OPEN,
		MOVE_TYPE_CLOSE
	} MoveType;

	void moveSteps(long steps, int velocity_sps, int accel_sps2);
	float getSmoothedTorque();
	float getInstantaneousTorque();
	bool checkTorqueLimit();
	void reportEvent(const char* statusType, const char* message);
	void setupHomeSensor();

	typedef enum {
		PHASE_IDLE,
		PHASE_START,
		PHASE_WAIT_TO_START,
		PHASE_MOVING
	} OperatingPhase;

	/**
	 * @enum HomingPhase
	 * @brief Defines the sub-states for the hall sensor homing sequence.
	 * @details The valve moves toward OPEN (negative direction) to find the sensor,
	 * backs off toward CLOSED (positive), then slowly re-approaches the sensor.
	 */
	typedef enum {
		HOMING_PHASE_IDLE,
		CHECK_SENSOR,                   ///< Check if already on the sensor; if so, skip to backoff.
		RAPID_APPROACH_START,           ///< Start rapid move toward open/sensor (negative direction).
		RAPID_APPROACH_WAIT_TO_START,   ///< Wait for the rapid move to begin.
		RAPID_APPROACH_MOVING,          ///< Moving toward sensor; stop when triggered.
		BACKOFF_START,                  ///< Start moving away from sensor (positive/closed direction).
		BACKOFF_WAIT_TO_START,          ///< Wait for the backoff move to begin.
		BACKOFF_MOVING,                 ///< Executing backoff move.
		SLOW_APPROACH_START,            ///< Start slow precision approach back toward sensor.
		SLOW_APPROACH_WAIT_TO_START,    ///< Wait for the slow move to begin.
		SLOW_APPROACH_MOVING,           ///< Moving slowly toward sensor; stop when triggered.
		SET_ZERO                        ///< Set current position as logical zero (open).
	} HomingPhase;

	const char* m_name;
	MotorDriver* m_motor;
	DigitalIn& m_homeSensor;
	Fillhead* m_controller;
	int m_nvmSlot;
	bool m_homeOnBoot;

	PinchValveState m_state;
	HomingPhase m_homingPhase;
	OperatingPhase m_opPhase;
	MoveType m_moveType;
	uint32_t m_moveStartTime;
	bool m_isHomed;
	uint32_t m_homingStartTime;

	float m_torqueLimit;
	float m_smoothedTorque;
	bool m_firstTorqueReading;

	char m_telemetryBuffer[160];

	const char* m_activeCommandStr;

	long m_homingDistanceSteps;
	long m_homingBackoffSteps;
	int m_homingRapidSps;
	int m_homingSlowSps;
	int m_homingAccelSps2;
};
