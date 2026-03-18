/**
 * @file pinch_valve_controller.cpp
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Implements the controller for a single motorized pinch valve.
 *
 * @details This file provides the concrete implementation for the `PinchValve` class.
 * It contains the logic for the state machines that manage homing, opening, closing,
 * and jogging operations, as well as motor control and torque monitoring.
 */
#include "pinch_valve_controller.h"
#include "fillhead.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/**
 * @brief Constructs the PinchValve controller.
 */
PinchValve::PinchValve(const char* name, MotorDriver* motor, Fillhead* controller) :
    m_name(name),
    m_motor(motor),
    m_controller(controller),
    m_state(VALVE_NOT_HOMED), // Default state is now NOT_HOMED
    m_homingPhase(HOMING_PHASE_IDLE),
    m_opPhase(PHASE_IDLE),
    m_moveType(MOVE_TYPE_NONE), // Initialize move type
    m_moveStartTime(0),
    m_isHomed(false),
    m_homingStartTime(0),
    m_torqueLimit(0.0f),
    m_smoothedTorque(0.0f),
    m_firstTorqueReading(true) {
}

/**
 * @brief Initializes the motor hardware for the pinch valve.
 */
void PinchValve::setup() {
	m_motor->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	m_motor->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
	m_motor->VelMax(PINCH_DEFAULT_VEL_MAX_SPS);
	m_motor->AccelMax(PINCH_DEFAULT_ACCEL_MAX_SPS2);
	m_motor->EnableRequest(true);
}

/**
 * @brief The main update loop for the valve's state machine.
 */
void PinchValve::updateState() {
    // Check for motor faults first, as this is a critical hardware issue.
    if (isInFault() && m_state != VALVE_ERROR) {
        m_state = VALVE_ERROR;
        m_isHomed = false; // Lost homed status due to error
        reportEvent(STATUS_PREFIX_ERROR, "Motor fault detected.");
    }

    switch (m_state) {
        case VALVE_HOMING: {
			// Homing logic remains the same...
			if (Milliseconds() - m_homingStartTime > MAX_HOMING_DURATION_MS) {
				abort();
				m_state = VALVE_ERROR;
				m_isHomed = false; // Lost homed status due to error
				m_homingPhase = HOMING_PHASE_IDLE;
				char errorMsg[128];
				snprintf(errorMsg, sizeof(errorMsg), "%s homing timeout.", m_name);
				reportEvent(STATUS_PREFIX_ERROR, errorMsg);
				return;
			}
			
			switch (m_homingPhase) {
				case INITIAL_BACKOFF_START:
					reportEvent(STATUS_PREFIX_INFO, "Homing: Performing initial backoff.");
					m_torqueLimit = m_homingBackoffTorque;
					moveSteps(-m_homingBackoffSteps, m_homingUnifiedSps, m_homingAccelSps2);
					m_homingPhase = INITIAL_BACKOFF_WAIT_TO_START;
					break;
				case INITIAL_BACKOFF_WAIT_TO_START:
					if (m_motor->StatusReg().bit.StepsActive) {
						m_homingPhase = INITIAL_BACKOFF_MOVING;
					} else if (Milliseconds() - m_homingStartTime > 500) {
						 reportEvent(STATUS_PREFIX_INFO, "Homing: Initial backoff did not start, proceeding.");
						 m_homingPhase = RAPID_SEARCH_START;
					}
					break;
				case INITIAL_BACKOFF_MOVING:
					if (checkTorqueLimit() || !m_motor->StatusReg().bit.StepsActive) {
						m_homingPhase = RAPID_SEARCH_START;
					}
					break;
				case RAPID_SEARCH_START:
					reportEvent(STATUS_PREFIX_INFO, "Homing: Starting rapid search.");
					m_torqueLimit = m_homingSearchTorque;
					moveSteps(m_homingDistanceSteps, m_homingUnifiedSps, m_homingAccelSps2);
					m_homingPhase = RAPID_SEARCH_WAIT_TO_START;
					break;
				case RAPID_SEARCH_WAIT_TO_START:
					if (m_motor->StatusReg().bit.StepsActive) m_homingPhase = RAPID_SEARCH_MOVING;
					break;
				case RAPID_SEARCH_MOVING:
					if (checkTorqueLimit()) {
						m_homingPhase = BACKOFF_START;
					} else if (!m_motor->StatusReg().bit.StepsActive) {
						m_state = VALVE_ERROR;
						m_isHomed = false; // Lost homed status due to error
						reportEvent(STATUS_PREFIX_ERROR, "Homing failed: move finished before hard stop.");
					}
					break;
				case BACKOFF_START:
					reportEvent(STATUS_PREFIX_INFO, "Homing: Backing off hard stop.");
					m_torqueLimit = m_homingBackoffTorque;
					moveSteps(-m_homingBackoffSteps, m_homingUnifiedSps, m_homingAccelSps2);
					m_homingPhase = BACKOFF_WAIT_TO_START;
					break;
				case BACKOFF_WAIT_TO_START:
					if (m_motor->StatusReg().bit.StepsActive) m_homingPhase = BACKOFF_MOVING;
					break;
				case BACKOFF_MOVING:
					if (!m_motor->StatusReg().bit.StepsActive) m_homingPhase = SLOW_SEARCH_START;
					break;
				case SLOW_SEARCH_START:
					reportEvent(STATUS_PREFIX_INFO, "Homing: Starting slow search.");
					m_torqueLimit = m_homingSearchTorque;
					moveSteps(m_homingBackoffSteps * 2, m_homingUnifiedSps, m_homingAccelSps2);
					m_homingPhase = SLOW_SEARCH_WAIT_TO_START;
					break;
				case SLOW_SEARCH_WAIT_TO_START:
					if (m_motor->StatusReg().bit.StepsActive) m_homingPhase = SLOW_SEARCH_MOVING;
					break;
				case SLOW_SEARCH_MOVING:
					if (checkTorqueLimit()) {
						m_homingPhase = SET_OFFSET_START;
					} else if (!m_motor->StatusReg().bit.StepsActive) {
						m_state = VALVE_ERROR;
						m_isHomed = false; // Lost homed status due to error
						reportEvent(STATUS_PREFIX_ERROR, "Homing failed during slow search.");
					}
					break;
				case SET_OFFSET_START:
					reportEvent(STATUS_PREFIX_INFO, "Homing: Moving to final offset.");
					m_torqueLimit = m_homingBackoffTorque;
					moveSteps(-m_homingFinalOffsetSteps, m_homingUnifiedSps, m_homingAccelSps2);
					m_homingPhase = SET_OFFSET_WAIT_TO_START;
					break;
				case SET_OFFSET_WAIT_TO_START:
					if (m_motor->StatusReg().bit.StepsActive) m_homingPhase = SET_OFFSET_MOVING;
					break;
				case SET_OFFSET_MOVING:
					if (checkTorqueLimit() || !m_motor->StatusReg().bit.StepsActive) m_homingPhase = SET_ZERO;
					break;
				case SET_ZERO:
					m_motor->PositionRefSet(0); 
					m_isHomed = true;
					m_state = VALVE_OPEN; // Transition to OPEN after homing
					m_homingPhase = HOMING_PHASE_IDLE;
					char msg[64];
					snprintf(msg, sizeof(msg), "%s homing complete. Valve is OPEN.", m_name);
					reportEvent(STATUS_PREFIX_DONE, msg);
					break;
				default:
					abort();
					m_state = VALVE_ERROR;
					m_isHomed = false; // Lost homed status due to error
					m_homingPhase = HOMING_PHASE_IDLE;
					break;
			}
			break;
		}
		
		case VALVE_MOVING: {
            // Shared logic for timeout and motor start
            switch(m_opPhase) {
                case PHASE_WAIT_TO_START:
                    if (m_motor->StatusReg().bit.StepsActive) {
                        m_opPhase = PHASE_MOVING;
                    } else if (Milliseconds() - m_moveStartTime > 500) {
                        m_state = VALVE_ERROR;
                        m_isHomed = false; // Lost homed status due to error
                        m_opPhase = PHASE_IDLE;
                        m_moveType = MOVE_TYPE_NONE;
                        reportEvent(STATUS_PREFIX_ERROR, "Move failed: Motor did not start.");
                    }
                    break;
                case PHASE_MOVING:
                    // --- Logic diverges based on move type ---
                    if (m_moveType == MOVE_TYPE_OPEN) {
                        // Success: move completes. Error: torque limit hit.
                        if (checkTorqueLimit()) {
                            abort();
                            m_state = VALVE_ERROR;
                            m_isHomed = false; // Lost homed status due to error
                            reportEvent(STATUS_PREFIX_ERROR, "Open failed: Torque limit hit unexpectedly.");
                        } else if (!m_motor->StatusReg().bit.StepsActive) {
                            m_state = VALVE_OPEN;
                            reportEvent(STATUS_PREFIX_DONE, "Open complete.");
                        }
                    } else if (m_moveType == MOVE_TYPE_CLOSE) {
                        // Success: torque limit hit. Error: move completes.
                        if (checkTorqueLimit()) {
                            abort(); // Stop the move now that we've hit the stop.
                            m_state = VALVE_CLOSED;
                            char msg[64];
                            snprintf(msg, sizeof(msg), "%s closed.", m_name);
                            reportEvent(STATUS_PREFIX_DONE, msg);
                        } else if (!m_motor->StatusReg().bit.StepsActive) {
                            m_state = VALVE_ERROR;
                            m_isHomed = false; // Lost homed status due to error
                            reportEvent(STATUS_PREFIX_ERROR, "Close failed: Did not reach torque limit.");
                        }
                    }
                    
                    // If state changed, the operation is over.
                    if (m_state != VALVE_MOVING) {
                        m_opPhase = PHASE_IDLE;
                        m_moveType = MOVE_TYPE_NONE;
                    }
                    break;
                default:
                    // This includes PHASE_IDLE and PHASE_START, which shouldn't be processed here.
                    break;
            }
        }
        break;

		case VALVE_JOGGING:
			switch(m_opPhase) {
				case PHASE_WAIT_TO_START:
					if (m_motor->StatusReg().bit.StepsActive) {
						m_opPhase = PHASE_MOVING;
					} else if (Milliseconds() - m_moveStartTime > 500) {
						m_state = VALVE_ERROR;
						m_isHomed = false; // Lost homed status due to error
						m_opPhase = PHASE_IDLE;
						reportEvent(STATUS_PREFIX_ERROR, "Move failed: Motor did not start.");
					}
					break;
				case PHASE_MOVING:
					if (checkTorqueLimit()) { // Expected stop for jog
						abort();
						m_state = m_isHomed ? VALVE_OPEN : VALVE_NOT_HOMED;
						m_opPhase = PHASE_IDLE;
					} else if (!m_motor->StatusReg().bit.StepsActive) {
						m_state = m_isHomed ? VALVE_OPEN : VALVE_NOT_HOMED; // Success
						m_opPhase = PHASE_IDLE;
						reportEvent(STATUS_PREFIX_DONE, "Jog complete.");
					}
					break;
				default:
					m_state = VALVE_ERROR;
					m_isHomed = false; // Lost homed status due to error
					m_opPhase = PHASE_IDLE;
					break;
			}
			break;
		
		case VALVE_RESETTING: {
            if (!m_motor->StatusReg().bit.StepsActive) {
                m_state = m_isHomed ? VALVE_OPEN : VALVE_NOT_HOMED;
            }
        }
        break;

		// Stationary states, no action needed.
        case VALVE_NOT_HOMED:
		case VALVE_OPEN:
		case VALVE_CLOSED:
		case VALVE_HALTED:
		case VALVE_ERROR:
			break;
    }
}

/**
 * @brief Routes a command to the appropriate handler function.
 */
void PinchValve::handleCommand(Command cmd, const char* args) {
    // Prevent new motion commands while busy, unless it's an abort/reset.
    if (isBusy() && cmd != CMD_ABORT && cmd != CMD_RESET) {
        reportEvent(STATUS_PREFIX_ERROR, "Valve is busy.");
        return;
    }
    // Prevent motion if in an error state.
    if (m_state == VALVE_ERROR && cmd != CMD_RESET) {
        reportEvent(STATUS_PREFIX_ERROR, "Valve is in an error state. Reset required.");
        return;
    }

	switch(cmd) {
		case CMD_INJECTION_VALVE_HOME_UNTUBED:
			home(false);
			break;
		case CMD_INJECTION_VALVE_HOME_TUBED:
			home(true);
			break;
		case CMD_VACUUM_VALVE_HOME_UNTUBED:
			home(false);
			break;
		case CMD_VACUUM_VALVE_HOME_TUBED:
			home(true);
			break;
		case CMD_INJECTION_VALVE_OPEN:
		case CMD_VACUUM_VALVE_OPEN:
			open();
			break;
		case CMD_INJECTION_VALVE_CLOSE:
		case CMD_VACUUM_VALVE_CLOSE:
			close();
			break;
		case CMD_INJECTION_VALVE_JOG:
		case CMD_VACUUM_VALVE_JOG:
			jog(args);
			break;
		default:
			break;
	}
}

/**
 * @brief Initiates the homing sequence for the pinch valve.
 */
void PinchValve::home(bool is_tubed) {
	m_isHomed = false;
	m_state = VALVE_HOMING;
	m_homingPhase = INITIAL_BACKOFF_START;
	m_homingStartTime = Milliseconds();

	if (is_tubed) {
		m_homingBackoffSteps = (long)(PINCH_VALVE_HOMING_BACKOFF_MM_TUBED * STEPS_PER_MM_PINCH);
		m_homingDistanceSteps = (long)(fabs(PINCH_HOMING_TUBED_STROKE_MM) * STEPS_PER_MM_PINCH);
		m_homingFinalOffsetSteps = (long)(PINCH_VALVE_TUBED_FINAL_OFFSET_MM * STEPS_PER_MM_PINCH);
		m_homingUnifiedSps = (int)fabs(PINCH_HOMING_TUBED_UNIFIED_VEL_MMS * STEPS_PER_MM_PINCH);
		m_homingAccelSps2 = (int)fabs(PINCH_HOMING_TUBED_ACCEL_MMSS * STEPS_PER_MM_PINCH);
		m_homingSearchTorque = PINCH_HOMING_TUBED_SEARCH_TORQUE_PERCENT;
		m_homingBackoffTorque = PINCH_HOMING_TUBED_BACKOFF_TORQUE_PERCENT;
	} else {
		m_homingBackoffSteps = (long)(PINCH_VALVE_HOMING_BACKOFF_MM_UNTUBED * STEPS_PER_MM_PINCH);
		m_homingDistanceSteps = (long)(fabs(PINCH_HOMING_UNTUBED_STROKE_MM) * STEPS_PER_MM_PINCH);
		m_homingFinalOffsetSteps = (long)(PINCH_VALVE_UNTUBED_FINAL_OFFSET_MM * STEPS_PER_MM_PINCH);
		m_homingUnifiedSps = (int)fabs(PINCH_HOMING_UNTUBED_UNIFIED_VEL_MMS * STEPS_PER_MM_PINCH);
		m_homingAccelSps2 = (int)fabs(PINCH_HOMING_UNTUBED_ACCEL_MMSS * STEPS_PER_MM_PINCH);
		m_homingSearchTorque = PINCH_HOMING_UNTUBED_SEARCH_TORQUE_PERCENT;
		m_homingBackoffTorque = PINCH_HOMING_UNTUBED_BACKOFF_TORQUE_PERCENT;
	}

	char msg[64];
	snprintf(msg, sizeof(msg), "%s homing started (%s).", m_name, is_tubed ? "tubed" : "untubed");
	reportEvent(STATUS_PREFIX_INFO, msg);
}

/**
 * @brief Kicks off the state machine to open the valve.
 */
void PinchValve::open() {
	if (!m_isHomed) {
		reportEvent(STATUS_PREFIX_ERROR, "Valve must be homed before opening.");
		return;
	}
	m_state = VALVE_MOVING;
	m_moveType = MOVE_TYPE_OPEN; // Set move type
	m_opPhase = PHASE_WAIT_TO_START;
	m_moveStartTime = Milliseconds();
	m_torqueLimit = JOG_DEFAULT_TORQUE_PERCENT; // High torque limit, not meant to be hit
	
	long target_steps = 0;
	long current_steps = m_motor->PositionRefCommanded();
	int vel_sps = (int)(PINCH_VALVE_OPEN_VEL_MMS * STEPS_PER_MM_PINCH);
	int accel_sps2 = (int)(PINCH_VALVE_OPEN_ACCEL_MMSS * STEPS_PER_MM_PINCH);

	moveSteps(target_steps - current_steps, vel_sps, accel_sps2);
}

/**
 * @brief Kicks off the state machine to close the valve.
 */
void PinchValve::close() {
	if (!m_isHomed) {
		reportEvent(STATUS_PREFIX_ERROR, "Valve must be homed before closing.");
		return;
	}
	m_state = VALVE_MOVING;
	m_moveType = MOVE_TYPE_CLOSE; // Set move type
	m_opPhase = PHASE_WAIT_TO_START;
	m_moveStartTime = Milliseconds();
	m_torqueLimit = PINCH_VALVE_PINCH_TORQUE_PERCENT; // Low torque limit, expected to be hit
	
	long long_move_steps = (long)(PINCH_HOMING_UNTUBED_STROKE_MM * STEPS_PER_MM_PINCH);
	int vel_sps = (int)(PINCH_VALVE_PINCH_VEL_MMS * STEPS_PER_MM_PINCH);
	int accel_sps2 = (int)(PINCH_JOG_DEFAULT_ACCEL_MMSS * STEPS_PER_MM_PINCH);

	moveSteps(long_move_steps, vel_sps, accel_sps2);
}

/**
 * @brief Kicks off the state machine to jog the valve.
 */
void PinchValve::jog(const char* args) {
	if (!args) {
		reportEvent(STATUS_PREFIX_ERROR, "Invalid jog command arguments.");
		return;
	}
	m_state = VALVE_JOGGING;
	m_moveType = MOVE_TYPE_NONE; // Ensure moveType is not set for jogs
	m_opPhase = PHASE_WAIT_TO_START;
	m_moveStartTime = Milliseconds();
	m_torqueLimit = JOG_DEFAULT_TORQUE_PERCENT;

	float dist_mm = atof(args);
	long steps = (long)(dist_mm * STEPS_PER_MM_PINCH);
	int vel_sps = (int)(PINCH_JOG_DEFAULT_VEL_MMS * STEPS_PER_MM_PINCH);
	int accel_sps2_val = (int)(PINCH_JOG_DEFAULT_ACCEL_MMSS * STEPS_PER_MM_PINCH);
	
	moveSteps(steps, vel_sps, accel_sps2_val);
}

/**
 * @brief Low-level move command.
 */
void PinchValve::moveSteps(long steps, int velocity_sps, int accel_sps2) {
	if (m_motor->StatusReg().bit.Enabled) {
		m_firstTorqueReading = true;
		m_motor->VelMax(velocity_sps);
		m_motor->AccelMax(accel_sps2);
		m_motor->Move(steps);
	} else {
		char errorMsg[128];
		snprintf(errorMsg, sizeof(errorMsg), "%s motor is not enabled.", m_name);
		reportEvent(STATUS_PREFIX_ERROR, errorMsg);
	}
}

void PinchValve::enable() {
	m_motor->EnableRequest(true);
	// Always set motor parameters on enable to ensure a known good state after a fault.
	m_motor->VelMax(PINCH_DEFAULT_VEL_MAX_SPS);
	m_motor->AccelMax(PINCH_DEFAULT_ACCEL_MAX_SPS2);
	char msg[64];
	snprintf(msg, sizeof(msg), "%s motor enabled.", m_name);
	reportEvent(STATUS_PREFIX_INFO, msg);
}

void PinchValve::disable() {
	m_motor->EnableRequest(false);
	char msg[64];
	snprintf(msg, sizeof(msg), "%s motor disabled.", m_name);
	reportEvent(STATUS_PREFIX_INFO, msg);
}

void PinchValve::abort() {
	if (m_state == VALVE_MOVING || m_state == VALVE_JOGGING) {
		m_motor->MoveStopAbrupt();
		m_state = VALVE_HALTED;
		m_opPhase = PHASE_IDLE;
		m_moveType = MOVE_TYPE_NONE;
	}
}

void PinchValve::reset() {
	// This function should only clear an error state.
	// If not in an error, do nothing.
	if (m_state != VALVE_ERROR) {
		return;
	}

	// If we are in an error state, we need to stop any potential motion
	// and reset the state machine to a known, safe state.
	if (m_motor->StatusReg().bit.StepsActive) {
		m_motor->MoveStopAbrupt();
	}
	m_state = VALVE_RESETTING; // Transition to a temporary state to wait for motor to stop
	m_homingPhase = HOMING_PHASE_IDLE;
	m_opPhase = PHASE_IDLE;
	m_moveType = MOVE_TYPE_NONE;
	m_firstTorqueReading = true;
	// The homed status was already cleared when the error occurred.
}

/**
 * @brief Gets the instantaneous, raw torque value from the motor.
 */
float PinchValve::getInstantaneousTorque() {
	float currentRawTorque = m_motor->HlfbPercent();
	if (currentRawTorque == TORQUE_HLFB_AT_POSITION) {
		return 0.0f;
	}
	return currentRawTorque;
}

/**
 * @brief Gets a smoothed torque value using an EWMA filter.
 */
float PinchValve::getSmoothedTorque() {
	float currentRawTorque = getInstantaneousTorque();
    if (m_firstTorqueReading) {
        m_smoothedTorque = currentRawTorque;
        m_firstTorqueReading = false;
    } else {
        m_smoothedTorque = EWMA_ALPHA_TORQUE * currentRawTorque + (1.0f - EWMA_ALPHA_TORQUE) * m_smoothedTorque;
    }
    return m_smoothedTorque;
}

/**
 * @brief Checks if the torque on the motor has exceeded the current limit.
 */
bool PinchValve::checkTorqueLimit() {
    if (m_motor->StatusReg().bit.StepsActive) {
        float torque = getInstantaneousTorque(); 
        if (torque > m_torqueLimit) {
            m_motor->MoveStopAbrupt();
            char torque_msg[STATUS_MESSAGE_BUFFER_SIZE];
            snprintf(torque_msg, sizeof(torque_msg), "%s TORQUE LIMIT REACHED (%.1f%%)", m_name, m_torqueLimit);
            reportEvent(STATUS_PREFIX_INFO, torque_msg);
            return true;
        }
    }
    return false;
}

void PinchValve::reportEvent(const char* statusType, const char* message) {
    char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
    snprintf(fullMsg, sizeof(fullMsg), "%s: %s", m_name, message);
    m_controller->reportEvent(statusType, fullMsg);
}

/**
 * @brief Assembles the telemetry string for this valve using smoothed torque.
 */
const char* PinchValve::getTelemetryString() {
	float displayTorque = getSmoothedTorque();
	snprintf(m_telemetryBuffer, sizeof(m_telemetryBuffer),
	"%s_pos:%.2f,%s_torque:%.1f,%s_homed:%d,%s_state:%d",
	m_name, (float)m_motor->PositionRefCommanded() / STEPS_PER_MM_PINCH,
	m_name, displayTorque,
	m_name, (int)m_isHomed,
	m_name, m_state
	);
	return m_telemetryBuffer;
}

bool PinchValve::isBusy() const {
	return m_state == VALVE_HOMING || m_state == VALVE_MOVING || m_state == VALVE_JOGGING || m_state == VALVE_RESETTING;
}

bool PinchValve::isInFault() const {
	return m_motor->StatusReg().bit.MotorInFault;
}

/**
 * @brief Checks if the valve has been successfully homed.
 */
bool PinchValve::isHomed() const {
	return m_isHomed;
}

/**
 * @brief Checks if the valve is currently in the fully open state.
 */
bool PinchValve::isOpen() const {
	return m_state == VALVE_OPEN;
}

/**
 * @brief Gets the current state of the valve as a human-readable string.
 */
const char* PinchValve::getState() const {
	switch (m_state) {
		case VALVE_NOT_HOMED: return "Not Homed";
		case VALVE_OPEN: return "Open";
		case VALVE_CLOSED: return "Closed";
		case VALVE_HALTED: return "Halted";
		case VALVE_MOVING: return "Moving";
		case VALVE_HOMING: return "Homing";
		case VALVE_JOGGING: return "Jogging";
		case VALVE_RESETTING: return "Resetting";
		case VALVE_ERROR: return "Error";
		default: return "Unknown";
	}
}