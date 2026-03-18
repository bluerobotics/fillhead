/**
 * @file vacuum_controller.cpp
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Implements the controller for the vacuum system.
 *
 * @details This file provides the concrete implementation for the `VacuumController` class.
 * It contains the logic for the state machine that manages vacuum pulldown, active hold,
 * and leak testing sequences. It also handles pressure sensing and command processing
 * for the vacuum system.
 */
#include "vacuum_controller.h"
#include "fillhead.h"
#include <cstdio>
#include <cstdlib>

VacuumController::VacuumController(Fillhead* controller) {
	m_controller = controller;
	m_state = VACUUM_OFF;
	m_vacuumPressurePsig = 0.0f;
	m_smoothedVacuumPsig = 0.0f;
	m_firstVacuumReading = true;
	m_stateStartTimeMs = 0;
	m_leakTestStartPressure = 0.0f;

	// Initialize parameters with default values from config, converting MS to S
	m_targetPsig = DEFAULT_VACUUM_TARGET_PSIG;
	m_rampTimeoutSec = (float)DEFAULT_VACUUM_RAMP_TIMEOUT_MS / 1000.0f;
	m_leakTestDeltaPsig = DEFAULT_LEAK_TEST_DELTA_PSIG;
	m_leakTestDurationSec = (float)DEFAULT_LEAK_TEST_DURATION_MS / 1000.0f;
}

void VacuumController::setup() {
	PIN_VACUUM_RELAY.Mode(Connector::OUTPUT_DIGITAL);
	PIN_VACUUM_TRANSDUCER.Mode(Connector::INPUT_ANALOG);
	PIN_VACUUM_VALVE_RELAY.Mode(Connector::OUTPUT_DIGITAL);

	PIN_VACUUM_RELAY.State(false);
	PIN_VACUUM_VALVE_RELAY.State(false);
}

/**
 * Handles a command from the CommsController.
 * @param cmd The command to execute.
 * @param args A string containing any arguments for the command.
 */
void VacuumController::handleCommand(Command cmd, const char* args) {
	// Prevent starting a new operation if one is already in progress (except for turning off)
	if (m_state != VACUUM_OFF && m_state != VACUUM_ON) {
		if (cmd == CMD_VACUUM_ON || cmd == CMD_VACUUM_LEAK_TEST) {
			reportEvent(STATUS_PREFIX_ERROR, "Vacuum command ignored: An operation is already in progress.");
			return;
		}
	}

	switch(cmd) {
		case CMD_VACUUM_ON:                 vacuumOn(args); break;
		case CMD_VACUUM_OFF:                vacuumOff(); break;
		case CMD_VACUUM_LEAK_TEST:          leakTest(args); break;
		case CMD_SET_VACUUM_TARGET:         setTarget(args); break;
		case CMD_SET_VACUUM_TIMEOUT_S:      setTimeout(args); break;
		case CMD_SET_LEAK_TEST_DELTA:       setLeakDelta(args); break;
		case CMD_SET_LEAK_TEST_DURATION_S:  setLeakDuration(args); break;
		default:
		// Not a vacuum command, ignore.
		break;
	}
}

void VacuumController::vacuumOn(const char* args) {
	if (args != NULL && args[0] != '\0') {
		float target = std::atof(args);
		if (target <= 0.0f && target > -15.0f) {
			m_targetPsig = target;
			char msg[STATUS_MESSAGE_BUFFER_SIZE];
			snprintf(msg, sizeof(msg), "vacuum_on received with target=%.2f PSIG. Ramping...", target);
			reportEvent(STATUS_PREFIX_START, msg);
			m_state = VACUUM_RAMP;
			m_stateStartTimeMs = Milliseconds();
			PIN_VACUUM_RELAY.State(true);
			PIN_VACUUM_VALVE_RELAY.State(true);
		} else {
			reportEvent(STATUS_PREFIX_ERROR, "Invalid vacuum target. Must be between 0 and -15 PSIG.");
		}
	} else {
		reportEvent(STATUS_PREFIX_START, "VACUUM_ON received. Pump is now running.");
		m_state = VACUUM_ON;
		PIN_VACUUM_RELAY.State(true);
		PIN_VACUUM_VALVE_RELAY.State(true);
	}
}

void VacuumController::vacuumOff() {
	if (m_state == VACUUM_OFF) {
		reportEvent(STATUS_PREFIX_INFO, "VACUUM_OFF ignored: System is already OFF.");
		return;
	}
	resetState();
	reportEvent(STATUS_PREFIX_DONE, "vacuum_off complete.");
}

void VacuumController::leakTest(const char* args) {
	float delta = m_leakTestDeltaPsig;
	float duration = m_leakTestDurationSec;
	if (args != NULL && args[0] != '\0') {
		float parsedDelta, parsedDuration;
		int parsed = sscanf(args, "%f %f", &parsedDelta, &parsedDuration);
		if (parsed >= 1) {
			if (parsedDelta > 0.0f && parsedDelta < 5.0f) {
				delta = parsedDelta;
			} else {
				reportEvent(STATUS_PREFIX_ERROR, "Invalid leak delta. Must be between 0 and 5 PSI.");
				return;
			}
		}
		if (parsed >= 2) {
			if (parsedDuration >= 1.0f && parsedDuration <= 120.0f) {
				duration = parsedDuration;
			} else {
				reportEvent(STATUS_PREFIX_ERROR, "Invalid leak duration. Must be between 1.0 and 120.0 seconds.");
				return;
			}
		}
	}
	m_leakTestDeltaPsig = delta;
	m_leakTestDurationSec = duration;
	char msg[STATUS_MESSAGE_BUFFER_SIZE];
	snprintf(msg, sizeof(msg), "LEAK_TEST initiated. Delta=%.3f PSI, Duration=%.1f s.", m_leakTestDeltaPsig, m_leakTestDurationSec);
	reportEvent(STATUS_PREFIX_START, msg);
	m_state = VACUUM_PULLDOWN;
	m_stateStartTimeMs = Milliseconds();
	PIN_VACUUM_RELAY.State(true);
	PIN_VACUUM_VALVE_RELAY.State(true);
}

void VacuumController::updateState() {
	// These states are terminal or passive, no automatic transitions needed.
	if (m_state == VACUUM_OFF || m_state == VACUUM_ON || m_state == VACUUM_ERROR) {
		return;
	}

	float elapsed_sec = (float)(Milliseconds() - m_stateStartTimeMs) / 1000.0f;

	if (m_state == VACUUM_RAMP) {
		if (m_vacuumPressurePsig <= m_targetPsig) {
			resetState();
			reportEvent(STATUS_PREFIX_DONE, "vacuum_on complete. Target pressure reached.");
		} else if (elapsed_sec > m_rampTimeoutSec) {
			m_state = VACUUM_ERROR;
			PIN_VACUUM_RELAY.State(false);
			PIN_VACUUM_VALVE_RELAY.State(false);
			reportEvent(STATUS_PREFIX_ERROR, "vacuum_on FAILED: Did not reach target pressure in time.");
		}
	}
	else if (m_state == VACUUM_PULLDOWN) {
		if (m_vacuumPressurePsig <= m_targetPsig) {
			PIN_VACUUM_RELAY.State(false);      // Pump off
			PIN_VACUUM_VALVE_RELAY.State(false);  // Close valve to seal system
			reportEvent(STATUS_PREFIX_INFO, "Leak Test: Target reached. Pump off. Settling...");
			m_state = VACUUM_SETTLING;
			m_stateStartTimeMs = Milliseconds();
		} else if (elapsed_sec > m_rampTimeoutSec) {
		m_state = VACUUM_ERROR;
		PIN_VACUUM_RELAY.State(false);
		PIN_VACUUM_VALVE_RELAY.State(false);
		reportEvent(STATUS_PREFIX_ERROR, "LEAK_TEST FAILED: Did not reach target pressure in time.");
	}
	}
	else if (m_state == VACUUM_SETTLING) {
		if (elapsed_sec > VACUUM_SETTLE_TIME_S) {
			m_leakTestStartPressure = m_smoothedVacuumPsig;
			char startMsg[STATUS_MESSAGE_BUFFER_SIZE];
			snprintf(startMsg, sizeof(startMsg), "Leak Test: Starting measurement at %.3f PSI.", m_leakTestStartPressure);
			reportEvent(STATUS_PREFIX_INFO, startMsg);
			m_state = VACUUM_LEAK_TESTING;
			m_stateStartTimeMs = Milliseconds();
		}
	}
	else if (m_state == VACUUM_LEAK_TESTING) {
		float pressure_delta = m_smoothedVacuumPsig - m_leakTestStartPressure;
		if (pressure_delta > m_leakTestDeltaPsig) {
			m_state = VACUUM_ERROR;
			char errorMsg[STATUS_MESSAGE_BUFFER_SIZE];
			snprintf(errorMsg, sizeof(errorMsg), "LEAK_TEST FAILED. Loss of %.3f PSI exceeded limit.", pressure_delta);
			reportEvent(STATUS_PREFIX_ERROR, errorMsg);
			} else if (elapsed_sec > m_leakTestDurationSec) {
			char passMsg[STATUS_MESSAGE_BUFFER_SIZE];
			snprintf(passMsg, sizeof(passMsg), "vacuum_leak_test complete. PASSED. Pressure loss was %.3f PSI.", pressure_delta);
			reportEvent(STATUS_PREFIX_DONE, passMsg);
			resetState();
		}
	}
}

void VacuumController::resetState() {
	m_state = VACUUM_OFF;
	PIN_VACUUM_RELAY.State(false);
	PIN_VACUUM_VALVE_RELAY.State(false);
}

void VacuumController::setTarget(const char* args) {
	float val = std::atof(args);
	if (val <= 0 && val > -15.0f) {
		m_targetPsig = val;
		char response[STATUS_MESSAGE_BUFFER_SIZE];
		snprintf(response, sizeof(response), "set_vacuum_target complete. Target=%.2f PSIG.", m_targetPsig);
		reportEvent(STATUS_PREFIX_DONE, response);
		} else {
		reportEvent(STATUS_PREFIX_ERROR, "Invalid vacuum target. Must be between 0 and -15.");
	}
}

void VacuumController::setTimeout(const char* args) {
	float val = std::atof(args);
	if (val >= 0.5f && val <= 60.0f) {
		m_rampTimeoutSec = val;
		char response[STATUS_MESSAGE_BUFFER_SIZE];
		snprintf(response, sizeof(response), "set_vacuum_timeout_s complete. Timeout=%.1f seconds.", m_rampTimeoutSec);
		reportEvent(STATUS_PREFIX_DONE, response);
		} else {
		reportEvent(STATUS_PREFIX_ERROR, "Invalid timeout. Must be between 0.5 and 60.0 seconds.");
	}
}

void VacuumController::setLeakDelta(const char* args) {
	float val = std::atof(args);
	if (val > 0.0f && val < 5.0f) {
		m_leakTestDeltaPsig = val;
		char response[STATUS_MESSAGE_BUFFER_SIZE];
		snprintf(response, sizeof(response), "set_leak_test_delta complete. Delta=%.3f PSI.", m_leakTestDeltaPsig);
		reportEvent(STATUS_PREFIX_DONE, response);
		} else {
		reportEvent(STATUS_PREFIX_ERROR, "Invalid leak delta. Must be between 0 and 5 PSI.");
	}
}

void VacuumController::setLeakDuration(const char* args) {
	float val = std::atof(args);
	if (val >= 1.0f && val <= 120.0f) {
		m_leakTestDurationSec = val;
		char response[STATUS_MESSAGE_BUFFER_SIZE];
		snprintf(response, sizeof(response), "set_leak_test_duration_s complete. Duration=%.1f seconds.", m_leakTestDurationSec);
		reportEvent(STATUS_PREFIX_DONE, response);
		} else {
		reportEvent(STATUS_PREFIX_ERROR, "Invalid leak duration. Must be between 1.0 and 120.0 seconds.");
	}
}

void VacuumController::updateVacuum() {
	uint16_t adc_val = PIN_VACUUM_TRANSDUCER.State();
	float measured_voltage = (float)adc_val * (TC_V_REF / 4095.0f);
	float voltage_span = VAC_V_OUT_MAX - VAC_V_OUT_MIN;
	float pressure_span = VAC_PRESSURE_MAX - VAC_PRESSURE_MIN;
	float voltage_percent = (measured_voltage - VAC_V_OUT_MIN) / voltage_span;
	float raw_psig = (voltage_percent * pressure_span) + VAC_PRESSURE_MIN;

	if (m_firstVacuumReading) {
		m_smoothedVacuumPsig = raw_psig;
		m_firstVacuumReading = false;
		} else {
		m_smoothedVacuumPsig = (EWMA_ALPHA_SENSORS * raw_psig) + ((1.0f - EWMA_ALPHA_SENSORS) * m_smoothedVacuumPsig);
	}
	m_vacuumPressurePsig = m_smoothedVacuumPsig + VACUUM_PSIG_OFFSET;
}

void VacuumController::reportEvent(const char* statusType, const char* message) {
	char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
	snprintf(fullMsg, sizeof(fullMsg), "Vacuum: %s", message);
	m_controller->reportEvent(statusType, fullMsg);
}

const char* VacuumController::getTelemetryString() {
	snprintf(m_telemetryBuffer, sizeof(m_telemetryBuffer),
	"vac_st:%d,vac_pv:%.2f,vac_sp:%.1f",
	(int)m_state, m_vacuumPressurePsig, m_targetPsig);
	return m_telemetryBuffer;
}

bool VacuumController::isBusy() const {
    return m_state != VACUUM_OFF && m_state != VACUUM_ERROR;
}

const char* VacuumController::getState() const {
	switch (m_state) {
		case VACUUM_OFF:            return "Off";
		case VACUUM_RAMP:           return "Ramping";
		case VACUUM_PULLDOWN:       return "Pulldown";
		case VACUUM_SETTLING:       return "Settling";
		case VACUUM_LEAK_TESTING:   return "Leak Test";
		case VACUUM_ON:             return "On";
		case VACUUM_ERROR:          return "Error";
		default:                    return "Unknown";
	}
}