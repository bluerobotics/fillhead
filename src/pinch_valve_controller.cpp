/**
 * @file pinch_valve_controller.cpp
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Implements the controller for a single motorized pinch valve.
 *
 * @details This file provides the concrete implementation for the `PinchValve` class.
 * It contains the logic for the state machines that manage homing (via hall effect
 * sensor), opening, closing, and jogging operations, as well as motor control,
 * torque monitoring, and NVM-backed configuration.
 */
#include "pinch_valve_controller.h"
#include "fillhead.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

PinchValve::PinchValve(const char* name, MotorDriver* motor, Connector& homeSensor, int nvmSlot, Fillhead* controller) :
    m_name(name),
    m_motor(motor),
    m_homeSensor(homeSensor),
    m_controller(controller),
    m_nvmSlot(nvmSlot),
    m_homeOnBoot(true),
    m_state(VALVE_NOT_HOMED),
    m_homingPhase(HOMING_PHASE_IDLE),
    m_opPhase(PHASE_IDLE),
    m_moveType(MOVE_TYPE_NONE),
    m_moveStartTime(0),
    m_isHomed(false),
    m_homingStartTime(0),
    m_torqueLimit(0.0f),
    m_smoothedTorque(0.0f),
    m_firstTorqueReading(true),
    m_homingDistanceSteps(0),
    m_homingBackoffSteps(0),
    m_homingRapidSps(0),
    m_homingSlowSps(0),
    m_homingAccelSps2(0) {
}

void PinchValve::setup() {
    m_motor->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    m_motor->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    m_motor->VelMax(PINCH_DEFAULT_VEL_MAX_SPS);
    m_motor->AccelMax(PINCH_DEFAULT_ACCEL_MAX_SPS2);
    m_motor->EnableRequest(true);

    setupHomeSensor();

    NvmManager &nvmMgr = NvmManager::Instance();
    int32_t hobValue = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(m_nvmSlot));
    m_homeOnBoot = (hobValue != 0);
}

void PinchValve::setupHomeSensor() {
    uint16_t filterSamples = HOME_SENSOR_FILTER_MS * 5;
    m_homeSensor.Mode(Connector::INPUT_DIGITAL);
    m_homeSensor.FilterLength(filterSamples);
}

bool PinchValve::isHomeSensorTriggered() const {
    return (m_homeSensor.State() != 0) == HOME_SENSOR_ACTIVE_STATE;
}

// --- State Machine ---

void PinchValve::updateState() {
    if (isInFault() && m_state != VALVE_ERROR) {
        m_state = VALVE_ERROR;
        m_isHomed = false;
        reportEvent(STATUS_PREFIX_ERROR, "Motor fault detected.");
    }

    switch (m_state) {
        case VALVE_HOMING: {
            if (Milliseconds() - m_homingStartTime > MAX_HOMING_DURATION_MS) {
                m_motor->MoveStopAbrupt();
                m_state = VALVE_ERROR;
                m_isHomed = false;
                m_homingPhase = HOMING_PHASE_IDLE;
                char errorMsg[128];
                snprintf(errorMsg, sizeof(errorMsg), "%s homing timeout.", m_name);
                reportEvent(STATUS_PREFIX_ERROR, errorMsg);
                return;
            }

            switch (m_homingPhase) {
                case CHECK_SENSOR:
                    if (isHomeSensorTriggered()) {
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Already on sensor, backing off.");
                        m_homingPhase = BACKOFF_START;
                    } else {
                        m_homingPhase = RAPID_APPROACH_START;
                    }
                    break;

                case RAPID_APPROACH_START:
                    reportEvent(STATUS_PREFIX_INFO, "Homing: Rapid search toward sensor.");
                    moveSteps(-m_homingDistanceSteps, m_homingRapidSps, m_homingAccelSps2);
                    m_homingPhase = RAPID_APPROACH_WAIT_TO_START;
                    break;
                case RAPID_APPROACH_WAIT_TO_START:
                    if (m_motor->StatusReg().bit.StepsActive) {
                        m_homingPhase = RAPID_APPROACH_MOVING;
                    } else if (Milliseconds() - m_homingStartTime > 500) {
                        m_state = VALVE_ERROR;
                        m_isHomed = false;
                        m_homingPhase = HOMING_PHASE_IDLE;
                        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: motor did not start.");
                    }
                    break;
                case RAPID_APPROACH_MOVING:
                    if (isHomeSensorTriggered()) {
                        m_motor->MoveStopAbrupt();
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Sensor triggered (rapid).");
                        m_homingPhase = BACKOFF_START;
                    } else if (!m_motor->StatusReg().bit.StepsActive) {
                        m_state = VALVE_ERROR;
                        m_isHomed = false;
                        m_homingPhase = HOMING_PHASE_IDLE;
                        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: sensor not found during rapid search.");
                    }
                    break;

                case BACKOFF_START:
                    reportEvent(STATUS_PREFIX_INFO, "Homing: Backing off sensor.");
                    moveSteps(m_homingBackoffSteps, m_homingSlowSps, m_homingAccelSps2);
                    m_homingPhase = BACKOFF_WAIT_TO_START;
                    break;
                case BACKOFF_WAIT_TO_START:
                    if (m_motor->StatusReg().bit.StepsActive) {
                        m_homingPhase = BACKOFF_MOVING;
                    } else if (Milliseconds() - m_homingStartTime > 500) {
                        m_state = VALVE_ERROR;
                        m_isHomed = false;
                        m_homingPhase = HOMING_PHASE_IDLE;
                        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: backoff motor did not start.");
                    }
                    break;
                case BACKOFF_MOVING:
                    if (!m_motor->StatusReg().bit.StepsActive) {
                        m_homingPhase = SLOW_APPROACH_START;
                    }
                    break;

                case SLOW_APPROACH_START:
                    reportEvent(STATUS_PREFIX_INFO, "Homing: Slow precision approach toward sensor.");
                    moveSteps(-m_homingBackoffSteps * 2, m_homingSlowSps, m_homingAccelSps2);
                    m_homingPhase = SLOW_APPROACH_WAIT_TO_START;
                    break;
                case SLOW_APPROACH_WAIT_TO_START:
                    if (m_motor->StatusReg().bit.StepsActive) {
                        m_homingPhase = SLOW_APPROACH_MOVING;
                    } else if (Milliseconds() - m_homingStartTime > 500) {
                        m_state = VALVE_ERROR;
                        m_isHomed = false;
                        m_homingPhase = HOMING_PHASE_IDLE;
                        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: slow approach motor did not start.");
                    }
                    break;
                case SLOW_APPROACH_MOVING:
                    if (isHomeSensorTriggered()) {
                        m_motor->MoveStopAbrupt();
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Sensor triggered (slow).");
                        m_homingPhase = SET_ZERO;
                    } else if (!m_motor->StatusReg().bit.StepsActive) {
                        m_state = VALVE_ERROR;
                        m_isHomed = false;
                        m_homingPhase = HOMING_PHASE_IDLE;
                        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: sensor not found during slow approach.");
                    }
                    break;

                case SET_ZERO: {
                    m_motor->PositionRefSet(0);
                    m_isHomed = true;
                    m_state = VALVE_OPEN;
                    m_homingPhase = HOMING_PHASE_IDLE;
                    char msg[64];
                    snprintf(msg, sizeof(msg), "%s homing complete. Valve is OPEN.", m_name);
                    reportEvent(STATUS_PREFIX_DONE, msg);
                    break;
                }
                default:
                    m_motor->MoveStopAbrupt();
                    m_state = VALVE_ERROR;
                    m_isHomed = false;
                    m_homingPhase = HOMING_PHASE_IDLE;
                    break;
            }
            break;
        }

        case VALVE_MOVING: {
            switch(m_opPhase) {
                case PHASE_WAIT_TO_START:
                    if (m_motor->StatusReg().bit.StepsActive) {
                        m_opPhase = PHASE_MOVING;
                    } else if (Milliseconds() - m_moveStartTime > 500) {
                        m_state = VALVE_ERROR;
                        m_isHomed = false;
                        m_opPhase = PHASE_IDLE;
                        m_moveType = MOVE_TYPE_NONE;
                        reportEvent(STATUS_PREFIX_ERROR, "Move failed: Motor did not start.");
                    }
                    break;
                case PHASE_MOVING:
                    if (m_moveType == MOVE_TYPE_OPEN) {
                        if (checkTorqueLimit()) {
                            abort();
                            m_state = VALVE_ERROR;
                            m_isHomed = false;
                            reportEvent(STATUS_PREFIX_ERROR, "Open failed: Torque limit hit unexpectedly.");
                        } else if (!m_motor->StatusReg().bit.StepsActive) {
                            m_state = VALVE_OPEN;
                            reportEvent(STATUS_PREFIX_DONE, "Open complete.");
                        }
                    } else if (m_moveType == MOVE_TYPE_CLOSE) {
                        if (checkTorqueLimit()) {
                            abort();
                            m_state = VALVE_CLOSED;
                            char msg[64];
                            snprintf(msg, sizeof(msg), "%s closed.", m_name);
                            reportEvent(STATUS_PREFIX_DONE, msg);
                        } else if (!m_motor->StatusReg().bit.StepsActive) {
                            m_state = VALVE_ERROR;
                            m_isHomed = false;
                            reportEvent(STATUS_PREFIX_ERROR, "Close failed: Did not reach torque limit.");
                        }
                    }

                    if (m_state != VALVE_MOVING) {
                        m_opPhase = PHASE_IDLE;
                        m_moveType = MOVE_TYPE_NONE;
                    }
                    break;
                default:
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
                        m_isHomed = false;
                        m_opPhase = PHASE_IDLE;
                        reportEvent(STATUS_PREFIX_ERROR, "Move failed: Motor did not start.");
                    }
                    break;
                case PHASE_MOVING:
                    if (checkTorqueLimit()) {
                        abort();
                        m_state = m_isHomed ? VALVE_OPEN : VALVE_NOT_HOMED;
                        m_opPhase = PHASE_IDLE;
                    } else if (!m_motor->StatusReg().bit.StepsActive) {
                        m_state = m_isHomed ? VALVE_OPEN : VALVE_NOT_HOMED;
                        m_opPhase = PHASE_IDLE;
                        reportEvent(STATUS_PREFIX_DONE, "Jog complete.");
                    }
                    break;
                default:
                    m_state = VALVE_ERROR;
                    m_isHomed = false;
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

        case VALVE_NOT_HOMED:
        case VALVE_OPEN:
        case VALVE_CLOSED:
        case VALVE_HALTED:
        case VALVE_ERROR:
            break;
    }
}

// --- Command Dispatch ---

void PinchValve::handleCommand(Command cmd, const char* args) {
    if (isBusy() && cmd != CMD_ABORT && cmd != CMD_RESET) {
        reportEvent(STATUS_PREFIX_ERROR, "Valve is busy.");
        return;
    }
    if (m_state == VALVE_ERROR && cmd != CMD_RESET) {
        reportEvent(STATUS_PREFIX_ERROR, "Valve is in an error state. Reset required.");
        return;
    }

    switch(cmd) {
        case CMD_INJECTION_VALVE_HOME:
        case CMD_VACUUM_VALVE_HOME:
            home();
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

// --- Motion Commands ---

void PinchValve::home() {
    m_isHomed = false;
    m_state = VALVE_HOMING;
    m_homingPhase = CHECK_SENSOR;
    m_homingStartTime = Milliseconds();

    m_homingDistanceSteps = (long)(PINCH_HOMING_STROKE_MM * STEPS_PER_MM_PINCH);
    m_homingBackoffSteps  = (long)(PINCH_HOMING_BACKOFF_MM * STEPS_PER_MM_PINCH);
    m_homingRapidSps      = (int)(PINCH_HOMING_RAPID_VEL_MMS * STEPS_PER_MM_PINCH);
    m_homingSlowSps       = (int)(PINCH_HOMING_SLOW_VEL_MMS * STEPS_PER_MM_PINCH);
    m_homingAccelSps2     = (int)(PINCH_HOMING_ACCEL_MMSS * STEPS_PER_MM_PINCH);

    char msg[64];
    snprintf(msg, sizeof(msg), "%s homing started.", m_name);
    reportEvent(STATUS_PREFIX_INFO, msg);
}

void PinchValve::open() {
    if (!m_isHomed) {
        reportEvent(STATUS_PREFIX_ERROR, "Valve must be homed before opening.");
        return;
    }
    m_state = VALVE_MOVING;
    m_moveType = MOVE_TYPE_OPEN;
    m_opPhase = PHASE_WAIT_TO_START;
    m_moveStartTime = Milliseconds();
    m_torqueLimit = JOG_DEFAULT_TORQUE_PERCENT;

    long target_steps = 0;
    long current_steps = m_motor->PositionRefCommanded();
    int vel_sps = (int)(PINCH_VALVE_OPEN_VEL_MMS * STEPS_PER_MM_PINCH);
    int accel_sps2 = (int)(PINCH_VALVE_OPEN_ACCEL_MMSS * STEPS_PER_MM_PINCH);

    moveSteps(target_steps - current_steps, vel_sps, accel_sps2);
}

void PinchValve::close() {
    if (!m_isHomed) {
        reportEvent(STATUS_PREFIX_ERROR, "Valve must be homed before closing.");
        return;
    }
    m_state = VALVE_MOVING;
    m_moveType = MOVE_TYPE_CLOSE;
    m_opPhase = PHASE_WAIT_TO_START;
    m_moveStartTime = Milliseconds();
    m_torqueLimit = PINCH_VALVE_PINCH_TORQUE_PERCENT;

    long long_move_steps = (long)(PINCH_HOMING_STROKE_MM * STEPS_PER_MM_PINCH);
    int vel_sps = (int)(PINCH_VALVE_PINCH_VEL_MMS * STEPS_PER_MM_PINCH);
    int accel_sps2 = (int)(PINCH_JOG_DEFAULT_ACCEL_MMSS * STEPS_PER_MM_PINCH);

    moveSteps(long_move_steps, vel_sps, accel_sps2);
}

void PinchValve::jog(const char* args) {
    if (!args) {
        reportEvent(STATUS_PREFIX_ERROR, "Invalid jog command arguments.");
        return;
    }
    m_state = VALVE_JOGGING;
    m_moveType = MOVE_TYPE_NONE;
    m_opPhase = PHASE_WAIT_TO_START;
    m_moveStartTime = Milliseconds();
    m_torqueLimit = JOG_DEFAULT_TORQUE_PERCENT;

    float dist_mm = atof(args);
    long steps = (long)(dist_mm * STEPS_PER_MM_PINCH);
    int vel_sps = (int)(PINCH_JOG_DEFAULT_VEL_MMS * STEPS_PER_MM_PINCH);
    int accel_sps2_val = (int)(PINCH_JOG_DEFAULT_ACCEL_MMSS * STEPS_PER_MM_PINCH);

    moveSteps(steps, vel_sps, accel_sps2_val);
}

// --- Low-Level Helpers ---

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
    if (m_state != VALVE_ERROR) {
        return;
    }
    if (m_motor->StatusReg().bit.StepsActive) {
        m_motor->MoveStopAbrupt();
    }
    m_state = VALVE_RESETTING;
    m_homingPhase = HOMING_PHASE_IDLE;
    m_opPhase = PHASE_IDLE;
    m_moveType = MOVE_TYPE_NONE;
    m_firstTorqueReading = true;
}

// --- Torque Monitoring (used for close/jog, not homing) ---

float PinchValve::getInstantaneousTorque() {
    float currentRawTorque = m_motor->HlfbPercent();
    if (currentRawTorque == TORQUE_HLFB_AT_POSITION) {
        return 0.0f;
    }
    return currentRawTorque;
}

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

// --- NVM Home-on-Boot ---

bool PinchValve::getHomeOnBoot() const {
    return m_homeOnBoot;
}

bool PinchValve::setHomeOnBoot(const char* enabled) {
    NvmManager &nvmMgr = NvmManager::Instance();
    if (strcmp(enabled, "true") == 0) {
        m_homeOnBoot = true;
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(m_nvmSlot), 1);
        return true;
    } else if (strcmp(enabled, "false") == 0) {
        m_homeOnBoot = false;
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(m_nvmSlot), 0);
        return true;
    }
    return false;
}

// --- Reporting & Telemetry ---

void PinchValve::reportEvent(const char* statusType, const char* message) {
    char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
    snprintf(fullMsg, sizeof(fullMsg), "%s: %s", m_name, message);
    m_controller->reportEvent(statusType, fullMsg);
}

const char* PinchValve::getTelemetryString() {
    float displayTorque = getSmoothedTorque();
    snprintf(m_telemetryBuffer, sizeof(m_telemetryBuffer),
        "%s_pos:%.2f,%s_torque:%.1f,%s_homed:%d,%s_state:%d,%s_home_sensor:%d",
        m_name, (float)m_motor->PositionRefCommanded() / STEPS_PER_MM_PINCH,
        m_name, displayTorque,
        m_name, (int)m_isHomed,
        m_name, m_state,
        m_name, (int)isHomeSensorTriggered()
    );
    return m_telemetryBuffer;
}

// --- Status Queries ---

bool PinchValve::isBusy() const {
    return m_state == VALVE_HOMING || m_state == VALVE_MOVING || m_state == VALVE_JOGGING || m_state == VALVE_RESETTING;
}

bool PinchValve::isInFault() const {
    return m_motor->StatusReg().bit.MotorInFault;
}

bool PinchValve::isHomed() const {
    return m_isHomed;
}

bool PinchValve::isOpen() const {
    return m_state == VALVE_OPEN;
}

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
