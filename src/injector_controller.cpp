/**
 * @file injector_controller.cpp
 * @author Eldin Miller-Stead
 * @date September 10, 2025
 * @brief Implements the controller for the dual-motor injector system.
 *
 * @details This file provides the concrete implementation for the `Injector` class.
 * It is a SUPERSET of Pressboi's MotorController merged with the original Fillhead
 * injector controller. It contains the logic for the hierarchical state machines that
 * manage homing (gantry-squaring with hall sensors), feeding (injection), general
 * moves (move_abs/move_inc), and jogging operations. It also includes command handlers,
 * motion control logic, NVM persistence, force/torque calibration, and telemetry.
 */

//==================================================================================================
// --- Includes ---
//==================================================================================================
#include "injector_controller.h"
#include "fillhead.h"
#include "events.h"
#include "error_log.h"
#include "NvmManager.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>

using ClearCore::NvmManager;

//==================================================================================================
// --- Constructor ---
//==================================================================================================

Injector::Injector(MotorDriver* motorA, MotorDriver* motorB, Fillhead* controller) {
    m_motorA = motorA;
    m_motorB = motorB;
    m_controller = controller;

    m_state = STATE_STANDBY;
    m_homingState = HOMING_NONE;
    m_homingPhase = HOMING_PHASE_IDLE;
    m_feedState = FEED_STANDBY;
    m_moveState = MOVE_STANDBY;
    m_enableState = ENABLE_IDLE;

    m_homingMachineDone = false;
    m_homingCartridgeDone = false;
    m_homingStartTime = 0;
    m_isEnabled = true;
    m_retractDone = false;
    m_pausedMessageSent = false;
    m_originalMoveCommand = nullptr;
    m_enableStartTime = 0;

    m_axisAHomeSensorTriggered = false;
    m_axisBHomeSensorTriggered = false;
    m_axisAStopped = false;
    m_axisBStopped = false;
    m_homeSensorsInitialized = false;

    strcpy(m_force_mode, "load_cell");
    m_motor_torque_scale = 0.0335f;
    m_motor_torque_offset = 1.04f;

    strcpy(m_polarity, "normal");
    m_home_on_boot = true;

    m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
    m_torqueOffset = DEFAULT_INJECTOR_TORQUE_OFFSET;
    m_smoothedTorqueValue0 = 0.0f;
    m_smoothedTorqueValue1 = 0.0f;
    m_firstTorqueReading0 = true;
    m_firstTorqueReading1 = true;

    m_machineHomeReferenceSteps = 0;
    m_cartridgeHomeReferenceSteps = 0;
    m_retractReferenceSteps = LONG_MIN;
    m_retract_position_mm = 0.0f;
    m_press_threshold_kg = 2.0f;

    m_cumulative_dispensed_ml = 0.0f;
    m_cumulative_distance_mm = 0.0f;

    m_cartridge_ml_per_mm = DEFAULT_CARTRIDGE_ML_PER_MM;
    m_feedDefaultTorquePercent = FEED_DEFAULT_TORQUE_PERCENT;
    m_feedDefaultVelocitySPS = FEED_DEFAULT_VELOCITY_SPS;
    m_feedDefaultAccelSPS2 = FEED_DEFAULT_ACCEL_SPS2;

    m_moveDefaultTorquePercent = MOVE_DEFAULT_TORQUE_PERCENT;
    m_moveDefaultVelocitySPS = MOVE_DEFAULT_VELOCITY_SPS;
    m_moveDefaultAccelSPS2 = MOVE_DEFAULT_ACCEL_SPS2;

    m_homingDistanceSteps = 0;
    m_homingBackoffSteps = 0;
    m_homingRapidSps = 0;
    m_homingTouchSps = 0;
    m_homingBackoffSps = 0;
    m_homingAccelSps2 = 0;

    m_feedStartTime = 0;
    m_moveStartTime = 0;

    m_active_op_target_position_steps = 0;
    m_joules = 0.0;
    m_endpoint_mm = 0.0f;
    m_press_startpoint_mm = 0.0f;
    m_prev_position_mm = 0.0;

    m_machineStrainBaselinePosMm = 0.0;
    m_prevMachineDeflectionMm = 0.0;
    m_prevTotalDeflectionMm = 0.0;
    m_machineEnergyJ = 0.0;
    m_machineStrainContactActive = false;
    m_jouleIntegrationActive = false;
    m_forceLimitTriggered = false;
    m_prevForceValid = false;
    m_prevForceKg = 0.0f;

    m_machineStrainCoeffs[0] = MACHINE_STRAIN_COEFF_X4;
    m_machineStrainCoeffs[1] = MACHINE_STRAIN_COEFF_X3;
    m_machineStrainCoeffs[2] = MACHINE_STRAIN_COEFF_X2;
    m_machineStrainCoeffs[3] = MACHINE_STRAIN_COEFF_X1;
    m_machineStrainCoeffs[4] = MACHINE_STRAIN_COEFF_C;

    m_retractSpeedMms = RETRACT_DEFAULT_SPEED_MMS;

    fullyResetActiveDispenseOperation();
    fullyResetActiveMove();
    m_activeFeedCommand = nullptr;
    m_activeJogCommand = nullptr;
    m_activeMoveCommand = nullptr;
}

//==================================================================================================
// --- Setup ---
//==================================================================================================

void Injector::setup() {
    m_motorA->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    m_motorA->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    m_motorA->VelMax(MOTOR_DEFAULT_VEL_MAX_SPS);
    m_motorA->AccelMax(MOTOR_DEFAULT_ACCEL_MAX_SPS2);

    m_motorB->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    m_motorB->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    m_motorB->VelMax(MOTOR_DEFAULT_VEL_MAX_SPS);
    m_motorB->AccelMax(MOTOR_DEFAULT_ACCEL_MAX_SPS2);

    m_motorA->EnableRequest(true);
    m_motorB->EnableRequest(true);

    uint32_t enableTimeout = Milliseconds() + 2000;
    while (Milliseconds() < enableTimeout) {
        if (m_motorA->StatusReg().bit.Enabled && m_motorB->StatusReg().bit.Enabled) {
            break;
        }
    }
    if (!m_motorA->StatusReg().bit.Enabled || !m_motorB->StatusReg().bit.Enabled) {
        g_errorLog.logf(LOG_WARNING, "Motor enable timeout. M0=%d, M1=%d",
                 m_motorA->StatusReg().bit.Enabled ? 1 : 0,
                 m_motorB->StatusReg().bit.Enabled ? 1 : 0);
    }

    setupHomeSensors();

    // --- NVM Initialization (magic "FLH1" = 0x464C4831) ---
    NvmManager &nvmMgr = NvmManager::Instance();
    const int32_t NVM_MAGIC_NUMBER = 0x464C4831;
    int32_t magicValue = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(7 * 4));

    if (magicValue != NVM_MAGIC_NUMBER) {
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(3 * 4), 0);  // Polarity: normal
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(4 * 4), 1);  // Force mode: load_cell
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(5 * 4), (int32_t)(0.0335f * 100000.0f));
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(6 * 4), (int32_t)(1.04f * 10000.0f));

        for (int i = 0; i < 5; ++i) {
            int32_t coeffBits;
            memcpy(&coeffBits, &m_machineStrainCoeffs[i], sizeof(float));
            nvmMgr.Int32(static_cast<NvmManager::NvmLocations>((8 + i) * 4), coeffBits);
        }

        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(13 * 4), 1);  // Home on boot: true

        float defaultRetract = 0.0f;
        int32_t retractBitsInit;
        memcpy(&retractBitsInit, &defaultRetract, sizeof(float));
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(14 * 4), retractBitsInit);

        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_INJ_VALVE_HOME_ON_BOOT), 1);
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_VAC_VALVE_HOME_ON_BOOT), 1);
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_CARTRIDGE_ML_PER_MM), (int32_t)(DEFAULT_CARTRIDGE_ML_PER_MM * 10000.0f));

        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(7 * 4), NVM_MAGIC_NUMBER);
    }

    // Load polarity
    int32_t polarityValue = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(3 * 4));
    bool isInverted = (polarityValue == 1);
    strcpy(m_polarity, isInverted ? "inverted" : "normal");
    m_motorA->PolarityInvertSDDirection(isInverted);
    m_motorB->PolarityInvertSDDirection(isInverted);

    // Load force mode (0 = motor_torque, 1 = load_cell, default load_cell)
    int32_t forceModeValue = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(4 * 4));
    strcpy(m_force_mode, (forceModeValue == 0) ? "motor_torque" : "load_cell");

    // Load motor torque calibration
    int32_t scaleValue = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(5 * 4));
    int32_t offsetValue = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(6 * 4));

    if (scaleValue > 0 && scaleValue < 20000 && scaleValue != -1) {
        m_motor_torque_scale = (float)scaleValue / 100000.0f;
    } else {
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(5 * 4), (int32_t)(0.0335f * 100000.0f));
    }

    if (offsetValue > -100000 && offsetValue < 100000 && offsetValue != 0 && offsetValue != -1) {
        m_motor_torque_offset = (float)offsetValue / 10000.0f;
    } else {
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(6 * 4), (int32_t)(1.04f * 10000.0f));
    }

    // Load machine strain coefficients
    for (int i = 0; i < 5; ++i) {
        int32_t coeffBits = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>((8 + i) * 4));
        if (coeffBits != 0 && coeffBits != -1) {
            float tempCoeff;
            memcpy(&tempCoeff, &coeffBits, sizeof(float));
            if (tempCoeff > -5e9f && tempCoeff < 5e9f && std::fabs(tempCoeff) < 1e4f) {
                m_machineStrainCoeffs[i] = tempCoeff;
                continue;
            }
        }
        float defaultCoeff;
        switch (i) {
            case 0: defaultCoeff = MACHINE_STRAIN_COEFF_X4; break;
            case 1: defaultCoeff = MACHINE_STRAIN_COEFF_X3; break;
            case 2: defaultCoeff = MACHINE_STRAIN_COEFF_X2; break;
            case 3: defaultCoeff = MACHINE_STRAIN_COEFF_X1; break;
            default: defaultCoeff = MACHINE_STRAIN_COEFF_C; break;
        }
        m_machineStrainCoeffs[i] = defaultCoeff;
        int32_t defaultBits;
        memcpy(&defaultBits, &defaultCoeff, sizeof(float));
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>((8 + i) * 4), defaultBits);
    }

    // Load home on boot
    int32_t homeOnBootValue = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(13 * 4));
    m_home_on_boot = (homeOnBootValue != 0);

    // Load retract position (offset in mm from home)
    int32_t retractBits = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(14 * 4));
    if (retractBits != 0 && retractBits != -1) {
        float tempRetract;
        memcpy(&tempRetract, &retractBits, sizeof(float));
        if (tempRetract >= -100.0f && tempRetract <= 100.0f) {
            m_retract_position_mm = tempRetract;
        }
    }

    // Load press threshold (default 2.0 kg)
    m_press_threshold_kg = 2.0f;
    int32_t thresholdBits = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(15 * 4));
    if (thresholdBits != 0 && thresholdBits != -1) {
        float tempThreshold;
        memcpy(&tempThreshold, &thresholdBits, sizeof(float));
        if (tempThreshold >= 0.1f && tempThreshold <= 50.0f) {
            m_press_threshold_kg = tempThreshold;
        }
    }

    // Load cartridge ratio (ml/mm, stored as int * 10000)
    int32_t ratioBits = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_CARTRIDGE_ML_PER_MM));
    if (ratioBits > 0 && ratioBits != -1) {
        float tempRatio = (float)ratioBits / 10000.0f;
        if (tempRatio >= 0.01f && tempRatio <= 100.0f) {
            m_cartridge_ml_per_mm = tempRatio;
        }
    }
}

//==================================================================================================
// --- Machine Strain Compensation ---
//==================================================================================================

float Injector::evaluateMachineStrainForceFromDeflection(float deflection_mm) const {
    float x = deflection_mm;
    if (x < 0.0f) x = 0.0f;

    float force = (((m_machineStrainCoeffs[0] * x + m_machineStrainCoeffs[1]) * x
                    + m_machineStrainCoeffs[2]) * x + m_machineStrainCoeffs[3]) * x
                    + m_machineStrainCoeffs[4];
    if (force < 0.0f) force = 0.0f;
    return force;
}

float Injector::estimateMachineDeflectionFromForce(float force_kg) const {
    if (force_kg <= 0.0f) return 0.0f;

    float min_force = evaluateMachineStrainForceFromDeflection(0.0f);
    if (force_kg <= min_force) return 0.0f;

    float low = 0.0f;
    float high = MACHINE_STRAIN_MAX_DEFLECTION_MM;

    const float MAX_DEFLECTION = MACHINE_STRAIN_MAX_DEFLECTION_MM * 4.0f;
    int expansion_iterations = 0;
    while (evaluateMachineStrainForceFromDeflection(high) < force_kg
           && high < MAX_DEFLECTION && expansion_iterations < 20) {
        high *= 1.5f;
        expansion_iterations++;
        if (high > MAX_DEFLECTION) { high = MAX_DEFLECTION; break; }
    }

    for (int i = 0; i < 20; ++i) {
        float mid = 0.5f * (low + high);
        float f_mid = evaluateMachineStrainForceFromDeflection(mid);
        if (f_mid < force_kg) low = mid;
        else high = mid;
    }
    return high;
}

void Injector::setMachineStrainCoeffs(float x4, float x3, float x2, float x1, float c) {
    m_machineStrainCoeffs[0] = x4;
    m_machineStrainCoeffs[1] = x3;
    m_machineStrainCoeffs[2] = x2;
    m_machineStrainCoeffs[3] = x1;
    m_machineStrainCoeffs[4] = c;
    m_prevForceValid = false;
    m_prevTotalDeflectionMm = 0.0;
    m_prevMachineDeflectionMm = 0.0;
    m_machineEnergyJ = 0.0;
    m_machineStrainContactActive = false;

    NvmManager &nvmMgr = NvmManager::Instance();
    for (int i = 0; i < 5; ++i) {
        int32_t bits;
        memcpy(&bits, &m_machineStrainCoeffs[i], sizeof(float));
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>((8 + i) * 4), bits);
    }
}

//==================================================================================================
// --- Main State Machine ---
//==================================================================================================

void Injector::updateState() {
    updateJoules();

    switch (m_state) {
        case STATE_STANDBY:
        case STATE_MOTOR_FAULT:
            break;

        // ==================================================================
        // HOMING — Gantry-squaring with independent hall-effect sensors
        // ==================================================================
        case STATE_HOMING: {
            if (Milliseconds() - m_homingStartTime > MAX_HOMING_DURATION_MS) {
                abortMove();
                reportEvent(STATUS_PREFIX_ERROR, "Homing failed: Timeout exceeded.");
                m_state = STATE_STANDBY;
                m_homingPhase = HOMING_PHASE_IDLE;
                break;
            }

            switch (m_homingPhase) {
                // --- RAPID APPROACH ---
                case RAPID_APPROACH_START: {
                    reportEvent(STATUS_PREFIX_INFO, "Homing: Starting rapid approach (gantry squaring).");
                    m_axisAStopped = false;
                    m_axisBStopped = false;
                    m_axisAHomeSensorTriggered = false;
                    m_axisBHomeSensorTriggered = false;
                    m_torqueLimit = INJECTOR_HOMING_BACKOFF_TORQUE_PERCENT;

                    long rapid_search_steps = m_homingDistanceSteps;
                    if (m_homingState == HOMING_MACHINE)
                        rapid_search_steps = -rapid_search_steps;

                    startMove(rapid_search_steps, m_homingRapidSps, m_homingAccelSps2);
                    m_homingPhase = RAPID_APPROACH_WAIT_TO_START;
                    break;
                }

                case RAPID_APPROACH_WAIT_TO_START: {
                    bool m0_moving = m_motorA->StatusReg().bit.StepsActive;
                    bool m1_moving = m_motorB->StatusReg().bit.StepsActive;
                    if (m0_moving && m1_moving) {
                        m_homingPhase = RAPID_APPROACH_MOVING;
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Rapid approach moving, monitoring sensors.");
                    } else if (Milliseconds() - m_homingStartTime > 500) {
                        abortMove();
                        char errorMsg[200];
                        snprintf(errorMsg, sizeof(errorMsg),
                            "Homing failed: Not all motors started. M0=%d M1=%d M0_status=0x%04X M1_status=0x%04X",
                            m0_moving ? 1 : 0, m1_moving ? 1 : 0,
                            (unsigned int)m_motorA->StatusReg().reg,
                            (unsigned int)m_motorB->StatusReg().reg);
                        reportEvent(STATUS_PREFIX_ERROR, errorMsg);
                        m_state = STATE_STANDBY;
                        m_homingPhase = HOMING_PHASE_IDLE;
                    }
                    break;
                }

                case RAPID_APPROACH_MOVING: {
                    if (!m_axisAStopped && isHomeSensorTriggered(0)) {
                        stopAxis(0);
                        m_axisAStopped = true;
                        m_axisAHomeSensorTriggered = true;
                        reportEvent(STATUS_PREFIX_INFO, "Homing: M0 sensor triggered (rapid).");
                    }
                    if (!m_axisBStopped && isHomeSensorTriggered(1)) {
                        stopAxis(1);
                        m_axisBStopped = true;
                        m_axisBHomeSensorTriggered = true;
                        reportEvent(STATUS_PREFIX_INFO, "Homing: M1 sensor triggered (rapid).");
                    }
                    if (checkTorqueLimit()) {
                        abortMove();
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Torque limit hit during rapid approach (backup safety).");
                        m_axisAStopped = true;
                        m_axisBStopped = true;
                    }

                    if (m_axisAStopped && m_axisBStopped) {
                        if (m_axisAHomeSensorTriggered || m_axisBHomeSensorTriggered) {
                            reportEvent(STATUS_PREFIX_INFO, "Homing: Rapid approach complete, starting backoff.");
                            m_homingPhase = BACKOFF_START;
                        } else {
                            reportEvent(STATUS_PREFIX_ERROR, "Homing failed: No sensors triggered during rapid approach.");
                            m_state = STATE_STANDBY;
                            m_homingPhase = HOMING_PHASE_IDLE;
                        }
                    } else if (!isMoving()) {
                        abortMove();
                        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: Motion stopped before sensors triggered.");
                        m_state = STATE_STANDBY;
                        m_homingPhase = HOMING_PHASE_IDLE;
                    }
                    break;
                }

                // --- BACKOFF ---
                case BACKOFF_START: {
                    reportEvent(STATUS_PREFIX_INFO, "Homing: Starting backoff.");
                    m_axisAStopped = false;
                    m_axisBStopped = false;
                    m_axisAHomeSensorTriggered = false;
                    m_axisBHomeSensorTriggered = false;
                    m_torqueLimit = INJECTOR_HOMING_BACKOFF_TORQUE_PERCENT;

                    long backoff_steps = (m_homingState == HOMING_MACHINE)
                                          ? m_homingBackoffSteps : -m_homingBackoffSteps;
                    startMove(backoff_steps, m_homingBackoffSps, m_homingAccelSps2);
                    m_homingPhase = BACKOFF_WAIT_TO_START;
                    break;
                }

                case BACKOFF_WAIT_TO_START:
                    if (isMoving()) m_homingPhase = BACKOFF_MOVING;
                    break;

                case BACKOFF_MOVING:
                    if (!isMoving()) {
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Backoff complete, starting slow approach.");
                        m_homingPhase = SLOW_APPROACH_START;
                    }
                    break;

                // --- SLOW APPROACH ---
                case SLOW_APPROACH_START: {
                    reportEvent(STATUS_PREFIX_INFO, "Homing: Starting slow approach for precision.");
                    m_axisAStopped = false;
                    m_axisBStopped = false;
                    m_axisAHomeSensorTriggered = false;
                    m_axisBHomeSensorTriggered = false;
                    m_torqueLimit = INJECTOR_HOMING_BACKOFF_TORQUE_PERCENT;

                    long slow_steps = (m_homingState == HOMING_MACHINE)
                                       ? -m_homingBackoffSteps * 2 : m_homingBackoffSteps * 2;
                    startMove(slow_steps, m_homingTouchSps, m_homingAccelSps2);
                    m_homingPhase = SLOW_APPROACH_WAIT_TO_START;
                    break;
                }

                case SLOW_APPROACH_WAIT_TO_START:
                    if (isMoving()) {
                        m_homingPhase = SLOW_APPROACH_MOVING;
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Slow approach moving, monitoring sensors.");
                    }
                    break;

                case SLOW_APPROACH_MOVING: {
                    if (!m_axisAStopped && isHomeSensorTriggered(0)) {
                        stopAxis(0);
                        m_axisAStopped = true;
                        m_axisAHomeSensorTriggered = true;
                        reportEvent(STATUS_PREFIX_INFO, "Homing: M0 sensor triggered (slow) - precise position found.");
                    }
                    if (!m_axisBStopped && isHomeSensorTriggered(1)) {
                        stopAxis(1);
                        m_axisBStopped = true;
                        m_axisBHomeSensorTriggered = true;
                        reportEvent(STATUS_PREFIX_INFO, "Homing: M1 sensor triggered (slow) - precise position found.");
                    }
                    if (checkTorqueLimit()) {
                        abortMove();
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Torque limit hit during slow approach (backup safety).");
                        m_axisAStopped = true;
                        m_axisBStopped = true;
                    }

                    if (m_axisAStopped && m_axisBStopped) {
                        if (m_axisAHomeSensorTriggered && m_axisBHomeSensorTriggered) {
                            reportEvent(STATUS_PREFIX_INFO, "Homing: Both sensors triggered, gantry squared. Moving to offset.");
                            m_homingPhase = FINAL_BACKOFF_START;
                        } else if (m_axisAHomeSensorTriggered || m_axisBHomeSensorTriggered) {
                            char warnMsg[128];
                            snprintf(warnMsg, sizeof(warnMsg),
                                     "Homing: Warning - only %s sensor triggered during slow approach.",
                                     m_axisAHomeSensorTriggered ? "M0" : "M1");
                            reportEvent(STATUS_PREFIX_INFO, warnMsg);
                            m_homingPhase = FINAL_BACKOFF_START;
                        } else {
                            reportEvent(STATUS_PREFIX_ERROR, "Homing failed: No sensors triggered during slow approach.");
                            m_state = STATE_STANDBY;
                            m_homingPhase = HOMING_PHASE_IDLE;
                        }
                    } else if (!isMoving()) {
                        abortMove();
                        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: Motion stopped before sensors triggered (slow).");
                        m_state = STATE_STANDBY;
                        m_homingPhase = HOMING_PHASE_IDLE;
                    }
                    break;
                }

                // --- FINAL BACKOFF ---
                case FINAL_BACKOFF_START: {
                    reportEvent(STATUS_PREFIX_INFO, "Homing: Moving to final offset position.");
                    m_torqueLimit = INJECTOR_HOMING_BACKOFF_TORQUE_PERCENT;
                    long offset_steps = (m_homingState == HOMING_MACHINE)
                                         ? m_homingBackoffSteps : -m_homingBackoffSteps;
                    startMove(offset_steps, m_homingBackoffSps, m_homingAccelSps2);
                    m_homingPhase = FINAL_BACKOFF_WAIT_TO_START;
                    break;
                }

                case FINAL_BACKOFF_WAIT_TO_START:
                    if (isMoving()) m_homingPhase = FINAL_BACKOFF_MOVING;
                    break;

                case FINAL_BACKOFF_MOVING:
                    if (!isMoving()) {
                        reportEvent(STATUS_PREFIX_INFO, "Homing: Final position reached.");
                        m_homingPhase = SET_ZERO;
                    }
                    break;

                // --- SET ZERO ---
                case SET_ZERO: {
                    const char* commandStr = (m_homingState == HOMING_MACHINE)
                                              ? CMD_STR_MACHINE_HOME_MOVE : CMD_STR_CARTRIDGE_HOME_MOVE;

                    if (m_homingState == HOMING_MACHINE) {
                        m_machineHomeReferenceSteps = m_motorA->PositionRefCommanded();
                        m_homingMachineDone = true;

                        if (m_retract_position_mm != 0.0f) {
                            m_retractReferenceSteps = m_machineHomeReferenceSteps
                                + static_cast<long>(m_retract_position_mm * STEPS_PER_MM_INJECTOR);
                            char dbg[128];
                            snprintf(dbg, sizeof(dbg),
                                     "Retract position recalculated after homing: %.2f mm (steps=%ld, home=%ld)",
                                     m_retract_position_mm, m_retractReferenceSteps, m_machineHomeReferenceSteps);
                            reportEvent(STATUS_PREFIX_INFO, dbg);
                        }

                        char sensorMsg[128];
                        snprintf(sensorMsg, sizeof(sensorMsg),
                                 "Homing complete. Sensor states: M0=%d, M1=%d",
                                 getHomeSensorState(0) ? 1 : 0, getHomeSensorState(1) ? 1 : 0);
                        reportEvent(STATUS_PREFIX_INFO, sensorMsg);
                    } else {
                        m_cartridgeHomeReferenceSteps = m_motorA->PositionRefCommanded();
                        m_homingCartridgeDone = true;
                    }

                    char doneMsg[STATUS_MESSAGE_BUFFER_SIZE];
                    snprintf(doneMsg, sizeof(doneMsg), "%s complete.", commandStr);
                    reportEvent(STATUS_PREFIX_DONE, doneMsg);

                    m_state = STATE_STANDBY;
                    m_homingPhase = HOMING_PHASE_IDLE;
                    break;
                }

                case HOMING_PHASE_ERROR:
                    reportEvent(STATUS_PREFIX_ERROR, "Injector homing sequence ended with error.");
                    m_state = STATE_STANDBY;
                    m_homingPhase = HOMING_PHASE_IDLE;
                    break;

                default:
                    abortMove();
                    reportEvent(STATUS_PREFIX_ERROR, "Unknown homing phase, aborting.");
                    m_state = STATE_STANDBY;
                    m_homingPhase = HOMING_PHASE_IDLE;
                    break;
            }
            break;
        }

        // ==================================================================
        // FEEDING — Injection operations (preserved from original Fillhead)
        // ==================================================================
        case STATE_FEEDING: {
            if (checkTorqueLimit()) {
                abortMove();
                reportEvent(STATUS_PREFIX_ERROR, "FEED_MODE: Torque limit! Operation stopped.");
                finalizeAndResetActiveDispenseOperation(false);
                m_state = STATE_STANDBY;
                return;
            }

            if (m_feedState == FEED_INJECT_ACTIVE && m_active_op_feed_force_limit_kg > 0.1f) {
                const char* mode = getForceMode();
                if (strcmp(mode, "load_cell") == 0) {
                    const char* errorMsg = nullptr;
                    if (checkForceSensorStatus(&errorMsg)) {
                        abortMove();
                        char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
                        snprintf(fullMsg, sizeof(fullMsg), "Inject stopped: %s", errorMsg);
                        reportEvent(STATUS_PREFIX_ERROR, fullMsg);
                        finalizeAndResetActiveDispenseOperation(false);
                        m_state = STATE_STANDBY;
                        return;
                    }
                    float current_force = m_controller->m_forceSensor.getForce();
                    if (current_force >= m_active_op_feed_force_limit_kg) {
                        char limit_desc[STATUS_MESSAGE_BUFFER_SIZE];
                        snprintf(limit_desc, sizeof(limit_desc), "Inject force limit (%.1f kg, actual: %.1f kg)",
                                 m_active_op_feed_force_limit_kg, current_force);
                        handleFeedLimitReached(limit_desc, current_force);
                        return;
                    }
                }
            }

            if (!isMoving() && m_feedState != FEED_INJECT_PAUSED) {
                bool isStarting = (m_feedState == FEED_INJECT_STARTING);
                uint32_t elapsed = Milliseconds() - m_feedStartTime;

                if (!isStarting || (isStarting && elapsed > MOVE_START_TIMEOUT_MS)) {
                    if (m_activeFeedCommand) {
                        char doneMsg[STATUS_MESSAGE_BUFFER_SIZE];
                        std::snprintf(doneMsg, sizeof(doneMsg), "%s complete.", m_activeFeedCommand);
                        reportEvent(STATUS_PREFIX_DONE, doneMsg);
                    }
                    finalizeAndResetActiveDispenseOperation(true);
                    m_state = STATE_STANDBY;
                }
            }

            if ((m_feedState == FEED_INJECT_STARTING || m_feedState == FEED_INJECT_RESUMING) && isMoving()) {
                m_feedState = FEED_INJECT_ACTIVE;
                m_active_op_segment_initial_axis_steps = m_motorA->PositionRefCommanded();
            }

            if (m_feedState == FEED_INJECT_ACTIVE) {
                if (m_active_op_steps_per_ml > 0.0001f) {
                    long current_pos = m_motorA->PositionRefCommanded();
                    long steps_moved_since_start = current_pos - m_active_op_initial_axis_steps;
                    m_active_op_total_dispensed_ml = (float)std::abs(steps_moved_since_start) / m_active_op_steps_per_ml;
                }
            }

            if (m_feedState == FEED_INJECT_PAUSED && !isMoving()) {
                if (m_active_op_steps_per_ml > 0.0001f) {
                    long total_steps_dispensed = (long)(m_active_op_total_dispensed_ml * m_active_op_steps_per_ml);
                    m_active_op_remaining_steps = m_active_op_total_target_steps - total_steps_dispensed;
                    if (m_active_op_remaining_steps < 0) m_active_op_remaining_steps = 0;
                }
                reportEvent(STATUS_PREFIX_INFO, "Feed Op: Operation Paused. Waiting for Resume/Cancel.");
            }
            break;
        }

        // ==================================================================
        // JOGGING — Manual jog moves (preserved from original Fillhead)
        // ==================================================================
        case STATE_JOGGING: {
            if (checkTorqueLimit()) {
                abortMove();
                reportEvent(STATUS_PREFIX_INFO, "JOG: Torque limit. Move stopped.");
                m_state = STATE_STANDBY;
                if (m_activeJogCommand) m_activeJogCommand = nullptr;
            } else if (!isMoving()) {
                if (m_activeJogCommand) {
                    char doneMsg[STATUS_MESSAGE_BUFFER_SIZE];
                    std::snprintf(doneMsg, sizeof(doneMsg), "%s complete.", m_activeJogCommand);
                    reportEvent(STATUS_PREFIX_DONE, doneMsg);
                    m_activeJogCommand = nullptr;
                }
                m_state = STATE_STANDBY;
            }
            break;
        }

        // ==================================================================
        // MOVING — General move_abs / move_inc (from Pressboi)
        // ==================================================================
        case STATE_MOVING: {
            // Force / torque limit checking during active moves
            if (m_moveState == MOVE_ACTIVE) {
                if (strcmp(m_active_op_force_mode, "load_cell") == 0) {
                    const char* errorMsg = nullptr;
                    if (checkForceSensorStatus(&errorMsg)) {
                        abortMove();
                        char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
                        snprintf(fullMsg, sizeof(fullMsg), "Move stopped: %s", errorMsg);
                        reportEvent(STATUS_PREFIX_ERROR, fullMsg);
                        m_moveState = MOVE_PAUSED;
                        return;
                    }
                    if (m_active_op_force_limit_kg > 0.1f) {
                        float current_force = m_controller->m_forceSensor.getForce();
                        if (current_force >= m_active_op_force_limit_kg) {
                            char limit_desc[STATUS_MESSAGE_BUFFER_SIZE];
                            snprintf(limit_desc, sizeof(limit_desc), "Force limit (%.1f kg, actual: %.1f kg)",
                                     m_active_op_force_limit_kg, current_force);
                            handleLimitReached(limit_desc, current_force);
                            return;
                        }
                    }
                } else {
                    if (checkTorqueLimit()) {
                        char limit_desc[STATUS_MESSAGE_BUFFER_SIZE];
                        snprintf(limit_desc, sizeof(limit_desc), "Torque limit (%.1f%%)", m_torqueLimit);
                        handleLimitReached(limit_desc, m_torqueLimit);
                        return;
                    }
                }
            }

            // Transition from STARTING/RESUMING to ACTIVE
            if ((m_moveState == MOVE_STARTING || m_moveState == MOVE_RESUMING) && isMoving()) {
                m_moveState = MOVE_ACTIVE;
                m_active_op_segment_initial_axis_steps = m_motorA->PositionRefCommanded();
            }

            // Check for move completion
            if (!isMoving() && m_moveState != MOVE_PAUSED) {
                bool isStarting = (m_moveState == MOVE_STARTING || m_moveState == MOVE_RESUMING);
                uint32_t elapsed = Milliseconds() - m_moveStartTime;

                if (isStarting && elapsed > MOVE_START_TIMEOUT_MS) {
                    if (m_activeMoveCommand) {
                        reportEvent(STATUS_PREFIX_ERROR, "Move timeout: Motor failed to start");
                    }
                    finalizeAndResetActiveMove(false);
                    m_state = STATE_STANDBY;
                } else if (!isStarting) {
                    // Record endpoint for press moves (not retracts)
                    if (m_activeMoveCommand &&
                        (strcmp(m_activeMoveCommand, "move_abs") == 0 ||
                         strcmp(m_activeMoveCommand, "move_inc") == 0)) {
                        long current_pos_steps = m_motorA->PositionRefCommanded();
                        m_endpoint_mm = static_cast<float>(
                            static_cast<double>(current_pos_steps - m_machineHomeReferenceSteps)
                            / STEPS_PER_MM_INJECTOR);
                    }

                    // Auto-retract after move completion if configured
                    if (strcmp(m_active_op_force_action, "retract") == 0) {
                        reportEvent(STATUS_PREFIX_INFO, "Move complete, retracting...");
                        m_originalMoveCommand = m_activeMoveCommand;
                        long retract_target = (m_retractReferenceSteps == LONG_MIN)
                            ? m_machineHomeReferenceSteps : m_retractReferenceSteps;
                        m_active_op_force_action[0] = '\0';
                        m_moveState = MOVE_TO_HOME;
                        m_activeMoveCommand = "retract";
                        m_active_op_target_position_steps = retract_target;
                        long current_pos = m_motorA->PositionRefCommanded();
                        long steps_to_retract = retract_target - current_pos;
                        m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
                        float speed_mms = (m_retractSpeedMms > 0.0f)
                            ? m_retractSpeedMms : RETRACT_DEFAULT_SPEED_MMS;
                        if (speed_mms > 100.0f) speed_mms = 100.0f;
                        int velocity_sps = static_cast<int>(speed_mms * STEPS_PER_MM_INJECTOR);
                        m_active_op_velocity_sps = velocity_sps;
                        m_active_op_accel_sps2 = m_moveDefaultAccelSPS2;
                        startMove(steps_to_retract, velocity_sps, m_moveDefaultAccelSPS2);
                        reportEvent(STATUS_PREFIX_START, "retract");
                    } else {
                        if (m_originalMoveCommand != nullptr) {
                            reportEvent(STATUS_PREFIX_DONE, m_originalMoveCommand);
                            m_originalMoveCommand = nullptr;
                        } else if (m_activeMoveCommand) {
                            reportEvent(STATUS_PREFIX_DONE, m_activeMoveCommand);
                        }
                        finalizeAndResetActiveMove(true);
                        m_state = STATE_STANDBY;
                    }
                }
            }

            // Track distance during active moves
            if (m_moveState == MOVE_ACTIVE) {
                long current_pos = m_motorA->PositionRefCommanded();
                long steps_moved = current_pos - m_active_op_initial_axis_steps;
                m_active_op_total_distance_mm = (float)std::abs(steps_moved) / STEPS_PER_MM_INJECTOR;
            }

            // Handle paused state (calculate remaining, report once)
            if (m_moveState == MOVE_PAUSED && !isMoving()) {
                if (!m_pausedMessageSent) {
                    long current_pos = m_motorA->PositionRefCommanded();
                    long steps_moved_so_far = std::abs(current_pos - m_active_op_initial_axis_steps);
                    m_active_op_remaining_steps = m_active_op_total_target_steps - steps_moved_so_far;
                    if (m_active_op_remaining_steps < 0) m_active_op_remaining_steps = 0;
                    reportEvent(STATUS_PREFIX_INFO, "Move: Operation Paused. Waiting for Resume/Cancel.");
                    m_pausedMessageSent = true;
                }
            } else {
                m_pausedMessageSent = false;
            }
            break;
        }
    }
}

//==================================================================================================
// --- Command Dispatch ---
//==================================================================================================

void Injector::handleCommand(Command cmd, const char* args) {
    if (!m_isEnabled) {
        reportEvent(STATUS_PREFIX_ERROR, "Injector command ignored: Motors are disabled.");
        return;
    }

    if (m_motorA->StatusReg().bit.MotorInFault || m_motorB->StatusReg().bit.MotorInFault) {
        char errorMsg[200];
        snprintf(errorMsg, sizeof(errorMsg),
                 "Injector command ignored: Motor in fault. M0 Status=0x%04X, M1 Status=0x%04X",
                 (unsigned int)m_motorA->StatusReg().reg,
                 (unsigned int)m_motorB->StatusReg().reg);
        reportEvent(STATUS_PREFIX_ERROR, errorMsg);
        return;
    }

    // Block new motion commands while busy
    if (m_state != STATE_STANDBY &&
        (cmd == CMD_JOG_MOVE || cmd == CMD_MACHINE_HOME_MOVE || cmd == CMD_CARTRIDGE_HOME_MOVE ||
         cmd == CMD_INJECT ||
         cmd == CMD_HOME || cmd == CMD_MOVE_ABS || cmd == CMD_MOVE_INC || cmd == CMD_RETRACT ||
         cmd == CMD_MOVE_TO_CARTRIDGE_HOME || cmd == CMD_MOVE_TO_CARTRIDGE_RETRACT)) {
        reportEvent(STATUS_PREFIX_ERROR, "Injector command ignored: Another operation is in progress.");
        return;
    }

    switch (cmd) {
        // --- Existing Fillhead Injection Commands ---
        case CMD_JOG_MOVE:                  jogMove(args); break;
        case CMD_MACHINE_HOME_MOVE:         machineHome(); break;
        case CMD_CARTRIDGE_HOME_MOVE:       cartridgeHome(); break;
        case CMD_MOVE_TO_CARTRIDGE_HOME:    moveToCartridgeHome(); break;
        case CMD_MOVE_TO_CARTRIDGE_RETRACT: moveToCartridgeRetract(args); break;
        case CMD_INJECT:
            initiateInjectMove(args);
            break;
        case CMD_SET_CARTRIDGE_ML_PER_MM:
            setCartridgeMlPerMm(args);
            break;
        case CMD_PAUSE_INJECTION:           pauseOperation(); break;
        case CMD_RESUME_INJECTION:          resumeOperation(); break;
        case CMD_CANCEL_INJECTION:          cancelOperation(); break;

        // --- Pressboi General Motion Commands ---
        case CMD_HOME:                      home(); break;
        case CMD_MOVE_ABS:                  moveAbsolute(args); break;
        case CMD_MOVE_INC:                  moveIncremental(args); break;
        case CMD_SET_RETRACT:               setRetract(args); break;
        case CMD_RETRACT:                   retract(args); break;
        case CMD_PAUSE:                     pauseGeneralMove(); break;
        case CMD_RESUME:                    resumeGeneralMove(); break;
        case CMD_CANCEL:                    cancelGeneralMove(); break;

        default: break;
    }
}

//==================================================================================================
// --- Enable / Disable ---
//==================================================================================================

void Injector::enable() {
    m_motorA->ClearAlerts();
    m_motorB->ClearAlerts();

    m_motorA->EnableRequest(true);
    m_motorB->EnableRequest(true);

    m_motorA->VelMax(MOTOR_DEFAULT_VEL_MAX_SPS);
    m_motorA->AccelMax(MOTOR_DEFAULT_ACCEL_MAX_SPS2);
    m_motorB->VelMax(MOTOR_DEFAULT_VEL_MAX_SPS);
    m_motorB->AccelMax(MOTOR_DEFAULT_ACCEL_MAX_SPS2);

    m_enableState = ENABLE_WAITING;
    m_enableStartTime = Milliseconds();
    m_isEnabled = true;

    reportEvent(STATUS_PREFIX_INFO, "Injector motors enabling...");
}

EnableState Injector::updateEnableState() {
    if (m_enableState == ENABLE_WAITING) {
        if (m_motorA->StatusReg().bit.Enabled && m_motorB->StatusReg().bit.Enabled) {
            m_enableState = ENABLE_COMPLETE;
            reportEvent(STATUS_PREFIX_INFO, "Injector motors enabled.");
        } else if (Milliseconds() - m_enableStartTime > 2000) {
            m_enableState = ENABLE_TIMEOUT;
            char warnMsg[128];
            snprintf(warnMsg, sizeof(warnMsg), "Motor enable timeout. M0=%d, M1=%d",
                     m_motorA->StatusReg().bit.Enabled ? 1 : 0,
                     m_motorB->StatusReg().bit.Enabled ? 1 : 0);
            reportEvent(STATUS_PREFIX_INFO, warnMsg);
        }
    }
    return m_enableState;
}

bool Injector::isEnableComplete() const {
    return m_enableState == ENABLE_COMPLETE || m_enableState == ENABLE_TIMEOUT;
}

void Injector::disable() {
    m_motorA->EnableRequest(false);
    m_motorB->EnableRequest(false);
    m_isEnabled = false;
    m_enableState = ENABLE_IDLE;
    reportEvent(STATUS_PREFIX_INFO, "Injector motors disabled.");
}

//==================================================================================================
// --- Abort / Reset ---
//==================================================================================================

void Injector::abortMove() {
    m_motorA->MoveStopDecel();
    m_motorB->MoveStopDecel();
}

void Injector::reset() {
    m_state = STATE_STANDBY;
    m_homingState = HOMING_NONE;
    m_homingPhase = HOMING_PHASE_IDLE;
    m_feedState = FEED_STANDBY;
    m_moveState = MOVE_STANDBY;
    m_enableState = ENABLE_IDLE;
    fullyResetActiveDispenseOperation();
    fullyResetActiveMove();
}

//==================================================================================================
// --- Homing Commands ---
//==================================================================================================

void Injector::home() {
    machineHome();
}

void Injector::machineHome() {
    if (!m_homeSensorsInitialized) {
        setupHomeSensors();
    }

    char statusMsg[200];
    snprintf(statusMsg, sizeof(statusMsg),
        "Home: Motor status: M0(en=%d,fault=%d,status=0x%04X) M1(en=%d,fault=%d,status=0x%04X)",
        m_motorA->StatusReg().bit.Enabled ? 1 : 0,
        m_motorA->StatusReg().bit.MotorInFault ? 1 : 0,
        (unsigned int)m_motorA->StatusReg().reg,
        m_motorB->StatusReg().bit.Enabled ? 1 : 0,
        m_motorB->StatusReg().bit.MotorInFault ? 1 : 0,
        (unsigned int)m_motorB->StatusReg().reg);
    reportEvent(STATUS_PREFIX_INFO, statusMsg);

    if (!m_motorA->StatusReg().bit.Enabled || !m_motorB->StatusReg().bit.Enabled) {
        char errorMsg[128];
        snprintf(errorMsg, sizeof(errorMsg),
            "Homing failed: Motors not enabled. M0=%d, M1=%d",
            m_motorA->StatusReg().bit.Enabled ? 1 : 0,
            m_motorB->StatusReg().bit.Enabled ? 1 : 0);
        reportEvent(STATUS_PREFIX_ERROR, errorMsg);
        return;
    }

    m_homingDistanceSteps = (long)(fabs(INJECTOR_HOMING_STROKE_MM) * STEPS_PER_MM_INJECTOR);
    m_homingBackoffSteps = (long)(INJECTOR_HOMING_BACKOFF_MM * STEPS_PER_MM_INJECTOR);
    m_homingRapidSps = (int)fabs(INJECTOR_HOMING_RAPID_VEL_MMS * STEPS_PER_MM_INJECTOR);
    m_homingBackoffSps = (int)fabs(INJECTOR_HOMING_BACKOFF_VEL_MMS * STEPS_PER_MM_INJECTOR);
    m_homingTouchSps = (int)fabs(INJECTOR_HOMING_TOUCH_VEL_MMS * STEPS_PER_MM_INJECTOR);
    m_homingAccelSps2 = (int)fabs(INJECTOR_HOMING_ACCEL_MMSS * STEPS_PER_MM_INJECTOR);

    m_active_op_target_position_steps = 0;

    char logMsg[200];
    snprintf(logMsg, sizeof(logMsg), "Homing params: dist_steps=%ld, rapid_sps=%d, touch_sps=%d, accel_sps2=%d",
             m_homingDistanceSteps, m_homingRapidSps, m_homingTouchSps, m_homingAccelSps2);
    reportEvent(STATUS_PREFIX_INFO, logMsg);

    char sensorMsg[128];
    snprintf(sensorMsg, sizeof(sensorMsg), "Home sensors before homing: M0(DI7)=%d, M1(DI6)=%d (active=%s)",
             getHomeSensorState(0) ? 1 : 0,
             getHomeSensorState(1) ? 1 : 0,
             HOME_SENSOR_ACTIVE_STATE ? "HIGH" : "LOW");
    reportEvent(STATUS_PREFIX_INFO, sensorMsg);

    if (m_homingDistanceSteps == 0) {
        reportEvent(STATUS_PREFIX_ERROR, "Homing failed: Calculated distance is zero. Check config.");
        return;
    }

    m_state = STATE_HOMING;
    m_homingState = HOMING_MACHINE;
    m_homingPhase = RAPID_APPROACH_START;
    m_homingStartTime = Milliseconds();
    m_homingMachineDone = false;

    m_axisAHomeSensorTriggered = false;
    m_axisBHomeSensorTriggered = false;
    m_axisAStopped = false;
    m_axisBStopped = false;

    m_joules = 0.0;
    m_press_startpoint_mm = 0.0f;
    long current_pos_steps = m_motorA->PositionRefCommanded();
    m_prev_position_mm = static_cast<double>(current_pos_steps - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;
    m_prevForceValid = false;
    m_forceLimitTriggered = false;
    m_jouleIntegrationActive = false;

    reportEvent(STATUS_PREFIX_START, "MACHINE_HOME_MOVE initiated (gantry squaring mode).");
}

void Injector::cartridgeHome() {
    if (!m_homeSensorsInitialized) {
        setupHomeSensors();
    }

    m_cumulative_dispensed_ml = 0.0f;

    m_homingDistanceSteps = (long)(fabs(INJECTOR_HOMING_STROKE_MM) * STEPS_PER_MM_INJECTOR);
    m_homingBackoffSteps = (long)(INJECTOR_HOMING_BACKOFF_MM * STEPS_PER_MM_INJECTOR);
    m_homingRapidSps = (int)fabs(INJECTOR_HOMING_RAPID_VEL_MMS * STEPS_PER_MM_INJECTOR);
    m_homingBackoffSps = (int)fabs(INJECTOR_HOMING_BACKOFF_VEL_MMS * STEPS_PER_MM_INJECTOR);
    m_homingTouchSps = (int)fabs(INJECTOR_HOMING_TOUCH_VEL_MMS * STEPS_PER_MM_INJECTOR);
    m_homingAccelSps2 = (int)fabs(INJECTOR_HOMING_ACCEL_MMSS * STEPS_PER_MM_INJECTOR);

    m_state = STATE_HOMING;
    m_homingState = HOMING_CARTRIDGE;
    m_homingPhase = RAPID_APPROACH_START;
    m_homingStartTime = Milliseconds();
    m_homingCartridgeDone = false;

    m_axisAHomeSensorTriggered = false;
    m_axisBHomeSensorTriggered = false;
    m_axisAStopped = false;
    m_axisBStopped = false;

    m_jouleIntegrationActive = false;

    reportEvent(STATUS_PREFIX_START, "CARTRIDGE_HOME_MOVE initiated.");
}

//==================================================================================================
// --- Injection Commands (Preserved from Fillhead) ---
//==================================================================================================

void Injector::moveToCartridgeHome() {
    if (!m_homingCartridgeDone) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Cartridge not homed.");
        return;
    }

    fullyResetActiveDispenseOperation();
    m_state = STATE_FEEDING;
    m_feedState = FEED_MOVING_TO_HOME;
    m_activeFeedCommand = CMD_STR_MOVE_TO_CARTRIDGE_HOME;

    long current_pos = m_motorA->PositionRefCommanded();
    long steps_to_move = m_cartridgeHomeReferenceSteps - current_pos;

    m_torqueLimit = (float)m_feedDefaultTorquePercent;
    startMove(steps_to_move, m_feedDefaultVelocitySPS, m_feedDefaultAccelSPS2);
}

void Injector::moveToCartridgeRetract(const char* args) {
    if (!m_homingCartridgeDone) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Cartridge not homed.");
        return;
    }

    float offset_mm = 0.0f;
    if (std::sscanf(args, "%f", &offset_mm) != 1 || offset_mm < 0) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Invalid offset for MOVE_TO_CARTRIDGE_RETRACT.");
        return;
    }

    fullyResetActiveDispenseOperation();
    m_state = STATE_FEEDING;
    m_feedState = FEED_MOVING_TO_RETRACT;
    m_activeFeedCommand = CMD_STR_MOVE_TO_CARTRIDGE_RETRACT;

    long offset_steps = (long)(offset_mm * STEPS_PER_MM_INJECTOR);
    long target_pos = m_cartridgeHomeReferenceSteps - offset_steps;
    long current_pos = m_motorA->PositionRefCommanded();
    long steps_to_move = target_pos - current_pos;

    m_torqueLimit = (float)m_feedDefaultTorquePercent;
    startMove(steps_to_move, m_feedDefaultVelocitySPS, m_feedDefaultAccelSPS2);
}

void Injector::initiateInjectMove(const char* args) {
    float volume_ml = 0.0f;
    float speed_ml_s = INJECT_DEFAULT_SPEED_MLS;
    float force_limit_kg = 0.0f;
    char force_action[16] = "abort";
    float accel_sps2 = (float)m_feedDefaultAccelSPS2;
    int torque_percent = m_feedDefaultTorquePercent;

    int parsed_count = std::sscanf(args, "%f %f %f %15s", &volume_ml, &speed_ml_s, &force_limit_kg, force_action);

    if (parsed_count >= 1) {
        float ml_per_mm = m_cartridge_ml_per_mm;
        float steps_per_ml = STEPS_PER_MM_INJECTOR / ml_per_mm;

        if (torque_percent <= 0 || torque_percent > 100) torque_percent = m_feedDefaultTorquePercent;
        if (volume_ml <= 0) { reportEvent(STATUS_PREFIX_ERROR, "Error: Inject volume must be positive."); return; }
        if (speed_ml_s <= 0) speed_ml_s = INJECT_DEFAULT_SPEED_MLS;

        fullyResetActiveDispenseOperation();
        m_state = STATE_FEEDING;
        m_feedState = FEED_INJECT_STARTING;
        m_active_op_target_ml = volume_ml;
        m_active_op_steps_per_ml = steps_per_ml;
        m_active_op_total_target_steps = (long)(volume_ml * steps_per_ml);
        m_active_op_remaining_steps = m_active_op_total_target_steps;
        m_active_op_initial_axis_steps = m_motorA->PositionRefCommanded();
        m_active_op_velocity_sps = (int)(speed_ml_s * steps_per_ml);
        m_active_op_accel_sps2 = (int)accel_sps2;
        m_active_op_torque_percent = torque_percent;
        m_active_op_feed_force_limit_kg = force_limit_kg;
        strncpy(m_active_op_feed_force_action, force_action, sizeof(m_active_op_feed_force_action) - 1);
        m_active_op_feed_force_action[sizeof(m_active_op_feed_force_action) - 1] = '\0';
        m_activeFeedCommand = CMD_STR_INJECT;
        m_feedStartTime = Milliseconds();

        char start_msg[128];
        snprintf(start_msg, sizeof(start_msg), "inject initiated. (ml/mm: %.4f, steps/ml: %.2f, force_limit: %.1f kg, action: %s)",
                 ml_per_mm, steps_per_ml, force_limit_kg, force_action);
        reportEvent(STATUS_PREFIX_START, start_msg);

        m_torqueLimit = (float)m_active_op_torque_percent;
        startMove(m_active_op_remaining_steps, m_active_op_velocity_sps, m_active_op_accel_sps2);
    } else {
        reportEvent(STATUS_PREFIX_ERROR, "Invalid inject format. Usage: inject <volume_ml> [speed_ml_s] [force_limit_kg] [force_action]");
    }
}

void Injector::setCartridgeMlPerMm(const char* args) {
    float ratio = 0.0f;
    if (std::sscanf(args, "%f", &ratio) == 1 && ratio > 0.0f) {
        m_cartridge_ml_per_mm = ratio;
        NvmManager &nvmMgr = NvmManager::Instance();
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_SLOT_CARTRIDGE_ML_PER_MM), (int32_t)(ratio * 10000.0f));
        char msg[128];
        snprintf(msg, sizeof(msg), "Cartridge ratio set to %.4f ml/mm and saved to NVM", ratio);
        reportEvent(STATUS_PREFIX_DONE, msg);
    } else {
        reportEvent(STATUS_PREFIX_ERROR, "Invalid set_cartridge_ml_per_mm format. Usage: set_cartridge_ml_per_mm <ml_per_mm>");
    }
}

// --- Injection Pause / Resume / Cancel ---

void Injector::pauseOperation() {
    if (m_state != STATE_FEEDING || m_feedState != FEED_INJECT_ACTIVE) {
        reportEvent(STATUS_PREFIX_INFO, "PAUSE ignored: No active injection to pause.");
        return;
    }
    abortMove();
    m_feedState = FEED_INJECT_PAUSED;
    reportEvent(STATUS_PREFIX_DONE, "PAUSE_INJECTION complete.");
}

void Injector::resumeOperation() {
    if (m_state != STATE_FEEDING || m_feedState != FEED_INJECT_PAUSED) {
        reportEvent(STATUS_PREFIX_INFO, "RESUME ignored: No operation was paused.");
        return;
    }
    if (m_active_op_remaining_steps <= 0) {
        reportEvent(STATUS_PREFIX_INFO, "RESUME ignored: No remaining volume to dispense.");
        fullyResetActiveDispenseOperation();
        m_state = STATE_STANDBY;
        return;
    }
    m_active_op_segment_initial_axis_steps = m_motorA->PositionRefCommanded();
    m_feedState = FEED_INJECT_RESUMING;
    m_torqueLimit = (float)m_active_op_torque_percent;
    startMove(m_active_op_remaining_steps, m_active_op_velocity_sps, m_active_op_accel_sps2);
    reportEvent(STATUS_PREFIX_DONE, "RESUME_INJECTION complete.");
}

void Injector::cancelOperation() {
    if (m_state != STATE_FEEDING) {
        reportEvent(STATUS_PREFIX_INFO, "CANCEL ignored: No active operation to cancel.");
        return;
    }
    abortMove();
    finalizeAndResetActiveDispenseOperation(false);
    m_state = STATE_STANDBY;
    reportEvent(STATUS_PREFIX_DONE, "CANCEL_INJECTION complete.");
}

//==================================================================================================
// --- General Move Commands (from Pressboi) ---
//==================================================================================================

void Injector::moveAbsolute(const char* args) {
    if (!m_homingMachineDone) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Must home before absolute moves.");
        return;
    }

    float position_mm = 0.0f;
    float speed_mms = MOVE_DEFAULT_VELOCITY_MMS;
    float force_kg = 0.0f;
    char force_action[32] = "hold";

    int parsed = std::sscanf(args, "%f %f %f %31s", &position_mm, &speed_mms, &force_kg, force_action);
    if (parsed < 1) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Invalid parameters for MOVE_ABS. Need at least position.");
        return;
    }

    if (speed_mms > 100.0f) {
        speed_mms = 100.0f;
        reportEvent(STATUS_PREFIX_INFO, "Speed limited to 100 mm/s for safety.");
    }

    // Validate force sensor in load_cell mode
    if (strcmp(m_force_mode, "load_cell") == 0) {
        const char* errorMsg = nullptr;
        if (checkForceSensorStatus(&errorMsg)) {
            char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
            snprintf(fullMsg, sizeof(fullMsg), "Move aborted: %s", errorMsg);
            reportEvent(STATUS_PREFIX_ERROR, fullMsg);
            return;
        }

        float current_force = m_controller->m_forceSensor.getForce();
        if (strcmp(force_action, "hold") == 0 && force_kg > 0.0f && current_force >= force_kg) {
            char msg[STATUS_MESSAGE_BUFFER_SIZE];
            snprintf(msg, sizeof(msg), "Force limit (%.2f kg) already reached. Current force: %.2f kg",
                     force_kg, current_force);
            reportEvent(STATUS_PREFIX_ERROR, msg);
            return;
        }
    }

    long target_steps = m_machineHomeReferenceSteps + (long)(position_mm * STEPS_PER_MM_INJECTOR);
    long current_pos = m_motorA->PositionRefCommanded();
    long steps_to_move = target_steps - current_pos;

    if (steps_to_move == 0) {
        reportEvent(STATUS_PREFIX_INFO, "Already at target position. Move complete.");
        reportEvent(STATUS_PREFIX_DONE, "move_abs");
        return;
    }

    int velocity_sps = (int)(speed_mms * STEPS_PER_MM_INJECTOR);

    // Set torque limit based on force mode and requested force
    if (force_kg > 0.0f) {
        if (strcmp(m_force_mode, "motor_torque") == 0) {
            if (force_kg < 50.0f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be >= 50 kg in motor_torque mode.");
                return;
            }
            if (force_kg > 2000.0f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be <= 2000 kg in motor_torque mode.");
                return;
            }
            m_torqueLimit = m_motor_torque_scale * force_kg + m_motor_torque_offset;
            char torque_msg[128];
            snprintf(torque_msg, sizeof(torque_msg), "Torque limit set: %.1f%% (from %.0f kg) in %s mode",
                     m_torqueLimit, force_kg, m_force_mode);
            reportEvent(STATUS_PREFIX_INFO, torque_msg);
        } else {
            if (force_kg < 0.2f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be >= 0.2 kg in load_cell mode.");
                return;
            }
            if (force_kg > 1000.0f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be <= 1000 kg in load_cell mode.");
                return;
            }
            m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
        }
    } else {
        m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
    }

    fullyResetActiveMove();
    m_state = STATE_MOVING;
    m_moveState = MOVE_STARTING;
    m_activeMoveCommand = "move_abs";
    m_active_op_target_position_steps = target_steps;
    m_active_op_initial_axis_steps = current_pos;
    m_active_op_total_target_steps = std::abs(steps_to_move);
    m_active_op_velocity_sps = velocity_sps;
    m_active_op_accel_sps2 = m_moveDefaultAccelSPS2;
    m_active_op_torque_percent = (int)m_torqueLimit;
    m_moveStartTime = Milliseconds();

    m_active_op_force_limit_kg = force_kg;
    strncpy(m_active_op_force_action, force_action, sizeof(m_active_op_force_action) - 1);
    m_active_op_force_action[sizeof(m_active_op_force_action) - 1] = '\0';
    strncpy(m_active_op_force_mode, m_force_mode, sizeof(m_active_op_force_mode) - 1);
    m_active_op_force_mode[sizeof(m_active_op_force_mode) - 1] = '\0';

    m_joules = 0.0;
    m_press_startpoint_mm = 0.0f;
    long cur_steps = m_motorA->PositionRefCommanded();
    m_prev_position_mm = static_cast<double>(cur_steps - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;
    m_machineStrainBaselinePosMm = m_prev_position_mm;
    m_prevMachineDeflectionMm = 0.0;
    m_prevTotalDeflectionMm = 0.0;
    m_machineEnergyJ = 0.0;
    m_machineStrainContactActive = false;
    m_forceLimitTriggered = false;
    m_jouleIntegrationActive = (strcmp(m_active_op_force_mode, "motor_torque") != 0);

    startMove(steps_to_move, velocity_sps, m_moveDefaultAccelSPS2);

    char msg[128];
    snprintf(msg, sizeof(msg), "move_abs to %.2f mm initiated (mode: %s)", position_mm, m_force_mode);
    reportEvent(STATUS_PREFIX_START, msg);
}

void Injector::moveIncremental(const char* args) {
    if (!m_homingMachineDone) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Must home before incremental moves.");
        return;
    }

    float distance_mm = 0.0f;
    float speed_mms = MOVE_DEFAULT_VELOCITY_MMS;
    float force_kg = 0.0f;
    char force_action[32] = "hold";

    int parsed = std::sscanf(args, "%f %f %f %31s", &distance_mm, &speed_mms, &force_kg, force_action);
    if (parsed < 1) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Invalid parameters for MOVE_INC. Need at least distance.");
        return;
    }

    if (speed_mms > 100.0f) {
        speed_mms = 100.0f;
        reportEvent(STATUS_PREFIX_INFO, "Speed limited to 100 mm/s for safety.");
    }

    if (strcmp(m_force_mode, "load_cell") == 0) {
        const char* errorMsg = nullptr;
        if (checkForceSensorStatus(&errorMsg)) {
            char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
            snprintf(fullMsg, sizeof(fullMsg), "Move aborted: %s", errorMsg);
            reportEvent(STATUS_PREFIX_ERROR, fullMsg);
            return;
        }

        float current_force = m_controller->m_forceSensor.getForce();
        if (strcmp(force_action, "hold") == 0 && force_kg > 0.0f && current_force >= force_kg) {
            char msg[STATUS_MESSAGE_BUFFER_SIZE];
            snprintf(msg, sizeof(msg), "Force limit (%.2f kg) already reached. Current force: %.2f kg",
                     force_kg, current_force);
            reportEvent(STATUS_PREFIX_ERROR, msg);
            return;
        }
    }

    long steps_to_move = (long)(distance_mm * STEPS_PER_MM_INJECTOR);
    if (steps_to_move == 0) {
        reportEvent(STATUS_PREFIX_INFO, "Move distance is zero. Move complete.");
        reportEvent(STATUS_PREFIX_DONE, "move_inc");
        return;
    }

    long current_pos = m_motorA->PositionRefCommanded();
    int velocity_sps = (int)(speed_mms * STEPS_PER_MM_INJECTOR);

    if (force_kg > 0.0f) {
        if (strcmp(m_force_mode, "motor_torque") == 0) {
            if (force_kg < 50.0f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be >= 50 kg in motor_torque mode.");
                return;
            }
            if (force_kg > 2000.0f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be <= 2000 kg in motor_torque mode.");
                return;
            }
            m_torqueLimit = m_motor_torque_scale * force_kg + m_motor_torque_offset;
            char torque_msg[128];
            snprintf(torque_msg, sizeof(torque_msg), "Torque limit set: %.1f%% (from %.0f kg) in %s mode",
                     m_torqueLimit, force_kg, m_force_mode);
            reportEvent(STATUS_PREFIX_INFO, torque_msg);
        } else {
            if (force_kg < 0.2f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be >= 0.2 kg in load_cell mode.");
                return;
            }
            if (force_kg > 1000.0f) {
                reportEvent(STATUS_PREFIX_ERROR, "Error: Force must be <= 1000 kg in load_cell mode.");
                return;
            }
            m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
        }
    } else {
        m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
    }

    fullyResetActiveMove();
    m_state = STATE_MOVING;
    m_moveState = MOVE_STARTING;
    m_activeMoveCommand = "move_inc";
    m_active_op_target_position_steps = current_pos + steps_to_move;
    m_active_op_initial_axis_steps = current_pos;
    m_active_op_total_target_steps = std::abs(steps_to_move);
    m_active_op_velocity_sps = velocity_sps;
    m_active_op_accel_sps2 = m_moveDefaultAccelSPS2;
    m_active_op_torque_percent = (int)m_torqueLimit;
    m_moveStartTime = Milliseconds();

    m_active_op_force_limit_kg = force_kg;
    strncpy(m_active_op_force_action, force_action, sizeof(m_active_op_force_action) - 1);
    m_active_op_force_action[sizeof(m_active_op_force_action) - 1] = '\0';
    strncpy(m_active_op_force_mode, m_force_mode, sizeof(m_active_op_force_mode) - 1);
    m_active_op_force_mode[sizeof(m_active_op_force_mode) - 1] = '\0';

    m_joules = 0.0;
    m_press_startpoint_mm = 0.0f;
    long cur_steps = m_motorA->PositionRefCommanded();
    m_prev_position_mm = static_cast<double>(cur_steps - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;
    m_machineStrainBaselinePosMm = m_prev_position_mm;
    m_prevMachineDeflectionMm = 0.0;
    m_prevTotalDeflectionMm = 0.0;
    m_machineEnergyJ = 0.0;
    m_machineStrainContactActive = false;
    m_forceLimitTriggered = false;
    m_jouleIntegrationActive = (strcmp(m_active_op_force_mode, "motor_torque") != 0);

    startMove(steps_to_move, velocity_sps, m_moveDefaultAccelSPS2);

    char msg[128];
    snprintf(msg, sizeof(msg), "move_inc by %.2f mm initiated (mode: %s)", distance_mm, m_force_mode);
    reportEvent(STATUS_PREFIX_START, msg);
}

void Injector::setRetract(const char* args) {
    if (!m_homingMachineDone) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Must home before setting retract position.");
        return;
    }

    float position_mm = 0.0f;
    float speed_mms = m_retractSpeedMms;
    int parsed = std::sscanf(args, "%f %f", &position_mm, &speed_mms);
    if (parsed < 1) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Invalid position for SET_RETRACT.");
        return;
    }

    if (parsed >= 2) {
        if (speed_mms <= 0.0f) {
            reportEvent(STATUS_PREFIX_ERROR, "Error: Retract speed must be > 0.");
            return;
        }
        if (speed_mms > 100.0f) {
            speed_mms = 100.0f;
            reportEvent(STATUS_PREFIX_INFO, "Retract speed limited to 100 mm/s for safety.");
        }
        m_retractSpeedMms = speed_mms;
    } else if (m_retractSpeedMms <= 0.0f) {
        m_retractSpeedMms = RETRACT_DEFAULT_SPEED_MMS;
    }
    if (m_retractSpeedMms > 100.0f) m_retractSpeedMms = 100.0f;

    long position_steps = (long)(position_mm * STEPS_PER_MM_INJECTOR);
    m_retractReferenceSteps = m_machineHomeReferenceSteps + position_steps;
    m_retract_position_mm = position_mm;

    NvmManager &nvmMgr = NvmManager::Instance();
    int32_t retractBits;
    memcpy(&retractBits, &m_retract_position_mm, sizeof(float));
    nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(14 * 4), retractBits);

    char msg[128];
    snprintf(msg, sizeof(msg), "Retract position set to %.2f mm (%ld steps from home) at %.2f mm/s",
             position_mm, position_steps, m_retractSpeedMms);
    reportEvent(STATUS_PREFIX_INFO, msg);
    char dbg[128];
    snprintf(dbg, sizeof(dbg), "Retract debug: home_steps=%ld, retract_steps=%ld", m_machineHomeReferenceSteps, m_retractReferenceSteps);
    reportEvent(STATUS_PREFIX_INFO, dbg);
    reportEvent(STATUS_PREFIX_DONE, "set_retract");
}

void Injector::retract(const char* args) {
    if (!m_homingMachineDone) {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Must home before moving to retract position.");
        return;
    }
    if (m_retractReferenceSteps == LONG_MIN) {
        char dbg[128];
        snprintf(dbg, sizeof(dbg), "Retract debug: reference steps not set (home=%ld)", m_machineHomeReferenceSteps);
        reportEvent(STATUS_PREFIX_INFO, dbg);
        reportEvent(STATUS_PREFIX_ERROR, "Error: Retract position not set. Use SET_RETRACT first.");
        return;
    }

    float speed_mms = (m_retractSpeedMms > 0.0f) ? m_retractSpeedMms : RETRACT_DEFAULT_SPEED_MMS;
    if (args && args[0] != '\0') {
        int parsed = std::sscanf(args, "%f", &speed_mms);
        if (parsed < 1) {
            reportEvent(STATUS_PREFIX_ERROR, "Error: Invalid speed for RETRACT.");
            return;
        }
    }
    if (speed_mms > 100.0f) {
        speed_mms = 100.0f;
        reportEvent(STATUS_PREFIX_INFO, "Speed limited to 100 mm/s for safety.");
    }

    fullyResetActiveMove();
    m_state = STATE_MOVING;
    m_moveState = MOVE_TO_HOME;
    m_activeMoveCommand = "retract";

    m_active_op_target_position_steps = m_retractReferenceSteps;
    long current_pos = m_motorA->PositionRefCommanded();
    long steps_to_retract = m_retractReferenceSteps - current_pos;

    int velocity_sps = (int)(speed_mms * STEPS_PER_MM_INJECTOR);
    m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;

    m_active_op_initial_axis_steps = current_pos;
    m_active_op_total_target_steps = std::abs(steps_to_retract);
    m_active_op_velocity_sps = velocity_sps;
    m_active_op_accel_sps2 = m_moveDefaultAccelSPS2;
    m_active_op_torque_percent = (int)m_torqueLimit;
    m_moveStartTime = Milliseconds();

    startMove(steps_to_retract, velocity_sps, m_moveDefaultAccelSPS2);

    char msg[128];
    snprintf(msg, sizeof(msg), "retract to %.3f mm at %.2f mm/s initiated",
             (float)(m_retractReferenceSteps - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR, speed_mms);
    reportEvent(STATUS_PREFIX_START, msg);
}

//==================================================================================================
// --- General Move Pause / Resume / Cancel ---
//==================================================================================================

void Injector::pauseGeneralMove() {
    if (m_state == STATE_HOMING) {
        abortMove();
        reportEvent(STATUS_PREFIX_INFO, "Homing paused. Send resume to continue.");
        reportEvent(STATUS_PREFIX_DONE, "pause");
    } else if (m_state == STATE_MOVING) {
        if (m_moveState == MOVE_ACTIVE || m_moveState == MOVE_STARTING ||
            m_moveState == MOVE_TO_HOME || m_moveState == MOVE_TO_RETRACT) {
            abortMove();
            m_moveState = MOVE_PAUSED;
            reportEvent(STATUS_PREFIX_INFO, "Move paused. Send resume to continue.");
            reportEvent(STATUS_PREFIX_DONE, "pause");
        } else {
            reportEvent(STATUS_PREFIX_INFO, "No active move to pause.");
            reportEvent(STATUS_PREFIX_DONE, "pause");
        }
    } else {
        reportEvent(STATUS_PREFIX_INFO, "No active operation to pause.");
        reportEvent(STATUS_PREFIX_DONE, "pause");
    }
}

void Injector::resumeGeneralMove() {
    if (m_state == STATE_HOMING) {
        reportEvent(STATUS_PREFIX_INFO, "Homing resumed.");
        reportEvent(STATUS_PREFIX_DONE, "resume");
    } else if (m_state == STATE_MOVING) {
        if (m_moveState == MOVE_PAUSED) {
            long current_pos = m_motorA->PositionRefCommanded();
            long steps_moved_so_far = current_pos - m_active_op_initial_axis_steps;
            long remaining_steps = m_active_op_total_target_steps - std::abs(steps_moved_so_far);

            if (remaining_steps > 0) {
                m_active_op_remaining_steps = remaining_steps;
                m_active_op_segment_initial_axis_steps = current_pos;
                m_moveState = MOVE_RESUMING;
                m_torqueLimit = (float)m_active_op_torque_percent;
                m_moveStartTime = Milliseconds();
                m_jouleIntegrationActive = (!m_forceLimitTriggered
                    && strcmp(m_active_op_force_mode, "load_cell") == 0);
                m_prevForceValid = false;
                m_prev_position_mm = static_cast<double>(current_pos - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;
                m_machineStrainBaselinePosMm = m_prev_position_mm;
                m_prevMachineDeflectionMm = 0.0;
                m_prevTotalDeflectionMm = 0.0;
                m_machineEnergyJ = 0.0;
                m_machineStrainContactActive = false;
                startMove(m_active_op_remaining_steps, m_active_op_velocity_sps, m_active_op_accel_sps2);
                reportEvent(STATUS_PREFIX_INFO, "Move resumed.");
                reportEvent(STATUS_PREFIX_DONE, "resume");
            } else {
                reportEvent(STATUS_PREFIX_INFO, "Move already complete.");
                fullyResetActiveMove();
                m_state = STATE_STANDBY;
                reportEvent(STATUS_PREFIX_DONE, "resume");
            }
        } else {
            reportEvent(STATUS_PREFIX_INFO, "No paused move to resume.");
            reportEvent(STATUS_PREFIX_DONE, "resume");
        }
    } else {
        reportEvent(STATUS_PREFIX_INFO, "No paused operation to resume.");
        reportEvent(STATUS_PREFIX_DONE, "resume");
    }
}

void Injector::cancelGeneralMove() {
    if (m_state == STATE_HOMING) {
        abortMove();
        m_homingPhase = HOMING_PHASE_IDLE;
        m_homingState = HOMING_NONE;
        m_state = STATE_STANDBY;
        reportEvent(STATUS_PREFIX_INFO, "Homing cancelled. Returning to standby.");
        reportEvent(STATUS_PREFIX_DONE, "cancel");
    } else if (m_state == STATE_MOVING) {
        abortMove();
        finalizeAndResetActiveMove(false);
        m_state = STATE_STANDBY;
        reportEvent(STATUS_PREFIX_INFO, "Move cancelled. Returning to standby.");
        reportEvent(STATUS_PREFIX_DONE, "cancel");
    } else {
        reportEvent(STATUS_PREFIX_INFO, "No active operation to cancel.");
        reportEvent(STATUS_PREFIX_DONE, "cancel");
    }
}

//==================================================================================================
// --- Jog Move ---
//==================================================================================================

void Injector::jogMove(const char* args) {
    float dist_mm1 = 0, dist_mm2 = 0, vel_mms = 0, accel_mms2 = 0;
    int torque_percent = 0;

    int parsed_count = std::sscanf(args, "%f %f %f %f %d", &dist_mm1, &dist_mm2, &vel_mms, &accel_mms2, &torque_percent);

    if (parsed_count == 5) {
        if (torque_percent <= 0 || torque_percent > 100) torque_percent = JOG_DEFAULT_TORQUE_PERCENT;
        if (vel_mms <= 0) vel_mms = JOG_DEFAULT_VEL_MMS;
        if (accel_mms2 <= 0) accel_mms2 = JOG_DEFAULT_ACCEL_MMSS;

        long steps1 = (long)(dist_mm1 * STEPS_PER_MM_INJECTOR);
        int velocity_sps = (int)(vel_mms * STEPS_PER_MM_INJECTOR);
        int accel_sps2_val = (int)(accel_mms2 * STEPS_PER_MM_INJECTOR);

        m_activeJogCommand = CMD_STR_JOG_MOVE;
        m_state = STATE_JOGGING;
        m_torqueLimit = (float)torque_percent;
        startMove(steps1, velocity_sps, accel_sps2_val);
    } else {
        char errorMsg[STATUS_MESSAGE_BUFFER_SIZE];
        std::snprintf(errorMsg, sizeof(errorMsg), "Invalid JOG_MOVE format. Expected 5 params, got %d.", parsed_count);
        reportEvent(STATUS_PREFIX_ERROR, errorMsg);
    }
}

//==================================================================================================
// --- Torque Offset ---
//==================================================================================================

void Injector::setTorqueOffset(const char* args) {
    float offset = 0.0f;
    if (std::sscanf(args, "%f", &offset) == 1) {
        m_torqueOffset = offset;
        char msg[128];
        snprintf(msg, sizeof(msg), "Torque offset set to %.2f%%", offset);
        reportEvent(STATUS_PREFIX_INFO, msg);
    } else {
        reportEvent(STATUS_PREFIX_ERROR, "Error: Invalid torque offset value.");
    }
}

//==================================================================================================
// --- Force Mode / Calibration / Polarity / Settings ---
//==================================================================================================

bool Injector::setForceMode(const char* mode) {
    NvmManager &nvmMgr = NvmManager::Instance();
    if (strcmp(mode, "motor_torque") == 0) {
        strcpy(m_force_mode, "motor_torque");
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(4 * 4), 0);
        return true;
    } else if (strcmp(mode, "load_cell") == 0) {
        strcpy(m_force_mode, "load_cell");
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(4 * 4), 1);
        return true;
    }
    return false;
}

const char* Injector::getForceMode() const {
    return m_force_mode;
}

void Injector::setForceCalibrationOffset(float offset) {
    if (strcmp(m_force_mode, "motor_torque") == 0) {
        m_motor_torque_offset = offset;
        NvmManager &nvmMgr = NvmManager::Instance();
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(6 * 4), (int32_t)(offset * 10000.0f));
    }
}

void Injector::setForceCalibrationScale(float scale) {
    if (strcmp(m_force_mode, "motor_torque") == 0) {
        m_motor_torque_scale = scale;
        NvmManager &nvmMgr = NvmManager::Instance();
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(5 * 4), (int32_t)(scale * 100000.0f));
    }
}

float Injector::getForceCalibrationOffset() const {
    if (strcmp(m_force_mode, "motor_torque") == 0) return m_motor_torque_offset;
    return 0.0f;
}

float Injector::getForceCalibrationScale() const {
    if (strcmp(m_force_mode, "motor_torque") == 0) return m_motor_torque_scale;
    return 1.0f;
}

bool Injector::setPolarity(const char* polarity) {
    NvmManager &nvmMgr = NvmManager::Instance();
    if (strcmp(polarity, "normal") == 0) {
        strcpy(m_polarity, "normal");
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(3 * 4), 0);
        m_motorA->PolarityInvertSDDirection(false);
        m_motorB->PolarityInvertSDDirection(false);
        return true;
    } else if (strcmp(polarity, "inverted") == 0) {
        strcpy(m_polarity, "inverted");
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(3 * 4), 1);
        m_motorA->PolarityInvertSDDirection(true);
        m_motorB->PolarityInvertSDDirection(true);
        return true;
    }
    return false;
}

const char* Injector::getPolarity() const {
    return m_polarity;
}

bool Injector::setHomeOnBoot(const char* enabled) {
    NvmManager &nvmMgr = NvmManager::Instance();
    if (strcmp(enabled, "true") == 0) {
        m_home_on_boot = true;
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(13 * 4), 1);
        return true;
    } else if (strcmp(enabled, "false") == 0) {
        m_home_on_boot = false;
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(13 * 4), 0);
        return true;
    }
    return false;
}

bool Injector::getHomeOnBoot() const {
    return m_home_on_boot;
}

bool Injector::setPressThreshold(float threshold_kg) {
    if (threshold_kg < 0.1f || threshold_kg > 50.0f) return false;
    m_press_threshold_kg = threshold_kg;
    NvmManager &nvmMgr = NvmManager::Instance();
    int32_t bits;
    memcpy(&bits, &threshold_kg, sizeof(float));
    nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(15 * 4), bits);
    return true;
}

//==================================================================================================
// --- Low-Level Motion ---
//==================================================================================================

void Injector::startMove(long steps, int velSps, int accelSps2) {
    m_firstTorqueReading0 = true;
    m_firstTorqueReading1 = true;

    char logMsg[128];
    snprintf(logMsg, sizeof(logMsg), "startMove: steps=%ld, vel=%d, accel=%d, torque=%.1f",
             steps, velSps, accelSps2, m_torqueLimit);
    reportEvent(STATUS_PREFIX_INFO, logMsg);

    if (steps == 0) {
        reportEvent(STATUS_PREFIX_INFO, "startMove called with 0 steps. No move will occur.");
        return;
    }

    m_motorA->VelMax(velSps);
    m_motorA->AccelMax(accelSps2);
    m_motorB->VelMax(velSps);
    m_motorB->AccelMax(accelSps2);

    m_motorA->Move(steps);
    m_motorB->Move(steps);
}

void Injector::startMoveAxis(int axis, long steps, int velSps, int accelSps2) {
    if (steps == 0) return;
    if (axis == 0) {
        m_motorA->VelMax(velSps);
        m_motorA->AccelMax(accelSps2);
        m_motorA->Move(steps);
    } else {
        m_motorB->VelMax(velSps);
        m_motorB->AccelMax(accelSps2);
        m_motorB->Move(steps);
    }
}

void Injector::stopAxis(int axis) {
    if (axis == 0) m_motorA->MoveStopDecel();
    else           m_motorB->MoveStopDecel();
}

bool Injector::isMoving() {
    if (!m_isEnabled) return false;
    return m_motorA->StatusReg().bit.StepsActive || m_motorB->StatusReg().bit.StepsActive;
}

bool Injector::isAxisMoving(int axis) {
    if (!m_isEnabled) return false;
    if (axis == 0) return m_motorA->StatusReg().bit.StepsActive;
    return m_motorB->StatusReg().bit.StepsActive;
}

//==================================================================================================
// --- Torque Sensing ---
//==================================================================================================

float Injector::getSmoothedTorque(MotorDriver *motor, float *smoothedValue, bool *firstRead) {
    if (!motor->StatusReg().bit.StepsActive && m_moveState != MOVE_ACTIVE) {
        *firstRead = true;
        return 0.0f;
    }

    float currentRawTorque = motor->HlfbPercent();

    if (currentRawTorque == TORQUE_HLFB_AT_POSITION) {
        if (m_moveState == MOVE_ACTIVE && !*firstRead) {
            return *smoothedValue + m_torqueOffset;
        }
        return 0.0f;
    }

    if (*firstRead) {
        *smoothedValue = currentRawTorque;
        *firstRead = false;
    } else {
        *smoothedValue = EWMA_ALPHA_TORQUE * currentRawTorque + (1.0f - EWMA_ALPHA_TORQUE) * (*smoothedValue);
    }
    return *smoothedValue + m_torqueOffset;
}

bool Injector::checkTorqueLimit() {
    if (isMoving()) {
        float torque0 = getSmoothedTorque(m_motorA, &m_smoothedTorqueValue0, &m_firstTorqueReading0);
        float torque1 = getSmoothedTorque(m_motorB, &m_smoothedTorqueValue1, &m_firstTorqueReading1);

        bool m0_over = (torque0 != TORQUE_HLFB_AT_POSITION && std::abs(torque0) > m_torqueLimit);
        bool m1_over = (torque1 != TORQUE_HLFB_AT_POSITION && std::abs(torque1) > m_torqueLimit);

        if (m0_over || m1_over) return true;
    }
    return false;
}

//==================================================================================================
// --- Force Sensor & Limit Handling ---
//==================================================================================================

bool Injector::checkForceSensorStatus(const char** errorMsg) {
    if (strcmp(m_force_mode, "motor_torque") == 0) {
        return false;
    }

    if (!m_controller->m_forceSensor.isConnected()) {
        *errorMsg = "Force sensor disconnected";
        return true;
    }

    float force = m_controller->m_forceSensor.getForce();

    if (force < FORCE_SENSOR_MIN_KG) {
        *errorMsg = "Force sensor error: reading below minimum (-10 kg)";
        return true;
    }

    if (force > FORCE_SENSOR_MAX_LIMIT_KG) {
        *errorMsg = "Force sensor error: reading above maximum (1440 kg)";
        return true;
    }

    return false;
}

void Injector::handleLimitReached(const char* limit_type, float limit_value) {
    abortMove();
    m_jouleIntegrationActive = false;
    m_forceLimitTriggered = true;
    m_prevForceValid = false;

    if (m_activeMoveCommand &&
        (strcmp(m_activeMoveCommand, "move_abs") == 0 || strcmp(m_activeMoveCommand, "move_inc") == 0)) {
        long pos = m_motorA->PositionRefCommanded();
        m_endpoint_mm = static_cast<float>(
            static_cast<double>(pos - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR);
    }

    char msg[STATUS_MESSAGE_BUFFER_SIZE];
    snprintf(msg, sizeof(msg), "%s reached.", limit_type);
    reportEvent(STATUS_PREFIX_INFO, msg);

    if (strcmp(m_active_op_force_action, "retract") == 0) {
        reportEvent(STATUS_PREFIX_INFO, "Force limit reached, retracting...");
        m_originalMoveCommand = m_activeMoveCommand;
        long retract_target = (m_retractReferenceSteps == LONG_MIN)
            ? m_machineHomeReferenceSteps : m_retractReferenceSteps;
        m_active_op_force_action[0] = '\0';
        m_moveState = MOVE_TO_HOME;
        m_activeMoveCommand = "retract";
        m_active_op_target_position_steps = retract_target;
        long cur = m_motorA->PositionRefCommanded();
        long steps = retract_target - cur;
        m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
        float spd = (m_retractSpeedMms > 0.0f) ? m_retractSpeedMms : RETRACT_DEFAULT_SPEED_MMS;
        if (spd > 100.0f) spd = 100.0f;
        int vel = static_cast<int>(spd * STEPS_PER_MM_INJECTOR);
        m_active_op_velocity_sps = vel;
        m_active_op_accel_sps2 = m_moveDefaultAccelSPS2;
        startMove(steps, vel, m_moveDefaultAccelSPS2);
        reportEvent(STATUS_PREFIX_START, "retract");
    } else if (strcmp(m_active_op_force_action, "abort") == 0) {
        if (m_activeMoveCommand) {
            char err[STATUS_MESSAGE_BUFFER_SIZE];
            snprintf(err, sizeof(err), "%s aborted due to force limit", m_activeMoveCommand);
            reportEvent(STATUS_PREFIX_ERROR, err);
        }
        long retract_target = (m_retractReferenceSteps == LONG_MIN)
            ? m_machineHomeReferenceSteps : m_retractReferenceSteps;
        m_active_op_force_action[0] = '\0';
        m_moveState = MOVE_TO_HOME;
        m_activeMoveCommand = "retract";
        m_active_op_target_position_steps = retract_target;
        long cur = m_motorA->PositionRefCommanded();
        long steps = retract_target - cur;
        m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
        float spd = (m_retractSpeedMms > 0.0f) ? m_retractSpeedMms : RETRACT_DEFAULT_SPEED_MMS;
        if (spd > 100.0f) spd = 100.0f;
        int vel = static_cast<int>(spd * STEPS_PER_MM_INJECTOR);
        m_active_op_velocity_sps = vel;
        m_active_op_accel_sps2 = m_moveDefaultAccelSPS2;
        startMove(steps, vel, m_moveDefaultAccelSPS2);
        reportEvent(STATUS_PREFIX_START, "retract");
    } else if (strcmp(m_active_op_force_action, "skip") == 0) {
        if (m_activeMoveCommand) {
            reportEvent(STATUS_PREFIX_DONE, m_activeMoveCommand);
        }
        finalizeAndResetActiveMove(true);
        m_state = STATE_STANDBY;
    } else {
        // "hold" — pause and signal host to hold script
        m_moveState = MOVE_PAUSED;
        sendEvent(EVENT_SCRIPT_HOLD);
    }
}

void Injector::handleFeedLimitReached(const char* limit_type, float limit_value) {
    abortMove();

    char msg[STATUS_MESSAGE_BUFFER_SIZE];
    snprintf(msg, sizeof(msg), "%s reached.", limit_type);
    reportEvent(STATUS_PREFIX_INFO, msg);

    const char* action = m_active_op_feed_force_action;

    if (strcmp(action, "retract") == 0 || strcmp(action, "abort") == 0) {
        if (strcmp(action, "abort") == 0) {
            reportEvent(STATUS_PREFIX_ERROR, "Injection aborted due to force limit");
        } else {
            reportEvent(STATUS_PREFIX_INFO, "Inject force limit reached, retracting...");
        }
        finalizeAndResetActiveDispenseOperation(false);
        long retract_target = (m_retractReferenceSteps == LONG_MIN)
            ? m_machineHomeReferenceSteps : m_retractReferenceSteps;
        m_state = STATE_MOVING;
        m_moveState = MOVE_TO_HOME;
        m_activeMoveCommand = "retract";
        m_active_op_target_position_steps = retract_target;
        long cur = m_motorA->PositionRefCommanded();
        long steps = retract_target - cur;
        m_torqueLimit = DEFAULT_INJECTOR_TORQUE_LIMIT;
        float spd = (m_retractSpeedMms > 0.0f) ? m_retractSpeedMms : RETRACT_DEFAULT_SPEED_MMS;
        if (spd > 100.0f) spd = 100.0f;
        int vel = static_cast<int>(spd * STEPS_PER_MM_INJECTOR);
        m_active_op_velocity_sps = vel;
        m_active_op_accel_sps2 = m_moveDefaultAccelSPS2;
        startMove(steps, vel, m_moveDefaultAccelSPS2);
        reportEvent(STATUS_PREFIX_START, "retract");
    } else if (strcmp(action, "skip") == 0) {
        reportEvent(STATUS_PREFIX_DONE, "inject");
        finalizeAndResetActiveDispenseOperation(true);
        m_state = STATE_STANDBY;
    } else {
        // "hold" — pause feed and signal host
        m_feedState = FEED_INJECT_PAUSED;
        sendEvent(EVENT_SCRIPT_HOLD);
    }
}

//==================================================================================================
// --- Dispense Operation Finalization ---
//==================================================================================================

void Injector::finalizeAndResetActiveDispenseOperation(bool success) {
    if (success) {
        m_last_completed_dispense_ml = m_active_op_total_dispensed_ml;
        m_cumulative_dispensed_ml += m_active_op_total_dispensed_ml;
    }
    fullyResetActiveDispenseOperation();
}

void Injector::fullyResetActiveDispenseOperation() {
    m_active_op_target_ml = 0.0f;
    m_active_op_total_dispensed_ml = 0.0f;
    m_last_completed_dispense_ml = 0.0f;
    m_active_op_total_target_steps = 0;
    m_active_op_remaining_steps = 0;
    m_active_op_segment_initial_axis_steps = 0;
    m_active_op_initial_axis_steps = 0;
    m_active_op_steps_per_ml = 0.0f;
    m_active_op_feed_force_limit_kg = 0.0f;
    m_active_op_feed_force_action[0] = '\0';
    m_activeFeedCommand = nullptr;
}

//==================================================================================================
// --- General Move Finalization ---
//==================================================================================================

void Injector::finalizeAndResetActiveMove(bool success) {
    if (success) {
        m_last_completed_distance_mm = m_active_op_total_distance_mm;
        m_cumulative_distance_mm += m_active_op_total_distance_mm;
    }
    fullyResetActiveMove();
}

void Injector::fullyResetActiveMove() {
    m_active_op_force_limit_kg = 0.0f;
    m_active_op_force_action[0] = '\0';
    strncpy(m_active_op_force_mode, "motor_torque", sizeof(m_active_op_force_mode) - 1);
    m_active_op_force_mode[sizeof(m_active_op_force_mode) - 1] = '\0';
    m_active_op_total_distance_mm = 0.0f;
    m_last_completed_distance_mm = 0.0f;
    m_active_op_total_target_steps = 0;
    m_active_op_remaining_steps = 0;
    m_active_op_segment_initial_axis_steps = 0;
    m_active_op_initial_axis_steps = 0;
    m_activeMoveCommand = nullptr;
    m_originalMoveCommand = nullptr;
    m_jouleIntegrationActive = false;
    m_forceLimitTriggered = false;
    m_prevForceValid = false;
    m_machineStrainBaselinePosMm = 0.0;
    m_prevMachineDeflectionMm = 0.0;
    m_prevTotalDeflectionMm = 0.0;
    m_machineEnergyJ = 0.0;
    m_machineStrainContactActive = false;
}

//==================================================================================================
// --- Joule Integration ---
//==================================================================================================

void Injector::updateJoules() {
    if (!m_jouleIntegrationActive || m_state != STATE_MOVING) {
        m_prevForceValid = false;
        return;
    }

    if (strcmp(m_active_op_force_mode, "motor_torque") == 0) {
        m_jouleIntegrationActive = false;
        m_prevForceValid = false;
        return;
    }

    // Load cell force × distance energy integration at 50Hz
    long current_pos_steps = m_motorA->PositionRefCommanded();
    double current_pos_mm = static_cast<double>(current_pos_steps - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;
    double distance_mm = current_pos_mm - m_prev_position_mm;
    double abs_distance_mm = fabs(distance_mm);

    float raw_force_sample = m_controller->m_forceSensor.getForce();
    if (!m_prevForceValid) {
        m_prevForceKg = raw_force_sample;
        if (m_prevForceKg < 0.0f) m_prevForceKg = 0.0f;
        m_prevMachineDeflectionMm = static_cast<double>(estimateMachineDeflectionFromForce(m_prevForceKg));
        m_prevForceValid = true;
        m_prev_position_mm = current_pos_mm;
        return;
    }

    float raw_force_kg = raw_force_sample;
    if (raw_force_kg < 0.0f) raw_force_kg = 0.0f;

    float clamped_force_kg = raw_force_kg;
    if (m_active_op_force_limit_kg > 0.0f && clamped_force_kg > m_active_op_force_limit_kg) {
        clamped_force_kg = m_active_op_force_limit_kg;
    }

    if (abs_distance_mm <= 0.0) {
        m_prev_position_mm = current_pos_mm;
        m_prevForceKg = clamped_force_kg;
        if (m_machineStrainContactActive) {
            m_prevMachineDeflectionMm = static_cast<double>(estimateMachineDeflectionFromForce(clamped_force_kg));
        } else {
            m_prevMachineDeflectionMm = 0.0;
        }
        return;
    }

    if (!m_machineStrainContactActive) {
        if (clamped_force_kg >= m_press_threshold_kg) {
            double contact_machine_def_mm = static_cast<double>(estimateMachineDeflectionFromForce(clamped_force_kg));
            if (contact_machine_def_mm < 0.0) contact_machine_def_mm = 0.0;
            m_machineStrainContactActive = true;
            m_machineStrainBaselinePosMm = current_pos_mm - contact_machine_def_mm;
            m_prevMachineDeflectionMm = contact_machine_def_mm;
            m_prevTotalDeflectionMm = contact_machine_def_mm;
            m_machineEnergyJ = 0.0;
            m_prev_position_mm = current_pos_mm;
            m_prevForceKg = clamped_force_kg;
            m_press_startpoint_mm = static_cast<float>(current_pos_mm);
            return;
        } else {
            m_machineStrainBaselinePosMm = current_pos_mm;
            m_prevMachineDeflectionMm = 0.0;
            m_prevTotalDeflectionMm = 0.0;
            m_machineEnergyJ = 0.0;
            m_prev_position_mm = current_pos_mm;
            m_prevForceKg = clamped_force_kg;
            return;
        }
    }

    double actual_force_avg = 0.5 * static_cast<double>(m_prevForceKg + clamped_force_kg);
    double total_deflection_mm = current_pos_mm - m_machineStrainBaselinePosMm;
    if (total_deflection_mm < 0.0) total_deflection_mm = 0.0;

    double machine_deflection_at_force = static_cast<double>(estimateMachineDeflectionFromForce(clamped_force_kg));

    double machine_ratio = 0.0;
    if (total_deflection_mm > 0.001) {
        machine_ratio = machine_deflection_at_force / total_deflection_mm;
        if (machine_ratio > 1.0) machine_ratio = 1.0;
        if (machine_ratio < 0.0) machine_ratio = 0.0;
    }

    double gross_increment = actual_force_avg * abs_distance_mm * 0.00981;
    double machine_increment = gross_increment * machine_ratio;
    double net_increment = gross_increment - machine_increment;
    if (net_increment < 0.0) net_increment = 0.0;

    m_joules += net_increment;
    m_machineEnergyJ += machine_increment;

    m_prev_position_mm = current_pos_mm;
    m_prevForceKg = clamped_force_kg;

    if (m_active_op_force_limit_kg > 0.0f && raw_force_kg >= m_active_op_force_limit_kg) {
        m_jouleIntegrationActive = false;
        m_forceLimitTriggered = true;
        m_prevForceValid = false;
    }
}

//==================================================================================================
// --- Event Reporting ---
//==================================================================================================

void Injector::reportEvent(const char* statusType, const char* message) {
    char fullMsg[STATUS_MESSAGE_BUFFER_SIZE];
    snprintf(fullMsg, sizeof(fullMsg), "Injector: %s", message);
    m_controller->reportEvent(statusType, fullMsg);
}

//==================================================================================================
// --- Telemetry ---
//==================================================================================================

const char* Injector::getTelemetryString() {
    float displayTorque0 = getSmoothedTorque(m_motorA, &m_smoothedTorqueValue0, &m_firstTorqueReading0);
    float displayTorque1 = getSmoothedTorque(m_motorB, &m_smoothedTorqueValue1, &m_firstTorqueReading1);

    long current_pos_steps_m0 = m_motorA->PositionRefCommanded();
    float machine_pos_mm = (float)(current_pos_steps_m0 - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;
    float cartridge_pos_mm = (float)(current_pos_steps_m0 - m_cartridgeHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;

    int enabled0 = m_isEnabled ? 1 : 0;
    int enabled1 = m_isEnabled ? 1 : 0;

    float live_cumulative_ml = m_cumulative_dispensed_ml + m_active_op_total_dispensed_ml;

    // Motor torque-derived force estimate (always available)
    float avg_torque = (displayTorque0 + displayTorque1) / 2.0f;
    float force_motor_torque = (avg_torque - m_motor_torque_offset) / m_motor_torque_scale;
    if (force_motor_torque < 0.0f) force_motor_torque = 0.0f;
    if (force_motor_torque > 2000.0f) force_motor_torque = 2000.0f;

    // Force limit for telemetry display
    float force_limit;
    if (m_state == STATE_MOVING && m_active_op_force_limit_kg > 0.1f) {
        force_limit = m_active_op_force_limit_kg;
    } else {
        force_limit = (strcmp(m_force_mode, "load_cell") == 0) ? 1000.0f : 2000.0f;
    }

    // Retract position
    float retract_pos;
    if (m_retractReferenceSteps == LONG_MIN) {
        retract_pos = 0.0f;
    } else {
        retract_pos = (float)(m_retractReferenceSteps - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR;
    }

    float target_pos = (float)((double)(m_active_op_target_position_steps - m_machineHomeReferenceSteps) / STEPS_PER_MM_INJECTOR);

    // Two-stage snprintf to fit as much as possible into 256 bytes.
    // Stage 1: Core Fillhead injection telemetry
    int n = std::snprintf(m_telemetryBuffer, sizeof(m_telemetryBuffer),
        "inj_t0:%.1f,inj_t1:%.1f,"
        "inj_h_mach:%d,inj_h_cart:%d,"
        "inj_mach_mm:%.2f,inj_cart_mm:%.2f,"
        "inj_cumulative_ml:%.2f,inj_active_ml:%.2f,inj_tgt_ml:%.2f,"
        "enabled0:%d,enabled1:%d,"
        "injector_state:%d",
        displayTorque0, displayTorque1,
        (int)m_homingMachineDone, (int)m_homingCartridgeDone,
        machine_pos_mm, cartridge_pos_mm,
        live_cumulative_ml, m_active_op_total_dispensed_ml, m_active_op_target_ml,
        enabled0, enabled1,
        (int)m_state);

    // Load cell force and raw ADC (from actual sensor)
    float force_load_cell = 0.0f;
    int32_t force_adc_raw = 0;
    if (m_controller->m_forceSensor.isConnected()) {
        force_load_cell = m_controller->m_forceSensor.getForce();
        force_adc_raw = (int32_t)m_controller->m_forceSensor.getRawValue();
    }

    // Stage 2: Pressboi-compatible fields (appended if space remains)
    if (n > 0 && (size_t)n < sizeof(m_telemetryBuffer) - 1) {
        int homed = (m_homingMachineDone && m_homingCartridgeDone) ? 1 : 0;
        std::snprintf(m_telemetryBuffer + n, sizeof(m_telemetryBuffer) - n,
            ",fmt:%.1f,flc:%.2f,far:%ld,fl:%.1f,j:%.4f,"
            "cp:%.3f,rp:%.2f,tp:%.2f,"
            "ep:%.3f,sp:%.3f,pt:%.1f,"
            "ta:%.1f,hm:%d,"
            "hs0:%d,hs1:%d,fm:%s,pol:%s",
            force_motor_torque, force_load_cell, (long)force_adc_raw,
            force_limit, (float)m_joules,
            machine_pos_mm, retract_pos, target_pos,
            m_endpoint_mm, m_press_startpoint_mm, m_press_threshold_kg,
            avg_torque, homed,
            getHomeSensorStateM0() ? 1 : 0, getHomeSensorStateM1() ? 1 : 0,
            m_force_mode, m_polarity);
    }

    return m_telemetryBuffer;
}

//==================================================================================================
// --- Status Queries ---
//==================================================================================================

bool Injector::isBusy() const {
    return m_state != STATE_STANDBY;
}

const char* Injector::getState() const {
    switch (m_state) {
        case STATE_STANDBY:     return "Standby";
        case STATE_HOMING:      return "Homing";
        case STATE_JOGGING:     return "Jogging";
        case STATE_FEEDING:     return "Feeding";
        case STATE_MOVING:      return "Moving";
        case STATE_MOTOR_FAULT: return "Fault";
        default:                return "Unknown";
    }
}

bool Injector::isInFault() const {
    return m_motorA->StatusReg().bit.MotorInFault || m_motorB->StatusReg().bit.MotorInFault;
}

//==================================================================================================
// --- Home Sensor Functions (Gantry Squaring) ---
//==================================================================================================

void Injector::setupHomeSensors() {
    uint16_t filterSamples = HOME_SENSOR_FILTER_MS * 5;

    HOME_SENSOR_M0.Mode(Connector::INPUT_DIGITAL);
    HOME_SENSOR_M1.Mode(Connector::INPUT_DIGITAL);

    HOME_SENSOR_M0.FilterLength(filterSamples);
    HOME_SENSOR_M1.FilterLength(filterSamples);

    m_homeSensorsInitialized = true;
    reportEvent(STATUS_PREFIX_INFO, "Home sensors initialized (DI6=M1, DI7=M0).");
}

bool Injector::isHomeSensorTriggered(int axis) {
    return (getHomeSensorState(axis) == HOME_SENSOR_ACTIVE_STATE);
}

bool Injector::getHomeSensorState(int axis) {
    if (axis == 0) return HOME_SENSOR_M0.State() != 0;
    return HOME_SENSOR_M1.State() != 0;
}

bool Injector::getHomeSensorStateM0() const {
    return HOME_SENSOR_M0.State() == (HOME_SENSOR_ACTIVE_STATE ? 1 : 0);
}

bool Injector::getHomeSensorStateM1() const {
    return HOME_SENSOR_M1.State() == (HOME_SENSOR_ACTIVE_STATE ? 1 : 0);
}
