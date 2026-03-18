/**
 * @file telemetry.h
 * @brief Telemetry structure and construction interface for the Fillhead controller.
 * @details Matches telemetry.json schema. Fillhead = Pressboi + pinch valves + heater + vacuum + injection.
 * All telemetry fields use FILLHEAD_TELEM prefix in messages.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//==================================================================================================
// Telemetry Field Keys
//==================================================================================================

/**
 * @name Telemetry Field Identifiers
 * @brief String keys used in telemetry messages.
 * Format: "FILLHEAD_TELEM: field1:value1,field2:value2,..."
 * @{
 */
#define TELEM_KEY_MAIN_STATE                     "main_state"
#define TELEM_KEY_INJECTOR_STATE                 "injector_state"
#define TELEM_KEY_INJ_VALVE_STATE                "inj_valve_state"
#define TELEM_KEY_VAC_VALVE_STATE                "vac_valve_state"
#define TELEM_KEY_HEATER_STATE                   "heater_state"
#define TELEM_KEY_VACUUM_STATE                   "vacuum_state"
#define TELEM_KEY_INJECTOR_TORQUE                "injector_torque"
#define TELEM_KEY_INJECTOR_HOMED                 "injector_homed"
#define TELEM_KEY_INJECTION_CUMULATIVE_ML        "injection_cumulative_ml"
#define TELEM_KEY_INJECTION_ACTIVE_ML            "injection_active_ml"
#define TELEM_KEY_INJECTION_TARGET_ML            "injection_target_ml"
#define TELEM_KEY_MOTORS_ENABLED                 "motors_enabled"
#define TELEM_KEY_INJ_VALVE_POS                  "inj_valve_pos"
#define TELEM_KEY_INJ_VALVE_TORQUE                "inj_valve_torque"
#define TELEM_KEY_INJ_VALVE_HOMED                 "inj_valve_homed"
#define TELEM_KEY_VAC_VALVE_POS                  "vac_valve_pos"
#define TELEM_KEY_VAC_VALVE_MOTOR_TORQUE         "vac_valve_motor_torque"
#define TELEM_KEY_VAC_VALVE_HOMED                "vac_valve_homed"
#define TELEM_KEY_TEMP_C                         "temp_c"
#define TELEM_KEY_HEATER_SETPOINT                "heater_setpoint"
#define TELEM_KEY_VACUUM_PSIG                    "vacuum_psig"
/* Pressboi-derived fields (Fillhead prefix in message) */
#define TELEM_KEY_FORCE_LOAD_CELL                "force_load_cell"
#define TELEM_KEY_FORCE_MOTOR_TORQUE             "force_motor_torque"
#define TELEM_KEY_FORCE_LIMIT                    "force_limit"
#define TELEM_KEY_FORCE_SOURCE                   "force_source"
#define TELEM_KEY_FORCE_ADC_RAW                  "force_adc_raw"
#define TELEM_KEY_FORCE_MODE                     "force_mode"
#define TELEM_KEY_JOULES                         "joules"
#define TELEM_KEY_ENABLED0                       "enabled0"
#define TELEM_KEY_ENABLED1                       "enabled1"
#define TELEM_KEY_CURRENT_POS                    "current_pos"
#define TELEM_KEY_RETRACT_POS                    "retract_pos"
#define TELEM_KEY_TARGET_POS                     "target_pos"
#define TELEM_KEY_ENDPOINT                       "endpoint"
#define TELEM_KEY_STARTPOINT                     "startpoint"
#define TELEM_KEY_PRESS_THRESHOLD                "press_threshold"
#define TELEM_KEY_TORQUE_AVG                     "torque_avg"
#define TELEM_KEY_HOMED                          "homed"
#define TELEM_KEY_HOME_SENSOR_M0                 "home_sensor_m0"
#define TELEM_KEY_HOME_SENSOR_M1                 "home_sensor_m1"
#define TELEM_KEY_POLARITY                       "polarity"
/** @} */

//==================================================================================================
// Telemetry Data Structure
//==================================================================================================

#define TELEM_STRING_MAX 32

/**
 * @struct TelemetryData
 * @brief Complete telemetry state for the Fillhead device.
 * @details Contains Fillhead-specific fields plus Pressboi-derived fields (force, position, etc.).
 */
typedef struct {
    /* Fillhead core */
    char         main_state[TELEM_STRING_MAX];
    int32_t      injector_state;
    int32_t      inj_valve_state;
    int32_t      vac_valve_state;
    int32_t      heater_state;
    int32_t      vacuum_state;
    float        injector_torque;
    int32_t      injector_homed;
    float        injection_cumulative_ml;
    float        injection_active_ml;
    float        injection_target_ml;
    bool         motors_enabled;
    float        inj_valve_pos;
    float        inj_valve_torque;
    bool         inj_valve_homed;
    float        vac_valve_pos;
    float        vac_valve_motor_torque;
    bool         vac_valve_homed;
    float        temp_c;
    float        heater_setpoint;
    float        vacuum_psig;
    /* Pressboi-derived (force/position) */
    float        force_load_cell;
    float        force_motor_torque;
    float        force_limit;
    char         force_source[TELEM_STRING_MAX];
    int32_t      force_adc_raw;
    char         force_mode[TELEM_STRING_MAX];
    float        joules;
    int32_t      enabled0;
    int32_t      enabled1;
    float        current_pos;
    float        retract_pos;
    float        target_pos;
    float        endpoint;
    float        startpoint;
    float        press_threshold;
    float        torque_avg;
    int32_t      homed;
    int32_t      home_sensor_m0;
    int32_t      home_sensor_m1;
    char         polarity[TELEM_STRING_MAX];
} TelemetryData;

//==================================================================================================
// Telemetry Construction Functions
//==================================================================================================

/**
 * @brief Initialize telemetry data structure with default values.
 * @param data Pointer to TelemetryData structure to initialize
 */
void telemetry_init(TelemetryData* data);

/**
 * @brief Build complete telemetry message string from data structure.
 * @param data Pointer to TelemetryData structure containing current values
 * @param buffer Output buffer to write telemetry message
 * @param buffer_size Size of output buffer
 * @return Number of characters written (excluding null terminator)
 *
 * @details Constructs a message in the format: "FILLHEAD_TELEM: field1:value1,field2:value2,..."
 */
int telemetry_build_message(const TelemetryData* data, char* buffer, size_t buffer_size);

/**
 * @brief Send telemetry message via Serial.
 * @param data Pointer to TelemetryData structure containing current values
 *
 * @details Builds and transmits the complete telemetry message.
 */
void telemetry_send(const TelemetryData* data);
