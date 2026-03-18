/**
 * @file telemetry.cpp
 * @brief Telemetry construction implementation for the Fillhead controller.
 * @details Matches telemetry.json schema. Uses FILLHEAD_TELEM prefix.
 */

#include "telemetry.h"
#include <stdio.h>
#include <string.h>
// #include "ClearCore.h"  // Include if using ClearCore hardware

#define TELEM_PREFIX "FILLHEAD_TELEM: "

//==================================================================================================
// Telemetry Initialization
//==================================================================================================

void telemetry_init(TelemetryData* data) {
    if (data == NULL) return;

    strncpy(data->main_state, "standby", TELEM_STRING_MAX - 1);
    data->main_state[TELEM_STRING_MAX - 1] = '\0';
    data->injector_state = 0;
    data->inj_valve_state = 0;
    data->vac_valve_state = 0;
    data->heater_state = 0;
    data->vacuum_state = 0;
    data->injector_torque = 0.0f;
    data->injector_homed = 0;
    data->injection_cumulative_ml = 0.0f;
    data->injection_active_ml = 0.0f;
    data->injection_target_ml = 0.0f;
    data->motors_enabled = true;
    data->inj_valve_pos = 0.0f;
    data->inj_valve_torque = 0.0f;
    data->inj_valve_homed = false;
    data->vac_valve_pos = 0.0f;
    data->vac_valve_motor_torque = 0.0f;
    data->vac_valve_homed = false;
    data->temp_c = 25.0f;
    data->heater_setpoint = 70.0f;
    data->vacuum_psig = 0.5f;
    /* Pressboi-derived */
    data->force_load_cell = 0.0f;
    data->force_motor_torque = 0.0f;
    data->force_limit = 1000.0f;
    strncpy(data->force_source, "load_cell", TELEM_STRING_MAX - 1);
    data->force_source[TELEM_STRING_MAX - 1] = '\0';
    data->force_adc_raw = 0;
    strncpy(data->force_mode, "load_cell", TELEM_STRING_MAX - 1);
    data->force_mode[TELEM_STRING_MAX - 1] = '\0';
    data->joules = 0.0f;
    data->enabled0 = 1;
    data->enabled1 = 1;
    data->current_pos = 0.0f;
    data->retract_pos = 0.0f;
    data->target_pos = 0.0f;
    data->endpoint = 0.0f;
    data->startpoint = 0.0f;
    data->press_threshold = 2.0f;
    data->torque_avg = 0.0f;
    data->homed = 0;
    data->home_sensor_m0 = 0;
    data->home_sensor_m1 = 0;
    strncpy(data->polarity, "normal", TELEM_STRING_MAX - 1);
    data->polarity[TELEM_STRING_MAX - 1] = '\0';
}

//==================================================================================================
// Telemetry Message Construction
//==================================================================================================

/* Append macros for telemetry field formatting */
#define APPEND_INT(buf, pos, size, key, val) \
    if ((pos) < (size)) (pos) += snprintf((buf) + (pos), (size) - (pos), "%s:%d,", (key), (int)(val))
#define APPEND_FLOAT(buf, pos, size, key, val, prec) \
    if ((pos) < (size)) (pos) += snprintf((buf) + (pos), (size) - (pos), "%s:%.*f,", (key), (prec), (double)(val))
#define APPEND_STR(buf, pos, size, key, val) \
    if ((pos) < (size)) (pos) += snprintf((buf) + (pos), (size) - (pos), "%s:%s,", (key), (val))

int telemetry_build_message(const TelemetryData* data, char* buffer, size_t buffer_size) {
    if (data == NULL || buffer == NULL || buffer_size == 0) return 0;

    int pos = 0;
    pos += snprintf(buffer + pos, buffer_size - pos, "%s", TELEM_PREFIX);

    APPEND_STR(buffer, pos, buffer_size, TELEM_KEY_MAIN_STATE, data->main_state);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_INJECTOR_STATE, data->injector_state);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_INJ_VALVE_STATE, data->inj_valve_state);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_VAC_VALVE_STATE, data->vac_valve_state);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_HEATER_STATE, data->heater_state);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_VACUUM_STATE, data->vacuum_state);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_INJECTOR_TORQUE, data->injector_torque, 1);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_INJECTOR_HOMED, data->injector_homed);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_INJECTION_CUMULATIVE_ML, data->injection_cumulative_ml, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_INJECTION_ACTIVE_ML, data->injection_active_ml, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_INJECTION_TARGET_ML, data->injection_target_ml, 2);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_MOTORS_ENABLED, data->motors_enabled ? 1 : 0);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_INJ_VALVE_POS, data->inj_valve_pos, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_INJ_VALVE_TORQUE, data->inj_valve_torque, 1);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_INJ_VALVE_HOMED, data->inj_valve_homed ? 1 : 0);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_VAC_VALVE_POS, data->vac_valve_pos, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_VAC_VALVE_MOTOR_TORQUE, data->vac_valve_motor_torque, 1);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_VAC_VALVE_HOMED, data->vac_valve_homed ? 1 : 0);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_TEMP_C, data->temp_c, 1);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_HEATER_SETPOINT, data->heater_setpoint, 1);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_VACUUM_PSIG, data->vacuum_psig, 2);
    /* Pressboi-derived */
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_FORCE_LOAD_CELL, data->force_load_cell, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_FORCE_MOTOR_TORQUE, data->force_motor_torque, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_FORCE_LIMIT, data->force_limit, 1);
    APPEND_STR(buffer, pos, buffer_size, TELEM_KEY_FORCE_SOURCE, data->force_source);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_FORCE_ADC_RAW, data->force_adc_raw);
    APPEND_STR(buffer, pos, buffer_size, TELEM_KEY_FORCE_MODE, data->force_mode);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_JOULES, data->joules, 3);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_ENABLED0, data->enabled0);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_ENABLED1, data->enabled1);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_CURRENT_POS, data->current_pos, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_RETRACT_POS, data->retract_pos, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_TARGET_POS, data->target_pos, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_ENDPOINT, data->endpoint, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_STARTPOINT, data->startpoint, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_PRESS_THRESHOLD, data->press_threshold, 2);
    APPEND_FLOAT(buffer, pos, buffer_size, TELEM_KEY_TORQUE_AVG, data->torque_avg, 1);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_HOMED, data->homed);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_HOME_SENSOR_M0, data->home_sensor_m0);
    APPEND_INT(buffer, pos, buffer_size, TELEM_KEY_HOME_SENSOR_M1, data->home_sensor_m1);
    APPEND_STR(buffer, pos, buffer_size, TELEM_KEY_POLARITY, data->polarity);

    /* Remove trailing comma from last field */
    if (pos > 0 && buffer[pos - 1] == ',') {
        buffer[pos - 1] = '\0';
        pos--;
    }
    return pos;
}

#undef APPEND_INT
#undef APPEND_FLOAT
#undef APPEND_STR

//==================================================================================================
// Telemetry Transmission
//==================================================================================================

// NOTE: You need to provide a sendMessage() implementation based on your comms setup
// For example:
// extern CommsController comms;
// #define sendMessage(msg) comms.enqueueTx(msg, comms.m_guiIp, comms.m_guiPort)

void telemetry_send(const TelemetryData* data) {
    char buffer[1024];
    int len = telemetry_build_message(data, buffer, sizeof(buffer));

    if (len > 0) {
        sendMessage(buffer);
    }
}
