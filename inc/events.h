/**
 * @file events.h
 * @brief Defines all event types and status prefixes for the Fillhead controller.
 * @details AUTO-GENERATED FILE - DO NOT EDIT MANUALLY
 * Generated from events.json
 * 
 * This header file defines all events sent FROM the Fillhead device TO the host.
 * Events are asynchronous notifications that can trigger host-side actions.
 * For command definitions (host → device), see commands.h
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

//==================================================================================================
// Status Message Prefixes (Device → Host)
//==================================================================================================

/**
 * @name Status Message Prefixes
 * @{
 */
#define STATUS_PREFIX_INFO                  "FILLHEAD_INFO: "
#define STATUS_PREFIX_START                 "FILLHEAD_START: "
#define STATUS_PREFIX_DONE                  "FILLHEAD_DONE: "
#define STATUS_PREFIX_ERROR                 "FILLHEAD_ERROR: "
#define STATUS_PREFIX_RECOVERY              "FILLHEAD_RECOVERY: "
#define STATUS_PREFIX_DISCOVERY             "DISCOVERY_RESPONSE: "
/** @} */

/**
 * @name Telemetry Prefix
 * @{
 */
#define TELEM_PREFIX                        "FILLHEAD_TELEM: "
/** @} */

/**
 * @name Event Prefix
 * @{
 */
#define EVENT_PREFIX                        "FILLHEAD_EVENT: "
/** @} */

//==================================================================================================
// Event String Definitions
//==================================================================================================

/**
 * @name Event String Identifiers
 * @{
 */
#define EVENT_STR_SCRIPT_RUN                          "script_run"
#define EVENT_STR_SCRIPT_HOLD                         "script_hold"
#define EVENT_STR_SCRIPT_RESET                        "script_reset"
/** @} */

//==================================================================================================
// Event Enum
//==================================================================================================

typedef enum {
    EVENT_UNKNOWN,

    EVENT_SCRIPT_RUN,
    EVENT_SCRIPT_HOLD,
    EVENT_SCRIPT_RESET
} Event;

//==================================================================================================
// Event Sending Functions
//==================================================================================================

void sendEvent(Event event);
void sendEventInt(Event event, int32_t param);
void sendEventString(Event event, const char* param);
void sendEventMulti(Event event, int32_t param1, int32_t param2);
