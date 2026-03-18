/**
 * @file events.cpp
 * @brief Event sending implementation for the Fillhead controller.
 */

#include "events.h"
#include "commands.h"
#include <stdio.h>
#include <string.h>

extern void sendMessage(const char* msg);

void sendEvent(Event event) {
    char buffer[256];
    const char* eventStr = NULL;
    
    switch (event) {
        case EVENT_SCRIPT_RUN:  eventStr = EVENT_STR_SCRIPT_RUN; break;
        case EVENT_SCRIPT_HOLD: eventStr = EVENT_STR_SCRIPT_HOLD; break;
        case EVENT_SCRIPT_RESET: eventStr = EVENT_STR_SCRIPT_RESET; break;
        case EVENT_UNKNOWN:
        default:
            return;
    }
    
    snprintf(buffer, sizeof(buffer), "%s%s", EVENT_PREFIX, eventStr);
    sendMessage(buffer);
}

void sendEventInt(Event event, int32_t param) {
    switch (event) {
        default:
            sendEvent(event);
            break;
    }
}

void sendEventString(Event event, const char* param) {
    char buffer[256];
    const char* eventStr = NULL;
    
    switch (event) {
        case EVENT_SCRIPT_RUN:  eventStr = EVENT_STR_SCRIPT_RUN; break;
        case EVENT_SCRIPT_HOLD: eventStr = EVENT_STR_SCRIPT_HOLD; break;
        case EVENT_SCRIPT_RESET: eventStr = EVENT_STR_SCRIPT_RESET; break;
        default:
            sendEvent(event);
            return;
    }
    
    snprintf(buffer, sizeof(buffer), "%s%s %s", EVENT_PREFIX, eventStr, param);
    sendMessage(buffer);
}

void sendEventMulti(Event event, int32_t param1, int32_t param2) {
    switch (event) {
        default:
            sendEventInt(event, param1);
            break;
    }
}
