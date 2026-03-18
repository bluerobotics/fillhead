/**
 * @file responses.h
 * @brief Response message formats for the Fillhead controller.
 * @details Status prefixes (FILLHEAD_INFO, FILLHEAD_DONE, FILLHEAD_ERROR, etc.) are defined in events.h.
 * This file provides backward compatibility and usage documentation.
 *
 * For command definitions (host → device), see commands.h
 * For telemetry structure, see telemetry.h
 */
#pragma once

#include "events.h"

//==================================================================================================
// Usage Examples
//==================================================================================================

/**
 * @section Status Message Example
 * @code
 * // Send an info message
 * Serial.print(STATUS_PREFIX_INFO);
 * Serial.println("System initialized");
 *
 * // Send a completion message
 * Serial.print(STATUS_PREFIX_DONE);
 * Serial.println("HEATER_ON");
 * @endcode
 *
 * @section Telemetry Message Example
 * @code
 * // Use the telemetry.h interface for sending telemetry
 * #include "telemetry.h"
 *
 * TelemetryData telem;
 * telemetry_init(&telem);
 * // ... update telem fields ...
 * telemetry_send(&telem);
 * @endcode
 */
