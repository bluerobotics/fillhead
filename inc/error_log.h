#pragma once

#include <stdint.h>
#include <cstring>

// Log configuration
#define ERROR_LOG_SIZE 100           ///< Maximum number of log entries (circular buffer)
#define ERROR_LOG_MSG_LENGTH 80      ///< Maximum length of each log message
#define HEARTBEAT_LOG_SIZE 2880      ///< 24 hours of heartbeats at 30-second intervals

/**
 * @enum LogLevel
 * @brief Defines the severity level of log entries.
 */
enum LogLevel : uint8_t {
    LOG_DEBUG,      ///< Debug information (verbose)
    LOG_INFO,       ///< General information
    LOG_WARNING,    ///< Warning - something unexpected but not critical
    LOG_ERROR,      ///< Error - something went wrong
    LOG_CRITICAL    ///< Critical error - system may be in bad state
};

/**
 * @struct LogEntry
 * @brief Represents a single log entry in the circular buffer.
 */
struct LogEntry {
    uint32_t timestamp;                  ///< Millisecond timestamp when entry was created
    LogLevel level;                      ///< Severity level of the entry
    char message[ERROR_LOG_MSG_LENGTH];  ///< The log message
};

/**
 * @struct HeartbeatEntry
 * @brief Compact heartbeat status entry (only 8 bytes vs 88 bytes for LogEntry).
 */
struct HeartbeatEntry {
    uint32_t timestamp;      ///< Millisecond timestamp
    uint8_t usbConnected;    ///< 1 if USB host connected, 0 if disconnected
    uint8_t networkActive;   ///< 1 if network link up, 0 if down
    uint8_t usbAvailable;    ///< Bytes available in USB TX buffer (0-255)
    uint8_t reserved;        ///< Reserved for future use
};

/**
 * @class ErrorLog
 * @brief Manages a circular buffer of log entries for firmware diagnostics.
 */
class ErrorLog {
public:
    ErrorLog();

    void log(LogLevel level, const char* message);
    void logf(LogLevel level, const char* format, ...);

    int getEntryCount() const;
    bool getEntry(int index, LogEntry* entry) const;
    void clear();

private:
    LogEntry m_buffer[ERROR_LOG_SIZE];
    int m_head;
    int m_count;
};

/**
 * @class HeartbeatLog
 * @brief Manages a compact circular buffer for system health heartbeats.
 */
class HeartbeatLog {
public:
    HeartbeatLog();

    void log(bool usbConnected, bool networkActive, uint8_t usbAvailable);

    int getEntryCount() const;
    bool getEntry(int index, HeartbeatEntry* entry) const;
    void clear();

private:
    HeartbeatEntry m_buffer[HEARTBEAT_LOG_SIZE];
    int m_head;
    int m_count;
};

// Global log instances
extern ErrorLog g_errorLog;
extern HeartbeatLog g_heartbeatLog;

