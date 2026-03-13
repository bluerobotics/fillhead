#include "error_log.h"
#include "ClearCore.h"
#include <cstdarg>
#include <cstdio>

// Global log instances
ErrorLog g_errorLog;
HeartbeatLog g_heartbeatLog;

ErrorLog::ErrorLog() {
    m_head = 0;
    m_count = 0;
}

void ErrorLog::log(LogLevel level, const char* message) {
    (void)level; // Level is stored for diagnostics but not used to filter here.

    uint32_t timestamp = Milliseconds();

    m_buffer[m_head].timestamp = timestamp;
    m_buffer[m_head].level = level;
    strncpy(m_buffer[m_head].message, message, ERROR_LOG_MSG_LENGTH - 1);
    m_buffer[m_head].message[ERROR_LOG_MSG_LENGTH - 1] = '\0';

    m_head = (m_head + 1) % ERROR_LOG_SIZE;
    if (m_count < ERROR_LOG_SIZE) {
        m_count++;
    }
}

void ErrorLog::logf(LogLevel level, const char* format, ...) {
    char buffer[ERROR_LOG_MSG_LENGTH];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, ERROR_LOG_MSG_LENGTH, format, args);
    va_end(args);

    log(level, buffer);
}

int ErrorLog::getEntryCount() const {
    return m_count;
}

bool ErrorLog::getEntry(int index, LogEntry* entry) const {
    if (index < 0 || index >= m_count) {
        return false;
    }

    int bufferIndex;
    if (m_count < ERROR_LOG_SIZE) {
        bufferIndex = index;
    } else {
        bufferIndex = (m_head + index) % ERROR_LOG_SIZE;
    }

    *entry = m_buffer[bufferIndex];
    return true;
}

void ErrorLog::clear() {
    m_head = 0;
    m_count = 0;
}

HeartbeatLog::HeartbeatLog() {
    m_head = 0;
    m_count = 0;
}

void HeartbeatLog::log(bool usbConnected, bool networkActive, uint8_t usbAvailable) {
    m_buffer[m_head].timestamp = Milliseconds();
    m_buffer[m_head].usbConnected = usbConnected ? 1 : 0;
    m_buffer[m_head].networkActive = networkActive ? 1 : 0;
    m_buffer[m_head].usbAvailable = usbAvailable;
    m_buffer[m_head].reserved = 0;

    m_head = (m_head + 1) % HEARTBEAT_LOG_SIZE;
    if (m_count < HEARTBEAT_LOG_SIZE) {
        m_count++;
    }
}

int HeartbeatLog::getEntryCount() const {
    return m_count;
}

bool HeartbeatLog::getEntry(int index, HeartbeatEntry* entry) const {
    if (index < 0 || index >= m_count) {
        return false;
    }

    int bufferIndex;
    if (m_count < HEARTBEAT_LOG_SIZE) {
        bufferIndex = index;
    } else {
        bufferIndex = (m_head + index) % HEARTBEAT_LOG_SIZE;
    }

    *entry = m_buffer[bufferIndex];
    return true;
}

void HeartbeatLog::clear() {
    m_head = 0;
    m_count = 0;
}

