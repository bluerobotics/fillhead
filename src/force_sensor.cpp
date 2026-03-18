/**
 * @file force_sensor.cpp
 * @author Eldin Miller-Stead
 * @date November 3, 2025
 * @brief Implementation of force sensor communication with Rugeduino/HX711.
 */

#include "force_sensor.h"
#include "NvmManager.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace ClearCore;

// NVM locations for force sensor calibration (8 bytes starting at user area)
#define NVM_LOC_FORCE_OFFSET    NvmManager::NVM_LOC_USER_START      // 4 bytes
#define NVM_LOC_FORCE_SCALE     (NvmManager::NVM_LOC_USER_START + 4) // 4 bytes

ForceSensor::ForceSensor() {
    m_force_kg = 0.0f;
    m_raw_value = 0;
    m_last_reading_time = 0;
    m_buffer_index = 0;
    m_offset_kg = FORCE_SENSOR_OFFSET_KG;  // Default from config
    m_scale = FORCE_SENSOR_SCALE_FACTOR;  // Default scale from config
    memset(m_serial_buffer, 0, sizeof(m_serial_buffer));
}

void ForceSensor::setup() {
    // Configure COM-0 for TTL UART communication
    ConnectorCOM0.Mode(Connector::TTL);
    ConnectorCOM0.Speed(115200);  // Match Rugeduino baud rate
    ConnectorCOM0.PortOpen();
    
    // Load calibration from NVM
    loadCalibrationFromNVM();
    
    // Wait for port to stabilize
    Delay_ms(100);
}

void ForceSensor::update() {
    // Read any available data from COM-0
    int c;
    while ((c = ConnectorCOM0.CharGet()) != -1) {
        // Process character
        if (c == '\n' || c == '\r') {
            // End of line - parse the accumulated data
            if (m_buffer_index > 0) {
                m_serial_buffer[m_buffer_index] = '\0';
                parseSerialData();
                m_buffer_index = 0;
            }
        } else if (m_buffer_index < sizeof(m_serial_buffer) - 1) {
            // Add character to buffer
            m_serial_buffer[m_buffer_index++] = (char)c;
        } else {
            // Buffer overflow - reset
            m_buffer_index = 0;
        }
    }
}

void ForceSensor::parseSerialData() {
    // Expected format from Rugeduino: "123456" (raw tared ADC value as integer)
    long raw_adc = 0;
    
    if (sscanf(m_serial_buffer, "%ld", &raw_adc) == 1) {
        // Store raw value
        m_raw_value = raw_adc;
        
        // Apply calibration equation: kg = (raw_adc × scale) + offset
        m_force_kg = (raw_adc * m_scale) + m_offset_kg;
        m_last_reading_time = Milliseconds();
    }
}

bool ForceSensor::isConnected() const {
    // Consider connected if we received data in the last second
    return (Milliseconds() - m_last_reading_time) < 1000;
}

void ForceSensor::tare() {
    // No-op: Rugeduino only sends raw values
}

void ForceSensor::setOffset(float offset_kg) {
    m_offset_kg = offset_kg;
    // Store as int32 (reinterpret float bits as int32)
    int32_t offset_bits;
    memcpy(&offset_bits, &offset_kg, sizeof(float));
    
    // Access NvmMgr from ClearCore namespace
    NvmManager &nvmMgr = NvmManager::Instance();
    nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_OFFSET), offset_bits);
}

void ForceSensor::setScale(float scale) {
    m_scale = scale;
    // Store as int32 (reinterpret float bits as int32)
    int32_t scale_bits;
    memcpy(&scale_bits, &scale, sizeof(float));
    
    // Access NvmMgr from ClearCore namespace
    NvmManager &nvmMgr = NvmManager::Instance();
    nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_SCALE), scale_bits);
}

void ForceSensor::loadCalibrationFromNVM() {
    // Access NvmMgr from ClearCore namespace
    NvmManager &nvmMgr = NvmManager::Instance();
    
    // Load offset
    int32_t offset_bits = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_OFFSET));
    // Check if NVM contains valid data (non-zero and not 0xFFFFFFFF which is erased flash)
    if (offset_bits != 0 && offset_bits != -1) {
        float temp_offset;
        memcpy(&temp_offset, &offset_bits, sizeof(float));
        // Validate offset is in reasonable range (-50 to +50 kg)
        if (temp_offset > -50.0f && temp_offset < 50.0f) {
            m_offset_kg = temp_offset;
        } else {
            // Invalid - write default to NVM
            int32_t default_bits;
            memcpy(&default_bits, &m_offset_kg, sizeof(float));
            nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_OFFSET), default_bits);
        }
    } else {
        // Empty NVM - initialize with default from config.h
        int32_t default_bits;
        memcpy(&default_bits, &m_offset_kg, sizeof(float));
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_OFFSET), default_bits);
    }
    
    // Load scale
    int32_t scale_bits = nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_SCALE));
    if (scale_bits != 0 && scale_bits != -1) {
        float temp_scale;
        memcpy(&temp_scale, &scale_bits, sizeof(float));
        // Validate scale is in reasonable range (-0.01 to +0.01, excluding very small values)
        if ((temp_scale > -0.01f && temp_scale < -0.00001f) || (temp_scale > 0.00001f && temp_scale < 0.01f)) {
            m_scale = temp_scale;
        } else {
            // Invalid - write default to NVM
            int32_t default_bits;
            memcpy(&default_bits, &m_scale, sizeof(float));
            nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_SCALE), default_bits);
        }
    } else {
        // Empty NVM - initialize with default from config.h
        int32_t default_bits;
        memcpy(&default_bits, &m_scale, sizeof(float));
        nvmMgr.Int32(static_cast<NvmManager::NvmLocations>(NVM_LOC_FORCE_SCALE), default_bits);
    }
}

