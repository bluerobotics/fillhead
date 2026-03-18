/**
 * @file force_sensor.h
 * @author Eldin Miller-Stead
 * @date November 3, 2025
 * @brief Defines the force sensor interface for reading HX711 data via Rugeduino.
 *
 * @details This class manages communication with a Rugeduino (Arduino Uno) connected to COM-0.
 * The Rugeduino reads the HX711 load cell amplifier and sends force readings via serial.
 */
#pragma once

#include "config.h"
#include "ClearCore.h"
#include "SerialDriver.h"

/**
 * @class ForceSensor
 * @brief Manages force readings from HX711 via Rugeduino on COM-0.
 * 
 * @details Communicates with Rugeduino (Arduino Uno) over UART (TTL mode) to receive
 * force readings from an HX711 load cell amplifier.
 */
class ForceSensor {
public:
    /**
     * @brief Constructs a new ForceSensor object.
     */
    ForceSensor();

    /**
     * @brief Initializes the serial port for communication with Rugeduino.
     * @details Configures COM-0 in TTL mode at 115200 baud.
     */
    void setup();

    /**
     * @brief Updates force reading by parsing incoming serial data.
     * @details Call this repeatedly in the main loop to process incoming data.
     */
    void update();

    /**
     * @brief Gets the most recent force reading.
     * @return Force in kilograms (kg)
     */
    float getForce() const { return m_force_kg; }
    
    /**
     * @brief Gets the raw ADC value from the HX711.
     * @return Raw 24-bit value
     */
    long getRawValue() const { return m_raw_value; }
    
    /**
     * @brief Checks if sensor is receiving data.
     * @return true if data received in last second
     */
    bool isConnected() const;
    
    /**
     * @brief Sends tare command to Rugeduino to zero the scale.
     */
    void tare();
    
    /**
     * @brief Set force sensor offset (kg) and save to NVM.
     * @param offset_kg Offset in kilograms to add to all readings
     */
    void setOffset(float offset_kg);
    
    /**
     * @brief Set force sensor scale factor and save to NVM.
     * @param scale Multiplicative scale factor (default 1.0)
     */
    void setScale(float scale);
    
    /**
     * @brief Get current offset value.
     * @return Offset in kilograms
     */
    float getOffset() const { return m_offset_kg; }
    
    /**
     * @brief Get current scale value.
     * @return Scale factor
     */
    float getScale() const { return m_scale; }

private:
    /**
     * @brief Parses incoming serial data from Rugeduino.
     */
    void parseSerialData();

    float m_force_kg;              ///< Current force reading in kg
    long m_raw_value;              ///< Raw ADC value from HX711
    uint32_t m_last_reading_time;  ///< Timestamp of last valid reading
    char m_serial_buffer[64];      ///< Buffer for incoming serial data
    uint8_t m_buffer_index;        ///< Current position in serial buffer
    float m_offset_kg;             ///< Calibration offset in kg (loaded from NVM)
    float m_scale;                 ///< Calibration scale factor (loaded from NVM)
    
    void loadCalibrationFromNVM(); ///< Load calibration from non-volatile memory
};

