/**
 * @file Calibration.h
 * @brief Calibration Manager for ESP32-S2 Digital Multimeter
 * @author Omnimeter Project
 * @version 1.0.0
 * 
 * @details
 * Provides runtime calibration with persistent storage in NVS (Non-Volatile Storage).
 * Supports two-point calibration for each voltage range to correct both offset and gain.
 * 
 * Calibration Procedure:
 * 1. Connect a known precision voltage source
 * 2. Use serial commands to calibrate each range
 * 3. Calibration data is automatically saved to NVS
 * 
 * Serial Commands:
 *   CAL:ZERO              - Calibrate zero offset (short inputs to ground)
 *   CAL:REF:<voltage>     - Calibrate with reference voltage (e.g., CAL:REF:5.000)
 *   CAL:RANGE:<n>:<volt>  - Calibrate specific range with voltage
 *   CAL:SHOW              - Display current calibration values
 *   CAL:RESET             - Reset calibration to factory defaults
 *   CAL:SAVE              - Force save calibration to NVS
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Preferences.h>
#include "Voltmeter.h"

// ============================================================================
// CALIBRATION CONSTANTS
// ============================================================================

constexpr uint8_t CAL_NUM_RANGES = 5;
constexpr uint16_t CAL_SAMPLES = 100;           // Samples for calibration averaging
constexpr uint32_t CAL_SETTLE_TIME_MS = 500;    // Settling time before calibration
constexpr float CAL_ZERO_THRESHOLD = 0.050f;    // Max voltage for zero calibration (50mV)
constexpr uint32_t CAL_MAGIC_NUMBER = 0xCA11B001;  // Magic number for valid calibration

// ============================================================================
// CALIBRATION DATA STRUCTURES
// ============================================================================

/**
 * @brief Calibration data for a single range
 */
struct RangeCalibration {
    float offset;       // Zero offset in volts (additive correction)
    float gain;         // Gain correction multiplier (multiplicative correction)
    bool calibrated;    // True if this range has been calibrated
};

/**
 * @brief Complete calibration data set
 */
struct CalibrationData {
    uint32_t magic;                         // Magic number for validation
    uint32_t version;                       // Calibration data version
    RangeCalibration ranges[CAL_NUM_RANGES]; // Per-range calibration
    uint32_t timestamp;                     // Last calibration timestamp
    uint32_t checksum;                      // Data integrity checksum
};

/**
 * @brief Calibration state machine states
 */
enum class CalibrationState : uint8_t {
    IDLE,
    WAITING_SETTLE,
    MEASURING,
    COMPLETE,
    ERROR
};

// ============================================================================
// CALIBRATION MANAGER CLASS
// ============================================================================

class CalibrationManager {
public:
    /**
     * @brief Construct CalibrationManager with reference to Voltmeter
     * @param voltmeter Reference to the voltmeter instance
     */
    explicit CalibrationManager(Voltmeter& voltmeter);
    
    /**
     * @brief Initialize calibration manager and load saved data
     * @return true if initialization successful
     */
    bool begin();
    
    /**
     * @brief Process serial commands for calibration
     * @param command Command string to process
     * @return true if command was handled
     */
    bool processCommand(const String& command);
    
    /**
     * @brief Get calibration offset for a range
     * @param range Voltage range
     * @return Calibration offset in volts
     */
    float getOffset(VoltageRange range) const;
    
    /**
     * @brief Get calibration gain for a range
     * @param range Voltage range
     * @return Calibration gain multiplier
     */
    float getGain(VoltageRange range) const;
    
    /**
     * @brief Check if a range has been calibrated
     * @param range Voltage range
     * @return true if calibrated
     */
    bool isCalibrated(VoltageRange range) const;
    
    /**
     * @brief Calibrate zero offset (inputs shorted to ground)
     * @return true if successful
     */
    bool calibrateZero();
    
    /**
     * @brief Calibrate with a known reference voltage
     * @param referenceVoltage The known accurate voltage applied
     * @return true if successful
     */
    bool calibrateReference(float referenceVoltage);
    
    /**
     * @brief Calibrate a specific range with known voltage
     * @param range Range to calibrate
     * @param referenceVoltage Known accurate voltage
     * @return true if successful
     */
    bool calibrateRange(VoltageRange range, float referenceVoltage);
    
    /**
     * @brief Reset all calibration to factory defaults
     */
    void resetToDefaults();
    
    /**
     * @brief Save calibration data to NVS
     * @return true if successful
     */
    bool saveCalibration();
    
    /**
     * @brief Load calibration data from NVS
     * @return true if valid data was loaded
     */
    bool loadCalibration();
    
    /**
     * @brief Print current calibration values to Serial
     */
    void printCalibration() const;
    
    /**
     * @brief Print calibration help/commands to Serial
     */
    void printHelp() const;

private:
    Voltmeter& m_voltmeter;
    Preferences m_prefs;
    CalibrationData m_data;
    CalibrationState m_state;
    
    /**
     * @brief Calculate checksum for calibration data
     * @return Calculated checksum
     */
    uint32_t calculateChecksum() const;
    
    /**
     * @brief Validate calibration data integrity
     * @return true if data is valid
     */
    bool validateData() const;
    
    /**
     * @brief Take averaged measurement for calibration
     * @param range Range to measure
     * @param samples Number of samples to average
     * @return Averaged raw ADC millivolts
     */
    float takeCalibrationMeasurement(VoltageRange range, uint16_t samples);
    
    /**
     * @brief Determine best range for a given voltage
     * @param voltage Voltage to measure
     * @return Optimal range for that voltage
     */
    VoltageRange selectRangeForVoltage(float voltage) const;
    
    /**
     * @brief Parse voltage value from string (with input sanitization)
     * @param str String containing voltage
     * @return Parsed voltage, or -1 on error
     */
    float parseVoltage(const String& str) const;
    
    /**
     * @brief Apply current calibration data to the Voltmeter instance
     * @details Called after loading from NVS or resetting to defaults
     */
    void applyCalibrationToVoltmeter();
};

#endif // CALIBRATION_H
