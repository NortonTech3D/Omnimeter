/**
 * @file Calibration.cpp
 * @brief Implementation of Calibration Manager
 * @author Omnimeter Project
 * @version 1.0.0
 */

#include "Calibration.h"

// NVS namespace for calibration data
static const char* NVS_NAMESPACE = "omnical";
static const char* NVS_KEY_DATA = "caldata";

// ============================================================================
// CONSTRUCTOR
// ============================================================================

CalibrationManager::CalibrationManager(Voltmeter& voltmeter)
    : m_voltmeter(voltmeter)
    , m_state(CalibrationState::IDLE)
{
    // Initialize with defaults
    resetToDefaults();
}

// ============================================================================
// PUBLIC METHODS
// ============================================================================

bool CalibrationManager::begin() {
    Serial.println(F("[Calibration] Initializing..."));
    
    // Try to load saved calibration
    if (loadCalibration()) {
        Serial.println(F("[Calibration] Loaded saved calibration data"));
        // CRITICAL: Apply loaded calibration to Voltmeter instance
        applyCalibrationToVoltmeter();
        return true;
    }
    
    Serial.println(F("[Calibration] No valid calibration found, using defaults"));
    resetToDefaults();
    // Apply defaults (gain=1.0, offset=0.0) to Voltmeter
    applyCalibrationToVoltmeter();
    return true;
}

bool CalibrationManager::processCommand(const String& command) {
    String cmd = command;
    cmd.trim();
    cmd.toUpperCase();
    
    // Check if it's a calibration command
    if (!cmd.startsWith("CAL:")) {
        return false;
    }
    
    // Remove "CAL:" prefix
    cmd = cmd.substring(4);
    
    if (cmd == "HELP" || cmd == "?") {
        printHelp();
        return true;
    }
    
    if (cmd == "ZERO") {
        Serial.println(F("\n[CAL] Starting zero calibration..."));
        Serial.println(F("[CAL] Ensure all inputs are shorted to ground!"));
        if (calibrateZero()) {
            Serial.println(F("[CAL] Zero calibration complete!"));
            saveCalibration();
        } else {
            Serial.println(F("[CAL] Zero calibration FAILED!"));
        }
        return true;
    }
    
    if (cmd == "SHOW") {
        printCalibration();
        return true;
    }
    
    if (cmd == "RESET") {
        Serial.println(F("[CAL] Resetting calibration to factory defaults..."));
        resetToDefaults();
        saveCalibration();
        Serial.println(F("[CAL] Calibration reset complete!"));
        return true;
    }
    
    if (cmd == "SAVE") {
        if (saveCalibration()) {
            Serial.println(F("[CAL] Calibration saved to NVS!"));
        } else {
            Serial.println(F("[CAL] Failed to save calibration!"));
        }
        return true;
    }
    
    // CAL:REF:<voltage>
    if (cmd.startsWith("REF:")) {
        String voltStr = cmd.substring(4);
        float voltage = parseVoltage(voltStr);
        if (voltage < 0) {
            Serial.println(F("[CAL] Invalid voltage format. Use CAL:REF:5.000"));
            return true;
        }
        
        Serial.print(F("\n[CAL] Reference calibration with "));
        Serial.print(voltage, 4);
        Serial.println(F(" V"));
        
        if (calibrateReference(voltage)) {
            Serial.println(F("[CAL] Reference calibration complete!"));
            saveCalibration();
        } else {
            Serial.println(F("[CAL] Reference calibration FAILED!"));
        }
        return true;
    }
    
    // CAL:RANGE:<n>:<voltage>
    if (cmd.startsWith("RANGE:")) {
        String params = cmd.substring(6);
        int colonPos = params.indexOf(':');
        if (colonPos < 0) {
            Serial.println(F("[CAL] Invalid format. Use CAL:RANGE:0:5.000"));
            Serial.println(F("[CAL] Ranges: 0=60V, 1=20V, 2=10V, 3=5V, 4=2.5V"));
            return true;
        }
        
        int rangeNum = params.substring(0, colonPos).toInt();
        float voltage = parseVoltage(params.substring(colonPos + 1));
        
        if (rangeNum < 0 || rangeNum > 4) {
            Serial.println(F("[CAL] Invalid range. Use 0-4"));
            return true;
        }
        
        if (voltage < 0) {
            Serial.println(F("[CAL] Invalid voltage format"));
            return true;
        }
        
        VoltageRange range = static_cast<VoltageRange>(rangeNum);
        
        Serial.print(F("\n[CAL] Calibrating range "));
        Serial.print(Voltmeter::getRangeString(range));
        Serial.print(F(" with "));
        Serial.print(voltage, 4);
        Serial.println(F(" V"));
        
        if (calibrateRange(range, voltage)) {
            Serial.println(F("[CAL] Range calibration complete!"));
            saveCalibration();
        } else {
            Serial.println(F("[CAL] Range calibration FAILED!"));
        }
        return true;
    }
    
    Serial.println(F("[CAL] Unknown command. Type CAL:HELP for commands."));
    return true;
}

float CalibrationManager::getOffset(VoltageRange range) const {
    uint8_t idx = static_cast<uint8_t>(range);
    if (idx < CAL_NUM_RANGES) {
        return m_data.ranges[idx].offset;
    }
    return 0.0f;
}

float CalibrationManager::getGain(VoltageRange range) const {
    uint8_t idx = static_cast<uint8_t>(range);
    if (idx < CAL_NUM_RANGES) {
        return m_data.ranges[idx].gain;
    }
    return 1.0f;
}

bool CalibrationManager::isCalibrated(VoltageRange range) const {
    uint8_t idx = static_cast<uint8_t>(range);
    if (idx < CAL_NUM_RANGES) {
        return m_data.ranges[idx].calibrated;
    }
    return false;
}

bool CalibrationManager::calibrateZero() {
    Serial.println(F("[CAL] Measuring zero offset for all ranges..."));
    Serial.println(F("[CAL] Please wait..."));
    
    delay(CAL_SETTLE_TIME_MS);
    
    bool success = true;
    
    for (uint8_t i = 0; i < CAL_NUM_RANGES; i++) {
        VoltageRange range = static_cast<VoltageRange>(i);
        
        Serial.print(F("[CAL]   Range "));
        Serial.print(Voltmeter::getRangeString(range));
        Serial.print(F(": "));
        
        // Take calibration measurement
        float rawMv = takeCalibrationMeasurement(range, CAL_SAMPLES);
        float rawVolts = rawMv * MV_TO_V;
        
        // For zero cal, we expect very low voltage
        // The offset is the negative of what we read (to subtract it later)
        // But we need to account for the existing multiplier
        float multiplier = m_voltmeter.getRangeMultiplier(range);
        float measuredVoltage = rawVolts * multiplier;
        
        if (measuredVoltage > CAL_ZERO_THRESHOLD) {
            Serial.print(F("ERROR - Input not grounded ("));
            Serial.print(measuredVoltage * 1000, 1);
            Serial.println(F(" mV)"));
            success = false;
            continue;
        }
        
        // Store the offset (will be subtracted during measurement)
        m_data.ranges[i].offset = -measuredVoltage;
        
        Serial.print(F("Offset = "));
        Serial.print(m_data.ranges[i].offset * 1000, 2);
        Serial.println(F(" mV"));
    }
    
    m_data.timestamp = millis();
    return success;
}

bool CalibrationManager::calibrateReference(float referenceVoltage) {
    if (referenceVoltage <= 0 || referenceVoltage > 60.0f) {
        Serial.println(F("[CAL] Reference voltage must be between 0 and 60V"));
        return false;
    }
    
    // Find the best range for this voltage
    VoltageRange range = selectRangeForVoltage(referenceVoltage);
    
    Serial.print(F("[CAL] Using range: "));
    Serial.println(Voltmeter::getRangeString(range));
    
    return calibrateRange(range, referenceVoltage);
}

bool CalibrationManager::calibrateRange(VoltageRange range, float referenceVoltage) {
    uint8_t idx = static_cast<uint8_t>(range);
    if (idx >= CAL_NUM_RANGES) {
        return false;
    }
    
    float maxVoltage = Voltmeter::getRangeMaxVoltage(range);
    
    // Check if reference voltage is appropriate for this range
    if (referenceVoltage <= 0 || referenceVoltage > maxVoltage) {
        Serial.print(F("[CAL] Reference voltage must be between 0 and "));
        Serial.print(maxVoltage, 1);
        Serial.println(F(" V for this range"));
        return false;
    }
    
    // For best accuracy, reference should be 50-90% of range
    if (referenceVoltage < maxVoltage * 0.3f) {
        Serial.println(F("[CAL] Warning: Reference voltage is low for this range"));
        Serial.println(F("[CAL] For best accuracy, use 50-90% of full scale"));
    }
    
    Serial.println(F("[CAL] Measuring... Please wait..."));
    delay(CAL_SETTLE_TIME_MS);
    
    // Take measurement
    float rawMv = takeCalibrationMeasurement(range, CAL_SAMPLES);
    float rawVolts = rawMv * MV_TO_V;
    
    // Calculate what voltage we measured with current calibration
    float multiplier = m_voltmeter.getRangeMultiplier(range);
    float currentOffset = m_data.ranges[idx].offset;
    float currentGain = m_data.ranges[idx].gain;
    
    // Raw measured voltage (before calibration)
    float measuredRaw = rawVolts * multiplier;
    
    // Apply existing offset
    float measuredWithOffset = measuredRaw + currentOffset;
    
    Serial.print(F("[CAL] Raw ADC: "));
    Serial.print(rawMv, 2);
    Serial.println(F(" mV"));
    
    Serial.print(F("[CAL] Measured (uncal): "));
    Serial.print(measuredRaw, 4);
    Serial.println(F(" V"));
    
    // Calculate new gain correction
    // We want: measuredWithOffset * newGain = referenceVoltage
    if (measuredWithOffset <= 0.001f) {
        Serial.println(F("[CAL] ERROR: Measured voltage too low"));
        return false;
    }
    
    float newGain = referenceVoltage / measuredWithOffset;
    
    // Sanity check - gain should be close to 1.0 (within 20%)
    if (newGain < 0.8f || newGain > 1.2f) {
        Serial.print(F("[CAL] WARNING: Gain correction is large ("));
        Serial.print(newGain, 4);
        Serial.println(F(")"));
        Serial.println(F("[CAL] This may indicate a hardware issue"));
    }
    
    // Apply new gain
    m_data.ranges[idx].gain = currentGain * newGain;
    m_data.ranges[idx].calibrated = true;
    m_data.timestamp = millis();
    
    Serial.print(F("[CAL] New gain: "));
    Serial.println(m_data.ranges[idx].gain, 6);
    
    // Verify
    float verifyVoltage = (measuredRaw + m_data.ranges[idx].offset) * m_data.ranges[idx].gain;
    Serial.print(F("[CAL] Verified: "));
    Serial.print(verifyVoltage, 4);
    Serial.print(F(" V (error: "));
    Serial.print((verifyVoltage - referenceVoltage) * 1000, 2);
    Serial.println(F(" mV)"));
    
    return true;
}

void CalibrationManager::resetToDefaults() {
    m_data.magic = CAL_MAGIC_NUMBER;
    m_data.version = 1;
    m_data.timestamp = 0;
    
    for (uint8_t i = 0; i < CAL_NUM_RANGES; i++) {
        m_data.ranges[i].offset = 0.0f;
        m_data.ranges[i].gain = 1.0f;
        m_data.ranges[i].calibrated = false;
    }
    
    m_data.checksum = calculateChecksum();
}

bool CalibrationManager::saveCalibration() {
    m_data.checksum = calculateChecksum();
    
    if (!m_prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println(F("[CAL] Failed to open NVS"));
        return false;
    }
    
    size_t written = m_prefs.putBytes(NVS_KEY_DATA, &m_data, sizeof(CalibrationData));
    m_prefs.end();
    
    if (written != sizeof(CalibrationData)) {
        Serial.println(F("[CAL] Failed to write calibration data"));
        return false;
    }
    
    return true;
}

bool CalibrationManager::loadCalibration() {
    if (!m_prefs.begin(NVS_NAMESPACE, true)) {
        return false;
    }
    
    size_t read = m_prefs.getBytes(NVS_KEY_DATA, &m_data, sizeof(CalibrationData));
    m_prefs.end();
    
    if (read != sizeof(CalibrationData)) {
        return false;
    }
    
    return validateData();
}

void CalibrationManager::printCalibration() const {
    Serial.println(F("\n╔════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║            OMNIMETER CALIBRATION DATA                      ║"));
    Serial.println(F("╠════════════════════════════════════════════════════════════╣"));
    
    Serial.print(F("║ Data Valid: "));
    Serial.print(validateData() ? "YES" : "NO ");
    Serial.print(F("        Last Cal: "));
    if (m_data.timestamp > 0) {
        Serial.print(m_data.timestamp / 1000);
        Serial.println(F(" sec              ║"));
    } else {
        Serial.println(F("Never                    ║"));
    }
    
    Serial.println(F("╠══════════╦══════════════╦══════════════╦══════════════════╣"));
    Serial.println(F("║  Range   ║    Offset    ║     Gain     ║   Calibrated     ║"));
    Serial.println(F("╠══════════╬══════════════╬══════════════╬══════════════════╣"));
    
    for (uint8_t i = 0; i < CAL_NUM_RANGES; i++) {
        VoltageRange range = static_cast<VoltageRange>(i);
        const RangeCalibration& cal = m_data.ranges[i];
        
        Serial.print(F("║  "));
        Serial.print(Voltmeter::getRangeString(range));
        // Pad range string
        size_t len = strlen(Voltmeter::getRangeString(range));
        for (size_t j = len; j < 6; j++) Serial.print(' ');
        Serial.print(F("║ "));
        
        // Offset in mV
        char buf[12];
        snprintf(buf, sizeof(buf), "%+8.3f mV", cal.offset * 1000);
        Serial.print(buf);
        Serial.print(F(" ║ "));
        
        // Gain
        snprintf(buf, sizeof(buf), "%10.6f", cal.gain);
        Serial.print(buf);
        Serial.print(F(" ║       "));
        
        Serial.print(cal.calibrated ? "YES" : "NO ");
        Serial.println(F("        ║"));
    }
    
    Serial.println(F("╚══════════╩══════════════╩══════════════╩══════════════════╝"));
    Serial.println();
}

void CalibrationManager::printHelp() const {
    Serial.println(F("\n╔════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║            CALIBRATION COMMANDS                            ║"));
    Serial.println(F("╠════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ CAL:HELP              - Show this help                     ║"));
    Serial.println(F("║ CAL:SHOW              - Display current calibration        ║"));
    Serial.println(F("║ CAL:ZERO              - Calibrate zero (short to GND)      ║"));
    Serial.println(F("║ CAL:REF:<voltage>     - Auto-range calibration             ║"));
    Serial.println(F("║                         Example: CAL:REF:5.000             ║"));
    Serial.println(F("║ CAL:RANGE:<n>:<volt>  - Calibrate specific range           ║"));
    Serial.println(F("║                         Ranges: 0=60V 1=20V 2=10V          ║"));
    Serial.println(F("║                                 3=5V  4=2.5V               ║"));
    Serial.println(F("║                         Example: CAL:RANGE:3:4.500         ║"));
    Serial.println(F("║ CAL:RESET             - Reset to factory defaults          ║"));
    Serial.println(F("║ CAL:SAVE              - Force save to flash                ║"));
    Serial.println(F("╠════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ CALIBRATION PROCEDURE:                                     ║"));
    Serial.println(F("║ 1. Short all inputs to ground, send: CAL:ZERO              ║"));
    Serial.println(F("║ 2. Apply known voltage (50-90% of range), send:            ║"));
    Serial.println(F("║    CAL:REF:<exact_voltage>                                 ║"));
    Serial.println(F("║ 3. Repeat step 2 for other ranges if needed                ║"));
    Serial.println(F("╚════════════════════════════════════════════════════════════╝\n"));
}

// ============================================================================
// PRIVATE METHODS
// ============================================================================

uint32_t CalibrationManager::calculateChecksum() const {
    uint32_t sum = 0;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&m_data);
    
    // Sum all bytes except the checksum field itself
    size_t checksumOffset = offsetof(CalibrationData, checksum);
    for (size_t i = 0; i < checksumOffset; i++) {
        sum += data[i];
        sum = (sum << 1) | (sum >> 31);  // Rotate left
    }
    
    return sum ^ 0xDEADBEEF;
}

bool CalibrationManager::validateData() const {
    if (m_data.magic != CAL_MAGIC_NUMBER) {
        return false;
    }
    
    if (m_data.checksum != calculateChecksum()) {
        return false;
    }
    
    // Sanity check calibration values
    for (uint8_t i = 0; i < CAL_NUM_RANGES; i++) {
        if (m_data.ranges[i].gain < 0.5f || m_data.ranges[i].gain > 2.0f) {
            return false;
        }
        if (m_data.ranges[i].offset < -1.0f || m_data.ranges[i].offset > 1.0f) {
            return false;
        }
    }
    
    return true;
}

float CalibrationManager::takeCalibrationMeasurement(VoltageRange range, uint16_t samples) {
    uint8_t pin;
    
    switch (range) {
        case VoltageRange::RANGE_60V:  pin = PIN_RANGE_60V; break;
        case VoltageRange::RANGE_20V:  pin = PIN_RANGE_20V; break;
        case VoltageRange::RANGE_10V:  pin = PIN_RANGE_10V; break;
        case VoltageRange::RANGE_5V:   pin = PIN_RANGE_5V;  break;
        case VoltageRange::RANGE_2V5:  pin = PIN_RANGE_2V5; break;
        default: return 0.0f;
    }
    
    // Discard first few readings for settling
    for (int i = 0; i < 10; i++) {
        analogReadMilliVolts(pin);
        delay(1);
    }
    
    // Take averaged measurement
    uint32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        sum += analogReadMilliVolts(pin);
        delayMicroseconds(500);  // Small delay between samples
    }
    
    return static_cast<float>(sum) / static_cast<float>(samples);
}

VoltageRange CalibrationManager::selectRangeForVoltage(float voltage) const {
    // Select the smallest range that can measure this voltage
    // with some headroom (use 90% of range max)
    if (voltage <= 2.5f * 0.9f) return VoltageRange::RANGE_2V5;
    if (voltage <= 5.0f * 0.9f) return VoltageRange::RANGE_5V;
    if (voltage <= 10.0f * 0.9f) return VoltageRange::RANGE_10V;
    if (voltage <= 20.0f * 0.9f) return VoltageRange::RANGE_20V;
    return VoltageRange::RANGE_60V;
}

float CalibrationManager::parseVoltage(const String& str) const {
    if (str.length() == 0) return -1.0f;
    
    // Input sanitization: check for valid numeric characters only
    // Allow: digits, decimal point, leading minus (though we reject negative later)
    bool hasDecimal = false;
    bool hasDigit = false;
    
    for (size_t i = 0; i < str.length(); i++) {
        char c = str.charAt(i);
        
        // Allow leading whitespace
        if (c == ' ' || c == '\t') {
            if (hasDigit) continue;  // Trailing whitespace OK after digits
            continue;
        }
        
        // Allow one decimal point
        if (c == '.') {
            if (hasDecimal) return -1.0f;  // Multiple decimals = invalid
            hasDecimal = true;
            continue;
        }
        
        // Allow leading minus (but will reject negative result later)
        if (c == '-' && i == 0) continue;
        
        // Must be a digit
        if (c < '0' || c > '9') {
            Serial.println(F("[CAL] Invalid character in voltage string"));
            return -1.0f;  // Invalid character
        }
        hasDigit = true;
    }
    
    if (!hasDigit) return -1.0f;  // No digits found
    
    float voltage = str.toFloat();
    
    // Validate range
    if (voltage < 0.0f) {
        Serial.println(F("[CAL] Negative voltage not allowed"));
        return -1.0f;
    }
    
    if (voltage > 65.0f) {
        Serial.println(F("[CAL] Voltage exceeds maximum (65V)"));
        return -1.0f;
    }
    
    // Check for NaN/Inf (can happen with malformed input)
    if (isnan(voltage) || isinf(voltage)) {
        Serial.println(F("[CAL] Invalid voltage value (NaN/Inf)"));
        return -1.0f;
    }
    
    return voltage;
}

// ============================================================================
// Apply calibration data to the Voltmeter instance
// ============================================================================
void CalibrationManager::applyCalibrationToVoltmeter() {
    for (uint8_t i = 0; i < CAL_NUM_RANGES; i++) {
        VoltageRange range = static_cast<VoltageRange>(i);
        
        // Ensure safe defaults even if data is corrupted
        float offset = m_data.ranges[i].offset;
        float gain = m_data.ranges[i].gain;
        
        // Sanity clamp - offset should be small
        if (offset < -1.0f) offset = -1.0f;
        if (offset > 1.0f) offset = 1.0f;
        
        // Sanity clamp - gain should be close to 1.0
        if (gain < 0.5f) gain = 1.0f;  // Default to 1.0 if corrupted
        if (gain > 2.0f) gain = 1.0f;
        
        m_voltmeter.setCalibration(range, offset, gain);
    }
    
    Serial.println(F("[Calibration] Applied calibration to Voltmeter"));
}
