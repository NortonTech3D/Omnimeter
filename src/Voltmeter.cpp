/**
 * @file Voltmeter.cpp
 * @brief Implementation of the Precision ADC Driver for ESP32-S2 Digital Multimeter
 * @author Omnimeter Project
 * @version 1.1.0
 * 
 * ============================================================================
 * OPTIMIZATION CHANGES (v1.1.0):
 * - Added setResult() helper to eliminate 9x duplicated result-building code
 * - Implemented hysteresis-aware auto-ranging with m_currentRange tracking
 * - Added range debouncing to prevent rapid switching
 * - Explicit 13-bit ADC resolution configuration
 * ============================================================================
 */

#include "Voltmeter.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Voltmeter::Voltmeter()
    : m_initialized(false)
    , m_oversampleCount(ADC_OVERSAMPLE_COUNT)
    , m_lastResult{0.0f, 0.0f, VoltageRange::RANGE_INVALID, false, false, 0}
    , m_currentRange(VoltageRange::RANGE_60V)
    , m_rangeStableCount(0)
    , m_emaVoltage(0.0f)
    , m_emaInitialized(false)
{
    // Initialize runtime calibration to SAFE DEFAULTS
    // Critical: gain = 1.0 ensures device works even without calibration
    for (int i = 0; i < 5; i++) {
        m_calOffsets[i] = 0.0f;  // No offset correction
        m_calGains[i] = 1.0f;    // Unity gain (no scaling)
    }
}

// ============================================================================
// PUBLIC METHODS
// ============================================================================

bool Voltmeter::begin() {
    // Configure ADC resolution (ESP32-S2 supports up to 13-bit)
    analogReadResolution(13);
    
    // Set ADC attenuation for ~2.5V max input range
    // ADC_11db attenuation gives approximately 0-2500mV range
    // Note: This is labeled as ~12dB in hardware but API uses ADC_11db constant
    analogSetAttenuation(ADC_11db);
    
    // Configure individual pins
    analogSetPinAttenuation(PIN_RANGE_60V, ADC_11db);
    analogSetPinAttenuation(PIN_RANGE_20V, ADC_11db);
    analogSetPinAttenuation(PIN_RANGE_10V, ADC_11db);
    analogSetPinAttenuation(PIN_RANGE_5V, ADC_11db);
    analogSetPinAttenuation(PIN_RANGE_2V5, ADC_11db);
    
    // Initialize ADC calibration characteristics using factory calibration
    // ESP32-S2 stores calibration data in eFuse
    esp_adc_cal_value_t calType = esp_adc_cal_characterize(
        ADC_UNIT_1,                    // ADC unit
        ADC_ATTEN_DB_12,               // Attenuation (12dB, formerly 11dB)
        ADC_WIDTH_BIT_13,              // Resolution (13-bit for ESP32-S2)
        1100,                          // Default VREF (overridden by eFuse if available)
        &m_adcChars                    // Output characteristics structure
    );
    
    // Log calibration source
    Serial.print(F("[Voltmeter] ADC Calibration type: "));
    switch (calType) {
        case ESP_ADC_CAL_VAL_EFUSE_VREF:
            Serial.println(F("eFuse VREF"));
            break;
        case ESP_ADC_CAL_VAL_EFUSE_TP:
            Serial.println(F("eFuse Two Point"));
            break;
        case ESP_ADC_CAL_VAL_DEFAULT_VREF:
            Serial.println(F("Default VREF (no eFuse calibration)"));
            break;
        default:
            Serial.println(F("Unknown"));
            break;
    }
    
    // Perform initial dummy reads to stabilize ADC
    for (int i = 0; i < 10; i++) {
        analogReadMilliVolts(PIN_RANGE_60V);
    }
    
    m_initialized = true;
    
    Serial.println(F("[Voltmeter] Initialization complete"));
    Serial.print(F("[Voltmeter] Oversample count: "));
    Serial.println(m_oversampleCount);
    
    // Print multiplier values for debugging
    Serial.println(F("[Voltmeter] Range multipliers:"));
    Serial.print(F("  60V: ")); Serial.println(MULTIPLIER_60V, 4);
    Serial.print(F("  20V: ")); Serial.println(MULTIPLIER_20V, 4);
    Serial.print(F("  10V: ")); Serial.println(MULTIPLIER_10V, 4);
    Serial.print(F("   5V: ")); Serial.println(MULTIPLIER_5V, 4);
    Serial.print(F(" 2.5V: ")); Serial.println(MULTIPLIER_2V5, 4);
    
    return true;
}

MeasurementResult Voltmeter::measure() {
    if (!m_initialized) {
        return setResult(0.0f, 0.0f, VoltageRange::RANGE_INVALID, false, false);
    }
    
    // ========================================================================
    // HYSTERESIS-AWARE AUTO-RANGING CASCADE
    // ========================================================================
    // Start with the highest range (60V) and cascade down to find optimal.
    // Uses different thresholds for UP vs DOWN transitions to prevent
    // oscillation at boundary voltages.
    //
    // SAFETY: Always read highest range first to detect dangerous voltages
    // ========================================================================
    
    // Step 1: Read 60V range first (always safe)
    float adcMv60V = readAdcMillivolts(PIN_RANGE_60V);
    float voltage60V = calculateVoltage(adcMv60V, VoltageRange::RANGE_60V);
    
    // Check for overrange (>60V - dangerous!)
    if (isSaturated(adcMv60V)) {
        m_currentRange = VoltageRange::RANGE_60V;
        return setResult(voltage60V, adcMv60V, VoltageRange::RANGE_60V, false, true);
    }
    
    // Determine target range based on voltage and current range (hysteresis)
    VoltageRange targetRange = determineOptimalRange(voltage60V);
    
    // Debounce: only switch after stable readings
    if (targetRange != m_currentRange) {
        m_rangeStableCount++;
        if (m_rangeStableCount < RANGE_DEBOUNCE_COUNT) {
            // Not stable yet, use current range
            targetRange = m_currentRange;
        } else {
            // Stable, commit to new range
            m_currentRange = targetRange;
            m_rangeStableCount = 0;
        }
    } else {
        m_rangeStableCount = 0;
    }
    
    // Read from the selected range
    return measureAtRange(targetRange, voltage60V, adcMv60V);
}

// ============================================================================
// Helper: Set result and return (eliminates code duplication)
// ============================================================================
MeasurementResult Voltmeter::setResult(float voltage, float adcMv, 
                                        VoltageRange range, bool valid, bool overrange) {
    // Apply EMA filter for smoothing (only for valid measurements)
    if (valid && !overrange) {
        if (!m_emaInitialized) {
            // First valid measurement - initialize EMA with raw value
            m_emaVoltage = voltage;
            m_emaInitialized = true;
        } else {
            // EMA formula: filtered = alpha * new_value + (1 - alpha) * old_filtered
            m_emaVoltage = EMA_ALPHA * voltage + (1.0f - EMA_ALPHA) * m_emaVoltage;
        }
        // Use smoothed voltage for the result
        voltage = m_emaVoltage;
    }
    
    m_lastResult.voltage = voltage;
    m_lastResult.rawAdcMillivolts = adcMv;
    m_lastResult.activeRange = range;
    m_lastResult.valid = valid;
    m_lastResult.overrange = overrange;
    m_lastResult.timestamp = millis();
    return m_lastResult;
}

// ============================================================================
// Helper: Determine optimal range with hysteresis
// ============================================================================
VoltageRange Voltmeter::determineOptimalRange(float voltage) {
    // Use different thresholds based on current range direction
    // This prevents oscillation at boundary voltages
    
    switch (m_currentRange) {
        case VoltageRange::RANGE_60V:
            // Currently on 60V, check if we can go DOWN
            if (voltage < THRESHOLD_DOWN_TO_20V) return VoltageRange::RANGE_20V;
            return VoltageRange::RANGE_60V;
            
        case VoltageRange::RANGE_20V:
            // Check UP threshold (with hysteresis)
            if (voltage >= THRESHOLD_UP_TO_60V) return VoltageRange::RANGE_60V;
            // Check DOWN threshold
            if (voltage < THRESHOLD_DOWN_TO_10V) return VoltageRange::RANGE_10V;
            return VoltageRange::RANGE_20V;
            
        case VoltageRange::RANGE_10V:
            if (voltage >= THRESHOLD_UP_TO_20V) return VoltageRange::RANGE_20V;
            if (voltage < THRESHOLD_DOWN_TO_5V) return VoltageRange::RANGE_5V;
            return VoltageRange::RANGE_10V;
            
        case VoltageRange::RANGE_5V:
            if (voltage >= THRESHOLD_UP_TO_10V) return VoltageRange::RANGE_10V;
            if (voltage < THRESHOLD_DOWN_TO_2V5) return VoltageRange::RANGE_2V5;
            return VoltageRange::RANGE_5V;
            
        case VoltageRange::RANGE_2V5:
            if (voltage >= THRESHOLD_UP_TO_5V) return VoltageRange::RANGE_5V;
            return VoltageRange::RANGE_2V5;
            
        default:
            return VoltageRange::RANGE_60V;
    }
}

// ============================================================================
// Helper: Measure at specific range with fallback
// ============================================================================
MeasurementResult Voltmeter::measureAtRange(VoltageRange range, 
                                             float fallbackVoltage, float fallbackAdcMv) {
    if (range == VoltageRange::RANGE_60V) {
        return setResult(fallbackVoltage, fallbackAdcMv, VoltageRange::RANGE_60V, true, false);
    }
    
    uint8_t pin = getRangePin(range);
    float adcMv = readAdcMillivolts(pin);
    
    // Check saturation - if saturated, fall back to previous range
    if (isSaturated(adcMv)) {
        VoltageRange fallbackRange = static_cast<VoltageRange>(
            static_cast<uint8_t>(range) - 1);
        return setResult(fallbackVoltage, fallbackAdcMv, 
                        (fallbackRange >= VoltageRange::RANGE_60V) ? fallbackRange : VoltageRange::RANGE_60V, 
                        true, false);
    }
    
    float voltage = calculateVoltage(adcMv, range);
    return setResult(voltage, adcMv, range, true, false);
}
MeasurementResult Voltmeter::measureRange(VoltageRange range) {
    if (!m_initialized || range == VoltageRange::RANGE_AUTO) {
        return measure();  // Fallback to auto-ranging
    }
    
    uint8_t pin = getRangePin(range);
    float adcMv = readAdcMillivolts(pin);
    bool saturated = isSaturated(adcMv);
    float voltage = calculateVoltage(adcMv, range);
    
    return setResult(voltage, adcMv, range, !saturated, saturated);
}

const MeasurementResult& Voltmeter::getLastMeasurement() const {
    return m_lastResult;
}

const char* Voltmeter::getRangeString(VoltageRange range) {
    switch (range) {
        case VoltageRange::RANGE_60V:  return "60V";
        case VoltageRange::RANGE_20V:  return "20V";
        case VoltageRange::RANGE_10V:  return "10V";
        case VoltageRange::RANGE_5V:   return "5V";
        case VoltageRange::RANGE_2V5:  return "2.5V";
        case VoltageRange::RANGE_AUTO: return "AUTO";
        default:                       return "INVALID";
    }
}

float Voltmeter::getRangeMaxVoltage(VoltageRange range) {
    static constexpr float MAX_VOLTAGES[] = { 60.0f, 20.0f, 10.0f, 5.0f, 2.5f };
    uint8_t idx = static_cast<uint8_t>(range);
    return (idx < 5) ? MAX_VOLTAGES[idx] : 0.0f;
}

void Voltmeter::setOversampleCount(uint16_t count) {
    if (count < 1) count = 1;
    if (count > 256) count = 256;
    m_oversampleCount = count;
}

uint16_t Voltmeter::getOversampleCount() const {
    return m_oversampleCount;
}

bool Voltmeter::isInitialized() const {
    return m_initialized;
}

// ============================================================================
// PRIVATE METHODS
// ============================================================================

float Voltmeter::readAdcMillivolts(uint8_t pin) {
    // Oversampling loop for noise reduction
    // Using analogReadMilliVolts() which applies factory calibration
    uint32_t sum = 0;
    
    for (uint16_t i = 0; i < m_oversampleCount; i++) {
        sum += analogReadMilliVolts(pin);
    }
    
    // Return averaged value
    return static_cast<float>(sum) / static_cast<float>(m_oversampleCount);
}

bool Voltmeter::isSaturated(float millivolts) {
    // Reading is saturated if it's above the threshold
    // This indicates the actual voltage exceeds the range's maximum
    return millivolts >= ADC_SATURATION_MV;
}

float Voltmeter::calculateVoltage(float millivolts, VoltageRange range) {
    // Convert millivolts to volts using multiplication (faster than division)
    float adcVolts = millivolts * MV_TO_V;
    
    // Get multiplier from lookup table
    float multiplier = getRangeMultiplier(range);
    
    // Get runtime calibration values
    uint8_t idx = static_cast<uint8_t>(range);
    float calOffset = (idx < 5) ? m_calOffsets[idx] : 0.0f;
    float calGain = (idx < 5) ? m_calGains[idx] : 1.0f;
    
    // Calculate actual input voltage:
    // Vin = (Vadc × Multiplier × CalGain) + CalOffset
    float voltage = (adcVolts * multiplier * calGain) + calOffset;
    
    // Clamp to non-negative (no negative DC voltage expected)
    return (voltage > 0.0f) ? voltage : 0.0f;
}

void Voltmeter::setCalibration(VoltageRange range, float offset, float gain) {
    uint8_t idx = static_cast<uint8_t>(range);
    if (idx < 5) {
        m_calOffsets[idx] = offset;
        m_calGains[idx] = gain;
    }
}

float Voltmeter::getCalibrationOffset(VoltageRange range) const {
    uint8_t idx = static_cast<uint8_t>(range);
    return (idx < 5) ? m_calOffsets[idx] : 0.0f;
}

float Voltmeter::getCalibrationGain(VoltageRange range) const {
    uint8_t idx = static_cast<uint8_t>(range);
    return (idx < 5) ? m_calGains[idx] : 1.0f;
}

uint8_t Voltmeter::getRangePin(VoltageRange range) {
    static constexpr uint8_t PINS[] = {
        PIN_RANGE_60V, PIN_RANGE_20V, PIN_RANGE_10V, PIN_RANGE_5V, PIN_RANGE_2V5
    };
    uint8_t idx = static_cast<uint8_t>(range);
    return (idx < 5) ? PINS[idx] : PIN_RANGE_60V;
}

float Voltmeter::getRangeMultiplier(VoltageRange range) {
    static constexpr float MULTIPLIERS[] = {
        MULTIPLIER_60V, MULTIPLIER_20V, MULTIPLIER_10V, MULTIPLIER_5V, MULTIPLIER_2V5
    };
    uint8_t idx = static_cast<uint8_t>(range);
    return (idx < 5) ? MULTIPLIERS[idx] : 1.0f;
}
