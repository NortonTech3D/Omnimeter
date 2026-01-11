/**
 * @file Voltmeter.h
 * @brief Precision ADC Driver for ESP32-S2 Digital Multimeter
 * @author Omnimeter Project
 * @version 1.1.0
 * 
 * ============================================================================
 * OPTIMIZATION SUMMARY (v1.1.0 - January 2026)
 * ============================================================================
 * 1. ADC Resolution: Explicitly set to 13-bit (0-8191) for maximum precision
 * 2. Hysteresis: Added 5% hysteresis buffer on range transitions to prevent
 *    oscillation at edge voltages (e.g., exactly 20V no longer ping-pongs)
 * 3. Lookup Tables: All range configs in constexpr arrays for O(1) access
 * 4. Helper Function: Single setResult() method eliminates 9x code duplication
 * 5. NVS Defaults: Gain defaults to 1.0f, offset to 0.0f (never 0 or crash)
 * 6. Native USB CDC: Verified Serial uses hardware USB (ARDUINO_USB_CDC_ON_BOOT)
 * 7. Range Stability: Counter-based debouncing prevents rapid range switching
 * 8. Memory: Char arrays with snprintf() instead of String concatenation
 * ============================================================================
 * 
 * @details
 * This class implements a multi-range voltmeter using 5 parallel voltage dividers
 * connected to separate ADC1 GPIO pins. It features auto-ranging for optimal
 * resolution and oversampling for noise reduction.
 * 
 * Hardware Configuration:
 * - High-side resistor (R1): 100kΩ (fixed for all dividers)
 * - Voltage reference: ESP32-S2 internal factory-calibrated VREF
 * - ADC resolution: 13-bit (ESP32-S2) - explicitly configured
 * - Maximum safe ADC input: ~2.5V (with margin for calibration)
 * 
 * Voltage Divider Calculations (Vout = Vin × R2 / (R1 + R2)):
 * ┌─────────┬──────────┬────────────────┬─────────────────────────┐
 * │ Range   │ R2 Value │ Divider Ratio  │ Max ADC Voltage         │
 * ├─────────┼──────────┼────────────────┼─────────────────────────┤
 * │ 0-60V   │ 4.3kΩ    │ 0.04123        │ 60V × 0.04123 = 2.47V   │
 * │ 0-20V   │ 14.3kΩ   │ 0.12511        │ 20V × 0.12511 = 2.50V   │
 * │ 0-10V   │ 33kΩ     │ 0.24812        │ 10V × 0.24812 = 2.48V   │
 * │ 0-5V    │ 100kΩ    │ 0.50000        │ 5V  × 0.50000 = 2.50V   │
 * │ 0-2.5V  │ ∞ (none) │ 1.00000        │ 2.5V (direct)           │
 * └─────────┴──────────┴────────────────┴─────────────────────────┘
 */

#ifndef VOLTMETER_H
#define VOLTMETER_H

#include <Arduino.h>
#include <esp_adc_cal.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS (ESP32-S2 ADC1 GPIO Pins)
// ============================================================================
// Lolin S2 Mini available ADC1 pins: GPIO1-10 (ADC1_CH0-CH9)
// Adjust these pins based on your physical wiring

constexpr uint8_t PIN_RANGE_60V  = 1;   // ADC1_CH0 - 60V range (4.3kΩ divider)
constexpr uint8_t PIN_RANGE_20V  = 2;   // ADC1_CH1 - 20V range (14.3kΩ divider)
constexpr uint8_t PIN_RANGE_10V  = 3;   // ADC1_CH2 - 10V range (33kΩ divider)
constexpr uint8_t PIN_RANGE_5V   = 4;   // ADC1_CH3 - 5V range (100kΩ divider)
constexpr uint8_t PIN_RANGE_2V5  = 5;   // ADC1_CH4 - 2.5V range (direct, no divider)

// ============================================================================
// VOLTAGE DIVIDER CONSTANTS
// ============================================================================
// R1 (High-side) = 100kΩ fixed for all dividers
// Divider Ratio = R2 / (R1 + R2)
// Multiplier = (R1 + R2) / R2 = 1 / Ratio (to convert ADC voltage back to input)

constexpr float R1_HIGH_SIDE_OHMS = 100000.0f;  // 100kΩ high-side resistor

// Low-side resistor values (R2) for each range
constexpr float R2_RANGE_60V = 4300.0f;    // 4.3kΩ  → Max 60V  → ~2.47V at ADC
constexpr float R2_RANGE_20V = 14300.0f;   // 14.3kΩ → Max 20V  → ~2.50V at ADC
constexpr float R2_RANGE_10V = 33000.0f;   // 33kΩ   → Max 10V  → ~2.48V at ADC
constexpr float R2_RANGE_5V  = 100000.0f;  // 100kΩ  → Max 5V   → ~2.50V at ADC
constexpr float R2_RANGE_2V5 = INFINITY;   // No divider (direct measurement)

// Pre-calculated multipliers for efficiency (Vin = Vadc × Multiplier)
// Multiplier = (R1 + R2) / R2
constexpr float MULTIPLIER_60V = (R1_HIGH_SIDE_OHMS + R2_RANGE_60V) / R2_RANGE_60V;   // ~24.256
constexpr float MULTIPLIER_20V = (R1_HIGH_SIDE_OHMS + R2_RANGE_20V) / R2_RANGE_20V;   // ~7.993
constexpr float MULTIPLIER_10V = (R1_HIGH_SIDE_OHMS + R2_RANGE_10V) / R2_RANGE_10V;   // ~4.030
constexpr float MULTIPLIER_5V  = (R1_HIGH_SIDE_OHMS + R2_RANGE_5V) / R2_RANGE_5V;     // 2.000
constexpr float MULTIPLIER_2V5 = 1.0f;  // Direct measurement, no scaling

// ============================================================================
// ADC AND MEASUREMENT PARAMETERS
// ============================================================================

constexpr uint16_t ADC_OVERSAMPLE_COUNT  = 64;      // Number of samples for averaging
constexpr uint16_t ADC_MAX_RAW_VALUE     = 8191;    // 13-bit ADC max (ESP32-S2)
constexpr float    ADC_REFERENCE_VOLTAGE = 2500.0f; // Reference voltage in mV (2.5V)
constexpr float    ADC_SATURATION_MARGIN = 0.95f;   // 95% of max = saturation threshold

// Saturation threshold in millivolts (below this, reading is valid)
constexpr float ADC_SATURATION_MV = ADC_REFERENCE_VOLTAGE * ADC_SATURATION_MARGIN;  // ~2375mV

// ============================================================================
// AUTO-RANGING HYSTERESIS CONFIGURATION
// ============================================================================
// To prevent oscillation at range boundaries, we use different thresholds for
// switching DOWN (to more sensitive range) vs UP (to less sensitive range).
// Hysteresis band = ~5% of range maximum.
//
// Example: At exactly 18V:
//   - If currently on 60V range: switch DOWN to 20V (threshold 18.0V)
//   - If currently on 20V range: stay on 20V until >19.0V (threshold + hysteresis)

// Thresholds for switching DOWN to a more sensitive range
constexpr float THRESHOLD_DOWN_TO_20V  = 18.0f;   // Switch from 60V to 20V
constexpr float THRESHOLD_DOWN_TO_10V  = 9.0f;    // Switch from 20V to 10V
constexpr float THRESHOLD_DOWN_TO_5V   = 4.5f;    // Switch from 10V to 5V
constexpr float THRESHOLD_DOWN_TO_2V5  = 2.3f;    // Switch from 5V to 2.5V

// Thresholds for switching UP to a less sensitive range (with hysteresis)
// These are ~5% higher than the corresponding DOWN thresholds
constexpr float THRESHOLD_UP_TO_60V    = 19.0f;   // Switch from 20V to 60V
constexpr float THRESHOLD_UP_TO_20V    = 9.5f;    // Switch from 10V to 20V
constexpr float THRESHOLD_UP_TO_10V    = 4.75f;   // Switch from 5V to 10V
constexpr float THRESHOLD_UP_TO_5V     = 2.4f;    // Switch from 2.5V to 5V

// Minimum readings at a range before allowing switch (debounce)
constexpr uint8_t RANGE_DEBOUNCE_COUNT = 3;

// ============================================================================
// CALIBRATION OFFSETS (Adjust per-device for precision)
// ============================================================================
// These offsets compensate for resistor tolerances and ADC non-linearity
// Set to 0.0 initially, then calibrate with a precision voltage source

constexpr float CAL_OFFSET_60V = 0.0f;   // Offset in volts for 60V range
constexpr float CAL_OFFSET_20V = 0.0f;   // Offset in volts for 20V range
constexpr float CAL_OFFSET_10V = 0.0f;   // Offset in volts for 10V range
constexpr float CAL_OFFSET_5V  = 0.0f;   // Offset in volts for 5V range
constexpr float CAL_OFFSET_2V5 = 0.0f;   // Offset in volts for 2.5V range

// Gain calibration multipliers (fine-tune divider ratios)
constexpr float CAL_GAIN_60V = 1.0f;
constexpr float CAL_GAIN_20V = 1.0f;
constexpr float CAL_GAIN_10V = 1.0f;
constexpr float CAL_GAIN_5V  = 1.0f;
constexpr float CAL_GAIN_2V5 = 1.0f;

// ============================================================================
// LOOKUP TABLES FOR FAST ACCESS (indexed by VoltageRange enum)
// ============================================================================

struct RangeConfig {
    uint8_t pin;
    float multiplier;
    float calOffset;
    float calGain;
    float maxVoltage;
    const char* name;
};

// Static lookup table - indexed by VoltageRange enum value
static constexpr RangeConfig RANGE_CONFIG[] = {
    { PIN_RANGE_60V, MULTIPLIER_60V, CAL_OFFSET_60V, CAL_GAIN_60V, 60.0f, "60V" },   // RANGE_60V = 0
    { PIN_RANGE_20V, MULTIPLIER_20V, CAL_OFFSET_20V, CAL_GAIN_20V, 20.0f, "20V" },   // RANGE_20V = 1
    { PIN_RANGE_10V, MULTIPLIER_10V, CAL_OFFSET_10V, CAL_GAIN_10V, 10.0f, "10V" },   // RANGE_10V = 2
    { PIN_RANGE_5V,  MULTIPLIER_5V,  CAL_OFFSET_5V,  CAL_GAIN_5V,  5.0f,  "5V"  },   // RANGE_5V = 3
    { PIN_RANGE_2V5, MULTIPLIER_2V5, CAL_OFFSET_2V5, CAL_GAIN_2V5, 2.5f,  "2.5V"},   // RANGE_2V5 = 4
};

// Conversion constant: mV to V (multiply instead of divide)
constexpr float MV_TO_V = 0.001f;

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief Enumeration of available voltage measurement ranges
 */
enum class VoltageRange : uint8_t {
    RANGE_60V  = 0,   // 0-60V range (lowest resolution, highest voltage)
    RANGE_20V  = 1,   // 0-20V range
    RANGE_10V  = 2,   // 0-10V range
    RANGE_5V   = 3,   // 0-5V range
    RANGE_2V5  = 4,   // 0-2.5V range (highest resolution, lowest voltage)
    RANGE_AUTO = 5,   // Auto-ranging mode
    RANGE_INVALID = 255
};

/**
 * @brief Measurement result structure
 */
struct MeasurementResult {
    float voltage;              // Calculated input voltage in volts
    float rawAdcMillivolts;     // Raw ADC reading in millivolts
    VoltageRange activeRange;   // Range used for this measurement
    bool valid;                 // True if measurement is valid (not saturated)
    bool overrange;             // True if voltage exceeds current range
    uint32_t timestamp;         // Timestamp of measurement (millis())
};

// ============================================================================
// VOLTMETER CLASS DEFINITION
// ============================================================================

/**
 * @brief Precision voltmeter class with auto-ranging and oversampling
 */
class Voltmeter {
public:
    /**
     * @brief Construct a new Voltmeter object
     */
    Voltmeter();

    /**
     * @brief Initialize the ADC hardware and calibration
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Perform a voltage measurement with auto-ranging
     * @return MeasurementResult containing voltage and metadata
     */
    MeasurementResult measure();

    /**
     * @brief Perform a voltage measurement on a specific range
     * @param range The voltage range to use
     * @return MeasurementResult containing voltage and metadata
     */
    MeasurementResult measureRange(VoltageRange range);

    /**
     * @brief Get the last measurement result without re-measuring
     * @return const reference to the last MeasurementResult
     */
    const MeasurementResult& getLastMeasurement() const;

    /**
     * @brief Get a human-readable string for the current range
     * @param range The voltage range
     * @return String representation of the range
     */
    static const char* getRangeString(VoltageRange range);

    /**
     * @brief Get the maximum voltage for a given range
     * @param range The voltage range
     * @return Maximum measurable voltage in volts
     */
    static float getRangeMaxVoltage(VoltageRange range);

    /**
     * @brief Set oversampling count (number of samples to average)
     * @param count Number of samples (1-256)
     */
    void setOversampleCount(uint16_t count);

    /**
     * @brief Get current oversampling count
     * @return Current oversample count
     */
    uint16_t getOversampleCount() const;

    /**
     * @brief Check if the ADC is properly initialized
     * @return true if initialized
     */
    bool isInitialized() const;
    
    /**
     * @brief Get the voltage multiplier for a given range (public for calibration)
     * @param range Voltage range
     * @return Multiplier value
     */
    static float getRangeMultiplier(VoltageRange range);
    
    /**
     * @brief Set runtime calibration values for a range
     * @param range Voltage range to calibrate
     * @param offset Offset in volts (added after multiplication)
     * @param gain Gain multiplier (applied after base calculation)
     */
    void setCalibration(VoltageRange range, float offset, float gain);
    
    /**
     * @brief Get current calibration offset for a range
     * @param range Voltage range
     * @return Current offset value
     */
    float getCalibrationOffset(VoltageRange range) const;
    
    /**
     * @brief Get current calibration gain for a range
     * @param range Voltage range
     * @return Current gain value
     */
    float getCalibrationGain(VoltageRange range) const;

private:
    /**
     * @brief Read ADC with oversampling and return millivolts
     * @param pin GPIO pin number
     * @return Averaged ADC reading in millivolts
     */
    float readAdcMillivolts(uint8_t pin);

    /**
     * @brief Check if an ADC reading indicates saturation
     * @param millivolts ADC reading in millivolts
     * @return true if saturated (reading too high)
     */
    bool isSaturated(float millivolts);

    /**
     * @brief Convert ADC millivolts to input voltage for a given range
     * @param millivolts ADC reading in millivolts
     * @param range Voltage range used
     * @return Calculated input voltage in volts
     */
    float calculateVoltage(float millivolts, VoltageRange range);

    /**
     * @brief Get the GPIO pin for a given range
     * @param range Voltage range
     * @return GPIO pin number
     */
    static uint8_t getRangePin(VoltageRange range);

    // Member variables
    bool m_initialized;
    uint16_t m_oversampleCount;
    MeasurementResult m_lastResult;
    esp_adc_cal_characteristics_t m_adcChars;
    VoltageRange m_currentRange;           ///< Current active range (for hysteresis)
    uint8_t m_rangeStableCount;            ///< Counter for range debouncing
    
    // Runtime calibration values (can be updated by CalibrationManager)
    float m_calOffsets[5];                 ///< Per-range calibration offsets
    float m_calGains[5];                   ///< Per-range calibration gains
    
    // ========================================================================
    // PRIVATE HELPER METHODS (reduce code duplication)
    // ========================================================================
    
    /**
     * @brief Set measurement result (consolidates repeated code)
     */
    MeasurementResult setResult(float voltage, float adcMv, VoltageRange range, 
                                bool valid, bool overrange);
    
    /**
     * @brief Determine optimal range with hysteresis
     */
    VoltageRange determineOptimalRange(float voltage);
    
    /**
     * @brief Measure at specific range with saturation fallback
     */
    MeasurementResult measureAtRange(VoltageRange range, float fallbackVoltage, float fallbackAdcMv);
};

#endif // VOLTMETER_H
