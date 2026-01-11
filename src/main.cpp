/**
 * @file main.cpp
 * @brief Main Application Entry Point for ESP32-S2 Precision Digital Multimeter
 * @author Omnimeter Project
 * @version 1.1.0
 * 
 * ============================================================================
 * OPTIMIZATION SUMMARY (v1.1.0 - January 2026)
 * ============================================================================
 * 
 * 1. ADC RESOLUTION (ESP32-S2 Specific):
 *    - Explicitly configured to 13-bit (8192 levels) via analogReadResolution(13)
 *    - Provides maximum precision for voltage measurements
 * 
 * 2. AUTO-RANGING HYSTERESIS:
 *    - Separate UP and DOWN thresholds with ~5% hysteresis band
 *    - Range debouncing (3 stable readings before switching)
 *    - Prevents oscillation at boundary voltages (e.g., exactly 20V)
 * 
 * 3. NVS FIRST-BOOT SAFETY:
 *    - Calibration defaults to gain=1.0, offset=0.0 if no saved data
 *    - Calibration automatically applied to Voltmeter on begin()
 *    - Sanity clamping prevents corrupted data from causing errors
 * 
 * 4. INPUT SANITIZATION:
 *    - parseVoltage() validates character-by-character
 *    - Rejects non-numeric characters, multiple decimals, NaN, Inf
 *    - Web API uses same sanitization via calibration commands
 * 
 * 5. NATIVE USB CDC (ESP32-S2):
 *    - ARDUINO_USB_CDC_ON_BOOT=1 enables hardware USB serial
 *    - No UART converter needed, direct USB connection
 * 
 * 6. ASYNC WEB SERVER:
 *    - ESPAsyncWebServer handles requests in background
 *    - Does not block ADC sampling loop
 * 
 * 7. CODE REFACTORING:
 *    - setResult() helper eliminates 9x duplicated result-building
 *    - Static char buffers in snprintf prevent heap fragmentation
 *    - Lookup tables for O(1) range configuration access
 * 
 * 8. MEMORY OPTIMIZATION:
 *    - Static object allocation (no 'new' in setup)
 *    - snprintf with pre-allocated buffers instead of String concat
 *    - Minimized heap allocations in main loop
 * 
 * ============================================================================
 * 
 * @details
 * This is the main entry point for the Omnimeter firmware. It initializes
 * all subsystems and runs the main non-blocking loop.
 * 
 * Architecture:
 * - Non-blocking design using millis() timing
 * - Modular class-based structure
 * - Dual output: Serial (USB CDC) and Web Interface
 * 
 * Hardware: ESP32-S2FN4R2 (Lolin S2 Mini)
 * Framework: Arduino / PlatformIO
 */

#include <Arduino.h>
#include "Voltmeter.h"
#include "WebInterface.h"
#include "Calibration.h"

// ============================================================================
// TIMING CONFIGURATION (Non-blocking intervals)
// ============================================================================

constexpr uint32_t SERIAL_UPDATE_INTERVAL_MS = 250;   // Serial output every 250ms
constexpr uint32_t ADC_SAMPLE_INTERVAL_MS = 100;      // ADC sampling every 100ms
constexpr uint32_t STATUS_LED_INTERVAL_MS = 1000;     // Status LED toggle every 1s
constexpr uint32_t STARTUP_DELAY_MS = 1000;           // Startup stabilization delay

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

constexpr uint8_t PIN_STATUS_LED = 15;  // Lolin S2 Mini built-in LED (active LOW)

// ============================================================================
// GLOBAL OBJECTS - Static allocation (avoids heap fragmentation)
// ============================================================================

static Voltmeter voltmeterInstance;
static WebInterface* webInterface = nullptr;  // Needs voltmeter reference, init in setup
static CalibrationManager* calibration = nullptr;
Voltmeter* voltmeter = &voltmeterInstance;

// Serial command buffer
static String serialBuffer;

// ============================================================================
// TIMING VARIABLES (for non-blocking loop)
// ============================================================================

uint32_t lastSerialUpdate = 0;
uint32_t lastAdcSample = 0;
uint32_t lastLedToggle = 0;
bool ledState = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeHardware();
void initializeSubsystems();
void updateStatusLED();
void outputSerialData(const MeasurementResult& result);
void processSerialCommands();
void printStartupBanner();
void printDivider();

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
    // LED first - gives visual feedback immediately
    pinMode(PIN_STATUS_LED, OUTPUT);
    
    // Blink LED 3 times quickly to show we're alive (even before Serial)
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_STATUS_LED, LOW);   // LED ON
        delay(200);
        digitalWrite(PIN_STATUS_LED, HIGH);  // LED OFF
        delay(200);
    }
    
    // Initialize serial communication (USB CDC on ESP32-S2)
    Serial.begin(115200);
    
    // Fixed delay for USB CDC enumeration - no conditional check
    delay(3000);
    
    Serial.println();
    Serial.println(F("====================================="));
    Serial.println(F("  OMNIMETER BOOT - Stage 1: Serial OK"));
    Serial.println(F("====================================="));
    Serial.flush();
    
    // Blink to show Serial init passed
    for (int i = 0; i < 2; i++) {
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(100);
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(100);
    }
    
    Serial.println(F("Stage 2: Initializing objects..."));
    
    // voltmeter is already statically allocated
    // Create WebInterface with reference to voltmeter
    static WebInterface webInterfaceInstance(voltmeterInstance);
    webInterface = &webInterfaceInstance;
    
    // Create Calibration Manager
    static CalibrationManager calibrationInstance(voltmeterInstance);
    calibration = &calibrationInstance;
    
    Serial.println(F("Stage 3: Objects ready"));
    
    // Startup delay for power stabilization
    delay(STARTUP_DELAY_MS);
    
    printStartupBanner();
    
    // Initialize hardware peripherals
    initializeHardware();
    
    // Initialize subsystems
    initializeSubsystems();
    
    printDivider();
    Serial.println(F("System initialization complete. Starting main loop..."));
    printDivider();
    Serial.println();
}

// ============================================================================
// LOOP - Main non-blocking execution loop
// ============================================================================

void loop() {
    uint32_t currentTime = millis();
    
    // ========================================================================
    // Task 1: ADC Sampling (every 100ms)
    // ========================================================================
    if (currentTime - lastAdcSample >= ADC_SAMPLE_INTERVAL_MS) {
        lastAdcSample = currentTime;
        
        // Perform auto-ranging measurement
        voltmeter->measure();
    }
    
    // ========================================================================
    // Task 2: Serial Output (every 250ms)
    // ========================================================================
    if (currentTime - lastSerialUpdate >= SERIAL_UPDATE_INTERVAL_MS) {
        lastSerialUpdate = currentTime;
        
        // Get latest measurement and output to serial
        const MeasurementResult& result = voltmeter->getLastMeasurement();
        outputSerialData(result);
    }
    
    // ========================================================================
    // Task 3: Status LED Toggle (every 1000ms)
    // ========================================================================
    if (currentTime - lastLedToggle >= STATUS_LED_INTERVAL_MS) {
        lastLedToggle = currentTime;
        updateStatusLED();
    }
    
    // ========================================================================
    // Task 4: Web Interface Processing
    // ========================================================================
    webInterface->loop();
    
    // ========================================================================
    // Task 5: Serial Command Processing
    // ========================================================================
    processSerialCommands();
    
    // ========================================================================
    // Yield to system tasks (WiFi, etc.)
    // ========================================================================
    yield();
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Initialize hardware peripherals
 */
void initializeHardware() {
    Serial.println(F("[Main] Initializing hardware..."));
    
    // Configure status LED
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);  // LED off (active LOW on Lolin S2 Mini)
    
    Serial.println(F("[Main] Hardware initialization complete"));
}

/**
 * @brief Initialize software subsystems
 */
void initializeSubsystems() {
    Serial.println(F("[Main] Initializing subsystems..."));
    printDivider();
    
    // Initialize Voltmeter (ADC)
    Serial.println(F("[Main] Starting Voltmeter..."));
    if (!voltmeter->begin()) {
        Serial.println(F("[Main] ERROR: Voltmeter initialization failed!"));
    }
    Serial.println(F("[Main] Voltmeter started"));
    
    printDivider();
    
    // Initialize Web Interface (WiFi + Web Server)
    Serial.println(F("[Main] Starting Web Interface..."));
    
    // Choose mode based on configuration
    WiFiOperatingMode wifiMode = WiFiOperatingMode::MODE_AP;
    
    #if defined(WIFI_STA_ENABLED) && WIFI_STA_ENABLED
        wifiMode = WiFiOperatingMode::MODE_AP_STA;
    #endif
    
    if (!webInterface->begin(wifiMode)) {
        Serial.println(F("[Main] WARNING: Web Interface initialization issues"));
    }
    
    Serial.println(F("[Main] Web Interface started"));
    printDivider();
    
    // Initialize Calibration Manager
    Serial.println(F("[Main] Loading calibration..."));
    calibration->begin();
    calibration->printCalibration();
    
    Serial.println(F("[Main] Subsystem initialization complete"));
}

/**
 * @brief Toggle status LED to indicate system activity (non-blocking)
 */
void updateStatusLED() {
    static uint8_t errorBlinkPhase = 0;
    
    // Check system health and adjust LED behavior
    const MeasurementResult& result = voltmeter->getLastMeasurement();
    
    if (!result.valid || result.overrange) {
        // Fast blink pattern for error (toggle every call = 2Hz blink)
        errorBlinkPhase++;
        digitalWrite(PIN_STATUS_LED, (errorBlinkPhase & 1) ? LOW : HIGH);
    } else {
        // Normal heartbeat (toggle every call = 0.5Hz)
        ledState = !ledState;
        digitalWrite(PIN_STATUS_LED, ledState ? LOW : HIGH);
        errorBlinkPhase = 0;
    }
}

/**
 * @brief Output measurement data to serial console (optimized single write)
 * @param result Measurement result to display
 */
void outputSerialData(const MeasurementResult& result) {
    // Pre-allocated buffer for entire line (reduces multiple Serial.print overhead)
    static char outputBuffer[80];
    
    const char* status = result.valid ? "OK" : (result.overrange ? "OVR" : "ERR");
    const char* rangeStr = Voltmeter::getRangeString(result.activeRange);
    
    if (result.overrange) {
        snprintf(outputBuffer, sizeof(outputBuffer),
                 "V: OVERRANGE! | Range: %s | ADC: %7.1f mV | %s",
                 rangeStr, result.rawAdcMillivolts, status);
    } else if (!result.valid) {
        snprintf(outputBuffer, sizeof(outputBuffer),
                 "V: INVALID    | Range: %s | ADC: %7.1f mV | %s",
                 rangeStr, result.rawAdcMillivolts, status);
    } else {
        // Adaptive precision based on voltage magnitude
        const char* fmt = (result.voltage < 10.0f) 
            ? "V: %7.4f V | Range: %s | ADC: %7.1f mV | %s"
            : "V: %7.3f V | Range: %s | ADC: %7.1f mV | %s";
        snprintf(outputBuffer, sizeof(outputBuffer), fmt,
                 result.voltage, rangeStr, result.rawAdcMillivolts, status);
    }
    
    Serial.println(outputBuffer);
}

/**
 * @brief Print startup banner to serial
 */
void printStartupBanner() {
    Serial.println();
    printDivider();
    Serial.println(F("    ____  __  __ _   _ _____ __  __ _____ _____ _____ ____  "));
    Serial.println(F("   / __ \\|  \\/  | \\ | |_   _|  \\/  | ____|_   _| ____|  _ \\ "));
    Serial.println(F("  | |  | | |\\/| |  \\| | | | | |\\/| |  _|   | | |  _| | |_) |"));
    Serial.println(F("  | |__| | |  | | |\\  | | | | |  | | |___  | | | |___|  _ < "));
    Serial.println(F("   \\____/|_|  |_|_| \\_|_|_| |_|  |_|_____| |_| |_____|_| \\_\\"));
    Serial.println();
    Serial.println(F("  ESP32-S2 Precision Digital Multimeter v1.0.0"));
    Serial.println(F("  Hardware: Lolin S2 Mini (ESP32-S2FN4R2)"));
    printDivider();
    Serial.println();
    
    // Print chip info
    Serial.print(F("  Chip: "));
    Serial.println(ESP.getChipModel());
    Serial.print(F("  CPU Freq: "));
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(F(" MHz"));
    Serial.print(F("  Flash: "));
    Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
    Serial.println(F(" MB"));
    Serial.print(F("  Free Heap: "));
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(F(" KB"));
    Serial.println();
}

/**
 * @brief Print a divider line to serial
 */
void printDivider() {
    Serial.println(F("================================================================"));
}

/**
 * @brief Process incoming serial commands (non-blocking)
 */
void processSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                // Echo command
                Serial.print(F("> "));
                Serial.println(serialBuffer);
                
                // Try calibration commands first
                if (!calibration->processCommand(serialBuffer)) {
                    // Handle other commands
                    String cmd = serialBuffer;
                    cmd.trim();
                    cmd.toUpperCase();
                    
                    if (cmd == "HELP" || cmd == "?") {
                        Serial.println(F("\n=== OMNIMETER COMMANDS ==="));
                        Serial.println(F("HELP, ?     - Show this help"));
                        Serial.println(F("STATUS      - Show system status"));
                        Serial.println(F("INFO        - Show device info"));
                        Serial.println(F("CAL:HELP    - Calibration commands"));
                        Serial.println();
                    }
                    else if (cmd == "STATUS") {
                        Serial.println(F("\n=== SYSTEM STATUS ==="));
                        Serial.print(F("Uptime: "));
                        Serial.print(millis() / 1000);
                        Serial.println(F(" seconds"));
                        Serial.print(F("Free Heap: "));
                        Serial.print(ESP.getFreeHeap());
                        Serial.println(F(" bytes"));
                        Serial.print(F("WiFi AP Clients: "));
                        Serial.println(WiFi.softAPgetStationNum());
                        const MeasurementResult& r = voltmeter->getLastMeasurement();
                        Serial.print(F("Last Voltage: "));
                        Serial.print(r.voltage, 4);
                        Serial.print(F(" V ("));
                        Serial.print(Voltmeter::getRangeString(r.activeRange));
                        Serial.println(F(")"));
                        Serial.println();
                    }
                    else if (cmd == "INFO") {
                        printStartupBanner();
                    }
                    else if (cmd.length() > 0) {
                        Serial.println(F("Unknown command. Type HELP for commands."));
                    }
                }
                
                serialBuffer = "";
            }
        }
        else if (c >= 32 && c < 127) {  // Printable ASCII
            if (serialBuffer.length() < 64) {
                serialBuffer += c;
            }
        }
    }
}
