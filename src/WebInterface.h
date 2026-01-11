/**
 * @file WebInterface.h
 * @brief Dual-Mode WiFi Network Interface and Web Server for ESP32-S2 Multimeter
 * @author Omnimeter Project
 * @version 1.0.0
 * 
 * @details
 * This class manages WiFi connectivity (AP and Station modes) and serves
 * a web interface for remote voltage monitoring and configuration.
 * 
 * Features:
 * - Access Point mode for standalone operation
 * - Station mode for integration with existing networks
 * - Async web server for non-blocking operation
 * - REST API for voltage data (JSON format)
 * - Embedded HTML interface with AJAX updates
 */

#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include "Voltmeter.h"

// Include secrets if available (defines WIFI_STA_SSID, WIFI_STA_PASSWORD)
#if __has_include("secrets.h")
    #include "secrets.h"
#endif

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================

// Access Point (AP) Mode Configuration
// Use build-time macros if defined, otherwise use defaults
#ifndef WIFI_AP_SSID
    #define WIFI_AP_SSID "ESP_Multimeter"
#endif
#ifndef WIFI_AP_PASSWORD
    #define WIFI_AP_PASSWORD "12345678"
#endif

constexpr const char* AP_SSID = WIFI_AP_SSID;
constexpr const char* AP_PASSWORD = WIFI_AP_PASSWORD;  // Min 8 characters, empty for open network
constexpr uint8_t AP_CHANNEL = 1;
constexpr bool AP_HIDDEN = false;
constexpr uint8_t AP_MAX_CONNECTIONS = 4;

// Station (STA) Mode Configuration
// Set WIFI_STA_ENABLED to true and configure credentials to connect to existing network
#ifndef WIFI_STA_SSID
    #define WIFI_STA_SSID "YourNetworkSSID"
#endif
#ifndef WIFI_STA_PASSWORD
    #define WIFI_STA_PASSWORD "YourNetworkPassword"
#endif

constexpr bool WIFI_STA_ENABLED = false;
constexpr const char* STA_SSID = WIFI_STA_SSID;
constexpr const char* STA_PASSWORD = WIFI_STA_PASSWORD;
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 15000;  // 15 second timeout

// mDNS Configuration
constexpr const char* MDNS_HOSTNAME = "omnimeter";

// OTA Configuration
constexpr const char* OTA_PASSWORD = "omnimeter";
constexpr uint16_t OTA_PORT = 3232;

// Web Server Configuration
constexpr uint16_t WEB_SERVER_PORT = 80;
constexpr uint32_t WEB_UPDATE_INTERVAL_MS = 50;  // Default AJAX update interval

// Static IP Configuration for AP Mode
constexpr uint8_t AP_IP_ADDR[4] = {192, 168, 4, 1};
constexpr uint8_t AP_GATEWAY[4] = {192, 168, 4, 1};
constexpr uint8_t AP_SUBNET[4] = {255, 255, 255, 0};

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief WiFi operating mode
 */
enum class WiFiOperatingMode : uint8_t {
    MODE_AP = 0,        // Access Point mode
    MODE_STATION = 1,   // Station mode (connected to router)
    MODE_AP_STA = 2,    // Both AP and Station active
    MODE_DISABLED = 3   // WiFi disabled
};

/**
 * @brief Network connection status
 */
enum class NetworkStatus : uint8_t {
    STATUS_DISCONNECTED = 0,
    STATUS_CONNECTING = 1,
    STATUS_CONNECTED = 2,
    STATUS_AP_ACTIVE = 3,
    STATUS_ERROR = 4
};

// ============================================================================
// WEB INTERFACE CLASS DEFINITION
// ============================================================================

/**
 * @brief Web interface and WiFi manager for the multimeter
 */
class WebInterface {
public:
    /**
     * @brief Construct a new WebInterface object
     * @param voltmeter Reference to the Voltmeter instance for data access
     */
    explicit WebInterface(Voltmeter& voltmeter);

    /**
     * @brief Destructor
     */
    ~WebInterface();

    /**
     * @brief Initialize WiFi and start web server
     * @param mode Operating mode (AP, Station, or both)
     * @return true if initialization successful
     */
    bool begin(WiFiOperatingMode mode = WiFiOperatingMode::MODE_AP);

    /**
     * @brief Process network tasks (call in main loop)
     * @note This is non-blocking and should be called frequently
     */
    void loop();

    /**
     * @brief Get current network status
     * @return NetworkStatus enum value
     */
    NetworkStatus getStatus() const;

    /**
     * @brief Get current operating mode
     * @return WiFiOperatingMode enum value
     */
    WiFiOperatingMode getMode() const;

    /**
     * @brief Get IP address as string
     * @return IP address string
     */
    String getIPAddress() const;

    /**
     * @brief Get number of connected clients (AP mode)
     * @return Number of connected WiFi clients
     */
    uint8_t getConnectedClients() const;

    /**
     * @brief Check if web server is running
     * @return true if server is active
     */
    bool isServerRunning() const;

    /**
     * @brief Set custom AJAX update interval
     * @param intervalMs Update interval in milliseconds
     */
    void setUpdateInterval(uint32_t intervalMs);

    /**
     * @brief Connect to a different WiFi network (Station mode)
     * @param ssid Network SSID
     * @param password Network password
     * @return true if connection initiated
     */
    bool connectToNetwork(const char* ssid, const char* password);

    /**
     * @brief Disconnect from current network
     */
    void disconnect();

    /**
     * @brief Get RSSI (signal strength) when in Station mode
     * @return RSSI in dBm, or 0 if not connected
     */
    int8_t getRSSI() const;

private:
    /**
     * @brief Initialize Access Point mode
     * @return true if successful
     */
    bool initAccessPoint();

    /**
     * @brief Initialize Station mode
     * @return true if connection successful
     */
    bool initStation();

    /**
     * @brief Initialize mDNS responder
     * @return true if successful
     */
    bool initMDNS();

    /**
     * @brief Initialize OTA update handler
     */
    void initOTA();

    /**
     * @brief Configure and start the async web server
     */
    void setupWebServer();

    /**
     * @brief Generate the main HTML page content
     * @return HTML string
     */
    String generateHTML();

    /**
     * @brief Generate JSON response for voltage data
     * @return JSON string
     */
    String generateJSON();

    /**
     * @brief Handle root page request
     */
    void handleRoot(AsyncWebServerRequest* request);

    /**
     * @brief Handle API data request
     */
    void handleAPI(AsyncWebServerRequest* request);

    /**
     * @brief Handle not found requests
     */
    void handleNotFound(AsyncWebServerRequest* request);

    // Member variables
    Voltmeter& m_voltmeter;
    AsyncWebServer* m_server;
    WiFiOperatingMode m_mode;
    NetworkStatus m_status;
    uint32_t m_updateIntervalMs;
    bool m_serverRunning;
    uint32_t m_lastStatusCheck;
    String m_cachedHTML;
};

#endif // WEB_INTERFACE_H
