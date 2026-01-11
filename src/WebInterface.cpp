/**
 * @file WebInterface.cpp
 * @brief Implementation of the Web Interface and WiFi Manager
 * @author Omnimeter Project
 * @version 1.1.0
 * 
 * ============================================================================
 * OPTIMIZATION CHANGES (v1.1.0):
 * - Replaced String concatenation in generateJSON() with snprintf char arrays
 * - Pre-allocated static JSON buffer to prevent heap fragmentation
 * - AsyncWebServer already non-blocking (verified)
 * ============================================================================
 */

#include "WebInterface.h"

// ============================================================================
// FALLBACK HTML (used if LittleFS file not found)
// ============================================================================

static const char FALLBACK_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>Omnimeter</title></head>
<body style="font-family:sans-serif;text-align:center;padding:50px;">
<h1>Omnimeter</h1>
<p>LittleFS not mounted or index.html missing.</p>
<p>Upload filesystem with: <code>pio run --target uploadfs</code></p>
<p><a href="/api/voltage">View JSON API</a></p>
</body>
</html>
)rawliteral";

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

WebInterface::WebInterface(Voltmeter& voltmeter)
    : m_voltmeter(voltmeter)
    , m_server(nullptr)
    , m_mode(WiFiOperatingMode::MODE_DISABLED)
    , m_status(NetworkStatus::STATUS_DISCONNECTED)
    , m_updateIntervalMs(WEB_UPDATE_INTERVAL_MS)
    , m_serverRunning(false)
    , m_lastStatusCheck(0)
{
}

WebInterface::~WebInterface() {
    if (m_server) {
        m_server->end();
        delete m_server;
        m_server = nullptr;
    }
    WiFi.disconnect(true);
}

// ============================================================================
// PUBLIC METHODS
// ============================================================================

bool WebInterface::begin(WiFiOperatingMode mode) {
    m_mode = mode;
    bool success = false;
    
    // Set WiFi mode based on configuration
    switch (mode) {
        case WiFiOperatingMode::MODE_AP:
            WiFi.mode(WIFI_AP);
            success = initAccessPoint();
            break;
            
        case WiFiOperatingMode::MODE_STATION:
            WiFi.mode(WIFI_STA);
            success = initStation();
            // Fall back to AP mode if station connection fails
            if (!success) {
                Serial.println(F("[WebInterface] Station mode failed, falling back to AP mode"));
                WiFi.mode(WIFI_AP);
                success = initAccessPoint();
                m_mode = WiFiOperatingMode::MODE_AP;
            }
            break;
            
        case WiFiOperatingMode::MODE_AP_STA:
            WiFi.mode(WIFI_AP_STA);
            initAccessPoint();
            success = initStation();
            if (!success) {
                // AP is still running even if STA fails
                success = true;
            }
            break;
            
        default:
            WiFi.mode(WIFI_OFF);
            return false;
    }
    
    if (success) {
        initMDNS();
        initOTA();
        setupWebServer();
    }
    
    return success;
}

void WebInterface::loop() {
    // Non-blocking status monitoring
    uint32_t now = millis();
    
    if (now - m_lastStatusCheck >= 5000) {  // Check every 5 seconds
        m_lastStatusCheck = now;
        
        // Update status based on current WiFi state
        if (m_mode == WiFiOperatingMode::MODE_STATION || 
            m_mode == WiFiOperatingMode::MODE_AP_STA) {
            if (WiFi.status() == WL_CONNECTED) {
                m_status = NetworkStatus::STATUS_CONNECTED;
            } else if (WiFi.status() == WL_DISCONNECTED) {
                m_status = NetworkStatus::STATUS_DISCONNECTED;
            }
        }
        
        // Log connection info periodically (debug)
        #if CORE_DEBUG_LEVEL > 0
        if (m_mode == WiFiOperatingMode::MODE_AP || 
            m_mode == WiFiOperatingMode::MODE_AP_STA) {
            Serial.printf("[WebInterface] AP Clients: %d\n", WiFi.softAPgetStationNum());
        }
        if (m_mode == WiFiOperatingMode::MODE_STATION && 
            WiFi.status() == WL_CONNECTED) {
            Serial.printf("[WebInterface] RSSI: %d dBm\n", WiFi.RSSI());
        }
        #endif
    }
    
    // AsyncWebServer handles requests automatically, no polling needed
}

NetworkStatus WebInterface::getStatus() const {
    return m_status;
}

WiFiOperatingMode WebInterface::getMode() const {
    return m_mode;
}

String WebInterface::getIPAddress() const {
    if (m_mode == WiFiOperatingMode::MODE_AP) {
        return WiFi.softAPIP().toString();
    } else if (m_mode == WiFiOperatingMode::MODE_STATION && 
               WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP().toString();
    } else if (m_mode == WiFiOperatingMode::MODE_AP_STA) {
        if (WiFi.status() == WL_CONNECTED) {
            return WiFi.localIP().toString();
        }
        return WiFi.softAPIP().toString();
    }
    return "0.0.0.0";
}

uint8_t WebInterface::getConnectedClients() const {
    return WiFi.softAPgetStationNum();
}

bool WebInterface::isServerRunning() const {
    return m_serverRunning;
}

void WebInterface::setUpdateInterval(uint32_t intervalMs) {
    m_updateIntervalMs = intervalMs;
}

bool WebInterface::connectToNetwork(const char* ssid, const char* password) {
    Serial.printf("[WebInterface] Connecting to: %s\n", ssid);
    
    WiFi.begin(ssid, password);
    
    uint32_t startTime = millis();
    m_status = NetworkStatus::STATUS_CONNECTING;
    
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > STA_CONNECT_TIMEOUT_MS) {
            Serial.println(F("[WebInterface] Connection timeout"));
            m_status = NetworkStatus::STATUS_ERROR;
            return false;
        }
        delay(100);  // Small delay during connection only
        Serial.print(".");
    }
    
    Serial.println();
    Serial.printf("[WebInterface] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    m_status = NetworkStatus::STATUS_CONNECTED;
    return true;
}

void WebInterface::disconnect() {
    WiFi.disconnect();
    m_status = NetworkStatus::STATUS_DISCONNECTED;
}

int8_t WebInterface::getRSSI() const {
    if (WiFi.status() == WL_CONNECTED) {
        return WiFi.RSSI();
    }
    return 0;
}

// ============================================================================
// PRIVATE METHODS
// ============================================================================

bool WebInterface::initAccessPoint() {
    Serial.println(F("[WebInterface] Starting Access Point..."));
    
    // Configure static IP for AP
    IPAddress apIP(AP_IP_ADDR[0], AP_IP_ADDR[1], AP_IP_ADDR[2], AP_IP_ADDR[3]);
    IPAddress gateway(AP_GATEWAY[0], AP_GATEWAY[1], AP_GATEWAY[2], AP_GATEWAY[3]);
    IPAddress subnet(AP_SUBNET[0], AP_SUBNET[1], AP_SUBNET[2], AP_SUBNET[3]);
    
    WiFi.softAPConfig(apIP, gateway, subnet);
    
    // Start AP with or without password
    bool success;
    if (strlen(AP_PASSWORD) >= 8) {
        success = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, AP_HIDDEN, AP_MAX_CONNECTIONS);
    } else {
        success = WiFi.softAP(AP_SSID, nullptr, AP_CHANNEL, AP_HIDDEN, AP_MAX_CONNECTIONS);
    }
    
    if (success) {
        Serial.println(F("[WebInterface] Access Point started successfully"));
        Serial.printf("[WebInterface] SSID: %s\n", AP_SSID);
        Serial.printf("[WebInterface] IP Address: %s\n", WiFi.softAPIP().toString().c_str());
        m_status = NetworkStatus::STATUS_AP_ACTIVE;
    } else {
        Serial.println(F("[WebInterface] Failed to start Access Point"));
        m_status = NetworkStatus::STATUS_ERROR;
    }
    
    return success;
}

bool WebInterface::initStation() {
    if (!WIFI_STA_ENABLED) {
        Serial.println(F("[WebInterface] Station mode disabled in configuration"));
        return false;
    }
    
    return connectToNetwork(STA_SSID, STA_PASSWORD);
}

bool WebInterface::initMDNS() {
    Serial.println(F("[WebInterface] Starting mDNS responder..."));
    
    if (!MDNS.begin(MDNS_HOSTNAME)) {
        Serial.println(F("[WebInterface] ERROR: mDNS responder failed to start"));
        return false;
    }
    
    // Add HTTP service to mDNS
    MDNS.addService("http", "tcp", WEB_SERVER_PORT);
    
    Serial.printf("[WebInterface] mDNS started: http://%s.local/\n", MDNS_HOSTNAME);
    return true;
}

void WebInterface::initOTA() {
    Serial.println(F("[WebInterface] Configuring OTA updates..."));
    
    ArduinoOTA.setHostname(MDNS_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.setPort(OTA_PORT);
    
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        Serial.printf("[OTA] Starting %s update...\n", type.c_str());
        // Unmount LittleFS during filesystem update
        if (ArduinoOTA.getCommand() == U_SPIFFS) {
            LittleFS.end();
        }
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println(F("\n[OTA] Update complete! Rebooting..."));
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static uint8_t lastPercent = 0;
        uint8_t percent = (progress * 100) / total;
        if (percent != lastPercent && percent % 10 == 0) {
            Serial.printf("[OTA] Progress: %u%%\n", percent);
            lastPercent = percent;
        }
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        switch (error) {
            case OTA_AUTH_ERROR:    Serial.println(F("Auth Failed")); break;
            case OTA_BEGIN_ERROR:   Serial.println(F("Begin Failed")); break;
            case OTA_CONNECT_ERROR: Serial.println(F("Connect Failed")); break;
            case OTA_RECEIVE_ERROR: Serial.println(F("Receive Failed")); break;
            case OTA_END_ERROR:     Serial.println(F("End Failed")); break;
            default:                Serial.println(F("Unknown")); break;
        }
    });
    
    ArduinoOTA.begin();
    Serial.printf("[WebInterface] OTA ready on port %d (password protected)\n", OTA_PORT);
}

void WebInterface::setupWebServer() {
    if (m_server) {
        delete m_server;
    }
    
    m_server = new AsyncWebServer(WEB_SERVER_PORT);
    
    // Route: Root page (/)
    m_server->on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleRoot(request);
    });
    
    // Route: API endpoint for voltage data
    m_server->on("/api/voltage", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleAPI(request);
    });
    
    // Route: Health check endpoint
    m_server->on("/health", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/plain", "OK");
    });
    
    // Route: Device info endpoint
    m_server->on("/api/info", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // Use static buffer to avoid heap fragmentation
        static char infoBuffer[200];
        snprintf(infoBuffer, sizeof(infoBuffer),
            "{\"device\":\"ESP32-S2 Multimeter\",\"version\":\"1.1.0\",\"ip\":\"%s\",\"uptime\":%lu,\"heap\":%u,\"rssi\":%d}",
            getIPAddress().c_str(),
            millis() / 1000,
            ESP.getFreeHeap(),
            getRSSI()
        );
        request->send(200, "application/json", infoBuffer);
    });
    
    // 404 handler
    m_server->onNotFound([this](AsyncWebServerRequest* request) {
        handleNotFound(request);
    });
    
    // Start server
    m_server->begin();
    m_serverRunning = true;
    
    Serial.println(F("[WebInterface] Web server started"));
    Serial.printf("[WebInterface] Access at: http://%s/\n", getIPAddress().c_str());
}

String WebInterface::generateHTML() {
    // Return the fallback HTML (used when LittleFS not available)
    return String(FPSTR(FALLBACK_HTML));
}

String WebInterface::generateJSON() {
    const MeasurementResult& result = m_voltmeter.getLastMeasurement();
    
    // Use static buffer to avoid heap fragmentation
    // Format: {"voltage":XX.XXXX,"adc_mv":XXXX.XX,"range":"XXXXX","valid":XXXXX,"overrange":XXXXX,"timestamp":XXXXXXXXXX}
    static char jsonBuffer[160];
    
    snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{\"voltage\":%.4f,\"adc_mv\":%.2f,\"range\":\"%s\",\"valid\":%s,\"overrange\":%s,\"timestamp\":%u}",
        result.voltage,
        result.rawAdcMillivolts,
        Voltmeter::getRangeString(result.activeRange),
        result.valid ? "true" : "false",
        result.overrange ? "true" : "false",
        (unsigned int)result.timestamp
    );
    
    return String(jsonBuffer);
}

void WebInterface::handleRoot(AsyncWebServerRequest* request) {
    // Try to serve from LittleFS first
    if (LittleFS.exists("/index.html")) {
        request->send(LittleFS, "/index.html", "text/html");
    } else {
        // Fallback to minimal HTML if file not found
        request->send(200, "text/html", FPSTR(FALLBACK_HTML));
    }
}

void WebInterface::handleAPI(AsyncWebServerRequest* request) {
    // Take a fresh measurement for the API response
    m_voltmeter.measure();
    
    String json = generateJSON();
    
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", json);
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
}

void WebInterface::handleNotFound(AsyncWebServerRequest* request) {
    String message = "404 Not Found\n\n";
    message += "URI: " + request->url() + "\n";
    message += "Method: " + String((request->method() == HTTP_GET) ? "GET" : "POST") + "\n";
    
    request->send(404, "text/plain", message);
}
