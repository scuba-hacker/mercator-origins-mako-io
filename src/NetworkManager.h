#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Update.h>
#include "M5StickCPlusDisplayManager.h"

// Network configuration structure
struct NetworkConfig {
    // WiFi networks
    const char* ssid_1;
    const char* password_1;
    const char* label_1;
    uint32_t timeout_1;
    
    const char* ssid_2;
    const char* password_2;
    const char* label_2;
    uint32_t timeout_2;
    
    const char* ssid_3;
    const char* password_3;
    const char* label_3;
    uint32_t timeout_3;
    
    // Network settings
    const char* device_hostname;
    
    // OTA device label
    const char* otaDeviceLabel;
};

// Main NetworkManager class
class NetworkManager {

private:
    // Configuration
    NetworkConfig config;
    M5StickCPlusDisplayManager& displayManager;
    
    // Network state
    WiFiClient wifiClient;
    String ssid_connected;
    String ssid_not_connected;
    char IPBuffer[16];
    char IPLocalGateway[16];
    char WiFiSSID[36];
    
    // Status labels
    const char* no_wifi_label;
    const char* wait_ip_label;
    const char* lost_ip_label;
    
    // OTA and web server
    bool otaActive;
    bool haltAllProcessingDuringOTAUpload;
    AsyncWebServer* asyncWebServer;
    AsyncElegantOtaClass* elegantOTA;
    
    // External dependencies (injected)
    std::function<void()> prepareEntireSystemForOTA;
    
    // Private helper methods
    void setupWebServerRoutes();
    
    // Static callback wrappers for WiFi events
    static void wifiStationConnectedWrapper(WiFiEvent_t event, WiFiEventInfo_t info);
    static void wifiGotIPWrapper(WiFiEvent_t event, WiFiEventInfo_t info);
    static void wifiLostIPWrapper(WiFiEvent_t event, WiFiEventInfo_t info);
    static void wifiStationDisconnectedWrapper(WiFiEvent_t event, WiFiEventInfo_t info);
    
    // Static callback wrappers for OTA
    static void uploadOTABeginCallbackWrapper(AsyncElegantOtaClass* originator);
    static void uploadOTAProgressCallbackWrapper(AsyncElegantOtaClass* originator, size_t progress, size_t total);
    static void uploadOTASucceededCallbackWrapper(AsyncElegantOtaClass* originator);
    
    // Static instance pointer for callbacks
    static NetworkManager* instance;
    
public:
    bool restartForGoodOTAScheduled;
    uint32_t restartAfterGoodOTAUpdateAt;

    // Constructor
    NetworkManager(const NetworkConfig& networkConfig, 
                   M5StickCPlusDisplayManager& displayMgr);
    
    // Destructor
    ~NetworkManager();
    
    // Initialization
    void begin();
    void loop();
    
    // External dependency injection
    void setPrepareEntireSystemForOTA(std::function<void()> callback) { prepareEntireSystemForOTA = callback; }
    
    // Main networking functions
    void toggleOTAActive();
    void toggleWiFiActive(bool wait);

    bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts);
    const char* scanForKnownNetworkAsync();
    const char* scanForKnownNetwork();
    bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly);
    
    // WiFi event handlers
    void wifiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info);
    void wifiGotIP(WiFiEvent_t event, WiFiEventInfo_t info);
    void wifiLostIP(WiFiEvent_t event, WiFiEventInfo_t info);
    void wifiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info);
        
    // OTA callbacks
    void uploadOTABeginCallback();
    void uploadOTAProgressCallback(size_t progress, size_t total);
    void uploadOTASucceededCallback();
    
    // Status getters
    bool isOTAActive() const { return otaActive; }
    bool isWiFiConnected() const { return WiFi.status() == WL_CONNECTED; }
    String getConnectedSSID() const { return ssid_connected; }
    String getLocalIP() const { return WiFi.localIP().toString(); }
    String getGatewayIP() const { return String(IPLocalGateway); }
    bool isHaltingForOTA() const { return haltAllProcessingDuringOTAUpload; }
    bool isRestartScheduled() const { return restartForGoodOTAScheduled; }
    uint32_t getRestartTime() const { return restartAfterGoodOTAUpdateAt; }
};