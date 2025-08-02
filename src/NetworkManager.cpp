#include "NetworkManager.h"
#include "SerialConfig.h"

// Static instance pointer for callbacks
NetworkManager* NetworkManager::instance = nullptr;

// NetworkManager implementation
NetworkManager::NetworkManager(const NetworkConfig& networkConfig, 
                               M5StickCPlusDisplayManager& displayMgr)
    : config(networkConfig)
    , displayManager(displayMgr)
    , ssid_not_connected("-")
    , ssid_connected(ssid_not_connected)
    , otaActive(false)
    , restartForGoodOTAScheduled(false)
    , restartAfterGoodOTAUpdateAt(0)
    , haltAllProcessingDuringOTAUpload(false)
    , asyncWebServer(nullptr)
    , elegantOTA(nullptr)
{
    // Set static instance for callbacks
    instance = this;

    // Initialize buffers
    strcpy(IPBuffer, no_wifi_label);
    strcpy(IPLocalGateway, "");
    strcpy(WiFiSSID, "");

    // MBJ - THIS NEEDS TO BE DEFERRED TO WHEN OTA IS ENABLED
    asyncWebServer = new AsyncWebServer(80);
    elegantOTA = new AsyncElegantOtaClass();
}

NetworkManager::~NetworkManager() {
    delete asyncWebServer;
    delete elegantOTA;
        
    if (instance == this) {
        instance = nullptr;
    }
}

void NetworkManager::begin() {
    // Register WiFi event handlers
    WiFi.onEvent(wifiStationConnectedWrapper, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(wifiGotIPWrapper, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(wifiLostIPWrapper, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_LOST_IP);
    WiFi.onEvent(wifiStationDisconnectedWrapper, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
}

void NetworkManager::loop() {
    // Handle OTA restart if scheduled
    if (restartForGoodOTAScheduled && millis() >= restartAfterGoodOTAUpdateAt) {
        ESP.restart();
    }
    
    // Skip processing during OTA upload
    if (haltAllProcessingDuringOTAUpload) {
        return;
    }
}

void NetworkManager::toggleWiFiActive(bool wait)
{
  displayManager.display.fillScreen(TFT_ORANGE);
  displayManager.display.setCursor(10, 10);
  displayManager.display.setTextSize(3);
  displayManager.display.setTextColor(TFT_WHITE, TFT_BLUE);

//  if (ESPNowActive)  MBJMBJMBJ
//    toggleESPNowActive();

  if (WiFi.status() == WL_CONNECTED)
  {
    if (otaActive)
    {
      asyncWebServer->end();
      displayManager.display.println("OTA Disabled");
      otaActive = false;
    }

    WiFi.disconnect();
    ssid_connected = ssid_not_connected;
    displayManager.display.setCursor(0, 10);
    displayManager.display.printf("Wifi Disabled");
  }
  else
  {
    displayManager.display.printf("Wifi Connecting");

    const bool wifiOnly = true;
    const int scanAttempts = 3;
    connectToWiFiAndInitOTA(wifiOnly,scanAttempts);

    displayManager.display.fillScreen(TFT_ORANGE);
    displayManager.display.setCursor(10, 10);
    displayManager.display.setTextSize(3);
    displayManager.display.setRotation(1);
    displayManager.display.setTextColor(TFT_WHITE, TFT_BLUE);

    displayManager.display.printf(WiFi.status() == WL_CONNECTED ? "Wifi Enabled" : "No Connect");
  }

  if (wait)
    delay(2000);

  displayManager.display.fillScreen(TFT_BLACK);
}


void NetworkManager::toggleOTAActive()
{
  displayManager.display.fillScreen(TFT_ORANGE);
  displayManager.display.setCursor(10, 10);
  displayManager.display.setTextSize(3);
  displayManager.display.setTextColor(TFT_WHITE, TFT_BLUE);
  displayManager.display.setRotation(1);

  if (otaActive)
  {
    asyncWebServer->end();
    displayManager.display.println("OTA Disabled");
    otaActive = false;
    delay (500);
  }
  else
  {
    bool wifiToggled = false;
    if (WiFi.status() != WL_CONNECTED)
    {
      toggleWiFiActive(false);  // don't wait
      wifiToggled = true;

      displayManager.display.fillScreen(TFT_ORANGE);
      displayManager.display.setCursor(10, 10);
      displayManager.display.setTextColor(TFT_WHITE, TFT_BLUE);
    }
    
    if (WiFi.status() == WL_CONNECTED)
    {
      asyncWebServer->on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/plain", "To upload firmware use /update");
      });

      elegantOTA->setID(config.otaDeviceLabel);
      elegantOTA->setUploadBeginCallback(uploadOTABeginCallback);
      elegantOTA->begin(asyncWebServer);    // Start AsyncElegantOTA
      asyncWebServer->begin();

      if (wifiToggled)
      {
        // Clear the QR Code from new wifi connection
        displayManager.display.fillScreen(TFT_ORANGE);
        displayManager.display.setCursor(10, 10);
        displayManager.display.setTextSize(3);
        displayManager.display.setRotation(1);
        displayManager.display.setTextColor(TFT_WHITE, TFT_BLUE);
        
        displayManager.display.printf(" OTA & WiFi\n   Enabled");
      }
      else
      {
        displayManager.display.printf("OTA Enabled");
      }
              
      otaActive = true;
    }
    else
    {
      displayManager.display.println("Error: Enable Wifi First");
    }
    
    delay (500);
  }
  
  if (otaActive)
  {
    writeLogToSerial = false;
    disableFeaturesForOTA(); 
    haltAllProcessingDuringOTAUpload = true;
  }

  showOTARecoveryScreen();
}


const char* NetworkManager::scanForKnownNetworkAsync() {
    const char* network = nullptr;

    // Append scanning status to scrolling line
//    displayManager.updateScrollingStatusLine("Scanning");
    displayManager.display.println("Scan WiFi\nSSIDs...");
    
    // Start progress animation for scanning
    // displayManager.startProgressAnimation();

    // Clear any previous scan results first
    WiFi.scanDelete();
    
    // Ensure WiFi is in station mode and ready for scanning
    USB_SERIAL_PRINTF("WiFi status before scan: %d\n", WiFi.status());
    WiFi.mode(WIFI_STA);
    delay(100);  // Give more time for mode change
    USB_SERIAL_PRINTF("WiFi status after mode set: %d\n", WiFi.status());
    
    // Check if there's an ongoing scan and wait for it to complete
    int16_t preScanCheck = WiFi.scanComplete();
    USB_SERIAL_PRINTF("Pre-scan check: %d\n", preScanCheck);
    if (preScanCheck == -1) {
        USB_SERIAL_PRINTF("Previous scan still running, waiting for completion...\n");
        int waitCount = 0;
        while (WiFi.scanComplete() == -1 && waitCount < 50) {  // Wait up to 5 seconds
            delay(100);
            waitCount++;
        }
        USB_SERIAL_PRINTF("Previous scan wait completed after %d iterations\n", waitCount);
        WiFi.scanDelete();  // Clean up
    }
    
    // Start async WiFi scan
    int16_t startResult = WiFi.scanNetworks(true, false);  // async=true, show_hidden=false
    
    USB_SERIAL_PRINTF("Async WiFi scan start result: %d\n", startResult);
    
    // Give a moment for scan to actually start
    delay(50);
    
    // Check if scan actually started
    int16_t initialCheck = WiFi.scanComplete();
    USB_SERIAL_PRINTF("Initial scan status check: %d\n", initialCheck);
    
    if (initialCheck == -2) {
  //      displayManager.stopProgressAnimation();
    //    displayManager.updateScrollingStatusLine("Scan failed to start");
        return nullptr;
    }
    
    // Poll for scan completion while showing progress
    int16_t scanResults = -1;  // -1 means scan is running
    int timeoutCount = 0;
    const int maxTimeout = 100; // 10 seconds max (100 * 100ms)
    
    USB_SERIAL_PRINTF("Starting scan polling loop (max %d iterations)\n", maxTimeout);
    
    while (timeoutCount < maxTimeout) {
        scanResults = WiFi.scanComplete();
        
        if (scanResults == -1) {
            // Scan still running
            if (timeoutCount % 10 == 0) {  // Log every 1 second
                USB_SERIAL_PRINTF("Scan still running... (iteration %d/%d)\n", timeoutCount, maxTimeout);
            }
//            displayManager.updateProgressAnimation();  // Show scanning progress
            delay(100);  // Check every 100ms
            timeoutCount++;
        } else if (scanResults == -2) {
            // No scan was started
            USB_SERIAL_PRINTF("ERROR: scanComplete() returned -2 (no scan) at iteration %d\n", timeoutCount);
  //          displayManager.stopProgressAnimation();
    //        displayManager.updateScrollingStatusLine("No scan started");
            return nullptr;
        } else {
            // Scan completed (scanResults >= 0)
            USB_SERIAL_PRINTF("Scan completed! Found %d networks after %d iterations\n", scanResults, timeoutCount);
            break;
        }
    }
    
    // Stop progress animation
//    displayManager.stopProgressAnimation();
    
    // Handle timeout
    if (timeoutCount >= maxTimeout) {
        USB_SERIAL_PRINTF("TIMEOUT: Scan timed out after %d iterations (%d seconds)\n", timeoutCount, timeoutCount/10);
  //      displayManager.updateScrollingStatusLine("Scan timeout");
        WiFi.scanDelete();
        return nullptr;
    }
    
    if (scanResults > 0) {
        USB_SERIAL_PRINTF("Processing %d scan results:\n", scanResults);
        for (int i = 0; i < scanResults; ++i) {
            // Print SSID and RSSI for each device found
            String SSID = WiFi.SSID(i);
            int32_t RSSI = WiFi.RSSI(i);
            USB_SERIAL_PRINTF("  [%d] %s (RSSI: %d)\n", i, SSID.c_str(), RSSI);

            // Check if the current device matches known networks
            if (strcmp(SSID.c_str(), config.ssid_1) == 0) {
                network = config.ssid_1;
                USB_SERIAL_PRINTF("  -> MATCH: Found known network %s\n", config.ssid_1);
            } else if (strcmp(SSID.c_str(), config.ssid_2) == 0) {
                network = config.ssid_2;
                USB_SERIAL_PRINTF("  -> MATCH: Found known network %s\n", config.ssid_2);
            } else if (strcmp(SSID.c_str(), config.ssid_3) == 0) {
                network = config.ssid_3;
                USB_SERIAL_PRINTF("  -> MATCH: Found known network %s\n", config.ssid_3);
            }

            if (network)
                break;
        }    
    } else {
        USB_SERIAL_PRINTF("No networks found in scan (scanResults = %d)\n", scanResults);
    }

    if (network) {
        // Append found network to scrolling line
    //    displayManager.updateScrollingStatusLine("Found " + String(network));
    } else {
        // Append not found to scrolling line
      //  displayManager.updateScrollingStatusLine("No known networks found");
    }

    // Clean up scan results
    WiFi.scanDelete();
    return network;
}

const char* NetworkManager::scanForKnownNetwork() {
    const char* network = nullptr;

    // Append scanning status to scrolling line
    //displayManager.updateScrollingStatusLine("Scanning");

    // Perform sync WiFi scan (fallback method)
    int8_t scanResults = WiFi.scanNetworks();
    
    if (scanResults > 0) {
        for (int i = 0; i < scanResults; ++i) {
            // Print SSID and RSSI for each device found
            String SSID = WiFi.SSID(i);

            // Check if the current device starts with the peerSSIDPrefix
            if (strcmp(SSID.c_str(), config.ssid_1) == 0)
                network = config.ssid_1;
            else if (strcmp(SSID.c_str(), config.ssid_2) == 0)
                network = config.ssid_2;
            else if (strcmp(SSID.c_str(), config.ssid_3) == 0)
                network = config.ssid_3;

            if (network)
                break;
        }    
    }

    if (network) {
        // Append found network to scrolling line
        //displayManager.updateScrollingStatusLine("Found " + String(network));
    } else {
        // Append not found to scrolling line
        //displayManager.updateScrollingStatusLine("No known networks found");
    }

    // clean up ram
    WiFi.scanDelete();

    return network;
}

bool NetworkManager::connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts) {
    USB_SERIAL_PRINTF("=== connectToWiFiAndInitOTA ENTRY: wifiOnly=%i repeatScanAttempts=%i WiFiStatus=%i otaActive=%i ===\n", wifiOnly, repeatScanAttempts, WiFi.status(), otaActive);
    
    if (wifiOnly && WiFi.status() == WL_CONNECTED)
        return true;

    // Initialize scrolling status line
//    displayManager.updateScrollingStatusLine("WiFi: Searching", false);

    while (repeatScanAttempts-- &&
           (WiFi.status() != WL_CONNECTED ||
            WiFi.status() == WL_CONNECTED && wifiOnly == false && otaActive == false)) {
        const char* network = scanForKnownNetworkAsync();
    
        if (!network) {
            // Append retry status to scrolling line
  //          displayManager.updateScrollingStatusLine("No networks found, retrying");
            
            delay(1000);
            continue;
        }
        
        int connectToFoundNetworkAttempts = 3;
        const int repeatDelay = 1000;
        
        // Append connecting status to scrolling line
    //    displayManager.updateScrollingStatusLine("Connecting to " + String(network), true, true);
        
        // Start progress animation
      //  displayManager.startProgressAnimation();
    
        if (strcmp(network, config.ssid_1) == 0) {
            while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(config.ssid_1, config.password_1, config.label_1, config.timeout_1, wifiOnly))
                delay(repeatDelay);
        } else if (strcmp(network, config.ssid_2) == 0) {
            while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(config.ssid_2, config.password_2, config.label_2, config.timeout_2, wifiOnly))
                delay(repeatDelay);
        } else if (strcmp(network, config.ssid_3) == 0) {
            while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(config.ssid_3, config.password_3, config.label_3, config.timeout_3, wifiOnly))
                delay(repeatDelay);
        }
        
        // Stop progress animation after connection attempts
        //displayManager.stopProgressAnimation();
    }

    bool connected = WiFi.status() == WL_CONNECTED;
    
    if (connected) {
        ssid_connected = WiFi.SSID();
//        displayManager.updateScrollingStatusLine("SUCCESS! Connected to " + ssid_connected, true, true);
        // Quietly add to display array for later scrolling, without refreshing screen
  //      displayManager.addDisplayLine("SUCCESS! Connected to " + ssid_connected, false, true);
    } else {
        ssid_connected = ssid_not_connected;
    //    displayManager.updateScrollingStatusLine("FAILED! Connection unsuccessful", true, true);
        // Quietly add to display array for later scrolling, without refreshing screen
      //  displayManager.addDisplayLine("FAILED! Connection unsuccessful", false, true);
    }
    
    return connected;
}

bool NetworkManager::setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly) {
    USB_SERIAL_PRINTF("=== setupOTAWebServer ENTRY: ssid=%s wifiOnly=%i otaActive=%i WiFiStatus=%i ===\n", _ssid, wifiOnly, otaActive, WiFi.status());
    
    if (wifiOnly && WiFi.status() == WL_CONNECTED) {
        USB_SERIAL_PRINTF("setupOTAWebServer: attempt to connect wifiOnly, already connected - otaActive=%i\n",otaActive);
        return true;
    }

    USB_SERIAL_PRINTF("setupOTAWebServer: attempt to connect %s wifiOnly=%i when otaActive=%i\n",_ssid, wifiOnly,otaActive);

    bool forcedCancellation = false;
    bool connected = false;
    WiFi.mode(WIFI_STA);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(config.device_hostname);

    WiFi.begin(_ssid, _password);
    
#ifdef ENABLE_TELEGRAM_BOT_AT_COMPILE_TIME
    if (secured_client && config.enableTelegram)
        secured_client->setCACert(TELEGRAM_CERTIFICATE_ROOT);
#endif

    // Wait for connection for max of timeout/1000 seconds
    const int progressStep = 100;
    int count = timeout / progressStep;
    while (WiFi.status() != WL_CONNECTED && --count > 0) {

        // Update progress animation
        displayManager.updateProgressAnimation();

        delay(progressStep);
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (wifiOnly == false && !otaActive) {
            USB_SERIAL_PRINTLN("setupOTAWebServer: WiFi connected ok, starting up OTA");  
            USB_SERIAL_PRINTLN("setupOTAWebServer: calling asyncWebServer->on");

            setupWebServerRoutes();
            
            USB_SERIAL_PRINTLN("setupOTAWebServer: calling elegantOTA->begin");

            elegantOTA->setID(config.otaDeviceLabel);
            elegantOTA->setUploadBeginCallback(uploadOTABeginCallbackWrapper);
            elegantOTA->setUploadProgressCallback(uploadOTAProgressCallbackWrapper);
            elegantOTA->setUploadSucceededCallback(uploadOTASucceededCallbackWrapper);
            elegantOTA->begin(asyncWebServer);

            USB_SERIAL_PRINTLN("setupOTAWebServer: calling asyncWebServer->begin");

            asyncWebServer->begin();
            USB_SERIAL_PRINTLN("setupOTAWebServer: OTA setup complete");
            otaActive = true;
            delay(2000);
            connected = true;
            }
    }

    return connected;
}

void NetworkManager::setupWebServerRoutes() {
    asyncWebServer->on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/plain", "To upload firmware use /update");
    });
}

void NetworkManager::uploadOTABeginCallback() {
    // Enable OTA mode to suppress all normal display updates
//    displayManager.setOTAMode(true);
    
    // Clear display first
//    displayManager.clearDisplay();
    
    // Set large bold font and draw centered text
//    displayManager.display.setFont(u8g2_font_ncenB14_tr);
    
    const char* line1 = "OTA Update";
    const char* line2 = "In Progress";
    
//    int line1Width = displayManager.display.getUTF8Width(line1);
//    int line2Width = displayManager.display.getUTF8Width(line2);
    
//    int x1 = (256 - line1Width) / 2;
//    int x2 = (256 - line2Width) / 2;
    
//    displayManager.display.setDrawColor(1);  // White (draw)
//    displayManager.display.drawUTF8(x1, 20, line1);  // First line at y=20
//    displayManager.display.drawUTF8(x2, 38, line2);  // Second line at y=38 (more spacing)
    
    // Draw progress bar bounding box (moved down to y=50)
//    int barWidth = 180;
//    int barHeight = 8;
//    int barX = (256 - barWidth) / 2;
//    int barY = 50;
//    displayManager.display.drawFrame(barX, barY, barWidth, barHeight);
    
//    displayManager.display.sendBuffer();
    
//    prepareNetworkForOTA();
}

void NetworkManager::uploadOTAProgressCallback(size_t progress, size_t total) {
    static int lastFillWidth = -1;
    
    // Skip if no total size available
    if (total == 0) {
        USB_SERIAL_PRINTF("OTA Progress: skipping, total=0\n");
        return;
    }
    
    USB_SERIAL_PRINTF("OTA Progress: %zu/%zu bytes\n", progress, total);

    /*
    // Progress bar dimensions - moved down to y=50, taller for better visibility
    int barWidth = 180;
    int barHeight = 8;
    int barX = (256 - barWidth) / 2;
    int barY = 50;
    
    // Calculate fill width based on actual progress ratio
    int fillWidth = ((barWidth - 2) * progress) / total;
    
    // Only update if we have at least 1 more pixel of progress bar to show
    if (fillWidth <= lastFillWidth) {
        return;
    }
    
    USB_SERIAL_PRINTF("OTA Progress: fillWidth: %d->%d\n", lastFillWidth, fillWidth);
    
    // Only clear and redraw the progress bar area, not entire screen  
    displayManager.display.setDrawColor(0);  // Black (erase)
    displayManager.display.drawBox(barX + 1, barY + 1, barWidth - 2, barHeight - 2);  // Clear progress area only
    
    displayManager.display.setDrawColor(1);  // White (draw)
    
    // Fill progress bar based on actual progress
    if (fillWidth > 0) {
        displayManager.display.drawBox(barX + 1, barY + 1, fillWidth, barHeight - 2);
    }
    
    // Use updateDisplayArea to only refresh the progress bar area
    int tileX = barX / 8;
    int tileY = barY / 8;  
    int tileWidth = (barWidth / 8) + 2;
    int tileHeight = 2;  // Cover progress bar only
    
    displayManager.display.updateDisplayArea(tileX, tileY, tileWidth, tileHeight);
    
    lastFillWidth = fillWidth;
    */
}

void NetworkManager::uploadOTASucceededCallback() {
    USB_SERIAL_PRINTF("OTA upload succeeded, scheduling restart in 3 seconds\n");
/*    
    // Show 100% completion - use same dimensions as progress callback
    int barWidth = 180;
    int barHeight = 8;
    int barX = (256 - barWidth) / 2;
    int barY = 50;
    
    // Fill progress bar to 100%
    displayManager.display.setDrawColor(0);  // Black (erase)
    displayManager.display.drawBox(barX + 1, barY + 1, barWidth - 2, barHeight - 2);  // Clear
    displayManager.display.setDrawColor(1);  // White (draw)
    displayManager.display.drawBox(barX + 1, barY + 1, barWidth - 2, barHeight - 2);  // Fill 100%
    
    // Update display
    int tileX = barX / 8;
    int tileY = barY / 8;  
    int tileWidth = (barWidth / 8) + 2;
    int tileHeight = 2;
    displayManager.display.updateDisplayArea(tileX, tileY, tileWidth, tileHeight);
  */  
    restartAfterGoodOTAUpdateAt = millis() + 3000;
    restartForGoodOTAScheduled = true;
}

// WiFi event handlers
void NetworkManager::wifiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    strcpy(IPBuffer, wait_ip_label);
}

void NetworkManager::wifiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    strcpy(IPBuffer, WiFi.localIP().toString().c_str());
    strcpy(IPLocalGateway, WiFi.gatewayIP().toString().c_str());
    strcpy(WiFiSSID, WiFi.SSID().c_str());
}

void NetworkManager::wifiLostIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    strcpy(IPBuffer, lost_ip_label);
    strcpy(IPLocalGateway, "");
    strcpy(WiFiSSID, WiFi.SSID().c_str());
}

void NetworkManager::wifiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    strcpy(IPBuffer, no_wifi_label);
    strcpy(IPLocalGateway, "");
    strcpy(WiFiSSID, "");
}

// Static callback wrappers
void NetworkManager::wifiStationConnectedWrapper(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (instance) instance->wifiStationConnected(event, info);
}

void NetworkManager::wifiGotIPWrapper(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (instance) instance->wifiGotIP(event, info);
}

void NetworkManager::wifiLostIPWrapper(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (instance) instance->wifiLostIP(event, info);
}

void NetworkManager::wifiStationDisconnectedWrapper(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (instance) instance->wifiStationDisconnected(event, info);
}

// OTA callback wrappers
void NetworkManager::uploadOTABeginCallbackWrapper(AsyncElegantOtaClass* originator) {
    if (instance) instance->uploadOTABeginCallback();
}

void NetworkManager::uploadOTAProgressCallbackWrapper(AsyncElegantOtaClass* originator, size_t progress, size_t total) {
    if (instance) instance->uploadOTAProgressCallback(progress, total);
}

void NetworkManager::uploadOTASucceededCallbackWrapper(AsyncElegantOtaClass* originator) {
    if (instance) instance->uploadOTASucceededCallback();
}