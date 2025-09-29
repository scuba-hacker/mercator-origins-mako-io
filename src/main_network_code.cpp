#ifdef BUILD_INCLUDE_MAIN_NETWORK_CODE

const char podTigerHostName[] = "Tiger";
const char testTigerHostName[] = "Tiger-Test";
const char *tigerHostName = podTigerHostName;
const char makoHostName[] = "Mako";
const char oceanicHostName[] = "Oceanic";
const char silkyHostName[] = "AudioPod";

// *************************** WiFi Persistence using Preferences ***************************

void saveLastSSID(const char* ssid) {
  persistedPreferences.putString("lastSSID", String(ssid));
  USB_SERIAL_PRINTF("Saved last SSID: %s\n", ssid);
}

String loadLastSSID() {
  String ssid = persistedPreferences.getString("lastSSID", "");
  if (ssid.length() > 0) {
    USB_SERIAL_PRINTF("Loaded last SSID: %s\n", ssid.c_str());
  } else {
    USB_SERIAL_PRINTLN("No saved SSID found");
  }
  return ssid;
}

const char* wifiStatusToString(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_DONE";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "FAILED";
    case WL_CONNECTION_LOST: return "LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

// *************************** Networking Functions ***************************

int8_t scanWiFiForSSIDs()
{
  return WiFi.scanNetworks(false,false,false,150U);
}

const char* scanForKnownNetwork() // return first known network found
{
  const char* network = nullptr;

  M5.Lcd.println("Scan WiFi\nSSIDs...");
  int8_t scanResults = scanWiFiForSSIDs();

  if (scanResults != 0)
  {
    for (int i = 0; i < scanResults; ++i) 
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      
      // Check if the current device starts with the peerSSIDPrefix
      if (strcmp(SSID.c_str(), ssid_1) == 0)
        network=ssid_1;
      else if (strcmp(SSID.c_str(), ssid_2) == 0)
        network=ssid_2;
      else if (strcmp(SSID.c_str(), ssid_3) == 0)
        network=ssid_3;

      if (network)
        break;
    }    
  }

  if (network)
  {
      M5.Lcd.printf("Found:\n%s",network);

      USB_SERIAL_PRINTF("Found:\n%s\n",network);
  }
  else
  {
    M5.Lcd.println("None\nFound");
      USB_SERIAL_PRINTLN("No networks Found\n");
  }

  // clean up ram
  WiFi.scanDelete();

  return network;
}

void uploadOTABeginCallback(AsyncElegantOtaClass* originator)
{
  haltAllProcessingDuringOTAUpload = true;   // prevent LCD call due to separate thread calling this
}

uint32_t otaProgressScreenUpdateAt = 0;
const uint32_t otaProgressUpdateDisplayDutyCycle = 200;

void uploadOTAProgressCallback(AsyncElegantOtaClass* originator, size_t progress, size_t total) 
{
  // Skip if no total size available
  if (total == 0) {
      USB_SERIAL_PRINTF("OTA Progress: skipping, total=0\n");
      return;
  }
  
  if (millis() > otaProgressScreenUpdateAt) 
  {
    OTAUploadFlashCurrentLEDPeriodicity = OTAUploadFlashInProgressLEDPeriodicity;
    otaProgressScreenUpdateAt = millis() + otaProgressUpdateDisplayDutyCycle;
    M5.Lcd.setCursor(3, 60);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("%7lu / %-7lu B", progress, total);
  }  
}

void uploadOTASucceededCallback(AsyncElegantOtaClass* originator)
{
    restartAfterGoodOTAUpdateAt = millis() + 3000;
    restartForGoodOTAScheduled = true;
}

bool connectToLastConnectedWifiNetwork(const bool wifiOnly)
{
  const uint32_t lastKnownNetworkTimeout = 5000;

  String lastSSID = loadLastSSID();
  if (lastSSID.length() > 0)
  {
    if (lastSSID.equals(ssid_1))
    {
      if (setupOTAWebServer(ssid_1, password_1, label_1, lastKnownNetworkTimeout, wifiOnly))
      {
        saveLastSSID(ssid_1);
        return true;
      }
    }
    else if (lastSSID.equals(ssid_2))
    {
      if (setupOTAWebServer(ssid_2, password_2, label_2, lastKnownNetworkTimeout, wifiOnly))
      {
        saveLastSSID(ssid_2);
        return true;
      }
    }
    else if (lastSSID.equals(ssid_3))
    {
      if (setupOTAWebServer(ssid_3, password_3, label_3, lastKnownNetworkTimeout, wifiOnly))
      {
        saveLastSSID(ssid_3);
        return true;
      }
    }
    else
    {
      saveLastSSID("");
    }
  }

  return WiFi.status() == WL_CONNECTED;
}

bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts)
{
  if (wifiOnly && WiFi.status() == WL_CONNECTED)
    return true;

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);

  if (connectToLastConnectedWifiNetwork(wifiOnly) == false)
  {
    // Fallback: Normal scan and connect process
    while (repeatScanAttempts-- &&
          (WiFi.status() != WL_CONNECTED ||
            WiFi.status() == WL_CONNECTED && wifiOnly == false && otaActive == false ) )
    {
      const char* network = scanForKnownNetwork();
    
      if (!network)
      {
        delay(500);
        continue;
      }

      int connectToFoundNetworkAttempts = 3;
      const int repeatDelay = 500;
    
      if (strcmp(network,ssid_1) == 0)
      {
        while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_1, password_1, label_1, timeout_1, wifiOnly))
          delay(repeatDelay);
        if (WiFi.status() == WL_CONNECTED)
          saveLastSSID(ssid_1);
      }
      else if (strcmp(network,ssid_2) == 0)
      {
        while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_2, password_2, label_2, timeout_2, wifiOnly))
          delay(repeatDelay);
        if (WiFi.status() == WL_CONNECTED)
          saveLastSSID(ssid_2);
      }
      else if (strcmp(network,ssid_3) == 0)
      {
        while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_3, password_3, label_3, timeout_3, wifiOnly))
          delay(repeatDelay);
        if (WiFi.status() == WL_CONNECTED)
          saveLastSSID(ssid_3);
      }
      
      delay(repeatDelay);
    }
  }

  if (WiFi.status() == WL_CONNECTED)  
  {
    ssid_connected = WiFi.SSID();
    return true;
  }
  
  ssid_connected = ssid_not_connected;
  
  return false;
}

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly)
{
  const bool showQRCode = false;    // disabled - kept for reference

  if (wifiOnly && WiFi.status() == WL_CONNECTED)
  {
     M5.Lcd.print("Already Connected!");
    return true;
  }

  bool forcedCancellation = false;

  M5.Lcd.setRotation(0);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.print("Connect\nWifi");

  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(makoHostName);

  wl_status_t beginResult = WiFi.begin(_ssid, _password);
  
  // Wait for connection for max of timeout milliseconds
  const int cycleTime = 500;
  int count = timeout / cycleTime;
  
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    M5.Lcd.print(".");
    delay(cycleTime);
  }
  
  if (WiFi.status() == WL_CONNECTED )
  {
    if (wifiOnly == false && !otaActive)
    {
      asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
      {
        request->send(200, "text/plain", "To upload firmware use /update. For compass calibration data use /calibration-data");
      });

      // Compass calibration data endpoint
      asyncWebServer.on("/calibration-data", HTTP_GET, [](AsyncWebServerRequest * request)
      {
        if (calibrationSampleCount == 0) {
          request->send(200, "text/plain", "No calibration data available. Start compass calibration first.");
          return;
        }

        AsyncWebServerResponse *response = request->beginChunkedResponse("text/csv", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
          static uint16_t currentSample = 0;

          // Reset on first chunk
          if (index == 0) {
            currentSample = 0;
          }

          size_t written = 0;

          // Write CSV header on first chunk
          if (currentSample == 0) {
            const char* header = "x,y,z\n";
            size_t headerLen = strlen(header);
            if (headerLen <= maxLen) {
              memcpy(buffer, header, headerLen);
              written = headerLen;
            }
          }

          // Write samples in chunks
          const size_t maxSamplesPerChunk = 50; // Process 50 samples per chunk
          size_t samplesInThisChunk = 0;

          while (currentSample < calibrationSampleCount &&
                 samplesInThisChunk < maxSamplesPerChunk &&
                 written < maxLen - 36) { // Leave room for one complete line

            char line[100];
            int lineLen = snprintf(line, sizeof(line), "%.6f,%.6f,%.6f\n",
                                 calibrationData[currentSample].x,
                                 calibrationData[currentSample].y,
                                 calibrationData[currentSample].z);

            if (written + lineLen > maxLen) {
              break; // Not enough space for this line
            }

            memcpy(buffer + written, line, lineLen);
            written += lineLen;
            currentSample++;
            samplesInThisChunk++;
          }

          return written;
        });

        request->send(response);
      });
        
      AsyncElegantOTA.setID(MERCATOR_OTA_DEVICE_LABEL);
      AsyncElegantOTA.setUploadBeginCallback(uploadOTABeginCallback);
      AsyncElegantOTA.setUploadProgressCallback(uploadOTAProgressCallback);
      AsyncElegantOTA.setUploadSucceededCallback(uploadOTASucceededCallback);
      AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
      asyncWebServer.begin();
      
      connected = true;
      otaActive = true;
    }
  }
  else
  {
    if (forcedCancellation)
      M5.Lcd.print("\nCancelled\nConnect\nAttempts");
    else
      M5.Lcd.print("No Connect");
  }

  M5.Lcd.fillScreen(TFT_BLACK);

  return connected;
}

void toggleWiFiActive(bool wait)
{
  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);

  if (ESPNowActive)
    toggleESPNowActive();

  if (WiFi.status() == WL_CONNECTED)
  {
    if (otaActive)
    {
      asyncWebServer.end();
      M5.Lcd.println("OTA Disabled");
      otaActive = false;
    }

    WiFi.disconnect();
    ssid_connected = ssid_not_connected;
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.printf("Wifi Disabled");
  }
  else
  {
    const bool wifiOnly = true;
    const int scanAttempts = 3;
    connectToWiFiAndInitOTA(wifiOnly,scanAttempts);

    M5.Lcd.fillScreen(TFT_ORANGE);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setRotation(1);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);

    M5.Lcd.printf(WiFi.status() == WL_CONNECTED ? "Wifi Enabled" : "No Connect");
  }

  if (wait)
    delay(2000);

  M5.Lcd.fillScreen(TFT_BLACK);
}

void disableAllWatchdogs() {
  // Disable task watchdog for current task
  esp_task_wdt_delete(NULL);

  // Disable system-wide task watchdogs
  esp_task_wdt_deinit();

  // Disable RTC watchdog
  
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  rtc_wdt_protect_on();
}


void disableFeaturesForOTA()
{
  disableAllWatchdogs();

  if (lemonTaskHandle != nullptr) {
    vTaskDelete(lemonTaskHandle);
    lemonTaskHandle = nullptr;
  }
  
  if (lemonRxQueue != nullptr) 
  {
    vQueueDelete(lemonRxQueue);
    lemonRxQueue = nullptr;
  }

  writeLogToSerial = false;
  enableDigitalCompass = enableTiltCompensation = enableSmoothedCompass = enableHumiditySensor = false;
  enableDepthSensor = enableIMUSensor = enableColourSensor = enableDownlinkComms = enableUplinkComms = enableDepthSensor = false;
  compassAvailable = imuAvailable = colourSensorAvailable =  depthAvailable = false;

  setRedLEDOn();
}

void toggleOTAActive()
{
  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_ORANGE);
  M5.Lcd.setRotation(1);

  if (otaActive)
  {
    asyncWebServer.end();
    M5.Lcd.println("OTA Disabled");
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

      M5.Lcd.fillScreen(TFT_ORANGE);
      M5.Lcd.setCursor(10, 10);
      M5.Lcd.setTextColor(TFT_WHITE, TFT_ORANGE);
    }
    
    if (WiFi.status() == WL_CONNECTED)
    {
      asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/plain", "To upload firmware use /update. For compass calibration data use /calibration-data");
      });

      // Compass calibration data endpoint
      asyncWebServer.on("/calibration-data", HTTP_GET, [](AsyncWebServerRequest * request)
      {
        if (calibrationSampleCount == 0) {
          request->send(200, "text/plain", "No calibration data available. Start compass calibration first.");
          return;
        }

        AsyncWebServerResponse *response = request->beginChunkedResponse("text/csv", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
          static uint16_t currentSample = 0;

          // Reset on first chunk
          if (index == 0) {
            currentSample = 0;
          }

          size_t written = 0;

          // Write CSV header on first chunk
          if (currentSample == 0) {
            const char* header = "x,y,z\n";
            size_t headerLen = strlen(header);
            if (headerLen <= maxLen) {
              memcpy(buffer, header, headerLen);
              written = headerLen;
            }
          }

          // Write samples in chunks
          const size_t maxSamplesPerChunk = 50; // Process 50 samples per chunk
          size_t samplesInThisChunk = 0;

          while (currentSample < calibrationSampleCount &&
                 samplesInThisChunk < maxSamplesPerChunk &&
                 written < maxLen - 36) { // Leave room for one complete line

            char line[100];
            int lineLen = snprintf(line, sizeof(line), "%.6f,%.6f,%.6f\n",
                                 calibrationData[currentSample].x,
                                 calibrationData[currentSample].y,
                                 calibrationData[currentSample].z);

            if (written + lineLen > maxLen) {
              break; // Not enough space for this line
            }

            memcpy(buffer + written, line, lineLen);
            written += lineLen;
            currentSample++;
            samplesInThisChunk++;
          }

          return written;
        });

        request->send(response);
      });

      AsyncElegantOTA.setID(MERCATOR_OTA_DEVICE_LABEL);
      AsyncElegantOTA.setUploadBeginCallback(uploadOTABeginCallback);
      AsyncElegantOTA.setUploadProgressCallback(uploadOTAProgressCallback);
      AsyncElegantOTA.setUploadSucceededCallback(uploadOTASucceededCallback);
      AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
      asyncWebServer.begin();

      if (wifiToggled)
      {
        // Clear the QR Code from new wifi connection
        M5.Lcd.fillScreen(TFT_ORANGE);
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextSize(3);
        M5.Lcd.setRotation(1);
        M5.Lcd.setTextColor(TFT_WHITE, TFT_ORANGE);
        
        M5.Lcd.printf(" OTA & WiFi\n   Enabled");
      }
      else
      {
        M5.Lcd.printf("OTA Enabled");
      }
              
      otaActive = true;
    }
    else
    {
      M5.Lcd.println("Error: Enable Wifi First");
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

#endif
