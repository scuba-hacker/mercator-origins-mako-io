#ifdef BUILD_INCLUDE_MAIN_NETWORK_CODE


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

bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts)
{
  if (wifiOnly && WiFi.status() == WL_CONNECTED)
    return true;

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);

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
    }
    else if (strcmp(network,ssid_2) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_2, password_2, label_2, timeout_2, wifiOnly))
        delay(repeatDelay);
    }
    else if (strcmp(network,ssid_3) == 0)
    {
      while (connectToFoundNetworkAttempts-- && !setupOTAWebServer(ssid_3, password_3, label_3, timeout_3, wifiOnly))
        delay(repeatDelay);
    }
    
    delay(repeatDelay);
  }

  bool connected=WiFi.status() == WL_CONNECTED;
  
  if (connected)
  {
    ssid_connected = WiFi.SSID();
  }
  else
  {
    ssid_connected = ssid_not_connected;
  }
  
  return connected;
}

bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly)
{
  if (wifiOnly && WiFi.status() == WL_CONNECTED)
    return true;

  bool forcedCancellation = false;

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("mako");

  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  M5.Lcd.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {

    M5.Lcd.print(".");
    delay(300);
  }

  M5.Lcd.print("\n\n");

  if (WiFi.status() == WL_CONNECTED )
  {
    if (wifiOnly == false && !otaActive)
    {
      asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) 
      {
        request->send(200, "text/plain", "To upload firmware use /update");
      });
        
      AsyncElegantOTA.setID(MERCATOR_OTA_DEVICE_LABEL);
      AsyncElegantOTA.setUploadBeginCallback(uploadOTABeginCallback);
      AsyncElegantOTA.setUploadProgressCallback(uploadOTAProgressCallback);
      AsyncElegantOTA.setUploadSucceededCallback(uploadOTASucceededCallback);
      AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
      asyncWebServer.begin();

      M5.Lcd.setRotation(0);
      
      M5.Lcd.fillScreen(TFT_BLACK);
      M5.Lcd.setCursor(0,155);
      M5.Lcd.setTextSize(2);
      M5.Lcd.printf("%s\n\n",WiFi.localIP().toString().c_str());
      M5.Lcd.println(WiFi.macAddress());
      connected = true;
      otaActive = true;
  
      M5.Lcd.qrcode("http://"+WiFi.localIP().toString()+"/update",0,0,135);
  
      delay(1000);

      connected = true;
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
    M5.Lcd.printf("Wifi Connecting");

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
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
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
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    }
    
    if (WiFi.status() == WL_CONNECTED)
    {
      asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/plain", "To upload firmware use /update");
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
        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
        
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
