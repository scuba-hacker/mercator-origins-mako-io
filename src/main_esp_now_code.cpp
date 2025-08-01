#ifdef BUILD_INCLUDE_MAIN_ESP_NOW_CODE

// *************************** ESP Now Functions ******************

// Init ESP Now with fallback
bool InitESPNow() 
{
  WiFi.disconnect();
  
  if (esp_now_init() == ESP_OK) 
  { 
      USB_SERIAL_PRINTLN("ESPNow Init Success");
    ESPNowActive = true;
  }
  else 
  {
      USB_SERIAL_PRINTLN("ESPNow Init Failed");
    // do nothing
    ESPNowActive = false;
  }
  
  return ESPNowActive;
}

bool TeardownESPNow()
{
  bool result = false;

  if (enableESPNow && ESPNowActive)
  {
    WiFi.disconnect();
    ESPNowActive = false;
    result = true;
  }
  
  return result;
}

void notifyESPNowNotActive()
{
  M5.Lcd.fillScreen(TFT_RED);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Error: ESPNow inactive");
  delay (2000);
  M5.Lcd.fillScreen(TFT_BLACK);
}

void displayESPNowSendDataResult(const esp_err_t result)
{
  if (result == ESP_OK)
    M5.Lcd.println("Success");
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
    M5.Lcd.println("ESPNOW not Init.");
  else if (result == ESP_ERR_ESPNOW_ARG)
    M5.Lcd.println("ESPNOW Invalid Argument");
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
    M5.Lcd.println("ESPNOW Internal Error");
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
    M5.Lcd.println("ESPNOW No Memory");
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
    M5.Lcd.println("ESPNOW Peer not found.");
  else
    M5.Lcd.println("ESPNOW Unknown Error");
}

void toggleESPNowActive()
{
  if (enableESPNow)
  {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);

    bool disabledWiFi = false;
    
    if (ESPNowActive == false)
    {
      if (otaActive)
        toggleOTAActive();

      if (WiFi.status() == WL_CONNECTED)
      {
        toggleWiFiActive();
        disabledWiFi = true;
      }

      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0,0);

      bool success = connectESPNow();

      if (success)
      {
        ESPNowActive = true;

        M5.Lcd.setRotation(0);
        M5.Lcd.println("  ESPNow\nSearching\n");
          USB_SERIAL_PRINTLN("Wifi\nDisabled\nESPNow\nEnabled");
        
        int peeringAttempts = 3;
        isPairedWithSilky = pairWithPeer(ESPNow_silky_peer,"AudioPod",peeringAttempts);
        
        if (isPairedWithSilky)
        {
          // set Silky volume to default.
          publishToSilkySetVolume(defaultSilkyVolume);
        }

        peeringAttempts = 5;
        isPairedWithTiger = pairWithPeer(ESPNow_tiger_peer,"Tiger",peeringAttempts);

        peeringAttempts = 5;
        isPairedWithOceanic = pairWithPeer(ESPNow_oceanic_peer,"Oceanic",peeringAttempts);

        // send message to tiger and oceanic to give first target
        // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
        publishToTigerAndOceanicCurrentTarget(nextWaypoint->_m5label);

        bool recordBreadCrumbTrail = false; // ensure trail not being record after pairing to get mako/oceanic in sync
        publishToOceanicBreadCrumbRecord(recordBreadCrumbTrail);
      }
      else
      {
        isPairedWithSilky = false;
        isPairedWithTiger = false;
        isPairedWithOceanic = false;
      }

      if (!isPairedWithSilky && !isPairedWithTiger && !isPairedWithOceanic)
      {
        TeardownESPNow();
        ESPNowActive = false;
  
        M5.Lcd.println("   ESPNow\nDisabled");
          USB_SERIAL_PRINTLN("ESPNow Disabled");
      }
    }
    else
    { 
      // disconnect ESPNow;
      TeardownESPNow();
 
      ESPNowActive = false;
      isPairedWithSilky = false;
      isPairedWithTiger = false;
      isPairedWithOceanic = false;

      M5.Lcd.println("ESPNow Disabled");
      
        USB_SERIAL_PRINTLN("ESPNow Disabled");
    }
    
    delay (500);
  
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}

bool connectESPNow()
{
  //Set device in STA mode to begin with
// MBJ Removed  WiFi.mode(WIFI_STA);
  WiFi.mode(WIFI_AP);
  
    USB_SERIAL_PRINTLN("ESPNow/Basic/Master Example");
    
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  configESPNowDeviceAP();
  
  // This is the mac address of the Master in AP Mode
    USB_SERIAL_PRINT("AP MAC: "); USB_SERIAL_PRINTLN(WiFi.softAPmacAddress());

    USB_SERIAL_PRINT("AP CHANNEL "); USB_SERIAL_PRINTLN(WiFi.channel());

  // Init ESPNow with a fallback logic
  bool result =  InitESPNow();

  if (result)
  {
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnESPNowDataSent);
    esp_now_register_recv_cb(OnESPNowDataRecv);
  }
  
  return result;
}

void configESPNowDeviceAP() 
{
  String Prefix = "Mako:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), ESPNOW_CHANNEL, 0);

  if (!result) 
  {
    USB_SERIAL_PRINTLN("AP Config failed.");
  } 
  else 
  {
    USB_SERIAL_PRINTF("AP Config Success. Broadcasting with AP: %s\n",String(SSID).c_str());
    USB_SERIAL_PRINTF("WiFi Channel: %d\n",WiFi.channel());
  }
}

void OnESPNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) 
{
  if (writeLogToSerial)
  {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    USB_SERIAL_PRINTF("Last Packet Recv from: %s\n",macStr);
    USB_SERIAL_PRINTF("Last Packet Recv 1st Byte: '%c'\n",*data);
    USB_SERIAL_PRINTF("Last Packet Recv Length: %d\n",data_len);
    USB_SERIAL_PRINTLN((char*)data);
  }

  xQueueSend(msgsReceivedQueue, (void*)data, (TickType_t)0);  // don't block on enqueue
}

// callback when data is sent from Master to Peer
void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    ESPNowMessagesDelivered++;
  }
  else
  {
    ESPNowMessagesFailedToDeliver++;
  }
}

// Scan for peers in AP mode
bool ESPNowScanForPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix)
{
  bool peerFound = false;
  
  M5.Lcd.printf("Scan For\n%s\n",peerSSIDPrefix);
  int8_t scanResults = scanWiFiForSSIDs();
  
  // reset on each scan 
  memset(&peer, 0, sizeof(peer));

  USB_SERIAL_PRINTLN("");

  if (scanResults == 0) 
  {   
    USB_SERIAL_PRINTLN("No WiFi devices in AP Mode found");

    peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
  } 
  else 
  {
    USB_SERIAL_PRINT("Found "); 
    USB_SERIAL_PRINT(scanResults); 
    USB_SERIAL_PRINTLN(" devices ");
    
    for (int i = 0; i < scanResults; ++i) 
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (ESPNOW_PRINTSCANRESULTS) 
      {
        USB_SERIAL_PRINT(i + 1);
        USB_SERIAL_PRINT(": ");
        USB_SERIAL_PRINT(SSID);
        USB_SERIAL_PRINT(" (");
        USB_SERIAL_PRINT(RSSI);
        USB_SERIAL_PRINT(")");
        USB_SERIAL_PRINTLN("");
      }
      
      // Check if the current device starts with the peerSSIDPrefix
      if (SSID.indexOf(peerSSIDPrefix) == 0) 
      {
        if (writeLogToSerial)
        {
          // SSID of interest
          USB_SERIAL_PRINTLN("Found a peer.");
          USB_SERIAL_PRINT(i + 1); 
          USB_SERIAL_PRINT(": "); 
          USB_SERIAL_PRINT(SSID); 
          USB_SERIAL_PRINT(" ["); 
          USB_SERIAL_PRINT(BSSIDstr); 
          USB_SERIAL_PRINT("]"); 
          USB_SERIAL_PRINT(" ("); 
          USB_SERIAL_PRINT(RSSI); 
          USB_SERIAL_PRINT(")"); 
          USB_SERIAL_PRINTLN("");
        }
                
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) 
        {
          for (int ii = 0; ii < 6; ++ii ) 
          {
            peer.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        peer.channel = ESPNOW_CHANNEL; // pick a channel
        peer.encrypt = 0; // no encryption

        peer.priv = (void*)peerSSIDPrefix;   // distinguish between different peers

        peerFound = true;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (peerFound) 
  {
    M5.Lcd.println("Peer Found");
      USB_SERIAL_PRINTLN("Peer Found, processing..");
  } 
  else 
  {
    M5.Lcd.println("Peer Not Found");
      USB_SERIAL_PRINTLN("Peer Not Found, trying again.");
  }
  
  // clean up ram
  WiFi.scanDelete();

  return peerFound;
}

bool pairWithPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, int maxAttempts)
{
  bool isPaired = false;
  while(maxAttempts-- && !isPaired)
  {
    bool result = ESPNowScanForPeer(peer,peerSSIDPrefix);

    // check if peer channel is defined
    if (result && peer.channel == ESPNOW_CHANNEL)
    { 
      isPaired = ESPNowManagePeer(peer);
      M5.Lcd.setTextColor(TFT_GREEN);
      M5.Lcd.printf("%s Pair\nok\n",peerSSIDPrefix);
      M5.Lcd.setTextColor(TFT_WHITE);
    }
    else
    {
      peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
      M5.Lcd.setTextColor(TFT_RED);
      M5.Lcd.printf("%s Pair\nfail\n",peerSSIDPrefix);
      M5.Lcd.setTextColor(TFT_WHITE);
    }
  }

  delay(500);
  
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setCursor(0,0);
  
  return isPaired;
}

// Check if the peer is already paired with the master.
// If not, pair the peer with master
bool ESPNowManagePeer(esp_now_peer_info_t& peer)
{
  bool result = false;
  
  if (peer.channel == ESPNOW_CHANNEL) 
  {
    if (ESPNOW_DELETEBEFOREPAIR) 
    {
      ESPNowDeletePeer(peer);
    }

      USB_SERIAL_PRINT("Peer Status: ");
      
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer.peer_addr);
    
    if (exists) 
    {
      // Peer already paired.
        USB_SERIAL_PRINTLN("Already Paired");

      M5.Lcd.println("Already paired");
      result = true;
    } 
    else 
    {
      // Peer not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&peer);
      
      if (addStatus == ESP_OK) 
      {
        // Pair success
          USB_SERIAL_PRINTLN("Pair success");
        M5.Lcd.println("Pair success");
        result = true;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) 
      {
        // How did we get so far!!
          USB_SERIAL_PRINTLN("ESPNOW Not Init");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_ARG) 
      {
            USB_SERIAL_PRINTLN("Invalid Argument");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_FULL) 
      {
            USB_SERIAL_PRINTLN("Peer list full");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) 
      {
          USB_SERIAL_PRINTLN("Out of memory");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_EXIST) 
      {
          USB_SERIAL_PRINTLN("Peer Exists");
        result = true;
      } 
      else 
      {
          USB_SERIAL_PRINTLN("Not sure what happened");
        result = false;
      }
    }
  }
  else 
  {
    // No peer found to process
      USB_SERIAL_PRINTLN("No Peer found to process");
    
    M5.Lcd.println("No Peer found to process");
    result = false;
  }

  return result;
}

void ESPNowDeletePeer(esp_now_peer_info_t& peer) 
{
  if (peer.channel != ESPNOW_NO_PEER_CHANNEL_FLAG)
  {
    esp_err_t delStatus = esp_now_del_peer(peer.peer_addr);
    
    if (writeLogToSerial)
    {
      USB_SERIAL_PRINT("Peer Delete Status: ");
      if (delStatus == ESP_OK) 
      {
        // Delete success
        USB_SERIAL_PRINTLN("ESPNowDeletePeer::Success");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) 
      {
        // How did we get so far!!
        USB_SERIAL_PRINTLN("ESPNowDeletePeer::ESPNOW Not Init");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_ARG) 
      {
        USB_SERIAL_PRINTLN("ESPNowDeletePeer::Invalid Argument");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) 
      {
        USB_SERIAL_PRINTLN("ESPNowDeletePeer::Peer not found.");
      } 
      else 
      {
        USB_SERIAL_PRINTLN("Not sure what happened");
      }
    }
  }
}


///////////// TEST

void readAndTestGoProButtons()
{
  BtnGoProTop.read();
  BtnGoProSide.read();

  bool btnTopPressed = BtnGoProTop.pressedFor(15);
  bool btnSidePressed = BtnGoProSide.pressedFor(15);
  
  if (btnTopPressed && btnSidePressed)
  {
    sideCount++;
    topCount++;
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.printf("TOP+SIDE %hu %hu", topCount, sideCount);
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.print("           \n");
    M5.Lcd.print("           \n");
  }
  else if (btnTopPressed)
  {
    topCount++;
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.printf("TOP %hu", topCount);
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.print("           \n");
    M5.Lcd.print("           \n");
  }
  else if (btnSidePressed)
  {
    sideCount++;
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.printf("SIDE %hu", sideCount);
    M5.Lcd.setCursor(5, M5.Lcd.height() - 75);
    M5.Lcd.print("           \n");
    M5.Lcd.print("           \n");
  }
}

#endif
