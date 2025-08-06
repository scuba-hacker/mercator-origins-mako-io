#ifdef BUILD_INCLUDE_MAIN_ESP_NOW_CODE

#include "esp_err.h"

// *************************** ESP Now Functions ******************

// Known MAC addresses for faster pairing (update these with your actual device MACs)
uint8_t LEMON_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};      // BSSID: the AP Mac, not STA Mac shown in router
uint8_t SILKY_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};      // BSSID: the AP Mac, not STA Mac shown in router
const char* silkyName = "Silky";

uint8_t TIGER_MAC[6]      = {0,0,0,0,0,0};
uint8_t TIGER_MAC_TEST[6]  = {0xE8, 0x9F, 0x6D, 0x00, 0x98, 0xA1};// BSSID: the AP Mac, not STA Mac shown in router
uint8_t TIGER_MAC_POD[6] = {0xE8, 0x9F, 0x6D, 0x09, 0xA7, 0x91};  // BSSID: the AP Mac, not STA Mac shown in router
const char* tigerName = "Tiger";

uint8_t OCEANIC_MAC[6] = {0x76, 0x4D, 0xBD, 0x7B, 0xA8, 0xE4};    // BSSID: the AP Mac, not STA Mac shown in router
const char* oceanicName = "Oceanic";

uint8_t MAKO_MAC[6] = {0x94, 0xB9, 0x7E, 0xAC, 0xF5, 0x45};       // BSSID: the AP Mac, not STA Mac shown in router
// Set to true to use known MAC addresses for faster pairing without wifi scans
bool USE_KNOWN_MACS = true;

void setKnownMAC(uint8_t* targetMAC, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5)
{
  targetMAC[0] = b0;
  targetMAC[1] = b1;
  targetMAC[2] = b2;
  targetMAC[3] = b3;
  targetMAC[4] = b4;
  targetMAC[5] = b5;
}

bool pairWithKnownMAC(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, const uint8_t* knownMAC);

void disableFastPairing()
{
  USE_KNOWN_MACS = false;
}

bool allDevicesPaired()
{
  return (isPairedWithSilky && isPairedWithTiger && isPairedWithOceanic);
}

int getUnpairedDeviceCount()
{
  int count = 0;
  if (!isPairedWithSilky) count++;
  if (!isPairedWithTiger) count++;
  if (!isPairedWithOceanic) count++;
  return count;
}

bool testDeviceConnectivity(esp_now_peer_info_t& peer, const char* deviceName)
{
  bool sent = false;
  if (ESPNowActive)
  {
    M5.Lcd.setCursor(5,12*7);
    
    if (pingSentTigerCount % 2)
      M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

    M5.Lcd.printf("Send Ping %i %i %i\n", pingSentTigerCount, attemptTigerPingSend, (int)ESPNow_tiger_peer.channel);

    if (deviceName == tigerName)
      publishToTigerPingEvent();
    else if (deviceName == oceanicName)
      publishToOceanicPingEvent();
    else if (deviceName == silkyName)
      publishToSilkyPingEvent();
  }
  
  return sent;
}

uint32_t verifyAllEspNowConnections()
{
  const uint32_t  verificationMinimumInterval = 3000;
  static uint32_t nextVerifyConnectionAttempt = 0;
  static uint32_t lastAttemptTime = 0;
  
  if (millis() > nextVerifyConnectionAttempt)
  {
    //if (isPairedWithSilky)
    //  testDeviceConnectivity(ESPNow_silky_peer, silkyName);
    
    if (isPairedWithTiger)
      testDeviceConnectivity(ESPNow_tiger_peer, tigerName);
    
    //if (isPairedWithOceanic)
   //   testDeviceConnectivity(ESPNow_oceanic_peer, oceanicName);
    
    lastAttemptTime = nextVerifyConnectionAttempt;
    nextVerifyConnectionAttempt = millis() + verificationMinimumInterval;
  }

  return lastAttemptTime;
}

// Init ESP Now with fallback
bool InitESPNow() 
{
  if (testTigerOutsidePod)
  {
    tigerHostName = testTigerHostName;
    memcpy(TIGER_MAC, TIGER_MAC_TEST, sizeof(TIGER_MAC_TEST));
  }
  else
  {
    tigerHostName = podTigerHostName;
    memcpy(TIGER_MAC, TIGER_MAC_POD, sizeof(TIGER_MAC_POD));
  }
 
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
    M5.Lcd.setTextSize(1);
    M5.Lcd.setRotation(0);

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

      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(0,0);
      M5.Lcd.setRotation(0);

      bool success = connectESPNow();

      if (success)
      {
        ESPNowActive = true;

        M5.Lcd.setRotation(1);
        
        if (USE_KNOWN_MACS)
        {
          // Fast pairing using known MAC addresses
          isPairedWithTiger = pairWithKnownMAC(ESPNow_tiger_peer, tigerHostName, TIGER_MAC);
          isPairedWithSilky = pairWithKnownMAC(ESPNow_silky_peer, silkyHostName, SILKY_MAC);
          isPairedWithOceanic = pairWithKnownMAC(ESPNow_oceanic_peer, oceanicHostName, OCEANIC_MAC);
        }
        else
        {
          // Standard pairing with scanning
          int peeringAttempts = 3;
          isPairedWithSilky = pairWithPeer(ESPNow_silky_peer,"AudioPod",peeringAttempts);
          
          peeringAttempts = 5;
          isPairedWithTiger = pairWithPeer(ESPNow_tiger_peer, tigerHostName, peeringAttempts);

          peeringAttempts = 5;
          isPairedWithOceanic = pairWithPeer(ESPNow_oceanic_peer,"Oceanic",peeringAttempts);
        }
        
        if (isPairedWithSilky)
        {
          // set Silky volume to default.
          publishToSilkySetVolume(defaultSilkyVolume);
        }

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

  // See Tiger configAndStartUpESPNow() for full explanation of why WIFI_AP_STA is needed for ESP NOW
  // It's a subtle ESP32/ESP-NOW implementation detail - the receive path uses the AP radio, but the transmit path
  // requires both interfaces to be properly configured in WIFI_AP_STA mode.
  
  WiFi.mode(WIFI_AP_STA);
  
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
  String Prefix = String(makoHostName) + String(":");
  String Mac = WiFi.softAPmacAddress();
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


bool pairWithKnownMAC(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, const uint8_t* mac_addr)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  USB_SERIAL_PRINTF("pairWithKnownMAC: %s %s\n",peerSSIDPrefix, macStr);

  // Skip scanning - use known MAC directly
  // IMPORTANT: Reset peer structure first (same as ESPNowScanForPeer does)
  memset(&peer, 0, sizeof(peer));
  
  // Setup peer structure with known MAC
  memcpy(peer.peer_addr, mac_addr, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = 0;
  peer.priv = (void*)peerSSIDPrefix;        // This is for our own purpose - ESPNow doesn't use this.
  
  USB_SERIAL_PRINTF("pairWithKnownMAC: peer.peer_addr = %02X:%02X:%02X:%02X:%02X:%02X\n",
        peer.peer_addr[0],
        peer.peer_addr[1],
        peer.peer_addr[2],
        peer.peer_addr[3],
        peer.peer_addr[4],
        peer.peer_addr[5]);
              
  return ESPNowManagePeer(peer);
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

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setCursor(0,0);
  
  return isPaired;
}

// Mitigation needed for fixing Send failing due to no WIFI_AP_STA mode being set before
void scanNetworksNoOp()
{
  USB_SERIAL_PRINTLN("scanNetworksNoOp: scanNetworks");
  WiFi.scanNetworks();
  USB_SERIAL_PRINTLN("scanNetworksNoOp: scanDelete");
  WiFi.scanDelete();
}

// Scan for peers in AP mode
bool ESPNowScanForPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix)
{
  bool peerFound = false;
  
  M5.Lcd.printf("Scan For %s\n",peerSSIDPrefix);
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
        
        M5.Lcd.printf("Found %s\n",BSSIDstr.c_str());
        delay(5000);
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
      else if (addStatus == ESP_ERR_ESPNOW_EXIST) 
      {
          USB_SERIAL_PRINTLN("Peer Exists");
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

  if (!result)
    peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;

  return result;
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

void OnESPNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) 
{
  // CRITICAL: Don't process ESP-NOW messages during OTA to prevent queue corruption
  if (haltAllProcessingDuringOTAUpload || otaActive) {
    return; // Silently drop messages during OTA
  }

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

  espMsgReceived++;
  
    // Handle ping responses by identifying sender MAC address
  if (data_len > 0 && *data == 'p') {
    uint32_t currentTime = millis();
    
    espPingMsgReceived++;

    // Check if this is from Tiger
    if (memcmp(mac_addr, TIGER_MAC, 6) == 0) {
      pingResponseReceivedFromTiger++;
      tigerLastPingResponse = currentTime;
    }
    // Check if this is from Silky
    else if (memcmp(mac_addr, SILKY_MAC, 6) == 0) {
      silkyLastPingResponse = currentTime;
    }
    // Check if this is from Oceanic
    else if (memcmp(mac_addr, OCEANIC_MAC, 6) == 0) {
      oceanicLastPingResponse = currentTime;
    }
    else
    {
      pingResponseReceivedFromUnknown++;
    }
  }

  if (msgsReceivedQueue && ESPNowActive)
  {
    xQueueSend(msgsReceivedQueue, (void*)data, (TickType_t)0);  // don't block on enqueue
  }
}

bool TeardownESPNow()
{
  bool result = false;

  if (enableESPNow && ESPNowActive)
  {
    ESPNowDeletePeer(ESPNow_silky_peer);
    ESPNowDeletePeer(ESPNow_tiger_peer);
    ESPNowDeletePeer(ESPNow_oceanic_peer);
    
    esp_now_deinit();
    
    WiFi.disconnect();
    ESPNowActive = false;
    result = true;
  }
  
  // Always reset pairing status when tearing down
  isPairedWithSilky = false;
  isPairedWithTiger = false;
  isPairedWithOceanic = false;
  
  // Reset ping response timestamps
  tigerLastPingResponse = 0;
  silkyLastPingResponse = 0;  
  oceanicLastPingResponse = 0;
  
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
