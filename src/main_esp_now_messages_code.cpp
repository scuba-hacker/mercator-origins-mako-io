
#ifdef BUILD_INCLUDE_MAIN_ESP_NOW_MESSAGES_CODE

// *************************** Tiger ESPNow Send Functions ******************

char tiger_espnow_buffer[256];

void publishToTigerBrightLightEvent()
{
  if (isPairedWithTiger && ESPNow_tiger_peer.channel == ESPNOW_CHANNEL)
  {
    memset(tiger_espnow_buffer,0,sizeof(tiger_espnow_buffer));
    tiger_espnow_buffer[0] = 'l';  // command l = light event
    tiger_espnow_buffer[1] = '\0';
    ESPNowSendResult = esp_now_send(ESPNow_tiger_peer.peer_addr, (uint8_t*)tiger_espnow_buffer, strlen(tiger_espnow_buffer)+1);
  }

  sendBrightLightEventToTiger = false;
}

void publishToTigerAndOceanicCurrentTarget(const char* currentTarget)
{     
  memset(tiger_espnow_buffer,0,sizeof(tiger_espnow_buffer));
  tiger_espnow_buffer[0] = 'c';  // command c= current target
  tiger_espnow_buffer[1] = '\0';
  strncpy(tiger_espnow_buffer+1,currentTarget,sizeof(tiger_espnow_buffer)-2);

  if (isPairedWithTiger && ESPNow_tiger_peer.channel == ESPNOW_CHANNEL)
  {
    ESPNowSendResult = esp_now_send(ESPNow_tiger_peer.peer_addr, (uint8_t*)tiger_espnow_buffer, strlen(currentTarget)+1);
  }

  if (isPairedWithOceanic && ESPNow_oceanic_peer.channel == ESPNOW_CHANNEL)
  {
    ESPNowSendResult = esp_now_send(ESPNow_oceanic_peer.peer_addr, (uint8_t*)tiger_espnow_buffer, strlen(currentTarget)+1);
  }
}

void publishToTigerAndOceanicLocationAndTarget(const char* currentTarget)
{     
  memset(tiger_espnow_buffer,0,sizeof(tiger_espnow_buffer));
  
  tiger_espnow_buffer[0] = 'X';  // command c=target code, location, heading, target

  const int maxCodeLength = 6;

  const int targetCodeOffset = 1;
  const int latitudeOffset = 8;
  const int longitudeOffset = 16;
  const int headingOffset = 24;
  const int currentTargetOffset = 32;
        
  const char* endOfCode=currentTarget;
  while (endOfCode - currentTarget < maxCodeLength && std::isalnum(*endOfCode++));

  if (endOfCode != currentTarget)
    memcpy(tiger_espnow_buffer+targetCodeOffset,currentTarget,endOfCode - currentTarget - 1);

  tiger_espnow_buffer[endOfCode - currentTarget + 1] = '\0';

  memcpy(tiger_espnow_buffer+latitudeOffset,&Lat,sizeof(Lat));
  memcpy(tiger_espnow_buffer+longitudeOffset,&Lng,sizeof(Lng));
  memcpy(tiger_espnow_buffer+headingOffset,&magnetic_heading,sizeof(magnetic_heading));

  strncpy(tiger_espnow_buffer+currentTargetOffset,currentTarget,sizeof(tiger_espnow_buffer)-2-currentTargetOffset);

  if (isPairedWithTiger && ESPNow_tiger_peer.channel == ESPNOW_CHANNEL)
  {
    ESPNowSendResult = esp_now_send(ESPNow_tiger_peer.peer_addr, (uint8_t*)tiger_espnow_buffer, currentTargetOffset+strlen(currentTarget)+1);
  }

  if (isPairedWithOceanic && ESPNow_oceanic_peer.channel == ESPNOW_CHANNEL)
  {
    ESPNowSendResult = esp_now_send(ESPNow_oceanic_peer.peer_addr, (uint8_t*)tiger_espnow_buffer, currentTargetOffset+strlen(currentTarget)+1);
  }
}

// *************************** Oceanic Send Functions ******************

char oceanic_espnow_buffer[256];

void publishToOceanicLightLevel(uint16_t lightLevel)
{
  if (isPairedWithOceanic && ESPNow_oceanic_peer.channel == ESPNOW_CHANNEL)
  {
    memset(oceanic_espnow_buffer,0,sizeof(oceanic_espnow_buffer));
    oceanic_espnow_buffer[0] = 'D';  // command D = send light level to oceanic
    oceanic_espnow_buffer[1] = lightLevel & 0xFF;
    oceanic_espnow_buffer[2] = (lightLevel >> 8);
    oceanic_espnow_buffer[3] = '\0';
    ESPNowSendResult = esp_now_send(ESPNow_oceanic_peer.peer_addr, (uint8_t*)oceanic_espnow_buffer, strlen(oceanic_espnow_buffer)+1);
  }

  sendLightLevelToOceanic = false;
}

void publishToOceanicBreadCrumbRecord(const bool record)
{
  if (isPairedWithOceanic && ESPNow_oceanic_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(oceanic_espnow_buffer,sizeof(oceanic_espnow_buffer),"B%c",(record ? 'Y' : 'N'));

      USB_SERIAL_PRINTLN("Sending ESP B msg to Oceanic...");

      USB_SERIAL_PRINTLN(oceanic_espnow_buffer);

    ESPNowSendResult = esp_now_send(ESPNow_oceanic_peer.peer_addr, reinterpret_cast<uint8_t*>(oceanic_espnow_buffer), strlen(oceanic_espnow_buffer)+1);
  }
}

void publishToOceanicPinPlaced(double latitude, double longitude, double heading, double depth)
{
  if (isPairedWithOceanic && ESPNow_oceanic_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(oceanic_espnow_buffer,sizeof(oceanic_espnow_buffer),"P%.7f %.7f %.0f %.1f",latitude,longitude,heading,depth);

      USB_SERIAL_PRINTLN("Sending ESP P msg to Oceanic...");

      USB_SERIAL_PRINTLN(oceanic_espnow_buffer);

    ESPNowSendResult = esp_now_send(ESPNow_oceanic_peer.peer_addr, reinterpret_cast<uint8_t*>(oceanic_espnow_buffer), strlen(oceanic_espnow_buffer)+1);
  }
}

// *************************** Silky Sound Send Functions ******************

void toggleSound()
{
 if (soundsOn)
    soundsOn = false;
  else
    soundsOn = true;  
}

void notifySoundsOnOffChanged()
{
  if (isPairedWithSilky && ESPNow_silky_peer.channel == ESPNOW_CHANNEL)
  {
    ESPNowSendResult = ESP_OK;
    audioAction = AUDIO_ACTION_SOUNDS_TOGGLE;
  }
}

void rotateToNextGuidanceSounds()
{
  switch(SFX_AHEAD)
  {
    case SFX_PIANO_AHEAD:
    {
      SFX_AHEAD = SFX_ORGAN_AHEAD;
      SFX_TURN_AROUND = SFX_ORGAN_BEHIND;
      SFX_ANTICLOCKWISE = SFX_ORGAN_LEFT;
      SFX_CLOCKWISE = SFX_ORGAN_RIGHT;
      break;
    }
    case SFX_ORGAN_AHEAD:
    {
      SFX_AHEAD = SFX_PAD_AHEAD;
      SFX_TURN_AROUND = SFX_PAD_BEHIND;
      SFX_ANTICLOCKWISE = SFX_PAD_LEFT;
      SFX_CLOCKWISE = SFX_PAD_RIGHT;
      break;
    }
    case SFX_PAD_AHEAD:
    {
      SFX_AHEAD = SFX_NONE;
      SFX_TURN_AROUND = SFX_NONE;
      SFX_ANTICLOCKWISE = SFX_NONE;
      SFX_CLOCKWISE = SFX_NONE;
      break;
    }
    case SFX_NONE:
    {
      SFX_AHEAD = SFX_PIANO_AHEAD;
      SFX_TURN_AROUND = SFX_PIANO_BEHIND;
      SFX_ANTICLOCKWISE = SFX_PIANO_LEFT;
      SFX_CLOCKWISE = SFX_PIANO_RIGHT;
      break;
    }
    default:
    {
      SFX_AHEAD = SFX_NONE;
      SFX_TURN_AROUND = SFX_NONE;
      SFX_ANTICLOCKWISE = SFX_NONE;
      SFX_CLOCKWISE = SFX_NONE;
    }
  }
  
  if (**(++currentSoundSet) == 0)
    currentSoundSet = soundSets;
  
  audioAction = AUDIO_ACTION_ROTATE_SOUND_SET;
}

void publishToSilkyPlayAudioGuidance(enum e_soundFX sound)
{
  if (isPairedWithSilky && soundsOn && ESPNow_silky_peer.channel == ESPNOW_CHANNEL && sound != SFX_NONE)
  {
    uint8_t ESPNow_Silky_data_to_send = (uint8_t)sound;
    const uint8_t *peer_addr = ESPNow_silky_peer.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_Silky_data_to_send, sizeof(ESPNow_Silky_data_to_send));

    audioAction = AUDIO_ACTION_NONE;
  }
}

void publishToSilkySkipToNextTrack()
{
  if (isPairedWithSilky && soundsOn && ESPNow_silky_peer.channel == ESPNOW_CHANNEL)
  {
    // Send byte command to Silky to say skip to next track
    uint8_t ESPNow_Silky_data_to_send = SILKY_ESPNOW_COMMAND_NEXT_TRACK;
    const uint8_t *peer_addr = ESPNow_silky_peer.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_Silky_data_to_send, sizeof(ESPNow_Silky_data_to_send));

    audioAction = AUDIO_ACTION_NEXT_SOUND;
  }
}

void publishToSilkyCycleVolumeUp()
{
  if (isPairedWithSilky && soundsOn && ESPNow_silky_peer.channel == ESPNOW_CHANNEL)
  {
    if (silkyVolume == maxSilkyVolume)
      silkyVolume = minSilkyVolume;
    else
      silkyVolume++;

    publishToSilkySetVolume(silkyVolume);

    audioAction = AUDIO_ACTION_CYCLE_VOLUME;
  }
}

void publishToSilkySetVolume(const uint8_t newVolume)
{
  if (isPairedWithSilky && ESPNow_silky_peer.channel == ESPNOW_CHANNEL)
  {
    silkyVolume = newVolume;
      
    // Send byte command to Silky to say set volume to silkyVolume
    uint16_t ESPNow_word_to_send = ((uint16_t)silkyVolume << 8) | (uint16_t)SILKY_ESPNOW_COMMAND_SET_VOLUME;
    const uint8_t *peer_addr = ESPNow_silky_peer.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, (uint8_t*)&ESPNow_word_to_send, sizeof(ESPNow_word_to_send));
    audioAction = AUDIO_ACTION_NONE;  // done on startup and no screen change needed.
  }
}

void publishToSilkyTogglePlayback()
{
  if (isPairedWithSilky && soundsOn && ESPNow_silky_peer.channel == ESPNOW_CHANNEL)
  {
    // Send byte command to Silky to say skip to next track
    uint8_t ESPNow_Silky_data_to_send = SILKY_ESPNOW_COMMAND_TOGGLE_PLAYBACK;
    const uint8_t *peer_addr = ESPNow_silky_peer.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_Silky_data_to_send, sizeof(ESPNow_Silky_data_to_send));
    audioAction = AUDIO_ACTION_PLAYBACK_TOGGLE;
  }
}

void publishToSilkyStopPlayback()
{
  if (isPairedWithSilky && !soundsOn && ESPNow_silky_peer.channel == ESPNOW_CHANNEL)
  {
    // Send byte command to Silky to say skip to next track
    uint8_t ESPNow_Silky_data_to_send  = SILKY_ESPNOW_COMMAND_STOP_PLAYBACK;
    const uint8_t *peer_addr = ESPNow_silky_peer.peer_addr;
    ESPNowSendResult = esp_now_send(peer_addr, &ESPNow_Silky_data_to_send, sizeof(ESPNow_Silky_data_to_send));
    audioAction = AUDIO_ACTION_STOP_PLAYBACK;
  }
}

#endif