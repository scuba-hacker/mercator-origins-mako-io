
#ifdef BUILD_INCLUDE_MAIN_TELEMETRY_CODE



/////////////////// BUTTON PRESS CODE

bool waitForBothButtonsReleased()
{
  while (!(BtnGoProTop.isReleased() && BtnGoProSide.isReleased()));  
}

const uint32_t buttonPressDurationToChangeScreen = 50;

void checkForButtonPresses()
{
  // ignore button presses whilst a temporary display is shown
  if (showTempDisplayEndTime != disabledTempDisplayEndTime)
    return;
    
  updateButtons();

/*
  // very strange behaviour with GPIO 38 triggering with finger proximity and not magnet
  // disabled. Needs a pull-up resistor to pin 38 and probably INPUT_PULLUP changed to INPUT in Button class
  const uint32_t reedTestDuration = 100;
  if (ReedGoProBottomRight.wasReleasefor(reedTestDuration))
  {
    M5.Lcd.fillScreen(TFT_GREEN);
    delay(1000);
  }
*/

  if (checkForDualButtonPresses())
    return;

  switch (display_to_show)
  {
    case SURVEY_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))
      {
        switchToNextDisplayToShow();
      }
  
      if (p_secondButton->wasReleasefor(500)) // Record Highlight
      {
        recordSurveyHighlight = true;
        recordHighlightExpireTime = millis() + recordHighlightDisplayDuration;
      }
      break;
    }

    case LOCATION_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(1000)) // Location Display: toggle ota (and wifi if needed)
      {
        toggleOTAActive();
      }
      else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))
      {
        switchToNextDisplayToShow();
      }
  
      if (p_secondButton->wasReleasefor(1000)) // Location Display: toggle wifi only
      {
        toggleWiFiActive();
      }
      break;
    }
    
    case JOURNEY_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(1000)) // Journey Course Display: toggle send uplink messages
      {
        toggleUplinkMessageProcessAndSend();
      }
      else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
      {
        switchToNextDisplayToShow();
      }
  
      if (p_secondButton->wasReleasefor(5000)) // Journey Course Display: toggle uptime
      {
        toggleUptimeGlobalDisplay();
      }
      else if (p_secondButton->wasReleasefor(1000)) // Journey Course Display: toggle async/sync mode for getting depth
      {
        toggleAsyncDepthDisplay();
      }
      break;
    }
    
    case AUDIO_TEST_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(10000))    // toggle between espnow and wifi
      {
        toggleESPNowActive();
      }
      if (p_primaryButton->wasReleasefor(2000))    // select next set of guidance sounds
      {
        rotateToNextGuidanceSounds();
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;        
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;
      }
      else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
      {
        switchToNextDisplayToShow();
      }  

      if (p_secondButton->wasReleasefor(10000)) // toggle sounds on and off
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        toggleSound();
        publishToSilkyStopPlayback();
        notifySoundsOnOffChanged();
      }
      else if (p_secondButton->wasReleasefor(5000)) // start/stop play
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        publishToSilkyTogglePlayback();
      }
      else if (p_secondButton->wasReleasefor(2000)) // cycle volume up and then low at max
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        publishToSilkyCycleVolumeUp();
      }
      else if (p_secondButton->wasReleasefor(50)) // Skip to next track
      {
        showTempDisplayEndTime = millis() + showTempAudioTestDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = AUDIO_ACTION_DISPLAY_TEMP;

        publishToSilkySkipToNextTrack();
      }

      if (display_to_show == AUDIO_ACTION_DISPLAY_TEMP)
      {
        M5.Lcd.fillScreen(TFT_GREEN);
        M5.Lcd.setTextColor(TFT_BLACK,TFT_GREEN);
      }

      break;
    }
    case COMPASS_CALIBRATION_DISPLAY:
    {
      if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
      {
        smoothedCompassCalcInProgress = true;
        switchToNextDisplayToShow();
      }

      if (p_secondButton->wasReleasefor(1000)) // stop calibration
      {
        M5.Lcd.print("Saving\nCalibration");  

        // save current max/min vectors to magnetometer_min and magnetometer_max
        magnetometer_min = calib_magnetometer_min;
        magnetometer_max = calib_magnetometer_max;
        
        smoothedCompassCalcInProgress = true;
        
        delay(10000);
        switchToNextDisplayToShow();
      }
      else if (p_secondButton->wasReleasefor(50)) // start calibration
      {

        M5.Lcd.fillScreen(TFT_BLACK);

        smoothedCompassCalcInProgress = false;
        
        // save current max/min vectors to magnetometer_min and magnetometer_max
        calib_magnetometer_min = vec<double>(initial_min_mag,initial_min_mag,initial_min_mag);
        calib_magnetometer_max = vec<double>(initial_max_mag,initial_max_mag,initial_max_mag);
      }

      break;
    }
    default:
    {
      if (p_primaryButton->wasReleasefor(10000)) // Nav Screens - switch to alternative dive plan
      {
        switchDivePlan();
        M5.Lcd.fillScreen(TFT_BLACK);
        delay(300);
        M5.Lcd.fillScreen(TFT_ORANGE);
        delay(300);
        M5.Lcd.fillScreen(TFT_BLACK);
        goto show_current_target;
      }
      else if (p_primaryButton->wasReleasefor(5000)) // Nav Screens : show lat long for 5 seconds
      {
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration;
        display_to_revert_to = display_to_show;
        display_to_show = SHOW_LAT_LONG_DISPLAY_TEMP;
        M5.Lcd.fillScreen(TFT_BLACK);
      }
      else if (p_primaryButton->wasReleasefor(2000)) // Record Highlight
      {
        recordSurveyHighlight = true;
        recordHighlightExpireTime = millis() + recordHighlightDisplayDuration;
      }
      else if (p_primaryButton->wasReleasefor(buttonPressDurationToChangeScreen))  // change display screen
      {
        switchToNextDisplayToShow();
      }

      if (p_secondButton->wasReleasefor(10000))     // Nav Screens: force reboot on
      {
        // force reboot
        esp_restart();
      }
      else if (p_secondButton->wasReleasefor(5000))     // Nav Screens: goto last dive target (jettie)
      {
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration / 2;

        // goto dive exit (the last target on the list)
        nextWaypoint = currentDiveWaypoints + *p_currentDiveWaypointExit;

        display_to_revert_to = display_to_show;
        display_to_show = NEXT_TARGET_DISPLAY_TEMP;
        firstLoopThroughTempScreen = true;
        M5.Lcd.fillScreen(TFT_BLACK);
      }
      else if (p_secondButton->wasReleasefor(1500))     // Nav Screens: switch to next target
      {
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration / 2;
        // head to next target, if at end of target list go to the top of the list

        if (++nextWaypoint == currentDiveWaypoints + *p_currentDiveWaypointCount)
          nextWaypoint = currentDiveWaypoints;
          
        display_to_revert_to = display_to_show;
        display_to_show = NEXT_TARGET_DISPLAY_TEMP;
        firstLoopThroughTempScreen = true;
        M5.Lcd.fillScreen(TFT_BLACK);
      }
      else if (p_secondButton->wasReleasefor(50))      // Nav Screens: remind diver of current target
      {
        show_current_target:
        showTempDisplayEndTime = millis() + showTempDisplayHoldDuration  / 2;

        // don't change target - remind diver of current target
        display_to_revert_to = display_to_show;
        display_to_show = THIS_TARGET_DISPLAY_TEMP;
        firstLoopThroughTempScreen = true;

        M5.Lcd.fillScreen(TFT_BLACK);
      }
        
      break;
    }
  }
}

bool checkForDualButtonPresses()
{
  static bool action500msReached = false;
  static bool action3000msReached = false;
  static bool action8000msReached = false;
  static bool anyActionTriggered = false;
  static uint32_t lastActionTime = 0;
  
  bool triggered = false;
  
  // Prevent any action within 10 seconds of the last action
  if (millis() - lastActionTime < 10000)
  {
    return false;
  }
  
  // Check if both buttons are currently pressed
  bool bothPressed = BtnGoProTop.isPressed() && BtnGoProSide.isPressed();
  
  if (!bothPressed)
  {
    // Buttons released - check which action to trigger based on how long they were held
    if (!anyActionTriggered && (action500msReached || action3000msReached || action8000msReached))
    {
      anyActionTriggered = true;  // Set this first to prevent re-entry
      lastActionTime = millis();  // Record time of action execution
      
      if (action8000msReached)
      {
        // 8 second hold completed - reboot
        esp_restart();
      }
      else if (action3000msReached)
      {
        // 3 second hold completed - toggle diagnostic screens
        skipDiagnosticDisplays = !skipDiagnosticDisplays;
        saveToEEPROMSkipDiagnosticDisplays();  // Save to EEPROM
        M5.Lcd.setCursor(5, M5.Lcd.height() - 90);
        M5.Lcd.printf("Skip Mode: %s", skipDiagnosticDisplays ? "ON" : "OFF");
        triggered = true;
      }
      else if (action500msReached)
      {
        // 0.5 second hold completed - OTA mode
        static uint32_t lastOTAToggle = 0;
        if (millis() - lastOTAToggle > 5000)  // Prevent rapid OTA toggles
        {
          lastOTAToggle = millis();
          toggleOTAActive();
        }
        triggered = true;
      }
    }
    
    // Only reset flags if both buttons are completely released
    if (BtnGoProTop.isReleased() && BtnGoProSide.isReleased())
    {
      action500msReached = false;
      action3000msReached = false;
      action8000msReached = false;
      anyActionTriggered = false;
    }
    
    return triggered;
  }
  
  // Buttons are pressed - track which thresholds have been reached
  // Only set the highest threshold reached to ensure mutual exclusivity
  if (BtnGoProTop.pressedFor(8000) && BtnGoProSide.pressedFor(8000))
  {
    action8000msReached = true;
    action3000msReached = false;  // Clear lower thresholds
    action500msReached = false;
  }
  else if (BtnGoProTop.pressedFor(3000) && BtnGoProSide.pressedFor(3000))
  {
    action3000msReached = true;
    action500msReached = false;   // Clear lower threshold
  }
  else if (BtnGoProTop.pressedFor(500) && BtnGoProSide.pressedFor(500))
  {
    action500msReached = true;
  }

  return false; // No action triggered while buttons are still pressed
}


bool enableOTAAtStartupIfTopButtonHeld()
{
  if (digitalRead(BUTTON_GOPRO_TOP_GPIO) == false)
  {
    // enable OTA mode immediately at startup
    topGoProButtonActiveAtStartup = true;
    return true;
  }
  else
    return false;
}

bool disableESPNowIfSideButtonHeld()
{
  if (digitalRead(BUTTON_GOPRO_SIDE_GPIO) == false)
  {
    sideGoProButtonActiveAtStartup = true;
    // disable espnow to speedup startup time when testing
    enableESPNowAtStartup = false;      
    // no function currently - could be used for show test track
    return true;
  }
  else
    return false; 
}

// END BUTTON PRESS CODE

#endif