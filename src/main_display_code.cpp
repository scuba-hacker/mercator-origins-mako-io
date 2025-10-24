#ifdef BUILD_INCLUDE_MAIN_DISPLAY_CODE


void switchToNextDisplayToShow()
{
  do {
    display_to_show = (e_mako_displays)((int)display_to_show + 1);

    if (display_to_show > last_display_rotation)
      display_to_show = first_display_rotation;
      
  } while (skipDiagnosticDisplays && 
           (display_to_show == LOCATION_DISPLAY ||

#ifdef ENABLE_ESP_NOW_DISPLAY
            display_to_show == ESP_NOW_DISPLAY ||  
#endif            
            display_to_show == JOURNEY_DISPLAY || 
            display_to_show == AUDIO_TEST_DISPLAY || 
            display_to_show == COMPASS_CALIBRATION_DISPLAY));

  M5.Lcd.fillScreen(TFT_BLACK);
  requestDisplayRefresh=true;
  lastWayMarker = BLACKOUT_MARKER;
  lastWayMarkerChangeTimestamp = 0;
}

void refreshDisplay()
{
  switch (display_to_show)
  {
    case SURVEY_DISPLAY:
    {
      drawSurveyDisplay();
      break;
    }
    case NAV_COMPASS_DISPLAY:
    {
      drawTargetSection();
      drawCompassSection();
      break;
    }
    case NAV_COURSE_DISPLAY:
    {
      drawTargetSection();
      drawCourseSection();
      break;
    }
    case LOCATION_DISPLAY:
    {
      drawLocationStats();
      break;
    }

#ifdef ENABLE_ESP_NOW_DISPLAY
    case ESP_NOW_DISPLAY:
    {
      drawEspNowDisplay();
      break;
    }
#endif

    case JOURNEY_DISPLAY:
    {
      drawJourneyStats();
      break;
    }
    case AUDIO_TEST_DISPLAY:
    {
      drawAudioTest();
      break;
    }
    case COMPASS_CALIBRATION_DISPLAY:
    {
      drawCompassCalibration();
      break;
    }
    case SHOW_LAT_LONG_DISPLAY_TEMP:
    {
      drawLatLong();
      break;
    }
    case NEXT_TARGET_DISPLAY_TEMP:
    {
      drawNextTarget();
      break;
    }
    case THIS_TARGET_DISPLAY_TEMP:
    {
      drawThisTarget();
      break;
    }
    case AUDIO_ACTION_DISPLAY_TEMP:
    {
      drawAudioActionDisplay();
      break;
    }
    default:
    {
      drawNullDisplay();
      break;
    }
  }

  // Used for test/debug when issues with restarts
  if (enableGlobalUptimeDisplay)
  {
    drawPowerOnTimeOverlay();
  }
}

void drawSurveyDisplay()
{
    M5.Lcd.setRotation(0);
    M5.Lcd.setCursor(15, 0);

    if (useGetDepthAsync)
      M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

    M5.Lcd.setTextSize(6);
    if (depth < 10.0)
      M5.Lcd.printf("%.1f\n", depth);    
    else
      M5.Lcd.printf("%.0fm\n", depth);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);
    M5.Lcd.print("\n");
    M5.Lcd.setTextSize(4);

    if (recordHighlightExpireTime != 0)
    {
      if (millis() < recordHighlightExpireTime)
      {
        M5.Lcd.printf("-PIN-\n");
      }
      else
      {
        recordHighlightExpireTime = 0;
        M5.Lcd.print("     \n");
        publishToOceanicPinPlaced(Lat,Lng,magnetic_heading,depth);
      }
    }
    else if (recordBreadCrumbTrail)
    {
        M5.Lcd.print("-REC-\n");
    }
    else
    {
      if  (millis() - journey_clear_period > last_journey_commit_time || journey_distance == 0)
      {
        M5.Lcd.print("     \n");
      }
      else
      {
        const bool surveyScreen = true;
        M5.Lcd.printf("%s\n", getCardinal(journey_course,surveyScreen).c_str());
      }
    }
        
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("\n");

    M5.Lcd.setTextSize(7);

    if (diveTimerRunning == false && minutesDurationDiving == 0)    
      M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);     // dive not started yet
    else if (diveTimerRunning == false && minutesDurationDiving > 0)
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);      // dive finished
    else if (diveTimerRunning == true && whenToStopTimerDueToLackOfDepth == 0)
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);    // dive in progress
    else if (diveTimerRunning == true && whenToStopTimerDueToLackOfDepth > 0)
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);   // dive in progress but not at minimum depth
    else
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);  // shouldn't get here.
      
    M5.Lcd.printf("%2hu'", minutesDurationDiving);

    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.printf("\n\n\n\n   %.1f", water_temperature);

    // print degrees sign
    int16_t x = M5.Lcd.getCursorX(), y = M5.Lcd.getCursorY();
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(x+2, y-4);
    M5.Lcd.print("o");
    M5.Lcd.setCursor(x, y);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("\n\n");

    M5.Lcd.setTextSize(3);

    // Set GPS 'G' character background color based on fix message timeout
    if (!isGPSStreamOk())
      M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);        // Red after 10 seconds no fix
    else if (isGPSTargetShortTimedOut())
      M5.Lcd.setTextColor(TFT_BLACK, TFT_YELLOW);     // Yellow after 3 seconds no fix  
    else
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);      // Black when GPS fixes current

    M5.Lcd.print("G");

    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

    M5.Lcd.printf(" %.0f%% ", humidity);

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    
    if (isInternetUploadOk())
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    
    M5.Lcd.printf("Q");
}


void drawTargetSection()
{
  directionMetric = (display_to_show == NAV_COURSE_DISPLAY ? JOURNEY_COURSE : COMPASS_HEADING);

  // Target Course and distance is shown in Green
  // Journey course and distance over last 10 seconds shown in Red
  // HDOP quality shown in top right corner as square block. Blue best at <=1.
  // Sat count shown underneath HDOP. Red < 4, Orange < 6, Yellow < 10, Blue 10+

  M5.Lcd.setRotation(0);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setCursor(0, 0);

  uint16_t x = 0, y = 0, degree_offset = 0, cardinal_offset = 0, hdop = 0, metre_offset = 0;

  if (GPS_status == GPS_NO_GPS_LIVE_IN_FLOAT && !hasEverReceivedGPSFix)
  {
    M5.Lcd.setCursor(5, 0);
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);

    M5.Lcd.print(" NO\n");
    M5.Lcd.setCursor(21, 41);
    M5.Lcd.print("GPS");

    M5.Lcd.setTextSize(2);
  }
  else if (GPS_status == GPS_NO_FIX_FROM_FLOAT && !hasEverReceivedGPSFix)
  {
    M5.Lcd.setCursor(5, 0);
    M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);

    M5.Lcd.print(" NO\n");
    M5.Lcd.setCursor(21, 41);
    M5.Lcd.print("FIX");

    M5.Lcd.setTextSize(2);
  }
  else // GPS_FIX_FROM_FLOAT or post-first-fix states - use color coding
  {
    M5.Lcd.setCursor(5, 0);
    
    // Set color based on GPS message timeout
    if (!isGPSStreamOk())
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);        // Red after 10 seconds
    else if (isGPSTargetShortTimedOut())
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);     // Yellow after 2.5 seconds
    else
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);      // Green when GPS is current

    // Display heading to target at top with degrees sign suffix
    M5.Lcd.printf("%3.0f", heading_to_target);
    M5.Lcd.setTextSize(2);
    degree_offset = -2;
    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();
    M5.Lcd.setCursor(x, y + degree_offset);
    M5.Lcd.print("o ");

    // Display Cardinal underneath degrees sign
    cardinal_offset = 21;
    M5.Lcd.setCursor(x, y + cardinal_offset);
    M5.Lcd.printf("%s ", getCardinal(heading_to_target).c_str());

    // Display HDOP signal quality as small coloured dot
    hdop = gps.hdop.hdop();
    if (hdop > 10)
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    else if (hdop > 5)
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    else if (hdop > 1)
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);

    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(x + 10, y - 25);
    M5.Lcd.print(".");

    // Display number of satellites
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(x + 8, y + 40);
    if (satellites < 4.0)
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    else if (satellites < 6.0)
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    else if (satellites < 10.0)
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);
      
    M5.Lcd.printf("%2lu", satellites);

    if (recordHighlightExpireTime != 0)
    {
      M5.Lcd.setTextSize(3);
      M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);
      M5.Lcd.setCursor(x, y);
      M5.Lcd.print("\n");
      M5.Lcd.setTextSize(4);
  
      if (millis() < recordHighlightExpireTime)
      {
        M5.Lcd.printf("-PIN-\n");
      }
      else
      {
        recordHighlightExpireTime = 0;
        M5.Lcd.print("     \n");
      }
    }
    else
    {
      // Display distance to target in metres, with by 'm' suffix
      // Set color based on GPS message timeout
      if (!isGPSStreamOk())
        M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);        // Red after 10 seconds
      else if (isGPSTargetShortTimedOut())
        M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);     // Yellow after 2.5 seconds
      else
        M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);      // Green when GPS is current
      
      uint8_t distanceTextSize = 0;

      M5.Lcd.setCursor(x, y);

      if (distance_to_target >= 1000)
      {
        x = M5.Lcd.getCursorX();
        y = M5.Lcd.getCursorY();

        if (distance_to_target >= 10000)
        {
          distanceTextSize=4;
          M5.Lcd.setTextSize(distanceTextSize);
          M5.Lcd.setCursor(x, y + 15);
          M5.Lcd.printf("\n%4.0f", distance_to_target / 1000.0);
          metre_offset = 14;
        }
        else
        {
          distanceTextSize=5;
          M5.Lcd.setTextSize(distanceTextSize);
          M5.Lcd.println("");
    
          x = M5.Lcd.getCursorX();
          y = M5.Lcd.getCursorY();

          M5.Lcd.setCursor(x+5, y);
          M5.Lcd.printf("%2.1f", distance_to_target / 1000.0);
          metre_offset = 22;
        }

        M5.Lcd.setTextSize(2);

        x = M5.Lcd.getCursorX();
        y = M5.Lcd.getCursorY();

        M5.Lcd.setCursor(x, y + metre_offset);
        M5.Lcd.print("km");
        M5.Lcd.setCursor(x, y);
      }
      else  // < 1000m
      {
        distanceTextSize = 5;
        M5.Lcd.setTextSize(distanceTextSize);
        M5.Lcd.printf("\n%3.0f", distance_to_target);
        M5.Lcd.setTextSize(3);

        x = M5.Lcd.getCursorX();
        y = M5.Lcd.getCursorY();

        metre_offset = 14;
        M5.Lcd.setCursor(x, y + metre_offset);
        M5.Lcd.print("m");
        M5.Lcd.setCursor(x, y);
      }

      M5.Lcd.setTextSize(distanceTextSize);
      M5.Lcd.println("");
    }
  }
}

void drawCompassSection()
{
    float directionOfTravel = magnetic_heading;

    // Display Journey Course with degrees suffix
    // this is the direction travelled in last x seconds
    // Black out the Journey Course if no recent movement
    M5.Lcd.setTextSize(5);

    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

    M5.Lcd.printf("%3.0f", directionOfTravel);

    uint16_t x = 0, y = 0, degree_offset = 0;

    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();

    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(x, y + degree_offset);

    // Display degrees
    M5.Lcd.printf("o ");

    // Display Cardinal underneath degrees sign
    const uint16_t cardinal_offset = 21;
    M5.Lcd.setCursor(x, y + cardinal_offset);
    M5.Lcd.printf("%s ", getCardinal(directionOfTravel).c_str());

    // Display temp and humidity
    M5.Lcd.setCursor(x, y);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("\n\n\n%2.1f", temperature);

    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();

    const uint16_t temp_degrees_offset = -2;
    M5.Lcd.setCursor(x + 3, y + temp_degrees_offset);
    M5.Lcd.setTextSize(0);
    M5.Lcd.printf("o ", temperature);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("C %3.0f%%", humidity);

    if (GPS_status == GPS_FIX_FROM_FLOAT)
      refreshDirectionGraphic(directionOfTravel, heading_to_target);

    M5.Lcd.setCursor(40, 146);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextFont(0);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.printf("%3.1fm", depth);

    blackout_journey_no_movement = false;
}


void drawTargetSection_smooth()
{
  directionMetric = (display_to_show == NAV_COURSE_DISPLAY ? JOURNEY_COURSE : COMPASS_HEADING);

  // Target Course and distance is shown in Green
  // Journey course and distance over last 10 seconds shown in Red
  // HDOP quality shown in top right corner as square block. Blue best at <=1.
  // Sat count shown underneath HDOP. Red < 4, Orange < 6, Yellow < 10, Blue 10+

  uint16_t width =  M5.Lcd.height();
  uint16_t height = M5.Lcd.width();
  uint16_t centre_x =  height/2;
  uint16_t centre_y = width/2;

  M5.Lcd.setRotation(0);

  uint16_t x = 0, y = 0, degree_offset = 0, cardinal_offset = 0, hdop = 0, metre_offset = 0;

  if (GPS_status == GPS_NO_GPS_LIVE_IN_FLOAT && !hasEverReceivedGPSFix)
  {
    uint8_t textSize=2;
    uint8_t textFont=4;  
  
    M5.Lcd.setTextSize(textSize);
    M5.Lcd.setTextFont(textFont);
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);

    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("NO",centre_x,0);
    M5.Lcd.drawString("GPS",centre_x,41);
    M5.Lcd.setTextDatum(TL_DATUM);  // default
  }
  else if (GPS_status == GPS_NO_FIX_FROM_FLOAT && !hasEverReceivedGPSFix)
  {
    uint8_t textSize=2;
    uint8_t textFont=4;  

    M5.Lcd.setTextSize(textSize);
    M5.Lcd.setTextFont(textFont);
    M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);

    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("NO",centre_x,0);
    M5.Lcd.drawString("FIX",centre_x,41);

    M5.Lcd.setTextDatum(TL_DATUM);  // default
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextFont(1);
  }
  else // GPS_FIX_FROM_FLOAT or post-first-fix states - use color coding
  {
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextFont(4);

    M5.Lcd.setCursor(5, 0);
    
    // Set color based on GPS message timeout
    if (!isGPSStreamOk())
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);        // Red after 10 seconds
    else if (isGPSTargetShortTimedOut())
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);     // Yellow after 2.5 seconds
    else
      M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);      // Green when GPS is current
    
    // Display heading to target at top with degrees sign suffix
    M5.Lcd.printf("%3.0f", heading_to_target);      // 5,0

    // Degrees Sign
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextFont(2);
    degree_offset = -2;
    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();
    M5.Lcd.setCursor(x, y + degree_offset);         // ?, ?
    M5.Lcd.print("o ");


    // Display Cardinal underneath degrees sign
    cardinal_offset = 21;
    M5.Lcd.setCursor(x, y + cardinal_offset);
    M5.Lcd.printf("%s ", getCardinal(heading_to_target).c_str());

    // Display HDOP signal quality as small coloured dot
    hdop = gps.hdop.hdop();
    if (hdop > 10)
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    else if (hdop > 5)
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    else if (hdop > 1)
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);

    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(x + 10, y - 25);
    M5.Lcd.print(".");

    // Display number of satellites
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(x + 8, y + 40);
    if (satellites < 4.0)
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    else if (satellites < 6.0)
      M5.Lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    else if (satellites < 10.0)
      M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    else
      M5.Lcd.setTextColor(TFT_BLUE, TFT_BLACK);
      
    M5.Lcd.printf("%2lu", satellites);

    if (recordHighlightExpireTime != 0)
    {
      M5.Lcd.setTextSize(3);
      M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);
      M5.Lcd.setCursor(x, y);
      M5.Lcd.print("\n");
      M5.Lcd.setTextSize(4);
  
      if (millis() < recordHighlightExpireTime)
      {
        M5.Lcd.printf("-PIN-\n");
      }
      else
      {
        recordHighlightExpireTime = 0;
        M5.Lcd.print("     \n");
      }
    }
    else
    {
      // Display distance to target in metres, with by 'm' suffix
      // Set color based on GPS message timeout
      if (!isGPSStreamOk())
        M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);        // Red after 10 seconds
      else if (isGPSTargetShortTimedOut())
        M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);     // Yellow after 2.5 seconds
      else
        M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);      // Green when GPS is current
      
      uint8_t distanceTextSize = 0;

      M5.Lcd.setCursor(x, y);

      if (distance_to_target >= 1000)
      {
        x = M5.Lcd.getCursorX();
        y = M5.Lcd.getCursorY();

        if (distance_to_target >= 10000)
        {
          distanceTextSize=4;
          M5.Lcd.setTextSize(distanceTextSize);
          M5.Lcd.setCursor(x, y + 15);
          M5.Lcd.printf("\n%4.0f", distance_to_target / 1000.0);
          metre_offset = 14;
        }
        else
        {
          distanceTextSize=5;
          M5.Lcd.setTextSize(distanceTextSize);
          M5.Lcd.println("");
    
          x = M5.Lcd.getCursorX();
          y = M5.Lcd.getCursorY();

          M5.Lcd.setCursor(x+5, y);
          M5.Lcd.printf("%2.1f", distance_to_target / 1000.0);
          metre_offset = 22;
        }

        M5.Lcd.setTextSize(2);

        x = M5.Lcd.getCursorX();
        y = M5.Lcd.getCursorY();

        M5.Lcd.setCursor(x, y + metre_offset);
        M5.Lcd.print("km");
        M5.Lcd.setCursor(x, y);
      }
      else  // < 1000m
      {
        distanceTextSize = 5;
        M5.Lcd.setTextSize(distanceTextSize);
        M5.Lcd.printf("\n%3.0f", distance_to_target);
        M5.Lcd.setTextSize(3);

        x = M5.Lcd.getCursorX();
        y = M5.Lcd.getCursorY();

        metre_offset = 14;
        M5.Lcd.setCursor(x, y + metre_offset);
        M5.Lcd.print("m");
        M5.Lcd.setCursor(x, y);
      }

      M5.Lcd.setTextSize(distanceTextSize);
      M5.Lcd.println("");
    }
  }
}

void drawCompassSection_smooth()
{
  uint16_t width =  M5.Lcd.width();
  uint16_t height = M5.Lcd.height();
  uint16_t centre_x =  width/2;
  uint16_t centre_y = height/2;

  float directionOfTravel = magnetic_heading;

  // Display Journey Course with degrees suffix
  // this is the direction travelled in last x seconds
  // Black out the Journey Course if no recent movement

  uint16_t headingSize=5;
  uint16_t headingFont=1;

  M5.Lcd.setTextSize(headingSize);
  M5.Lcd.setTextFont(headingFont);
  M5.Lcd.setTextDatum(TL_DATUM);

  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

  uint16_t xpos = 0;
  uint16_t ypos = 82, yposHeadingBottom = 0;

  char direction[4];
  snprintf(direction,sizeof(direction),"%3.0f",directionOfTravel);
  xpos += M5.Lcd.drawString(direction,xpos,ypos);
  yposHeadingBottom = ypos + M5.Lcd.fontHeight(); 

  uint16_t cardinalSize=2;
  uint16_t cardinalFont=1;

  M5.Lcd.setTextSize(cardinalSize);
  M5.Lcd.setTextFont(cardinalFont);
  M5.Lcd.setTextDatum(TL_DATUM);

  // Display degrees
//  M5.Lcd.print("o ",xpos,ypos);
  M5.Lcd.drawString("o ",xpos,ypos);

  // Display Cardinal underneath degrees sign
  const uint16_t cardinal_subscript_offset = 21;

  char cardinal[4];
  snprintf(cardinal,sizeof(cardinal),"%s ",getCardinal(directionOfTravel).c_str());
  M5.Lcd.drawString(cardinal,xpos,ypos + cardinal_subscript_offset);

  ypos = yposHeadingBottom;

  uint16_t x = xpos;
  uint16_t y = ypos;

  uint8_t tempSize=2;
  uint8_t tempFont=1;

  uint8_t degreesSize=2;
  uint8_t degreesFont=1;
  uint16_t degree_offset=2;

  M5.Lcd.setTextSize(tempSize);  M5.Lcd.setTextFont(tempFont);  M5.Lcd.setTextDatum(TL_DATUM);

  xpos = 0;
  xpos += M5.Lcd.drawNumber(temperature,xpos,ypos);

  M5.Lcd.setTextSize(degreesSize);  M5.Lcd.setTextFont(degreesFont);
  xpos += M5.Lcd.drawString("o",xpos,ypos-2);

  M5.Lcd.setTextSize(tempSize);  M5.Lcd.setTextFont(tempFont);
  xpos += M5.Lcd.drawString("C",xpos,ypos);

  char humidity[5];
  snprintf(humidity,sizeof(humidity),"%3.0f%%",humidity);
  M5.Lcd.setTextDatum(TR_DATUM);
  M5.Lcd.drawString(humidity,width,ypos);

  M5.Lcd.setTextDatum(TL_DATUM);

  if (GPS_status == GPS_FIX_FROM_FLOAT)
    refreshDirectionGraphic(directionOfTravel, heading_to_target);

  uint8_t depthSize=3, depthFont=0;
  char depthLabel[6]; snprintf(depthLabel,sizeof(depthLabel),"%3.1fm",depth);
  ypos = 146;
  M5.Lcd.setTextDatum(TC_DATUM);

  M5.Lcd.setTextSize(depthSize);
  M5.Lcd.setTextFont(depthFont);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.drawString(depthLabel,centre_x,ypos);
  M5.Lcd.setTextDatum(TL_DATUM);

  blackout_journey_no_movement = false;
}

void drawCourseSection()
{
    float directionOfTravel = journey_course;

    // Display Journey Course with degrees suffix
    // this is the direction travelled in last x seconds
    // Black out the Journey Course if no recent movement
    M5.Lcd.setTextSize(5);

    M5.Lcd.setTextColor((blackout_journey_no_movement ? TFT_BLACK : TFT_RED), TFT_BLACK);

    M5.Lcd.printf("%3.0f", directionOfTravel);

    uint16_t x = 0, y = 0, degree_offset = 0, cardinal_offset = 0, metre_offset = 0;
    
    x = M5.Lcd.getCursorX();
    y = M5.Lcd.getCursorY();

    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(x, y + degree_offset);

    if (GPS_status == GPS_FIX_FROM_FLOAT)
    {
      // Display small rotating line character to indicate a new journey datapoint has been recorded
      M5.Lcd.printf("o %c", activity_indicator[activity_count]);

      // Display Cardinal underneath degrees sign
      cardinal_offset = 21;
      M5.Lcd.setCursor(x, y + cardinal_offset);
      M5.Lcd.printf("%s ", getCardinal(directionOfTravel).c_str());

      // Display distance travelled during last journey course measurement with 'm' suffix
      M5.Lcd.setCursor(x, y);
      M5.Lcd.setTextSize(5);
      M5.Lcd.printf("\n%3.0f", journey_distance);

      M5.Lcd.setTextSize(3);

      x = M5.Lcd.getCursorX();
      y = M5.Lcd.getCursorY();

      M5.Lcd.setCursor(x, y + metre_offset);
      M5.Lcd.print("m");
      M5.Lcd.setTextSize(5);
      refreshDirectionGraphic(directionOfTravel, heading_to_target);
    }
    else
    {
      // do nothing
    } 
}

void drawNextTarget()
{
  if (firstLoopThroughTempScreen)
  {
    firstLoopThroughTempScreen = false;
    
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextFont(1);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(0, 5);
  
    // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
    M5.Lcd.printf ("Next:\n\n%i) %s", nextWaypoint-currentDiveWaypoints+1, nextWaypoint->_m5label); // needs to change to use navigation waypoints master _m5label
  
    // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
    publishToTigerAndOceanicCurrentTarget(nextWaypoint->_m5label);      // needs to change to send only the waypoint code
  }
    
  if (millis() > showTempDisplayEndTime)
  {
    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;
    // UPDATED NEEDED HERE /////
    publishToTigerAndOceanicCurrentTarget(nextWaypoint->_m5label);      // needs to change to send only the waypoint code
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}

void drawThisTarget()
{
  if (firstLoopThroughTempScreen)
  {
    firstLoopThroughTempScreen = false;
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextFont(1);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(0, 5);
  
    // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
    M5.Lcd.printf ("Towards\n\n%i) %s", nextWaypoint-currentDiveWaypoints+1, nextWaypoint->_m5label);  // needs to change to use navigation waypoints master _m5label
  }
  
  if (millis() > showTempDisplayEndTime)
  {
    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;

    // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
    publishToTigerAndOceanicCurrentTarget(nextWaypoint->_m5label);   // needs to change to use navigation waypoints master _m5label
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}

void drawAudioActionDisplay()
{
  M5.Lcd.setCursor(10, 10);

  switch (audioAction)
  {
    case AUDIO_ACTION_NEXT_SOUND:
      M5.Lcd.println("Silky:\nSkip to Next Sound\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_CYCLE_VOLUME:
      M5.Lcd.printf("Silky:\nCycle volume up %u\n",silkyVolume);
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_SOUNDS_TOGGLE:
      M5.Lcd.println(soundsOn ? "Silky:\nSounds On\n" : "Silky:\nSounds Off\n"); 
      break;

    case AUDIO_ACTION_PLAYBACK_TOGGLE:
      M5.Lcd.println("Silky:\nToggle Playback\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_STOP_PLAYBACK:
      M5.Lcd.println("Silky:\nStop Playback\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_SET_VOLUME:
      M5.Lcd.println("Silky:\nSet Volume\n");
      displayESPNowSendDataResult(ESPNowSendResult);
      break;

    case AUDIO_ACTION_ROTATE_SOUND_SET:
    {
      M5.Lcd.println("Silky:\nSound Set To\n");
      M5.Lcd.println(*currentSoundSet);
      break;
    }
    
    case AUDIO_ACTION_NONE:
      // shouldn't get here
      M5.Lcd.println("Silky:\nAudio Action None\n");
      break;

    case RESET_ESPNOW_SEND_RESULT:
      // shouldn't get here
      M5.Lcd.println("Silky:\nAudio Action Reset\n");
      break;

    default:
      // shouldn't get here
      M5.Lcd.println("Silky:\nUndefined Audio Action\n");
      break;
  }

  if (millis() > showTempDisplayEndTime)
  {
    ESPNowSendResult = RESET_ESPNOW_SEND_RESULT;
    
    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
  }
}

void drawLatLong()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0, 0);
  /*
          if (sendInstantMessageEmergencyNowFlag == true)
          {
            M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
            M5.Lcd.printf("SOS MSG SENT\n", Lat);
            sleep(3000);
          }
          else
          {
            M5.Lcd.setTextColor(TFT_BLUE, TFT_YELLOW);
            M5.Lcd.printf(" MSG SENT! \n", Lat);
          }
  */

  M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
  M5.Lcd.printf("Location Here\n");
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.printf("N:%.7f\n", Lat);
  M5.Lcd.printf("E:%.7f\n", Lng);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.printf("%.1fm %hu mins\n", depth, minutesDurationDiving);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.printf("%02d:%02d:%02d %02d%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month());

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  if (millis() > showTempDisplayEndTime)
  {
    // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
    publishToTigerAndOceanicLocationAndTarget(nextWaypoint->_m5label); 

    showTempDisplayEndTime = disabledTempDisplayEndTime;
    display_to_show = display_to_revert_to;
    M5.Lcd.fillScreen(TFT_BLACK);
  }
}
    
void drawLocationStats()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

  M5.Lcd.setCursor(5, 0);
  M5.Lcd.printf("La:%.6f   ", Lat);
  M5.Lcd.setCursor(5, 17);
  M5.Lcd.printf("Lo:%.6f   ", Lng);

  M5.Lcd.setCursor(5, 34);
  M5.Lcd.printf("Depth:%.0f m s:%hu ^%hu  ", depth,sensor_acquisition_time, max_sensor_acquisition_time);
//  M5.Lcd.printf("OverTgt:%d s:%hu ^%hu  ", overrideTarget,sensor_acquisition_time, max_sensor_acquisition_time);

  M5.Lcd.setCursor(5, 51);
  if (WiFi.status() == WL_CONNECTED)
  {
    M5.Lcd.printf("%s ", WiFi.localIP().toString().c_str());
    if (otaActive)
      M5.Lcd.println("OTA");
    else
      M5.Lcd.println("");
  }
  else
  {
    if (ESPNowActive)
    {
      M5.Lcd.print(WiFi.macAddress());
    }
    else
      M5.Lcd.print("WiFi & ESPNow: Off");
  }

  M5.Lcd.setCursor(5, 68);
 // M5.Lcd.printf("Tiger: %s\n",tigerMessage);
//  M5.Lcd.printf("Colour: %u %hu   \n",brightLightEvents, clear_light);
  M5.Lcd.printf("Baud: %i\n",UPLINK_BAUD_RATE);

  M5.Lcd.setCursor(5, 85);
//  M5.Lcd.printf("T-Reeds: %s\n",tigerReeds);
  M5.Lcd.printf("Rx/Tx: %lu / %lu\n",bytesReceivedFromLemon,bytesTransmittedToLemon);
  
  M5.Lcd.setCursor(5, 102);
  multi_heap_info_t info;
  heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task

  M5.Lcd.printf("Heap/Largest: %lu / %lu \n",info.total_free_bytes / 1024,info.largest_free_block / 1024);

//  M5.Lcd.printf("Silky: %s",silkyMessage);  
//  M5.Lcd.printf("T: (%d)\n%s", (int)(nextWaypoint - currentDiveWaypoints)+1, nextWaypoint->_m5label);
}

void drawEspNowDisplay()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(1); // Smaller text to fit more info

  // test every 5 seconds whilst on this screen
  verifyAllEspNowConnections();

  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setCursor(5, 0);
  M5.Lcd.printf("ESP Now Active: %i T: %lu\n", (int)ESPNowActive,millis() / 1000);
  M5.Lcd.printf("T/O/S:   %i %i %i \n",isPairedWithTiger, isPairedWithOceanic, isPairedWithSilky);
  M5.Lcd.printf("Ping Resp:   %i %i\n",pingResponseReceivedFromTiger, pingResponseReceivedFromUnknown);
  M5.Lcd.printf("ESP Msg Rx: %i %i\n",espMsgReceived, espPingMsgReceived);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.printf("MAKO AP MAC: %s\n",WiFi.softAPmacAddress().c_str());
  M5.Lcd.printf("Paired MACs:\n");

  M5.Lcd.setCursor(5, 48);
  if (isPairedWithTiger && ESPNow_tiger_peer.channel == ESPNOW_CHANNEL) {
    M5.Lcd.printf("Tiger: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  ESPNow_tiger_peer.peer_addr[0], ESPNow_tiger_peer.peer_addr[1], ESPNow_tiger_peer.peer_addr[2], 
                  ESPNow_tiger_peer.peer_addr[3], ESPNow_tiger_peer.peer_addr[4], ESPNow_tiger_peer.peer_addr[5]);
  } else {
    M5.Lcd.printf("Tiger: Not Paired\n");
  }

  M5.Lcd.setCursor(5, 60);
  if (isPairedWithOceanic && ESPNow_oceanic_peer.channel == ESPNOW_CHANNEL) {
    M5.Lcd.printf("Oceanic: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  ESPNow_oceanic_peer.peer_addr[0], ESPNow_oceanic_peer.peer_addr[1], ESPNow_oceanic_peer.peer_addr[2], 
                  ESPNow_oceanic_peer.peer_addr[3], ESPNow_oceanic_peer.peer_addr[4], ESPNow_oceanic_peer.peer_addr[5]);
  } else {
    M5.Lcd.printf("Oceanic: Not Paired\n");
  }
}

void drawJourneyStats()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

  M5.Lcd.setCursor(5, 0);
  M5.Lcd.printf("V:%.2fV I:%.0fmA  ", M5.Axp.GetVBusVoltage(), M5.Axp.GetVBusCurrent());
  M5.Lcd.setCursor(5, 17);


  M5.Lcd.printf("OTA:%hu Uplink:%hu", otaActive,  enableUplinkComms);

  M5.Lcd.setCursor(5, 34);  
  M5.Lcd.printf("Wifi:%hu %s", WiFi.status() == WL_CONNECTED, (ESPNowActive ? "ESPNow On" : ssid_connected.c_str()));
  M5.Lcd.setCursor(5, 51);
  M5.Lcd.printf("Fix:%lu Up:%lu", fixCount, uplinkMessageCount);
  M5.Lcd.setCursor(5, 68);
  M5.Lcd.printf("No Fix:%lu", noFixCount);
  M5.Lcd.setCursor(5, 85);
  M5.Lcd.printf("chk+:%lu chk-:%lu", newPassedChecksum, newFailedChecksum); 
//  M5.Lcd.printf("tc:%.0f d:%.0f  ", heading_to_target, distance_to_target);
  M5.Lcd.setCursor(5, 102);
  M5.Lcd.printf("%02d:%02d:%02d UTC", gps.time.hour(), gps.time.minute(), gps.time.second());
  M5.Lcd.setCursor(5, 119);
  M5.Lcd.printf("Uptime:%.1f", ((float)millis() / 1000.0));
}

void drawCompassCalibration()
{
  M5.Lcd.setRotation(0);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

  if (smoothedCompassCalcInProgress == false)
  {
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);

    M5.Lcd.printf("Calib Start\n           \n");

    sensors_event_t magEvent;

    // Collect calibration data for soft iron compensation
    collectCalibrationSample(magEvent);

    if (magEvent.magnetic.x < calib_magnetometer_min.x) calib_magnetometer_min.x = magEvent.magnetic.x;
    if (magEvent.magnetic.x > calib_magnetometer_max.x) calib_magnetometer_max.x = magEvent.magnetic.x;

    if (magEvent.magnetic.y < calib_magnetometer_min.y) calib_magnetometer_min.y = magEvent.magnetic.y;
    if (magEvent.magnetic.y > calib_magnetometer_max.y) calib_magnetometer_max.y = magEvent.magnetic.y;

    if (magEvent.magnetic.z < calib_magnetometer_min.z) calib_magnetometer_min.z = magEvent.magnetic.z;
    if (magEvent.magnetic.z > calib_magnetometer_max.z) calib_magnetometer_max.z = magEvent.magnetic.z;

    M5.Lcd.printf("x %.3f\n  %.3f\n\n",calib_magnetometer_min.x,calib_magnetometer_max.x);
    M5.Lcd.printf("y %.3f\n  %.3f\n\n",calib_magnetometer_min.y,calib_magnetometer_max.y);
    M5.Lcd.printf("z %.3f\n  %.3f\n\n",calib_magnetometer_min.z,calib_magnetometer_max.z);

    // Show calibration data collection status
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.printf("Samples:\b %d/%d", calibrationSampleCount, maxCalibrationSamples);
  }
  else
  {
    if (setHardIronOffsetsInHardwareRegisters)
    {
      M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
      M5.Lcd.println("To\ncalibrate\ndisable\nhardware\nregisters\n");
      M5.Lcd.println("Current:\nSetup\n\n");
      M5.Lcd.println(getSpoolSetupDescription(spool_setup,true));
    }
    else
    {
      M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
      M5.Lcd.printf("Bottom\nbutton\nto start\ncalibration\n\n");    
      M5.Lcd.printf("x %.3f\n  %.3f\n\n",calib_magnetometer_min.x,calib_magnetometer_max.x);
      M5.Lcd.printf("y %.3f\n  %.3f\n\n",calib_magnetometer_min.y,calib_magnetometer_max.y);
      M5.Lcd.printf("z %.3f\n  %.3f\n\n",calib_magnetometer_min.z,calib_magnetometer_max.z);
    }
  }
}

void drawAudioTest()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0, 0);

  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.printf("AUDIO TEST\nESPNow ");

  if (ESPNowActive)
    M5.Lcd.println("On");
  else
    M5.Lcd.println("Off");

  M5.Lcd.println(isPairedWithSilky ? "Audio Paired" : "Not Paired");

  if (isPairedWithSilky && soundsOn)
  {
    M5.Lcd.printf("Sounds On (%i)",silkyVolume);
  }
  else
  {
    M5.Lcd.println("Sounds Off");
  }

  if (writeLogToSerial)
  {
    USB_SERIAL_PRINTLN("Toggle ESPNow: Top 10s\n");
    USB_SERIAL_PRINTLN("Start/Stop Play: Side 0.5s\n");
    USB_SERIAL_PRINTLN("Vol cycle: Side 2s\n");
    USB_SERIAL_PRINTLN("Next Track: Side 5s\n");
  }
}

void drawNullDisplay()
{
  M5.Lcd.setCursor(5, 17);
  M5.Lcd.printf("NULL DISPLAY");
}

void drawPowerOnTimeOverlay()
{
  // overlay count up / power on time in seconds.
  M5.Lcd.setCursor(0, SCREEN_WIDTH - 15);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.printf(" Uptime: %.1f ", ((float)millis() / 1000.0));
}

void showOTARecoveryScreen()
{
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(TFT_GREEN);
  M5.Lcd.setCursor(5,5);
  M5.Lcd.setTextColor(TFT_BLACK,TFT_GREEN);
  M5.Lcd.setTextSize(3);

  if (otaActive)
  {
    M5.Lcd.println("  Mako OTA\n    Ready\n");
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("");
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("%s  Build:\n",WiFi.localIP().toString().c_str());
  }
  else
  {
    M5.Lcd.print("OTA\nOff\nNo WiFi\n\n");
  }

  M5.Lcd.setTextSize(1);
  M5.Lcd.println("");
  M5.Lcd.setTextSize(2);
  M5.Lcd.println(buildTimestamp);
}

void refreshGlobalStatusDisplay()
{
  if (power_up_no_fix_byte_loop_count > 0)
  {
    // Bytes have been received from the Float UART/Serial but no fix has been received since power on.
    // The 'No Fix' only shown on first acquisition.
    M5.Lcd.setCursor(55, 5);
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("No Fix\n");
    M5.Lcd.setCursor(110, 45);

    if (millis() > next_global_status_display_update)
    {
      next_global_status_display_update = millis() + global_status_display_update_period;
      activity_count = (activity_count+1) % 4;
    }

    M5.Lcd.printf("%c", activity_indicator[activity_count]);
  }
  else if (power_up_no_fix_byte_loop_count != -1)
  {
    // No GPS is reported when no bytes have ever been received on the Float UART/Serial.
    // Once messages start being received, this is blocked as it is normal
    // to have gaps in the stream. There is no indication if GPS stream hangs
    // after first byte received, eg no bytes within 10 seconds.
    M5.Lcd.setCursor(55, 5);
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("No GPS\n");
    M5.Lcd.setCursor(110, 45);

    if (millis() > next_global_status_display_update)
    {
      next_global_status_display_update = millis() + global_status_display_update_period;
      activity_count = (activity_count+1) % 4;
    }

    M5.Lcd.printf("%c", activity_indicator[activity_count]);
  }
  else
  {
    // do nothing - the two conditions above are dealing with initial conditions before any fix/msg received from float serial
  }
}

const int32_t durationBetweenGuidanceSounds = 3000;

void refreshDirectionGraphic( float directionOfTravel,  float headingToTarget)
{
  if (!enableNavigationGraphics)
    return;

  // Calculate whether the diver needs to continue straight ahead,
  // rotate clockwise or rotate anticlockwise and u pdate graphic.
  // Blacks out if no journey recorded.
  const int16_t edgeBound = 25;    // If journey course within +- 25 degrees of target heading then go ahead

  int16_t normaliser = (int16_t)(directionOfTravel);

  int16_t d = (int16_t)directionOfTravel - normaliser;  // directionofTravel normalised to zero
  int16_t t = (int16_t)headingToTarget - normaliser;    // headingToTarget normalised.
  if (t > 180)                  // normalise to range -179 to +180 degrees
    t -= 360;
  else if (t <= -180)
    t += 360;

  int16_t e1 = t - edgeBound;   // left-most edge to target
  if (e1 > 180)                 // normalise to range -179 to +180 degrees
    e1 -= 360;
  else if (e1 <= -180)
    e1 += 360;

  int16_t e2 = t + edgeBound;   // right-most edge to target
  if (e2 > 180)                 // normalise to range -179 to +180 degrees
    e2 -= 360;
  else if (e2 <= -180)
    e2 += 360;

  int16_t o = t + 180;          // opposite heading to target
  if (o > 180)                  // normalise to range -179 to +180 degrees
    o -= 360;
  else if (o <= -180)
    o += 360;


  if (blackout_journey_no_movement)
  {
    goBlackout();
    lastWayMarker = BLACKOUT_MARKER;
  }
  else
  {
    if (millis() - lastWayMarkerChangeTimestamp > durationBetweenGuidanceSounds)
    {
      lastWayMarkerChangeTimestamp = millis();

      if (e1 <= d && d <= e2)     // scenario 1
      {
        newWayMarker = GO_AHEAD_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAhead();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_AHEAD);
      }
      
      else if (e1 > e2)           // scenario 4
      {
        newWayMarker = GO_TURN_AROUND_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goTurnAround();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_TURN_AROUND);
      }
      else if (o <= d && d <= e1) // scenario 2
      {
        newWayMarker = GO_CLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_CLOCKWISE);
      }
      else if (e2 <= d && d <= o) // scenario 3
      {
        newWayMarker = GO_ANTICLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAntiClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_ANTICLOCKWISE);
      }
      else if (o <= d && d <= e1) // scenario 5
      {
        newWayMarker = GO_CLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_CLOCKWISE);
      }
      else if (e2 <= d && d <= o) // scenario 6
      {
        newWayMarker = GO_ANTICLOCKWISE_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goAntiClockwise();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_ANTICLOCKWISE);
      }
      else
      {
        newWayMarker = UNKNOWN_MARKER;

        if (lastWayMarker != newWayMarker)
        {
          goUnknown();
          lastWayMarker = newWayMarker;
        }
        publishToSilkyPlayAudioGuidance(SFX_UNKNOWN);
      }
    }
  }
}

void goBlackout()
{
  drawGoUnknown(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoAhead(false);
  drawGoTurnAround(false);
}

void goAhead()
{
  drawGoUnknown(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoTurnAround(false);
  drawGoAhead(true);
}

void goTurnAround()
{
  drawGoUnknown(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoAhead(false);
  drawGoTurnAround(true);
}

void goClockwise()
{
  drawGoUnknown(false);
  drawGoAhead(false);
  drawGoAntiClockwise(false);
  drawGoTurnAround(false);
  drawGoClockwise(true);
}

void goUnknown()
{
  drawGoAhead(false);
  drawGoTurnAround(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(false);
  drawGoUnknown(true);
}

void goAntiClockwise()
{
  drawGoUnknown(false);
  drawGoAhead(false);
  drawGoTurnAround(false);
  drawGoClockwise(false);
  drawGoAntiClockwise(true);
}

void drawGoAhead(const bool show)
{
  uint32_t colour = (show ? TFT_GREEN : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(0, screenHeight,
                      screenWidth, screenHeight,
                      screenWidth / 2, screenHeight - 70,
                      colour);

  if (show)
  {
    M5.Lcd.setCursor(40, 220);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
    M5.Lcd.print("Ahead");
  }
}

void drawGoTurnAround(const bool show)
{
  uint32_t colour = (show ? TFT_CYAN : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(0, screenHeight - 70,
                      screenWidth, screenHeight - 70,
                      screenWidth / 2, screenHeight,
                      colour);

  if (show)
  {
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_BLACK, TFT_CYAN);
    M5.Lcd.setCursor(40, 180);
    M5.Lcd.print("About");
    M5.Lcd.setCursor(45, 200);
    M5.Lcd.print("Turn");
  }
}

void drawGoAntiClockwise(const bool show)
{
  uint32_t colour = (show ? TFT_RED : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(0, screenHeight,
                      0, screenHeight - 70,
                      screenWidth / 2, screenHeight,
                      colour);

  if (show)
  {
    M5.Lcd.setCursor(5, 220);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.print("Anti");
  }
}

void drawGoClockwise(const bool show)
{
  uint32_t colour = (show ? TFT_BLUE : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.fillTriangle(screenWidth, screenHeight,
                      screenWidth, screenHeight - 70,
                      screenWidth / 2, screenHeight,
                      colour);
  if (show)
  {
    M5.Lcd.setCursor(96, 220);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    M5.Lcd.print("Clk");
  }
}

void drawGoUnknown(const bool show)
{
  uint32_t colour = (show ? TFT_MAGENTA : TFT_BLACK);
  const int screenWidth = M5.Lcd.width();
  const int screenHeight = M5.Lcd.height();

  M5.Lcd.setCursor(screenWidth / 2, 190);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setTextColor(colour, TFT_BLACK);
  M5.Lcd.print("!");
}

#endif
