
#ifdef BUILD_INCLUDE_MAIN_TELEMETRY_CODE


// TELEMETRY FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void performUplinkTasks()
{
  if (enableUplinkComms)
  {
    fp_sendUplinkMessage();

    uplinkMessageCount++;
  }    
}

void sendNoUplinkTelemetryMessages()
{
  /// do nothing
}

void sendFullUplinkTelemetryMessage()
{
  sendUplinkTelemetryMessageV5();
}

enum e_user_action{NO_USER_ACTION=0x0000, HIGHLIGHT_USER_ACTION=0x0001,RECORD_BREADCRUMB_TRAIL_USER_ACTION=0x0002,LEAK_DETECTED_USER_ACTION=0x0004};

uint16_t getOneShotUserActionForUplink()
{
  int userAction = NO_USER_ACTION;
  
  if (recordSurveyHighlight || recordHighlightExpireTime != 0)
  {
    // log the highlight in all messages for the time the highlight is shown on the screen (5 seconds)
    recordSurveyHighlight = false;
    userAction |= HIGHLIGHT_USER_ACTION;
  }

  if (recordBreadCrumbTrail)
  {
    userAction |= RECORD_BREADCRUMB_TRAIL_USER_ACTION;
  }

  if (sendLeakDetectedToLemon)
  {
    userAction |= LEAK_DETECTED_USER_ACTION;
  }

  return (uint16_t)userAction;
}

// send number of good messages received and number of bad messages received. 16 bit for both.
void sendUplinkTelemetryMessageV5()
{
  delay(lingerTimeMsBeforeUplink);

//  if (millis() > latestFixTimeStamp + lingerTimeMsBeforeUplink)
//  {
    latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

    // this is 57 words, 114 bytes including checksum (56 metrics)

    // fixed format

    uint16_t uplink_length = 114;   // bytes to transmit in this message - including length and checksum. 57 x 2 byte words == 114 bytes
    uint16_t uplink_msgtype = 0;   // 0 is full fat telemetry message zero!
    uint16_t uplink_depth = depth * 10.0;
    uint16_t uplink_water_pressure = water_pressure * 100.0;

    uint16_t uplink_water_temperature = water_temperature * 10.0;
    uint16_t uplink_enclosure_temperature = temperature * 10.0;
    uint16_t uplink_enclosure_humidity = humidity * 10.0;
    uint16_t uplink_enclosure_air_pressure = air_pressure * 10.0;

    uint16_t uplink_heading = magnetic_heading * 10.0;

    const char*    uplink_target_code = nextWaypoint->_label;
    uint16_t uplink_heading_to_target = heading_to_target * 10.0;
    uint16_t uplink_distance_to_target = (distance_to_target < 6499 ? distance_to_target * 10.0 : 64999);
    uint16_t uplink_journey_course = journey_course * 10.0;
    uint16_t uplink_journey_distance = journey_distance * 100.0;
    
    uint16_t uplink_mako_seconds_on = millis() / 1000.0 / 60.0;
    uint16_t uplink_mako_user_action = getOneShotUserActionForUplink();

    uint16_t uplink_mako_bad_checksum_msgs = newFailedChecksum;

    uint16_t uplink_mako_usb_voltage = M5.Axp.GetVBusVoltage() * 1000.0;
    uint16_t uplink_mako_usb_current = M5.Axp.GetVBusCurrent() * 100.0;


    float uplink_mako_lsm_acc_x = accelerometer_vector.x;
    float uplink_mako_lsm_acc_y = accelerometer_vector.y;
    float uplink_mako_lsm_acc_z = accelerometer_vector.z;

    float uplink_mako_imu_gyro_x = imu_gyro_vector.x;
    float uplink_mako_imu_gyro_y = imu_gyro_vector.y;
    float uplink_mako_imu_gyro_z = imu_gyro_vector.z;

    float uplink_mako_imu_lin_acc_x = imu_lin_acc_vector.x;
    float uplink_mako_imu_lin_acc_y = imu_lin_acc_vector.y;
    float uplink_mako_imu_lin_acc_z = imu_lin_acc_vector.z;

    float uplink_mako_imu_rot_acc_x = imu_rot_acc_vector.x;
    float uplink_mako_imu_rot_acc_y = imu_rot_acc_vector.y;
    float uplink_mako_imu_rot_acc_z = imu_rot_acc_vector.z;

    uint16_t uplink_mako_good_checksum_msgs = newPassedChecksum;

    uint16_t uplink_flags = sendInstantMessageLocationNowFlag | (sendInstantMessageEmergencyNowFlag << 1);

    uint16_t* nextMetric = telemetryMessage;

    // 21x16 bit words (21 x 2 byte metrics)
    *(nextMetric++) = uplink_length;
    *(nextMetric++) = uplink_msgtype;
    *(nextMetric++) = uplink_depth;
    *(nextMetric++) = uplink_water_pressure;
    *(nextMetric++) = uplink_water_temperature;
    *(nextMetric++) = uplink_enclosure_temperature;
    *(nextMetric++) = uplink_enclosure_humidity;
    *(nextMetric++) = uplink_enclosure_air_pressure;
    *(nextMetric++) = uplink_heading;
    *(nextMetric++) = uplink_heading_to_target;
    *(nextMetric++) = uplink_distance_to_target;
    *(nextMetric++) = uplink_journey_course;
    *(nextMetric++) = uplink_journey_distance;
    
    switch (display_to_show)
    {
      case NAV_COMPASS_DISPLAY: displayLabel[0] = navCompassDisplayLabel[0]; displayLabel[1] = navCompassDisplayLabel[1]; break;
      case NAV_COURSE_DISPLAY:  displayLabel[0] = navCourseDisplayLabel[0]; displayLabel[1] = navCourseDisplayLabel[1]; break;
      case LOCATION_DISPLAY:    displayLabel[0] = locationDisplayLabel[0]; displayLabel[1] = locationDisplayLabel[1]; break;
      case ESP_NOW_DISPLAY:    displayLabel[0] = espNowDisplayLabel[0]; displayLabel[1] = espNowDisplayLabel[1]; break;
      case JOURNEY_DISPLAY:     displayLabel[0] = journeyDisplayLabel[0]; displayLabel[1] = journeyDisplayLabel[1]; break;
      case SHOW_LAT_LONG_DISPLAY_TEMP: displayLabel[0] = showLatLongDisplayLabel[0]; displayLabel[1] = showLatLongDisplayLabel[1]; break;
      case AUDIO_TEST_DISPLAY:  displayLabel[0] = audioTestDisplayLabel[0]; displayLabel[1] = audioTestDisplayLabel[1]; break;
      case COMPASS_CALIBRATION_DISPLAY: displayLabel[0] = compassCalibrationDisplayLabel[0]; displayLabel[1] = compassCalibrationDisplayLabel[1]; 
      case SURVEY_DISPLAY:      displayLabel[0] = surveyDisplayLabel[0]; displayLabel[1] = surveyDisplayLabel[1]; break;
      case NEXT_TARGET_DISPLAY_TEMP: displayLabel[0] = nextWaypointDisplayLabel[0]; displayLabel[1] = nextWaypointDisplayLabel[1]; break;
      case THIS_TARGET_DISPLAY_TEMP: displayLabel[0] = thisTargetDisplayLabel[0]; displayLabel[1] = thisTargetDisplayLabel[1]; break;      
      case AUDIO_ACTION_DISPLAY_TEMP: displayLabel[0] = thisTargetDisplayLabel[0]; displayLabel[1] = thisTargetDisplayLabel[1]; break;      
      default:                  displayLabel[0] = audioActionDisplayLabel[0]; displayLabel[1] = audioActionDisplayLabel[1]; break;
    }

    *(nextMetric++) = (uint16_t)(displayLabel[0]) + (((uint16_t)(displayLabel[1])) << 8); // which display is shown on the console

    *(nextMetric++) = uplink_mako_seconds_on;
    *(nextMetric++) = uplink_mako_user_action;
    *(nextMetric++) = uplink_mako_bad_checksum_msgs;
    *(nextMetric++) = uplink_mako_usb_voltage;
    *(nextMetric++) = uplink_mako_usb_current;

    *(nextMetric++) = (uint16_t)(uplink_target_code[0]) + (((uint16_t)(uplink_target_code[1])) << 8);
    *(nextMetric++) = (uint16_t)(uplink_target_code[2]) + (((uint16_t)(uplink_target_code[3])) << 8);
    

    *(nextMetric++) = minimum_sensor_read_time;
    *(nextMetric++) = lingerTimeMsBeforeUplink;
    *(nextMetric++) = sensor_acquisition_time;
    *(nextMetric++) = max_sensor_acquisition_time;
    *(nextMetric++) = actual_sensor_acquisition_time;
    *(nextMetric++) = max_actual_sensor_acquisition_time;

    char* p = NULL;

    // 3x32 bit words (floats) - (6 x 2 byte metrics)
    p = (char*) &uplink_mako_lsm_acc_x;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_lsm_acc_y;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_lsm_acc_z;         *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);

    // 9x32 bit words (floats) - (18 x 2 byte metrics)
    p = (char*) &uplink_mako_imu_gyro_x;        *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_gyro_y;        *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_gyro_z;        *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_lin_acc_x;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_lin_acc_y;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_lin_acc_z;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_rot_acc_x;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_rot_acc_y;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);
    p = (char*) &uplink_mako_imu_rot_acc_z;     *(nextMetric++) = *(p++) | (*(p++) << 8); *(nextMetric++) = *(p++) | (*(p++) << 8);

    // 2 x 16 bit words (2 x 2 byte metrics)
    *(nextMetric++) = uplink_mako_good_checksum_msgs;
    *(nextMetric++) = newWayMarker; // guidance graphic enum

    switch (newWayMarker)         // guidance label
    {
      case BLACKOUT_MARKER:  newWayMarkerLabel[0] = shortBlackOut[0]; newWayMarkerLabel[1] = shortBlackOut[1]; break;
      case GO_ANTICLOCKWISE_MARKER:  newWayMarkerLabel[0] = shortAntiClockwise[0]; newWayMarkerLabel[1] = shortAntiClockwise[1]; break;
      case GO_AHEAD_MARKER:  newWayMarkerLabel[0] = shortAhead[0]; newWayMarkerLabel[1] = shortAhead[1]; break;
      case GO_CLOCKWISE_MARKER:  newWayMarkerLabel[0] = shortClockwise[0]; newWayMarkerLabel[1] = shortClockwise[1]; break;
      case GO_TURN_AROUND_MARKER:  newWayMarkerLabel[0] = shortTurnAround[0]; newWayMarkerLabel[1] = shortTurnAround[1]; break;
      case UNKNOWN_MARKER:  newWayMarkerLabel[0] = shortUnknownMarker[0]; newWayMarkerLabel[1] = shortUnknownMarker[1]; break;
      default: newWayMarkerLabel[0] = shortUndefinedMarker[0]; newWayMarkerLabel[1] = shortUndefinedMarker[1]; break; // undefined
    }

    switch (directionMetric)
    {
      case JOURNEY_COURSE: directionMetricLabel[0] = shortJourneyCourseDirectionMetric[0]; directionMetricLabel[1] = shortJourneyCourseDirectionMetric[1]; break;
      case COMPASS_HEADING: directionMetricLabel[0] = shortCompassHeadingDirectionMetric[0]; directionMetricLabel[1] = shortCompassHeadingDirectionMetric[1]; break;
      default: directionMetricLabel[0] = shortUndefinedDirectionMetric[0]; directionMetricLabel[1] = shortUndefinedDirectionMetric[1]; break;
    }

    // 2 x 16 bit words (2 x 2 byte metrics)
    *(nextMetric++) = (uint16_t)(newWayMarkerLabel[0]) + (((uint16_t)(newWayMarkerLabel[1])) << 8); // guidance graphic
    *(nextMetric++) = (uint16_t)(directionMetricLabel[0]) + (((uint16_t)(directionMetricLabel[1])) << 8); // whether course or compass heading is shown on display

    // 2 x 16 bit words (1 x 2 byte metric plus 2 byte checksum)
    *(nextMetric++) = uplink_flags;
    *(nextMetric++) = 0;   // initialise checksum to zero prior to computing checksum

    // calculate and store checksum by xor'ing all words, except the 
    uint16_t checksum = 0;
    uint16_t wordsToXOR = uplink_length / 2;
    for (int i = 0; i < wordsToXOR; i++)
    {
      checksum ^= telemetryMessage[i];
    }

    telemetryMessage[wordsToXOR-1] = checksum;

    float_serial.write(uplink_preamble_pattern2);

    float_serial.write((char*)telemetryMessage, uplink_length);

    bytesTransmittedToLemon += sizeof(uplink_preamble_pattern2) + uplink_length;

    // clear flags
    if (sendInstantMessageLocationNowFlag == true)
      sendInstantMessageLocationNowFlag = false;

    if (sendInstantMessageEmergencyNowFlag == true)
      sendInstantMessageEmergencyNowFlag = false;
//  }
}

void sendUplinkTestMessage()
{
  // if 5ms after fix received then send the uplink msg, only get here if no bytes received from Float Serial.
  const uint32_t lingerTimeMsBeforeUplink = 5;
  if (millis() > latestFixTimeStamp + lingerTimeMsBeforeUplink)
  {
    latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;

    // now send one uplink msg, cycling around the 5 msgs.

    float_serial.write(uplink_preamble_pattern);

    for (int i = 0; i < 4; i++)
    {
      float_serial.write(uplinkTestMessages[nextUplinkMessage]);
    }
    nextUplinkMessage = (nextUplinkMessage + 1) % 4;
  }
}

void toggleUplinkMessageProcessAndSend()
{
  enableUplinkComms = !enableUplinkComms;

  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(20, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  if (enableUplinkComms)
    M5.Lcd.println("Uplink On");
  else
    M5.Lcd.println("Uplink Off");

  delay(2000);

  M5.Lcd.fillScreen(TFT_BLACK);
}

#endif