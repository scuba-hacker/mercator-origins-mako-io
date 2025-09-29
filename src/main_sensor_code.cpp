#ifdef BUILD_INCLUDE_MAIN_SENSOR_CODE

// Calibration data collection functions
void startCalibrationDataCollection()
{
  collectingCalibrationData = true;
  calibrationSampleCount = 0;
  lastCalibrationSampleTime = 0;
  USB_SERIAL_PRINTLN("Started calibration data collection");
}

void stopCalibrationDataCollection()
{
  collectingCalibrationData = false;
  USB_SERIAL_PRINTF("Stopped calibration data collection. Collected %d samples\n", calibrationSampleCount);
}

void collectCalibrationSample()
{
  if (!collectingCalibrationData || calibrationSampleCount >= maxCalibrationSamples) {
    return;
  }

  uint32_t currentTime = millis();
  if (currentTime - lastCalibrationSampleTime < calibrationSampleInterval) {
    return; // Not time for next sample yet
  }

  sensors_event_t magEvent;
  mag.getEvent(&magEvent);

  // Store raw magnetometer reading
  calibrationData[calibrationSampleCount].x = magEvent.magnetic.x;
  calibrationData[calibrationSampleCount].y = magEvent.magnetic.y;
  calibrationData[calibrationSampleCount].z = magEvent.magnetic.z;

  calibrationSampleCount++;
  lastCalibrationSampleTime = currentTime;

  if (calibrationSampleCount % 100 == 0) {
    USB_SERIAL_PRINTF("Collected %d calibration samples\n", calibrationSampleCount);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void refreshAndCalculatePositionalAttributes()
{
  Lat = gps.location.lat();
  lat_str = String(Lat , 7);
  Lng = gps.location.lng();
  lng_str = String(Lng , 7);
  satellites = gps.satellites.value();
  internetUploadStatusGood = (gps.altitudeUnitsGeoid.value() == 'M');
  overrideTarget = gps.altitudeUnits.value();

  if (overrideTarget == 'M')
  {
    overrideTarget = -1;
  }
  else if (overrideTarget >= 0)
  {
    overrideTarget -= 33;
    nextWaypoint = const_cast<NavigationWaypoint*>(&WraysburyWaypoints::waypoints[overrideTarget]);
  }
  else if (overrideTarget == -2)
  {
    overrideTarget = 'M' - 33;
    nextWaypoint = const_cast<NavigationWaypoint*>(&WraysburyWaypoints::waypoints[overrideTarget]);
  }

  if (enableRecentCourseCalculation)
  {
    if (millis() - last_journey_commit_time > journey_calc_period)
    {
      double distanceTravelled = gps.distanceBetween(Lat, Lng, journey_lat, journey_lng);

      if (distanceTravelled > journey_min_dist)
      {
        // Must have travelled min distance and min period elapsed since last waypoint.
        // store the course travelled since last waypoint.
        journey_course = gps.courseTo(journey_lat, journey_lng, Lat, Lng);
        if (journey_course >= 359.5) 
          journey_course = 0;

        journey_distance = distanceTravelled;

        if (journey_distance > 50) journey_distance = 0;    // correct for initial sample

        last_journey_commit_time = millis();
        journey_lat = Lat;
        journey_lng = Lng;
        activity_count = (activity_count + 1) % 4;
      }
    }
  }
  else
  {
    journey_course = 1;
    journey_distance = 1.1;
  }

  if  (millis() - journey_clear_period > last_journey_commit_time || journey_distance == 0)
    blackout_journey_no_movement = true;
  else
    blackout_journey_no_movement = false;

  if (directionMetric == COMPASS_HEADING)
    blackout_journey_no_movement = false;

  if (enableNavigationTargeting)
  {
    // UPDATED NEEDED HERE TO USE MASTER LAT AND LONG IN NAV WAYPOINTS ARRAY/////
    heading_to_target = gps.courseTo(Lat, Lng, nextWaypoint->_lat, nextWaypoint->_long);
    distance_to_target = gps.distanceBetween(Lat, Lng, nextWaypoint->_lat, nextWaypoint->_long);
  }
  else
  {
    heading_to_target = 1.0;
    distance_to_target = 1.1;
  }
}

void acquireAllSensorReadings()
{        
  uint32_t start_time_millis = millis();
  uint32_t forced_standardised_sensor_read_time = start_time_millis+minimum_sensor_read_time;
  
  if (millis() > s_lastCompassNotSmoothedDisplayRefresh + s_compassNotSmoothedHeadingUpdateRate)
  {
    s_lastCompassNotSmoothedDisplayRefresh = millis();

    if (compassAvailable)
    {
      if (enableSmoothedCompass)
      {
        getSmoothedMagHeading(magnetic_heading);
      }
      else
      {
        if (enableTiltCompensation)
        {
          getMagHeadingTiltCompensated(magnetic_heading);
        }
        else
        {
          getMagHeadingNotTiltCompensated(magnetic_heading);
        }
      }
    }
    else
    {
      magnetic_heading = 0;
    }
  }
  
  if (millis() > s_lastTempHumidityDisplayRefresh + s_tempHumidityUpdatePeriod)
  {
    s_lastTempHumidityDisplayRefresh = millis();

    getTempAndHumidityAndAirPressureBME280(humidity, temperature, air_pressure, pressure_altitude);

    if (!useGetDepthAsync)
    {
      bool read_original_algorithm = (display_to_show == SURVEY_DISPLAY ? true : false);
      getDepth(depth, water_temperature, water_pressure, depth_altitude, read_original_algorithm);
    }

    getM5ImuSensorData(&imu_gyro_vector.x, &imu_gyro_vector.y, &imu_gyro_vector.z,
                     &imu_lin_acc_vector.x, &imu_lin_acc_vector.y, &imu_lin_acc_vector.z,
                     &imu_rot_acc_vector.x, &imu_rot_acc_vector.y, &imu_rot_acc_vector.z,
                     &imu_temperature);
  }

  if (colourSensorAvailable &&
      millis() > nextLightReadTime && 
      Adafruit_ColourSensor.colorDataReady())
  {
    Adafruit_ColourSensor.getColorData(&red_light, &green_light, &blue_light, &clear_light);
    currentLightLevel = clear_light;
    nextLightReadTime = millis() + readLightTimeWait;
    sendLightLevelToOceanic = true;
  }

/*
  if (colourSensorAvailable && depth > minimumDivingDepthToActivateLightSensor &&
      millis() > s_lastColourDisplayRefresh + s_colourUpdatePeriod && millis() > nextLightReadTime)
  {
    s_lastColourDisplayRefresh = millis();
    if (Adafruit_ColourSensor.colorDataReady())
    {
      Adafruit_ColourSensor.getColorData(&red_light, &green_light, &blue_light, &clear_light);
    }
    colourMeasurements[colourIndex++] = clear_light;
    colourIndex = colourIndex % maxColourMeasurements;

    uint16_t threshold = 4090;
    uint8_t samplesAtOrAboveThreshold=0;
    uint8_t samplesBelowThreshold=0;

    for (uint8_t i = 0; i < maxColourMeasurements; i++)
    {
      if (colourMeasurements[i] < threshold)
        samplesBelowThreshold++;
      else
        samplesAtOrAboveThreshold++;
    }

    if (samplesAtOrAboveThreshold >= 4 && samplesAtOrAboveThreshold <= 10) // between 800ms and 2000ms in a 4000ms (200x20) period.
    {
      brightLightEvents++;
      sendBrightLightEventToTiger = true;
      memset(colourMeasurements,0,sizeof(colourMeasurements));
      nextLightReadTime = millis() + readLightTimeWait;
    }
  }
*/

  actual_sensor_acquisition_time = (uint16_t)(millis() - start_time_millis);

  if (actual_sensor_acquisition_time > max_actual_sensor_acquisition_time)
    max_actual_sensor_acquisition_time = actual_sensor_acquisition_time;

  // equalise acquisition time to be set to a minimum - BLOCKING - later make this asynchronous, and use lingerTimeMsBeforeUplink
//  while (millis() < forced_standardised_sensor_read_time);
  
  sensor_acquisition_time = (uint16_t)(millis() - start_time_millis);
  if (sensor_acquisition_time > max_sensor_acquisition_time)
    max_sensor_acquisition_time = sensor_acquisition_time;
}

bool getSmoothedMagHeading(double& magHeading)
{
  magHeading = 0;

  if (getMagHeadingTiltCompensated(magHeading) == false)
  {
    magHeading = -1.0;
    return false; // use the not initialised state to ignore this reading as NaN
  }

  s_smoothedCompassHeading[s_nextCompassSampleIndex] = magHeading;

  s_nextCompassSampleIndex = (s_nextCompassSampleIndex + 1) % s_smoothedCompassBufferSize;

  if (!s_smoothedCompassBufferInitialised)
  {
    // populate the entire smoothing buffer before doing any calculations or showing results
    if (s_nextCompassSampleIndex == 0)
      s_smoothedCompassBufferInitialised = true;
    return s_smoothedCompassBufferInitialised;
  }

  // compute average from s_nextCompassSampleIndex % s_smoothedCompassBufferSize to s_nextCompassSampleIndex-1
  magHeading = 0.0;

  bool magHeadingInNWQuadrantFound = false;
  bool magHeadingInNEQuadrantFound = false;

  for (uint8_t index = s_nextCompassSampleIndex; index < s_nextCompassSampleIndex + s_smoothedCompassBufferSize; index++)
  {
    if (s_smoothedCompassHeading[index % s_smoothedCompassBufferSize] < 90.0)
      magHeadingInNEQuadrantFound = true;
    else if (s_smoothedCompassHeading[index % s_smoothedCompassBufferSize] > 270.0)
      magHeadingInNWQuadrantFound = true;
  }

  double offset = (magHeadingInNWQuadrantFound && magHeadingInNEQuadrantFound ? 90.0 : 0.0);

  double shifted = 0.0;
  for (uint8_t index = s_nextCompassSampleIndex; index < s_nextCompassSampleIndex + s_smoothedCompassBufferSize; index++)
  {
    shifted = s_smoothedCompassHeading[index % s_smoothedCompassBufferSize] + offset;
    if (shifted >= 360.0)
      shifted -= 360.0;

    magHeading = magHeading + shifted;
  }

  magHeading = (magHeading / (double)s_smoothedCompassBufferSize)  - offset;

  if (magHeading < 0.0)
    magHeading += 360.0;

  if (magHeading >= 359.5)
    magHeading = 0.0;

  return s_smoothedCompassBufferInitialised;
}

/*
   Returns the angular difference in the horizontal plane between the "from" vector and north, in degrees.
   Description of heading algorithm:
   Shift and scale the magnetic reading based on calibration data to find
   the North vector. Use the acceleration readings to determine the Up
   vector (gravity is measured as an upward acceleration). The cross
   product of North and Up vectors is East. The vectors East and North
   form a basis for the horizontal plane. The From vector is projected
   into the horizontal plane and the angle between the projected vector
   and horizontal north is returned.
*/
template <typename T> double calculateTiltCompensatedHeading(vec<T> from)
{
  sensors_event_t event;
  mag.getEvent(&event);
  magnetometer_vector = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

  accel.getEvent(&event);
  accelerometer_vector = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

  // Hard iron calibration only - subtract center point
  magnetometer_vector.x -= (magnetometer_min.x + magnetometer_max.x) / 2.0;
  magnetometer_vector.y -= (magnetometer_min.y + magnetometer_max.y) / 2.0;
  magnetometer_vector.z -= (magnetometer_min.z + magnetometer_max.z) / 2.0;

  // Compute east and north vectors
  vec<double> east;
  vec<double> north;
  vector_cross(&magnetometer_vector, &accelerometer_vector, &east);
  vector_normalize(&east);
  vector_cross(&accelerometer_vector, &east, &north);
  vector_normalize(&north);

  // compute heading
  float heading = atan2(vector_dot(&east, &from), vector_dot(&north, &from)) * 180.0 / PI;
  if (heading < 0.0) {
    heading += 360.0;
  }
  return heading;
}

template <typename Ta, typename Tb, typename To> void vector_cross(const vec<Ta> *a, const vec<Tb> *b, vec<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float vector_dot(const vec<Ta> *a, const vec<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vec<double> *a)
{
  double mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

/*
   Returns the angular difference in the horizontal plane between a default vector and north, in degrees.
   The default vector here is the +X axis as indicated by the silkscreen.
*/
bool getMagHeadingTiltCompensated(double& tiltCompensatedHeading)
{
  double tch = calculateTiltCompensatedHeading((vec<int>) {1, 0, 0});  // was 1

  if (isnan(tch))
  {
    tiltCompensatedHeading = 0;
    return false;
  }

  // correction applied according to my experimentation
  tiltCompensatedHeading = -tch;

  // Normalize to 0-360
  if (tiltCompensatedHeading < 0.0)
    tiltCompensatedHeading += 360.0;

  if (tiltCompensatedHeading >= 359.5)
    tiltCompensatedHeading = 0.0;

  // correct for reversed dev module in gopro case - points south instead of north when north is reported.
 
  const bool magnetometerReversedInCase = true; // set to false if module points towards north when reporting north

  if (magnetometerReversedInCase)
  {
    tiltCompensatedHeading += 180.0;
    
    if (tiltCompensatedHeading >= 359.5)
      tiltCompensatedHeading -= 360.0;
  }
  
  return true;
}

bool getMagHeadingNotTiltCompensated(double& newHeading)
{
  sensors_event_t magEvent;
  mag.getEvent(&magEvent);
  double heading = (atan2(magEvent.magnetic.y, magEvent.magnetic.x) * 180.0) / PI;

  if (isnan(heading))
    return false;

  newHeading = -heading;

  if (newHeading < 0.0)
    newHeading += 360.0;

  if (newHeading >= 359.5)
    newHeading = 0.0;

  return true;
}

std::string getCardinal(float b, bool surveyScreen) 
{
  std::string result = "---";

  if      (b > 337.5 || b <= 22.5) result = (surveyScreen ? "North" : "N  ");  // 0
  else if (b > 22.5 && b <= 67.5) result = (surveyScreen ? "  NE" : "NE ");  // 45
  else if (b > 67.5 && b <= 112.5) result = (surveyScreen ? "East" : "E  ");  // 90
  else if (b > 112.5 && b <= 157.5) result = (surveyScreen ? "  SE" : "SE "); // 135
  else if (b > 157.5 && b <= 202.5) result = (surveyScreen ? "South" : "S  "); // 180
  else if (b > 202.5 && b <= 247.5) result = (surveyScreen ? "  SW" : "SW "); // 225
  else if (b > 247.5 && b <= 292.5) result = (surveyScreen ? "West" : "W  "); // 270
  else if (b > 292.5 && b <= 337.5) result = (surveyScreen ? "  NW" : "NW "); // 315

  return result;
}

uint32_t nextDepthReadCompleteTime = 0xFFFFFFFF;
const uint32_t depthReadCompletePeriod = 250;

// Need a way to reset the depth sensor in dive - just a reboot.
bool getDepthAsync(float& d, float& d_t, float& d_p, float& d_a)
{
  bool dataAcquired = false;
  if (!enableDepthSensor || !depthAvailable)
  {
    d = d_t = d_p = d_a = 0.0;
    return dataAcquired;
  }

  uint32_t timeNow = millis();
  
  if (nextDepthReadCompleteTime > timeNow && BlueRobotics_DepthSensor.readAsync() == MS5837::READ_COMPLETE)
  {
    // Trigger a read of the depth sensor in one second's time once the read is complete.
    nextDepthReadCompleteTime = timeNow + depthReadCompletePeriod;
    
    float temp_d = BlueRobotics_DepthSensor.depth();
  
    if (temp_d > 100.0)  // reject outliers that seem to be occurring (of several 1000 metres)
    {
      return dataAcquired;
    }
    else if (temp_d < 0.0)
    {
      // correct for any negative number, eg -0.01 which will otherwise get cast to a large unsigned number
      d = 0.0;
    }
    else
    {
      d = temp_d;
    }
  
    d_t = BlueRobotics_DepthSensor.temperature();
    d_p = BlueRobotics_DepthSensor.pressure() / 1000.0;
    d_a = BlueRobotics_DepthSensor.altitude();
    dataAcquired = true;
  }
  
  return dataAcquired;
}

// depth in metres, temperature in C, water pressure in Bar, Altitude in m
void getDepth(float& d, float& d_t, float& d_p, float& d_a, bool original_read)
{
  if (!enableDepthSensor || !depthAvailable)
  {
    d = d_t = d_p = d_a = 0.3;
    return;
  }

/*
  bool result = BlueRobotics_DepthSensor.read_original();

  if (result)
  {
      redLEDStatus = !redLEDStatus;
      digitalWrite(RED_LED_GPIO, redLEDStatus);
  }
*/

  original_read ? BlueRobotics_DepthSensor.read_original() : BlueRobotics_DepthSensor.read();

  float temp_d = BlueRobotics_DepthSensor.depth();

  if (temp_d > 100.0)  // reject outliers that seem to be occurring (of several 1000 metres)
  {
    return;
  }
  else if (temp_d < 0.0)
  {
    // correct for any negative number, eg -0.01 which will otherwise get cast to a large unsigned number
    d = 0.0;
  }
  else
  {
    d = temp_d;
  }

  d_t = BlueRobotics_DepthSensor.temperature();
  d_p = BlueRobotics_DepthSensor.pressure() / 1000.0;
  d_a = BlueRobotics_DepthSensor.altitude();

//  d = testDepthTimer();
}

void checkDivingDepthForTimer(const float& d)
{
  if (!diveTimerRunning && minutesDurationDiving == 0 && d >= minimumDivingDepthToRunTimer)
  {
    startDiveTimer();
  }
  else
  {
    if (diveTimerRunning)
    {
      if (d < minimumDivingDepthToRunTimer)
      {
        notifyNotAtDivingDepth();
      }
      else
      {
        // at diving depth so reset the time that dive timer is stopped if at < 1m depth continuously
        whenToStopTimerDueToLackOfDepth = 0;
      }
    }
  }
  refreshDiveTimer();
}

void startDiveTimer()
{
  resetRealTimeClock();
  diveTimerRunning = true;
}

// If not at diving depth for >= 10 minutes then dive timer is stopped permanently.
void notifyNotAtDivingDepth()
{
  if (diveTimerRunning)
  {
    if (whenToStopTimerDueToLackOfDepth == 0)
    {
      RTC_TimeTypeDef TimeStruct;         // Hours, Minutes, Seconds
      M5.Rtc.GetTime(&TimeStruct);
  
      whenToStopTimerDueToLackOfDepth = TimeStruct.Hours*60 + TimeStruct.Minutes + minsToTriggerStopDiveTimer;
    }
    else if (minutesDurationDiving >= whenToStopTimerDueToLackOfDepth)
    {
      minutesDurationDiving -= minsToTriggerStopDiveTimer;
      diveTimerRunning = false;
    }
  }
}

void refreshDiveTimer()
{
  if (diveTimerRunning)
  {
    RTC_TimeTypeDef TimeStruct;         // Hours, Minutes, Seconds
    M5.Rtc.GetTime(&TimeStruct);
    
    minutesDurationDiving = TimeStruct.Hours*60 + TimeStruct.Minutes;
  }
}

void resetRealTimeClock()
{
  M5.Lcd.fillScreen(BLACK);
  RTC_TimeTypeDef TimeStruct;         // Hours, Minutes, Seconds 
  TimeStruct.Hours   = 0;
  TimeStruct.Minutes = 0;
  TimeStruct.Seconds = 0;

  M5.Rtc.SetTime(&TimeStruct);

  minutesDurationDiving = 0;
}

void getTempAndHumidityAndAirPressureBME280(float& h, float& t, float& p, float& p_a)
{
  if (humidityAvailable)
  {
    h = Adafruit_TempHumidityPressure.readHumidity();
    t = Adafruit_TempHumidityPressure.readTemperature();
    p = (float)(Adafruit_TempHumidityPressure.readPressure()) / 100.0 + pressure_correction;
    p_a = Adafruit_TempHumidityPressure.readAltitude(SEALEVELPRESSURE_HPA);
  }
  else
  {
    h = t = p = 0.0;
  }
}

void toggleRedLED()
{
  redLEDStatus = (redLEDStatus == HIGH ? LOW : HIGH );
  digitalWrite(RED_LED_GPIO, redLEDStatus);
}

void setRedLEDOff()
{
  redLEDStatus = HIGH;
  digitalWrite(RED_LED_GPIO, redLEDStatus);
}

void setRedLEDOn()
{
  redLEDStatus = LOW;
  digitalWrite(RED_LED_GPIO, redLEDStatus);
}


void toggleUptimeGlobalDisplay()
{
  enableGlobalUptimeDisplay = !enableGlobalUptimeDisplay;

  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(20, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  if (enableGlobalUptimeDisplay)
    M5.Lcd.println("Uptime On");
  else
    M5.Lcd.println("Uptime Off");

  delay(2000);

  M5.Lcd.fillScreen(TFT_BLACK);
}

void toggleAsyncDepthDisplay()
{
  useGetDepthAsync = !useGetDepthAsync;

  M5.Lcd.fillScreen(TFT_ORANGE);
  M5.Lcd.setCursor(20, 10);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  if (useGetDepthAsync)
    M5.Lcd.println("Async Depth On");
  else
    M5.Lcd.println("Async Depth Off");

  delay(2000);

  M5.Lcd.fillScreen(TFT_BLACK);
}



#endif
