#ifdef BUILD_INCLUDE_MAIN_SENSOR_CODE

template <typename T> double compensate_calibrated_heading_for_tilt(vec<T>& from_vector, vec<double>& magnetometer_vector, vec<double>& accelerometer_vector);
void apply_full_corrections_to_raw_magnetometer_readings(vec<double>& magnetometer_vector);
void apply_legacy_hard_iron_only_correction_to_raw_magnetometer_readings(vec<double>& magnetometer_vector);

// Advanced calibration: Hard iron + Soft iron compensation
// Calibration from ellipsoid fit (magcal_ellipsoid.txt)

vec<double> hard_iron_offset;
double soft_iron_matrix[3][3];

// Calibration for Oceanic being present and gopro mount for real gopro - but without the camera.
// hard iron compensation matrix - linear shift in each axis to account for fixed magnetic disturbance, eg metal brackets
vec<double> hard_iron_offset_without_camera = { 8.331061080431581445, -12.71955273800613107, 45.12891163824074425 };
//
// soft iron compensation matrix - transforms distorted ellipsoid to sphere with radius ~47 (in sensor units)
const double soft_iron_matrix_without_camara[3][3] = {
  { 0.9719287820482613860,   0.006939862106518941871, -0.01086593702250229256 },
  { 0.006939862106518939269, 1.042324789021345177,     0.006598418562658864804 },
  { -0.01086593702250229082,  0.006598418562658867406,  0.9873142116148320158 },
};

// TO DO - Calibration for Oceanic being present and gopro camera mounted
// hard iron compensation matrix - linear shift in each axis to account for fixed magnetic disturbance, eg metal brackets
vec<double> hard_iron_offset_with_camera = { 1, 1, 1 };
//
// soft iron compensation matrix - transforms distorted ellipsoid to sphere with radius ~47 (in sensor units)
const double soft_iron_matrix_with_camara[3][3] = {
  { 1, 1, 1},
  { 1, 1, 1},
  { 1, 1, 1},
};

// TO DO - Calibration for no Oceanic or GoPro Mount/Camera being present
// hard iron compensation matrix - linear shift in each axis to account for fixed magnetic disturbance, eg metal brackets
vec<double> hard_iron_offset_mako_tiger_only = { 1, 1, 1 };
//
// soft iron compensation matrix - transforms distorted ellipsoid to sphere with radius ~47 (in sensor units)
const double soft_iron_matrix_mako_tiger_only[3][3] = {
  { 1, 1, 1},
  { 1, 1, 1},
  { 1, 1, 1},
};

const char* getSpoolSetupDescription(e_spool_setup setup)
{
  switch (setup)
  {
    case SPOOL_45M_WITH_OCEANIC_AND_WITHOUT_CAMERA: return "45M Spool, Oceanic";
    case SPOOL_45M_WITH_OCEANIC_AND_CAMERA:         return "45M Spool, Oceanic, Camera";
    case SPOOL_45_MAKO_TIGER_ONLY:                  return "45M Spool only";
    default:                                        return "Unknown";
  }  
}

// LIS2MDL I2C address and offset registers
#define LIS2MDL_I2C_ADDR 0x1E
#define LIS2MDL_OFFSET_X_REG_L 0x45
#define LIS2MDL_OFFSET_X_REG_H 0x46
#define LIS2MDL_OFFSET_Y_REG_L 0x47
#define LIS2MDL_OFFSET_Y_REG_H 0x48
#define LIS2MDL_OFFSET_Z_REG_L 0x49
#define LIS2MDL_OFFSET_Z_REG_H 0x4A
#define LIS2MDL_MAG_LSB_UT 0.15  // LSB value in microTesla

void setMagHardIronOffsets(vec<double> hard_iron_offset, TwoWire *wire = &Wire, uint8_t i2c_addr = LIS2MDL_I2C_ADDR);
void resetMagHardIronOffsets(TwoWire *wire = &Wire, uint8_t i2c_addr = LIS2MDL_I2C_ADDR);

bool setHardIronOffsetsInHardwareRegisters = true;

void setCompassCalibrationSpoolSetup(e_spool_setup setup)
{
  switch (setup)
  {
    case SPOOL_45M_WITH_OCEANIC_AND_WITHOUT_CAMERA:
    {
      hard_iron_offset = hard_iron_offset_without_camera;
      memcpy(soft_iron_matrix, soft_iron_matrix_without_camara,sizeof(soft_iron_matrix));
      break;
    }
    case SPOOL_45M_WITH_OCEANIC_AND_CAMERA:
    {
      hard_iron_offset = hard_iron_offset_with_camera;
      memcpy(soft_iron_matrix, soft_iron_matrix_with_camara,sizeof(soft_iron_matrix));
    }
    case SPOOL_45_MAKO_TIGER_ONLY:
    default:
    {
      hard_iron_offset = hard_iron_offset_mako_tiger_only;
      memcpy(soft_iron_matrix, soft_iron_matrix_mako_tiger_only,sizeof(soft_iron_matrix));
    }
  }
}


void initSensors()
{
  if (enableIMUSensor)
  {
    imuAvailable = !M5.Imu.Init();
    if (imuAvailable)
      M5.Lcd.println("M5 IMU On");
  }
  else
  {
    USB_SERIAL_PRINTLN("IMU Sensor Off");
    M5.Lcd.println("IMU Sensor Off");
    imuAvailable = false;
  }

  if (enableHumiditySensor)
  {
    if (!Adafruit_TempHumidityPressure.begin())
    {
      USB_SERIAL_PRINTLN("Could not find BME280 Barometer");
  
      M5.Lcd.println("BE280 T/H/P bad");
      delay(5000);
      humidityAvailable = false;
    }
    else
    {
      M5.Lcd.println("Tmp/Humd Ok");
      M5.Lcd.println("Barometr Ok");
    }
  }
  else
  {
    USB_SERIAL_PRINTLN("BME280 Humidity Off");
    M5.Lcd.println("BME280 Humidity Off");
    humidityAvailable = false;
    temperature = 0.1;
    humidity = 0.1;
    air_pressure = 0.1;
  }

    // Measurements against real Suunto dive compass.
  // Suunto compass versus Mako digital compass
  //
  // 0 N on    suunto:  352 on digital compass (-8 difference)
  // 45 NE on  suunto:  47 (+2)
  // 90 E on   suunto:  100 (+10)
  // 135 SE on suunto:  156 (+21)
  // 180 S on  suunto:  186 (+6)
  // 225 SW on suunto:  220 (-5)
  // 270 W on  suunto:  260 (-10)
  // 315 NW on suunto:  315 (0)

  if (enableDigitalCompass)
  {
    // Initialize compass smoothing arrays
    for (uint8_t i = 0; i < s_smoothedCompassBufferSize; i++) {
      s_smoothedCompassHeading[i] = 0.0;
    }

    setCompassCalibrationSpoolSetup(spool_setup);

    if (!mag.begin())
    {
      USB_SERIAL_PRINTLN("Could not find LIS2MDL Magnetometer. Check wiring");
      M5.Lcd.println("LIS2MDL Magnetometer bad");
      delay(5000);
      compassAvailable = false;
    }
    else
    {
      if (setHardIronOffsetsInHardwareRegisters)
          setMagHardIronOffsets(hard_iron_offset);

      M5.Lcd.println("Compass Ok");
    }

    if (!accel.begin())
    {
      // In begin() the code Enables the accelerometer to 100Hz sampling using
      //    ctrl1.write(0x57);
      // This could be changed to 25Hz or 50Hz to reduce noise
      //     ctrl1.write(0x37);    // 25 Hz
      // or  ctrl1.write(0x47);    // 50 Hz
      USB_SERIAL_PRINTLN("Unable to initialize LSM303 accelerometer");
      M5.Lcd.println("LSM303 accelerometer bad");
      compassAvailable = false;
    }
    else
    {
      M5.Lcd.println("Accel Ok");
      accel.setMode(LSM303_MODE_HIGH_RESOLUTION); // 12 bit instead of default 10 bit resolution for acceleration - better tilt compensation
    }

    if (compassAvailable && enableSmoothedCompass)
      smoothedCompassCalcInProgress = true;
  }
  else
  {
    USB_SERIAL_PRINTLN("LSM303 Compass off");
    M5.Lcd.println("LSM303 Compass off");
    compassAvailable = false;
  }

  if (enableColourSensor)
  {
    if (!Adafruit_ColourSensor.begin())
    {
      USB_SERIAL_PRINTLN("Unable to init APDS9960 colour");
      M5.Lcd.println("APDS9960 colour bad");
      colourSensorAvailable=false;
    }
    else
    {
      M5.Lcd.println("Colour Ok");
      Adafruit_ColourSensor.enableColor(true);
      colourSensorAvailable=true;
    }
  }
  
  if (enableDepthSensor)
  {
    USB_SERIAL_PRINTLN("Attempt to begin depth sensor");

    if (!BlueRobotics_DepthSensor.begin())
    {
      USB_SERIAL_PRINTLN("Could not begin depth sensor");
      depthAvailable = false;
    }
    else
    {
      if (divingInTheSea)
        BlueRobotics_DepthSensor.setFluidDensitySaltWater();
      else
        BlueRobotics_DepthSensor.setFluidDensityFreshWater();

    	bool calibrated = BlueRobotics_DepthSensor.calibrateAtSurfaceForAtmosphericPressure();
      M5.Lcd.println(calibrated ? "Depth Ok (Cal)" : "Depth Ok (No Cal)");
      // select fastest conversion rate (smallest oversample rate) - approx 2ms total for D1 (pressure conversion) + D2 (temperature conversion)
      BlueRobotics_DepthSensor.setOSR(MS5837::OSR_256); 
    }
  }
  else
  {
    USB_SERIAL_PRINTLN("Depth Sensor Off");
    M5.Lcd.println("Depth Sensor Off");
    depthAvailable = false;
    depth = 0;
  }
}

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
        getSmoothedMagHeading(magnetic_heading,enableMedianHeadingSmoothing);
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

bool getSmoothedMagHeading(double& magHeading, bool useMedian)
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

  double offset = (magHeadingInNWQuadrantFound && magHeadingInNEQuadrantFound ? 180.0 : 0.0);

  if (useMedian)
  {
    // Calculate median
    double sorted[s_smoothedCompassBufferSize];
    for (uint8_t i = 0; i < s_smoothedCompassBufferSize; i++)
    {
      uint8_t index = (s_nextCompassSampleIndex + i) % s_smoothedCompassBufferSize;
      sorted[i] = s_smoothedCompassHeading[index] + offset;
      if (sorted[i] >= 360.0)
        sorted[i] -= 360.0;
    }

    // Simple bubble sort (small array, performance ok)
    for (uint8_t i = 0; i < s_smoothedCompassBufferSize - 1; i++)
    {
      for (uint8_t j = 0; j < s_smoothedCompassBufferSize - i - 1; j++)
      {
        if (sorted[j] > sorted[j + 1])
        {
          double temp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = temp;
        }
      }
    }

    // Median is average of middle two values for even-sized array
    magHeading = (sorted[s_smoothedCompassBufferSize / 2 - 1] + sorted[s_smoothedCompassBufferSize / 2]) / 2.0 - offset;
  }
  else
  {
    // Calculate mean
    double shifted = 0.0;
    for (uint8_t index = s_nextCompassSampleIndex; index < s_nextCompassSampleIndex + s_smoothedCompassBufferSize; index++)
    {
      shifted = s_smoothedCompassHeading[index % s_smoothedCompassBufferSize] + offset;
      if (shifted >= 360.0)
        shifted -= 360.0;

      magHeading = magHeading + shifted;
    }

    magHeading = (magHeading / (double)s_smoothedCompassBufferSize) - offset;
  }

  if (magHeading < 0.0)
    magHeading += 360.0;

  if (magHeading >= 359.5)
    magHeading = 0.0;

  return s_smoothedCompassBufferInitialised;
}

void apply_full_corrections_to_raw_magnetometer_readings(vec<double>& magnetometer_vector)
{
  if (!setHardIronOffsetsInHardwareRegisters)
  {
    // Remove hard iron offset (calculate in software rather than on chip)
    magnetometer_vector.x -= hard_iron_offset.x;
    magnetometer_vector.y -= hard_iron_offset.y;
    magnetometer_vector.z -= hard_iron_offset.z;
  }

  double corrected_x = soft_iron_matrix[0][0] * magnetometer_vector.x +
                        soft_iron_matrix[0][1] * magnetometer_vector.y +
                        soft_iron_matrix[0][2] * magnetometer_vector.z;
  double corrected_y = soft_iron_matrix[1][0] * magnetometer_vector.x +
                        soft_iron_matrix[1][1] * magnetometer_vector.y +
                        soft_iron_matrix[1][2] * magnetometer_vector.z;
  double corrected_z = soft_iron_matrix[2][0] * magnetometer_vector.x +
                        soft_iron_matrix[2][1] * magnetometer_vector.y +
                        soft_iron_matrix[2][2] * magnetometer_vector.z;

  magnetometer_vector.x = corrected_x;
  magnetometer_vector.y = corrected_y;
  magnetometer_vector.z = corrected_z;
}

template <typename T> double calculateTiltCompensatedHeading(vec<T> from_vector)
{
  sensors_event_t event;
  
  mag.getEvent(&event);
  magnetometer_vector = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

  apply_full_corrections_to_raw_magnetometer_readings(magnetometer_vector);

  accel.getEvent(&event);
  accelerometer_vector = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

  return compensate_calibrated_heading_for_tilt(from_vector, magnetometer_vector, accelerometer_vector);
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
template <typename T> double compensate_calibrated_heading_for_tilt(vec<T>& from_vector, vec<double>& magnetometer_vector, vec<double>& accelerometer_vector)
{
  // Compute east and north vectors
  vec<double> east;
  vec<double> north;
  vector_cross(&magnetometer_vector, &accelerometer_vector, &east);
  vector_normalize(&east);
  vector_cross(&accelerometer_vector, &east, &north);
  vector_normalize(&north);

  // compute heading
  float heading = atan2(vector_dot(&east, &from_vector), vector_dot(&north, &from_vector)) * 180.0 / PI;
  if (heading < 0.0) {
    heading += 360.0;
  }
  return heading;
}

void apply_legacy_hard_iron_only_correction_to_raw_magnetometer_readings(vec<double>& magnetometer_vector)
{
  // recalibration on 28 Sep 2025 only Oceanic mounted, plus the gopro mount arm.
  // Values obtained using Mako's calibration screen - just max and min as measured on device.
  const vec<double> magnetometer_min = { -40.05, -58.650, -2.550};
  const vec<double> magnetometer_max = { 56.4, 33.9, 94.35};

  // Hard iron calibration only - subtract center point
  magnetometer_vector.x -= (magnetometer_min.x + magnetometer_max.x) / 2.0;
  magnetometer_vector.y -= (magnetometer_min.y + magnetometer_max.y) / 2.0;
  magnetometer_vector.z -= (magnetometer_min.z + magnetometer_max.z) / 2.0;
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
  // Apply callibration corrections to magnetometer readings and then apply tilt compensation
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

void setMagHardIronOffsets(const vec<double> off, TwoWire* wire, uint8_t addr)
{
  // Convert from µT to LSB (values from getEvent are scaled by 0.15)
  // Hardware SUBTRACTS: H_out = H_meas - H_offset, so NO negation
  const int16_t ox = (int16_t)lround(off.x / LIS2MDL_MAG_LSB_UT);
  const int16_t oy = (int16_t)lround(off.y / LIS2MDL_MAG_LSB_UT);
  const int16_t oz = (int16_t)lround(off.z / LIS2MDL_MAG_LSB_UT);

  // Write each axis separately (LSB and MSB in separate transactions)
  // Cast to uint16_t first to avoid sign extension issues
  uint16_t ux = static_cast<uint16_t>(ox);
  uint16_t uy = static_cast<uint16_t>(oy);
  uint16_t uz = static_cast<uint16_t>(oz);

  wire->beginTransmission(addr);
  wire->write(LIS2MDL_OFFSET_X_REG_L);
  wire->write(static_cast<uint8_t>(ux & 0xFF));
  wire->endTransmission();

  wire->beginTransmission(addr);
  wire->write(LIS2MDL_OFFSET_X_REG_H);
  wire->write(static_cast<uint8_t>(ux >> 8));
  wire->endTransmission();

  wire->beginTransmission(addr);
  wire->write(LIS2MDL_OFFSET_Y_REG_L);
  wire->write(static_cast<uint8_t>(uy & 0xFF));
  wire->endTransmission();

  wire->beginTransmission(addr);
  wire->write(LIS2MDL_OFFSET_Y_REG_H);
  wire->write(static_cast<uint8_t>(uy >> 8));
  wire->endTransmission();

  wire->beginTransmission(addr);
  wire->write(LIS2MDL_OFFSET_Z_REG_L);
  wire->write(static_cast<uint8_t>(uz & 0xFF));
  wire->endTransmission();

  wire->beginTransmission(addr);
  wire->write(LIS2MDL_OFFSET_Z_REG_H);
  wire->write(static_cast<uint8_t>(uz >> 8));
  wire->endTransmission();
}

void resetMagHardIronOffsets(TwoWire *wire, uint8_t i2c_addr)
{
  setMagHardIronOffsets(vec<double>(0,0,0),wire,i2c_addr);
}

#endif
