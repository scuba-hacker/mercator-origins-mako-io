
#ifdef BUILD_INCLUDE_MAIN_TEST_CODE

float testDepthTimer()
{
  uint32_t now = millis();
  if (now < 30000)        // 30s
    return 0; 
  else if (now < 95000)   // 1 min 35s
    return 3;
  else if (now < 110000)  // 1 min 50s
    return 0;
  else if (now < 215000)  // 3 min 35s 
    return 4;
  else
    return 0;   // Should revert to 3 minutes after a further 4 minutes. at 7 min 35s
}

void loop_no_gps()
{
  M5.Axp.ScreenBreath(HALF_BRIGHT_DISPLAY);

  getTempAndHumidityAndAirPressureBME280(humidity, temperature, air_pressure, pressure_altitude);

  getDepth(depth, water_temperature, water_pressure, depth_altitude,true);

  getMagHeadingTiltCompensated(magnetic_heading);

  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(4);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setCursor(0, 0);

  M5.Lcd.printf("  %.2f m  \n", depth);

  M5.Lcd.setTextSize(1);
  M5.Lcd.println("");

  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("%.1fC   %.1f%%\n", temperature, humidity);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("");

  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf(" %.1f mBar \n", air_pressure);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("");

  M5.Lcd.setTextColor(TFT_YELLOW, TFT_MAGENTA);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("%.2fV %.1fmA \n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());

  sleep(250);
}

void sendTestSerialBytesWhenReady()
{
  uint32_t s_lastSendTestSerialBytesTest = 0;
  const uint32_t s_sendTestSerialBytesPeriodMs = 1000;

  if (millis() > s_lastSendTestSerialBytesTest + s_sendTestSerialBytesPeriodMs)
  {
    s_lastSendTestSerialBytesTest = millis();

    lemon_float_serial.write("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU");
  }
}

#endif