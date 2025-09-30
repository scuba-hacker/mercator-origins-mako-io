////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool goProButtonsPrimaryControl = true;     // false means use the M5 Stick physical buttons (eg out of gopro case bench test)
                                            // true means use the go pro buttons meaning it must be installed into the pod.
                                            // If set to false when Mako is in the pod, activate an reed switch to make
                                            // go pro buttons primary so that OTA can be done with fixed code. 
                                            // Relies on receiving ESP Now message from Tiger

bool divingInTheSea = false;                 // Salt / Fresh water depth sensor calibration

bool testTigerOutsidePod = false;
bool writeLogToSerial = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
Documentation needed:


Reed Controls / M5 Button Controls when under test
Serial Commands
ESPNow Commands 
*/

// https://www.hackster.io/pradeeplogu0/real-time-gps-monitoring-with-qubitro-and-m5stickc-a2bc7c

// https://github.com/mikalhart/TinyGPSPlus/blob/master/README.md
// http://arduiniana.org/libraries/tinygpsplus/

// Course to back left corner of garden at compost bin Lat = 51.39126, long=-0.28764
//
//possible fix to deepSleep with timer #31 - https://github.com/m5stack/M5StickC-Plus/pull/31
//Sleep causing unresponsive device #13 https://github.com/m5stack/M5StickC-Plus/issues/13
//AXP192.cpp SetSleep() is different than the one for M5StickC #1 https://github.com/m5stack/M5StickC-Plus/issues/1

#include <Arduino.h>

#include "SerialConfig.h"

#include <M5StickCPlus.h>

// rename the git file "mercator_secrets_template.c" to the filename below, filling in your wifi credentials etc.
#include "mercator_secrets.c"

#include <esp_now.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()

#include "driver/uart.h"

#include <memory.h>
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"

#include <NavigationWaypoints.h>
#include <Preferences.h>

// ************** Mako Control Parameters **************

bool usingSDP8600OptoSchmittDetector = true; // alternative is SDP8406 phototransistor which was the original detector

bool enableDigitalCompass = true;
bool enableTiltCompensation = true;
bool enableSmoothedCompass = true;
bool enableMedianHeadingSmoothing = true;
bool enableHumiditySensor = true;
bool enableDepthSensor = true;
bool enableIMUSensor = false;
bool enableColourSensor = true;
const uint32_t minimum_sensor_read_time = 0; // (ms) regulates upload of responses to lemon by normalising the time to process sensor readings. This contributes most to the round-trip latency.

bool enableDownlinkComms = true; // enable reading the feed from Lemon at surface
bool enableUplinkComms = true;  // enable writing of feed to Lemon. Can be toggled through UI

const bool enableNavigationGraphics = true;
const bool enableNavigationTargeting = true;
const bool enableRecentCourseCalculation = true;
bool enableGlobalUptimeDisplay = false;    // adds a timer to compass heading display so can see if a crash/reboot has happened

bool enableWifiAtStartup = false;   // set to true only if no espnow at startup
bool enableESPNowAtStartup = true;  // set to true only if no wifi at startup
int pingResponseReceivedFromTiger = 0;
int pingResponseReceivedFromUnknown = 0;
int espMsgReceived = 0;
int espPingMsgReceived = 0;
int pingSentTigerCount = 0;
int attemptTigerPingSend = 0;

bool otaActive = false; // OTA updates toggle

bool compassAvailable = true;
bool humidityAvailable = true;
bool colourSensorAvailable = false;
bool depthAvailable = true;
bool imuAvailable = true;

const uint8_t RED_LED_GPIO = 10;
int redLEDStatus = HIGH;

bool sendInstantMessageLocationNowFlag = false;
bool sendInstantMessageEmergencyNowFlag = false;

void toggleRedLED();
void setRedLEDOff();
void setRedLEDOn();

template <typename T> struct vec
{
  T x, y, z;
  vec<T>(T xx, T yy, T zz) : x(xx),y(yy),z(zz) {}
  vec<T>() : x(0),y(0),z(0) {}
};

enum e_audio_action {AUDIO_ACTION_NONE, AUDIO_ACTION_NEXT_SOUND, AUDIO_ACTION_CYCLE_VOLUME, AUDIO_ACTION_SOUNDS_TOGGLE, 
AUDIO_ACTION_PLAYBACK_TOGGLE, AUDIO_ACTION_STOP_PLAYBACK, AUDIO_ACTION_SET_VOLUME, AUDIO_ACTION_ROTATE_SOUND_SET};

enum e_soundFX {SFX_PIANO_AHEAD='0', SFX_PIANO_BEHIND='1',SFX_PIANO_LEFT='2',SFX_PIANO_RIGHT='3',
                SFX_ORGAN_AHEAD='4', SFX_ORGAN_BEHIND='5',SFX_ORGAN_LEFT='6',SFX_ORGAN_RIGHT='7',
                SFX_PAD_AHEAD='8', SFX_PAD_BEHIND='9',SFX_PAD_LEFT=':',SFX_PAD_RIGHT=';',SFX_NONE='_'};

void switchDivePlan();
void switchToNextDisplayToShow();
void getM5ImuSensorData(float* gyro_x, float* gyro_y, float* gyro_z,
                        float* lin_acc_x, float* lin_acc_y, float* lin_acc_z,
                        float* rot_acc_x, float* rot_acc_y, float* rot_acc_z,
                        float* IMU_temperature);
const char* scanForKnownNetwork();
bool connectToWiFiAndInitOTA(const bool wifiOnly, int repeatScanAttempts);
bool setupOTAWebServer(const char* _ssid, const char* _password, const char* label, uint32_t timeout, bool wifiOnly);
bool getMagHeadingTiltCompensated(double& tiltCompensatedHeading);
bool getMagHeadingNotTiltCompensated(double& heading);
bool getSmoothedMagHeading(double& b,bool useMedian = false);
std::string getCardinal(float b, bool surveyScreen = false);
void getTempAndHumidityAndAirPressureBME280(float& h, float& t, float& p, float& p_a);
void getDepth(float& d, float& d_t, float& d_p, float& d_a, bool original_read);
bool getDepthAsync(float& d, float& d_t, float& d_p, float& d_a);
bool manualInitDepthSensor();
void goBlackout();
void goAhead();
void goClockwise();
void goUnknown();
void goAntiClockwise();
void goTurnAround();
void drawGoAhead(const bool show);
void drawGoBlackout(const bool show);
void drawGoClockwise(const bool show);
void drawGoTurnAround(const bool show);
void drawGoUnknown(const bool show);
void drawGoAntiClockwise(const bool show);
void showOTARecoveryScreen();
void dumpHeapUsage(const char* msg);
void loop_no_gps();
void sendTestSerialBytesWhenReady();
bool isGPSStreamOk();
bool isLatestGPSMsgFix();
bool isGPSTargetShortTimedOut();
bool isInternetUploadOk();
bool processGPSMessageIfAvailable();
void refreshAndCalculatePositionalAttributes();
void initSensors();
void acquireAllSensorReadings();
void checkForButtonPresses();
void refreshDisplay();
void drawSurveyDisplay();
void drawTargetSection();
void drawCompassSection();
void drawCourseSection();
void drawNextTarget();
void drawAudioActionDisplay();
void drawLatLong();
void drawLocationStats();
void drawEspNowDisplay();
void drawJourneyStats();
void drawAudioTest();
void drawNullDisplay();
void drawPowerOnTimeOverlay();
void performUplinkTasks();
void refreshGlobalStatusDisplay();
void sendNoUplinkTelemetryMessages();
void sendFullUplinkTelemetryMessage();
uint16_t getOneShotUserActionForUplink();
void sendUplinkTelemetryMessageV5();
void sendUplinkTestMessage();
void refreshDirectionGraphic( float directionOfTravel,  float headingToTarget);
void vector_normalize(vec<double> *a);
bool getMagHeadingNotTiltCompensated(double& newHeading);
float testDepthTimer();
void checkDivingDepthForTimer(const float& d);
void startDiveTimer();
void notifyNotAtDivingDepth();
void refreshDiveTimer();
void resetRealTimeClock();
uint32_t verifyAllEspNowConnections();
void notifyESPNowNotActive();
void displayESPNowSendDataResult(const esp_err_t result);
void enableFastPairing();
void enableParallelPairing();
void toggleESPNowActive(bool adjustDisplay=false);
bool disableESPNowIfSideButtonHeld();
bool enableOTAAtStartupIfTopButtonHeld();
void toggleWiFiActive(bool wait=true);
void toggleOTAActive();
void disableFeaturesForOTA();
void toggleUptimeGlobalDisplay();
void toggleAsyncDepthDisplay();
void toggleUplinkMessageProcessAndSend();
void processIncomingESPNowMessages();
void publishToTigerBrightLightEvent();
void publishToTigerAndOceanicLocationAndTarget(const char* currentTarget);
void publishToTigerAndOceanicCurrentTarget(const char* currentTarget);
void publishToOceanicLightLevel(uint16_t lightLevel);
void publishToOceanicBreadCrumbRecord(const bool record);
void publishToOceanicPinPlaced(double latitude, double longitude, double heading, double depth);
void toggleSound();
void publishToSilkyPlayAudioGuidance(enum e_soundFX sound);
void publishToSilkySkipToNextTrack();
void publishToSilkyCycleVolumeUp();
void publishToSilkySetVolume(const uint8_t newVolume);
void publishToSilkyTogglePlayback();
void publishToSilkyStopPlayback();
void drawCompassCalibration();
void drawThisTarget();
void notifySoundsOnOffChanged();
void startCalibrationDataCollection();
void stopCalibrationDataCollection();
void collectCalibrationSample();
void setupCalibrationWebEndpoint();
void rotateToNextGuidanceSounds();
bool connectESPNow();
void configESPNowDeviceAP();
void readAndTestGoProButtons();
bool checkForDualButtonPresses();
bool InitESPNow();
bool TeardownESPNow();
void OnESPNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
bool ESPNowScanForPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix);
bool pairWithPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, int maxAttempts);
bool ESPNowManagePeer(esp_now_peer_info_t& peer);
void ESPNowDeletePeer(esp_now_peer_info_t& peer);

// ************** ESPNow variables **************
const int RESET_ESPNOW_SEND_RESULT = 0xFF;
esp_err_t ESPNowSendResult=(esp_err_t)RESET_ESPNOW_SEND_RESULT;

const uint8_t ESPNOW_CHANNEL = 1;
const uint8_t ESPNOW_NO_PEER_CHANNEL_FLAG = 0xFF;
const uint8_t ESPNOW_PRINTSCANRESULTS = 0;
const uint8_t ESPNOW_DELETEBEFOREPAIR = 0;

uint16_t ESPNowMessagesDelivered = 0;
uint16_t ESPNowMessagesFailedToDeliver = 0;

esp_now_peer_info_t ESPNow_silky_peer;
esp_now_peer_info_t ESPNow_tiger_peer;
esp_now_peer_info_t ESPNow_oceanic_peer;

QueueHandle_t espNowMsgsReceivedQueue;

char tigerMessage[16]="";
char tigerReeds[16]="";
char oceanicMessage[16]="";
char oceanicButtons[16]="";
char silkyMessage[16]="";

bool refreshTigerMsgShown = false;
bool refreshTigerReedsShown = false;
bool refreshOceanicButtonsShown = false;
bool refreshSilkyMsgShown = false;

bool sendLeakDetectedToLemon = false;

TaskHandle_t lemonTaskHandle = nullptr;

// ************** Silky / Sounds variables **************

bool soundsOn = true;
const uint8_t defaultSilkyVolume = 5;     // no software gain applied - best for sounds recorded at optimal amplitude.
const uint8_t minSilkyVolume = 1;         // quietest
const uint8_t maxSilkyVolume = 5;
uint8_t silkyVolume = defaultSilkyVolume;

e_audio_action audioAction = AUDIO_ACTION_NONE;

const uint8_t SILKY_ESPNOW_COMMAND_TOGGLE_PLAYBACK = (uint8_t)'A'; // 500 ms
const uint8_t SILKY_ESPNOW_COMMAND_CYCLE_VOLUME_UP = (uint8_t)'B'; // 2000 ms
const uint8_t SILKY_ESPNOW_COMMAND_NEXT_TRACK = (uint8_t)'C';   // 5000 ms
const uint8_t SILKY_ESPNOW_COMMAND_STOP_PLAYBACK = (uint8_t)'D';   // 10000 ms
const uint8_t SILKY_ESPNOW_COMMAND_SET_VOLUME = (uint8_t)'E';


e_soundFX SFX_AHEAD = SFX_NONE;   // default to no sounds, originally set to SFX_PIANO_AHEAD
e_soundFX SFX_TURN_AROUND = SFX_NONE;
e_soundFX SFX_ANTICLOCKWISE = SFX_NONE;
e_soundFX SFX_CLOCKWISE = SFX_NONE;
e_soundFX SFX_UNKNOWN = SFX_NONE;

const char *soundSets[] = {"Piano Sounds", "Organ Sounds", "Pad Sounds","No Sounds",""};
const char** currentSoundSet = soundSets;

// screen Rotation values:
// 1 = Button right
// 2 = Button above
// 3 = Button left
// 4 = Button below

uint8_t initialTextSize = 2;

#include "TinyGPSPlus.h"

// OTA updates start
#define MERCATOR_ELEGANTOTA_MAKO_BANNER
#define MERCATOR_OTA_DEVICE_LABEL "MAKO-IO"

#include <WiFi.h>
#include <Update.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
AsyncElegantOtaClass AsyncElegantOTA;
void uploadOTABeginCallback(AsyncElegantOtaClass* originator);

const int SCREEN_LENGTH = 240;
const int SCREEN_WIDTH = 135;

// const int UPLINK_BAUD_RATE = 9600;
// works for rx from lemon: 922190, 1100000,1500000,1700000,1900000,2100000 (with linger = 3ms)
// does not work for rx from lemon: 921600, 180000
// does not work up to lemon: 91000
// const int UPLINK_BAUD_RATE = 576000;  // max baudrate for mako Rx as Lemon Tx is using a real GPIO pin and this is Lemon Max that Mako can decode.

const uint32_t lingerTimeMsBeforeUplink = 0;
const int UPLINK_BAUD_RATE = 57600;

/*
// added 3ms delay before replying to Lemon when running at 2.1Mbit / sec
// can experiment with 2ms and 1ms.
// without the delay there's 14% loss/miss
const uint32_t lingerTimeMsBeforeUplink = 3; 
const int UPLINK_BAUD_RATE = 2100000 ;
*/

enum e_display_brightness {OFF_DISPLAY = 0, DIM_DISPLAY = 25, HALF_BRIGHT_DISPLAY = 50, BRIGHTEST_DISPLAY = 100};
enum e_uplinkMode {SEND_NO_UPLINK_MSG, SEND_TEST_UPLINK_MSG, SEND_FULL_UPLINK_MSG};


enum  e_mako_displays 
  {
  NAV_COMPASS_DISPLAY, 
  NAV_COURSE_DISPLAY, 
  SURVEY_DISPLAY, 
  LOCATION_DISPLAY,
#ifdef ENABLE_ESP_NOW_DISPLAY
  ESP_NOW_DISPLAY,  // uncomment to bring back
#endif
  JOURNEY_DISPLAY, 
  AUDIO_TEST_DISPLAY,
  COMPASS_CALIBRATION_DISPLAY,
  SHOW_LAT_LONG_DISPLAY_TEMP, 
  NEXT_TARGET_DISPLAY_TEMP, 
  THIS_TARGET_DISPLAY_TEMP,
  AUDIO_ACTION_DISPLAY_TEMP};
  
const e_mako_displays first_display_rotation = NAV_COMPASS_DISPLAY;
const e_mako_displays last_display_rotation = COMPASS_CALIBRATION_DISPLAY;

e_mako_displays display_to_show = first_display_rotation;
e_mako_displays display_to_revert_to = first_display_rotation;

const e_display_brightness ScreenBrightness = BRIGHTEST_DISPLAY;

Preferences persistedPreferences;
bool skipDiagnosticDisplays = true;
String latestConnectedWiFiSSID;

void sendNoUplinkTelemetryMessages();
void sendUplinkTestMessage();
void sendFullUplinkTelemetryMessage();
void saveToEEPROMSkipDiagnosticDisplays();

// feature switches

const e_uplinkMode uplinkMode = SEND_FULL_UPLINK_MSG;

uint16_t sensor_acquisition_time = 0;         // how long the acquireSensorReadings function took to run (including forced wait)
uint16_t max_sensor_acquisition_time = 0;     // maximum sensor acquistion time (including forced wait)
uint16_t actual_sensor_acquisition_time = 0;  // actual sensor acquisition time without forced wait
uint16_t max_actual_sensor_acquisition_time = 0;  // max val of above.

bool enableESPNow = true;
bool ESPNowActive = false;       // will be set to true on startup if set above - can be toggled through interface.
bool isPairedWithSilky = false;
bool isPairedWithTiger = false;
bool isPairedWithOceanic = false;

// Ping response tracking
uint32_t tigerLastPingResponse = 0;
uint32_t silkyLastPingResponse = 0;
uint32_t oceanicLastPingResponse = 0;

void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// compilation switches

void (*fp_sendUplinkMessage)() =  ( uplinkMode == SEND_NO_UPLINK_MSG ? &sendNoUplinkTelemetryMessages :
                                  ( uplinkMode == SEND_TEST_UPLINK_MSG ? &sendUplinkTestMessage :
                                  ( uplinkMode == SEND_FULL_UPLINK_MSG ? &sendFullUplinkTelemetryMessage : &sendNoUplinkTelemetryMessages)));

uint8_t telemetry_message_count = 0;

const uint32_t displayScreenRefreshMinimumInterval = 200; // milliseconds
uint32_t nextDisplayRefreshAt = 0;
bool requestDisplayRefresh = true;  // request display to refresh at startup of loop()

uint32_t map_screen_refresh_minimum_interval = 1000; // milliseconds
uint32_t nextMapScreenRefresh = 0;
bool requestMapScreenRefresh = false;

const String ssid_not_connected = "-";
String ssid_connected;

const bool enableOTAServer = true; // OTA updates
AsyncWebServer asyncWebServer(80);
uint32_t restartAfterGoodOTAUpdateAt = millis() + 3000;
uint32_t restartForGoodOTAScheduled = false;
// OTA updates end

const uint32_t disabledTempDisplayEndTime = 0xFFFFFFFF;
uint32_t showTempDisplayEndTime = disabledTempDisplayEndTime;
const uint32_t showTempDisplayHoldDuration = 5000;
const uint32_t showTempAudioTestDisplayHoldDuration = 2000;
bool firstLoopThroughTempScreen = false;

char uplink_preamble_pattern[] = "MBJAEJ";
char uplink_preamble_pattern2[] = "MBJMBJAEJ";

char uplinkTestMessages[][6] = {"MSG0 ", "MSG1 ", "MSG2 ", "MSG3 "};
char newWayMarkerLabel[2];
char directionMetricLabel[2];

const uint32_t minimumExpectedTimeBetweenFix = 10000;  // 10 seconds for red
const uint32_t gpsTargetYellowTimeout = 3000;        // 3 seconds for yellow

// I2C and framework
#include <Wire.h>                   // I2C framework
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>       // Magnetometer
#include <Adafruit_LSM303_Accel.h>  // Accelerometer
#include <Adafruit_BME280.h>
#include <Adafruit_APDS9960.h>      // colour sensor
#include <MS5837.h>                 // water pressure sensor

const uint16_t SEALEVELPRESSURE_HPA = 1000.00;    // could get Lemon to check atmospheric pressure from weather website and send to Mako to calibrate this

#include <math.h>

enum gpsStatus {GPS_NO_GPS_LIVE_IN_FLOAT, GPS_NO_FIX_FROM_FLOAT, GPS_FIX_FROM_FLOAT};
gpsStatus GPS_status = GPS_NO_GPS_LIVE_IN_FLOAT;
bool hasEverReceivedGPSFix = false;  // Track if GPS fix has ever been received since power-on

const int maxSizeTelemetryMessageToMakoInBytes = 200;
uint16_t telemetryMessage[maxSizeTelemetryMessageToMakoInBytes/sizeof(uint16_t)];

char shortBlackOut[] = "BL";
char shortAntiClockwise[] = "AC";
char shortAhead[] = "AH";
char shortClockwise[] = "CL";
char shortTurnAround[] = "TA";
char shortUnknownMarker[] = "UM";
char shortUndefinedMarker[] = "UD";

char shortCompassHeadingDirectionMetric[] ="CH";
char shortJourneyCourseDirectionMetric[] ="JC";
char shortUndefinedDirectionMetric[] ="JC";

char displayLabel[] = "??";

char navCompassDisplayLabel[] = "CM";
char navCourseDisplayLabel[] = "CO";
char locationDisplayLabel[] = "LO";
char espNowDisplayLabel[] = "ES";
char journeyDisplayLabel[] = "JO";
char showLatLongDisplayLabel[] = "LL";
char audioTestDisplayLabel[] = "AT";
char compassCalibrationDisplayLabel[] = "CC";
char surveyDisplayLabel[] = "SV";
char thisTargetDisplayLabel[] = "TT";
char nextWaypointDisplayLabel[] = "NT";
char audioActionDisplayLabel[] = "AA";
char undefinedDisplayLabel[] = "??";

uint32_t recordHighlightExpireTime = 0;
uint32_t recordHighlightDisplayDuration = 10000;
bool     recordSurveyHighlight = false;

/*
// Dover test
const uint8_t waypointCountDiveOne = 2;
const uint8_t waypointExitDiveOne = 1;

NavigationWaypoint diveOneWaypoints[waypointCountDiveOne] =
{
  [0] = { ._label = "Z06 Wreck Zone", ._m5label = "Z06\nWreck\nZone", ._cat=JETTY, ._lat = 51.066017, ._long = 1.270883},
  [1] = { ._label = "Z06 Wreck Zone", ._m5label = "Z06\nWreck\nZone", ._cat=JETTY, ._lat = 51.066017, ._long = 1.270883},
};

const uint8_t waypointCountDiveTwo = 2;
const uint8_t waypointExitDiveTwo = 1;

NavigationWaypoint diveTwoWaypoints[waypointCountDiveTwo] =
{
  [0] = { ._label = "Z06 Wreck Zone", ._m5label = "Z06\nWreck\nZone", ._cat=JETTY, ._lat = 51.066017, ._long = 1.270883},
  [1] = { ._label = "Z06 Wreck Zone", ._m5label = "Z06\nWreck\nZone", ._cat=JETTY, ._lat = 51.066017, ._long = 1.270883},
};
*/

/*
// Home test
const uint8_t waypointCountDiveOne = 3;
const uint8_t waypointExitDiveOne = 1;

NavigationWaypoint diveOneWaypoints[waypointCountDiveOne] =
{
  [0] = { ._label = "Z04 Home", ._m5label = "Z04\n\nHome", ._cat=JETTY, ._lat = 51.391513052111996, ._long = -0.2874361808098317},
  [1] = { ._label = "98N Home Test", ._m5label = "98N\n\nHome\nTest", ._cat=BLUE_BUOY, ._lat = 51.39140569365721, ._long = -0.2869881590541109},
  [2] = { ._label = "Z04 Home", ._m5label = "Z04\n\nHome", ._cat=JETTY, ._lat = 51.391513052111996, ._long = -0.2874361808098317},
};

const uint8_t waypointCountDiveTwo = 3;
const uint8_t waypointExitDiveTwo = 1;

NavigationWaypoint diveTwoWaypoints[waypointCountDiveOne] =
{
  [0] = { ._label = "Z04 Home", ._m5label = "Z04\n\nHome", ._cat=JETTY, ._lat = 51.391513052111996, ._long = -0.2874361808098317},
  [1] = { ._label = "98N Home Test", ._m5label = "98N\n\nHome\nTest", ._cat=BLUE_BUOY,._lat = 51.39140569365721, ._long = -0.2869881590541109},
  [2] = { ._label = "Z04 Home", ._m5label = "Z04\n\nHome", ._cat=JETTY, ._lat = 51.391513052111996, ._long = -0.2874361808098317},
};
*/

// Dive 1
NavigationWaypoint diveOneWaypoints[] =
{
  { ._label = "Z01 Cafe Jetty", ._m5label = "Z01\n\nCafe\nJetty", ._cat=JETTY, ._lat = 51.460015, ._long = -0.548316},
  { ._label = "10N Bus 2m", ._m5label = "10N\n\nBus\n\n2m", ._cat=NO_BUOY,._lat = 51.460073, ._long = -0.548515},
  { ._label = "X07 Boat with Chain Links", ._m5label = "X07\nChain\nLink\nBoat", ._cat=UNMARKED, ._lat = 51.4600385714286, ._long = -0.548724142857143},
  { ._label = "04N Spitfire Car 6m", ._m5label = "04N\nSpit\n  fire\nCar\n6m", ._cat=NO_BUOY, ._lat = 51.4601028571429, ._long = -0.54883835},
  { ._label = "03N Scimitar Car 5.5m",  ._m5label = "03N\nScimitar\nCar\n5.5m", ._cat=NO_BUOY, ._lat = 51.460347, ._long = -0.5489195},
  { ._label = "X02 Quarry Machine in Reeds", ._m5label = "X02\nQuarry\nMachine\nReeds", ._cat=UNMARKED, ._lat = 51.460434, ._long = -0.548921},
  { ._label = "05N Lightning Boat 5.5m", ._m5label = "05N\nLight\n  ning\nBoat\n5.5m", ._cat=NO_BUOY, ._lat = 51.4605855, ._long = -0.548901666666667},
  { ._label = "06aN Caves Centre", ._m5label = "06aN\nCaves\nCentre", ._cat=NO_BUOY, ._lat = 51.460947625, ._long = -0.54878325},
  { ._label = "12N Commer Van 6m", ._m5label = "12N\n\nCommer\nVan\n\n6m", BLUE_BUOY, ._lat = 51.4613355909091, ._long = -0.548469727272727},
  { ._label = "X21 Memorial Stone - Kit 7.5m", ._m5label = "X21\nMem\nStone\nKit\n7.5m", ._cat=UNMARKED, ._lat = 51.460993, ._long = -0.548006}, // Near die hard taxi
  { ._label = "24N Half Die Hard Taxi 8m", ._m5label = "24N\n\nHalf\nDie\nHard\nTaxi 8m", ._cat=NO_BUOY, ._lat = 51.460773, ._long = -0.547620875},
  { ._label = "X20 La Mouette Boat", ._m5label = "X20\nLa\nMouette\nBoat", ._cat=UNMARKED, ._lat = 51.460740, ._long = -0.547713}, // By die hard taxi
  { ._label = "27B Wreck Site 6m", ._m5label = "27B\n\nWreck\nSite\n\n6m", BLUE_BUOY, ._lat = 51.4604300973436, ._long = -0.547383365365033},
  { ._label = "43N Thorpe Orange Boat 5.5m", ._m5label = "43N\n\nThorpe\nOrange\nBoat\n\n5.5m", ._cat=NO_BUOY, ._lat = 51.4602073333333, ._long = -0.546787666666667},
  { ._label = "35N Dragon Boat 7.5m", ._m5label = "35N\n\nDragon\nBoat\n\n7.5m", ._cat=NO_BUOY, ._lat = 51.4599636666667, ._long = -0.547154333333333},
  { ._label = "46N Plane 6m", ._m5label = "46N\n\nPlane\n\n6m", ._cat=NO_BUOY, ._lat = 51.459745, ._long = -0.546649},
  { ._label = "50aN Swim Through - no crates 6m", ._m5label = "50aN\nSwim\nThrough\nno\ncrates\n\n6m", ._cat=UNMARKED, ._lat = 51.45914367, ._long = -0.546032333},
  { ._label = "X22 Fruit Machine 5.5m", ._m5label = "X22\nFruit\nMach\n5.5m", ._cat=UNMARKED, ._lat = 51.459353, ._long = -0.546939}, // By disused exit + 2 buried boats
  { ._label = "37N Dive Bell 4m", ._m5label = "37N\n\nDive\nBell\n\n4m", ._cat=NO_BUOY, ._lat = 51.4594757058824, ._long = -0.547087117647059},
  { ._label = "23N Traffic Lights 7m", ._m5label = "23N\n\nTraffic\nLights\n\n7m", ._cat=NO_BUOY, ._lat = 51.4600558888889, ._long = -0.547677333333333},
  { ._label = "X25 Dumpy Cylinder 6m", ._m5label = "X25\nDumpy\nCylinder\n6m",._cat=UNMARKED, ._lat = 51.4600631, ._long = -0.5480722},
  { ._label = "X15 Fireworks Launcher", ._m5label = "X15\nFirework\nLauncher", ._cat=UNMARKED, ._lat = 51.4599975, ._long = -0.5481015},
  { ._label = "Z02 Mid Jetty", ._m5label = "Z02\n\nMid\nJetty", ._cat=JETTY, ._lat = 51.459547, ._long = -0.547461},
};

const uint8_t waypointCountDiveOne = sizeof(diveOneWaypoints)/sizeof(NavigationWaypoint); const uint8_t waypointExitDiveOne = waypointCountDiveOne - 1;

// Dive 2
NavigationWaypoint diveTwoWaypoints[] =
{
  { ._label = "Z02 Mid Jetty", ._m5label = "Z02\n\nMid\nJetty", ._cat=JETTY, ._lat = 51.459547, ._long = -0.547461},
  { ._label = "44N VW Camper Van and Seahorse 5.5m", ._m5label = "44N\nVW\nCamper\nVan and\nSeahors\n\n5.5m", ._cat=NO_BUOY, ._lat = 51.459368, ._long = -0.546760142857143},
  { ._label = "51B Orca Van 5.5m", ._m5label = "51B\n\nOrca\nVan\n\n5.5m", BLUE_BUOY, ._lat = 51.4591431428571, ._long = -0.545936857142857},
  { ._label = "50aN Swim Through - no crates 6m", ._m5label = "50aN\nSwim\nThrough\nno\ncrates\n\n6m", ._cat=UNMARKED, ._lat = 51.45914367, ._long = -0.546032333},
  { ._label = "49B Claymore 6.5m", ._m5label = "49B\n\nClay-\n   more\n\n6.5m", BLUE_BUOY, ._lat = 51.459634435324, ._long = -0.54646635372985},
  { ._label = "46N Plane 6m", ._m5label = "46N\n\nPlane\n\n6m", ._cat=NO_BUOY, ._lat = 51.459745, ._long = -0.546649},
  { ._label = "41N Tin/Cabin Boat 7m", ._m5label = "41N\n\nTin\nCabin\nBoat\n\n7m", ._cat=NO_BUOY, ._lat = 51.459676625, ._long = -0.5468125},
  { ._label = "39N London Black Cab 7m", ._m5label = "39N\n\nLondon\nBlack\nCab\n\n7m", ._cat=NO_BUOY, ._lat = 51.459729, ._long = -0.546992857142857},
  { ._label = "38B Lifeboat 6.5m", ._m5label = "38B\n\nLife\nBoat\n\n6.5m", BLUE_BUOY, ._lat = 51.459839375, ._long = -0.5469307},
  { ._label = "35N Dragon Boat 7.5m", ._m5label = "35N\n\nDragon\nBoat\n\n7.5m", ._cat=NO_BUOY, ._lat = 51.4599636666667, ._long = -0.547154333333333},
  { ._label = "25N Boat In A Hole 7m", ._m5label = "25N\n\nBoat In\nA Hole\n\n7m", ._cat=NO_BUOY, ._lat = 51.4599545, ._long = -0.54755475},
  { ._label = "22B Lady of Kent Search Light 5m", ._m5label = "22B\nLady of\nKent\nSearch\nLight\n\n5m", BLUE_BUOY, ._lat = 51.4599185714286, ._long = -0.547681},
  { ._label = "20N Skittles Sweet Bowl 5.5m", ._m5label = "20N\n\nSkittle\nSweet\nBowl\n\n5.5m", ._cat=NO_BUOY, ._lat = 51.4600375, ._long = -0.5478815},
  { ._label = "X12 Cement Mixer", ._m5label = "X12\nCement\nMixer\n", ._cat=UNMARKED, ._lat = 51.46020025, ._long =	-0.5478815},
  { ._label = "18N Milk Float 6.5m", ._m5label = "18N\n\nMilk\nFloat\n\n6.5m", ._cat=NO_BUOY, ._lat = 51.4601745714286, ._long = -0.548058571428571},
  { ._label = "10N Bus 2m", ._m5label = "10N\n\nBus\n\n2m", ._cat=NO_BUOY,._lat = 51.460073, ._long = -0.548515},
  { ._label = "Z01 Cafe Jetty", ._m5label = "Z01\n\nCafe\nJetty", ._cat=JETTY, ._lat = 51.460015, ._long = -0.548316},
};
const uint8_t waypointCountDiveTwo =  sizeof(diveTwoWaypoints)/sizeof(NavigationWaypoint); const uint8_t waypointExitDiveTwo = waypointCountDiveTwo - 1;

const uint8_t* p_currentDiveWaypointCount = nullptr;
const uint8_t* p_currentDiveWaypointExit = nullptr;
NavigationWaypoint* currentDiveWaypoints = nullptr;

NavigationWaypoint* nextWaypoint = nullptr;

void switchDivePlan()
{
  if (p_currentDiveWaypointCount == nullptr || p_currentDiveWaypointCount == &waypointCountDiveTwo)
  {
    p_currentDiveWaypointCount = &waypointCountDiveOne;
    p_currentDiveWaypointExit = &waypointExitDiveOne;
    currentDiveWaypoints = diveOneWaypoints;
    nextWaypoint = currentDiveWaypoints;
  }
  else
  {
    p_currentDiveWaypointCount = &waypointCountDiveTwo;
    p_currentDiveWaypointExit = &waypointExitDiveTwo;
    currentDiveWaypoints = diveTwoWaypoints;
    nextWaypoint = currentDiveWaypoints;
  }
  
  // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
  publishToTigerAndOceanicCurrentTarget(nextWaypoint->_m5label);
}

enum e_way_marker {BLACKOUT_MARKER, GO_ANTICLOCKWISE_MARKER, GO_AHEAD_MARKER, GO_CLOCKWISE_MARKER, GO_TURN_AROUND_MARKER, UNKNOWN_MARKER};
enum e_direction_metric {JOURNEY_COURSE, COMPASS_HEADING};

double heading_to_target = 0, distance_to_target = 0;
double journey_lat = 0, journey_lng = 0, journey_course = 0, journey_distance = 0;
double magnetic_heading = 0;
float mag_accel_x = 0, mag_accel_y = 0, mag_accel_z = 0;
float mag_tesla_x = 0, mag_tesla_y = 0, mag_tesla_z = 0;
float humidity = 0, temperature = 0, air_pressure = 0, pressure_altitude = 0, depth = 0, water_temperature = 0, water_pressure = 0, depth_altitude = 0;
uint16_t red_sensor = 0, green_sensor = 0, blue_sensor = 0, clear_sensor = 0;
const float pressure_correction = 0;  // mbar, calibrated against Braggs Wunderground - not used now

uint32_t next_global_status_display_update = 0;
const uint32_t global_status_display_update_period = 250;   // ms

const uint32_t journey_calc_period = 10000;    // in milliseconds
const uint32_t journey_min_dist = 5;          // in metres

uint32_t last_journey_commit_time = 0;
uint32_t journey_clear_period = 15000;    // clear the journey info after 15 secs inactivity
uint32_t lastWayMarkerChangeTimestamp = 0;
e_way_marker lastWayMarker = BLACKOUT_MARKER;
e_way_marker newWayMarker = BLACKOUT_MARKER;
bool blackout_journey_no_movement = true;
uint8_t activity_count = 0;
void refreshDirectionGraphic(float directionOfTravel, float headingToTarget);

e_direction_metric directionMetric = COMPASS_HEADING;

uint32_t fixCount = 0;
uint32_t noFixCount = 0;
uint32_t newPassedChecksum = 0;
uint32_t newFailedChecksum = 0;
uint32_t uplinkMessageCount = 0;
uint32_t passedChecksumCount = 0;

char activity_indicator[] = "\\|/-";

bool           diveTimerRunning = false;
const float    minimumDivingDepthToRunTimer = 1.0;  // in metres
const float    minimumDivingDepthToActivateLightSensor = 1.0; // in metres
uint16_t       minutesDurationDiving = 0;
uint16_t       whenToStopTimerDueToLackOfDepth = 0;
uint16_t       minsToTriggerStopDiveTimer = 10;

bool recordBreadCrumbTrail = false;

TinyGPSPlus gps;
#define UART_NUMBER_LEMON_FLOAT 2

static constexpr int LEMON_RX_BUFFER_SIZE = 1024;
static constexpr size_t LEMON_RX_READ_CHUNK = 256;
static constexpr int LEMON_RX_QUEUE_SIZE = 10;
static constexpr TickType_t LEMON_RX_TIMEOUT = pdMS_TO_TICKS(100);

QueueHandle_t lemonRxQueue = nullptr;

// Data structure to send via queue
struct LemonDataPacket
{
  uint8_t data[LEMON_RX_READ_CHUNK];
  int length;
};

HardwareSerial lemon_float_serial(UART_NUMBER_LEMON_FLOAT);   // UART number 2: This uses Grove SCL=GPIO33 and SDA=GPIO32 for Hardware UART Tx and Rx

double Lat, Lng;
String  lat_str , lng_str;
int satellites = 0;
char internetUploadStatusGood = false;
int  overrideTarget = -1;
double b, c = 0;
int power_up_no_fix_byte_loop_count = 0;

uint8_t nextUplinkMessage = 0;

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_BME280 Adafruit_TempHumidityPressure;
Adafruit_APDS9960 Adafruit_ColourSensor;
uint16_t red_light=0,green_light=0,blue_light=0,clear_light=0;
uint32_t bytesReceivedFromLemon=0,bytesTransmittedToLemon=0;

uint32_t s_lastColourDisplayRefresh = 0;
const uint32_t s_colourUpdatePeriod = 200; // time between each colour sensor read
const uint8_t maxColourMeasurements=20;
uint16_t colourMeasurements[maxColourMeasurements];
uint8_t colourIndex=0;

const uint32_t readLightTimeWait = 10000;
uint32_t nextLightReadTime = 0;
uint8_t brightLightEvents = 0;
uint16_t currentLightLevel = 0;
bool sendBrightLightEventToTiger = false;
bool sendLightLevelToOceanic = false;

MS5837 BlueRobotics_DepthSensor;

// Stores min and max magnetometer values from calibration

const double initial_min_mag = 1000;
const double initial_max_mag = -1000;

vec<double> calib_magnetometer_max;
vec<double> calib_magnetometer_min;
vec<double> magnetometer_max;
vec<double> magnetometer_min;
vec<double> magnetometer_vector, accelerometer_vector;
vec<float> imu_gyro_vector, imu_lin_acc_vector, imu_rot_acc_vector;
float imu_temperature = 0.0;

// Calibration data collection for soft iron compensation
const uint16_t maxCalibrationSamples = 3000;
const uint32_t calibrationSampleInterval = 20; // Sample every 10ms during calibration
struct CalibrationSample {
  double x, y, z;
};
CalibrationSample calibrationData[maxCalibrationSamples];
uint16_t calibrationSampleCount = 0;
bool collectingCalibrationData = false;
uint32_t lastCalibrationSampleTime = 0;

void getM5ImuSensorData(float* gyro_x, float* gyro_y, float* gyro_z,
                        float* lin_acc_x, float* lin_acc_y, float* lin_acc_z,
                        float* rot_acc_x, float* rot_acc_y, float* rot_acc_z,
                        float* IMU_temperature)
{
  if (enableIMUSensor)
  {
    M5.IMU.getGyroData(gyro_x, gyro_y, gyro_z);
    M5.IMU.getAccelData(lin_acc_x, lin_acc_y, lin_acc_z);
    M5.IMU.getAhrsData(rot_acc_x, rot_acc_y, rot_acc_z);
    M5.IMU.getTempData(IMU_temperature);
  }
  else
  {
    *gyro_x = *gyro_y = *gyro_z = *lin_acc_x = *lin_acc_y = *lin_acc_z = *rot_acc_x = *rot_acc_y = *rot_acc_z = *IMU_temperature = 0.0;
  }
}

// Magnetic Compass averaging and refresh rate control
const uint8_t s_smoothedCompassBufferSize = 10;
double s_smoothedCompassHeading[s_smoothedCompassBufferSize];
uint8_t s_nextCompassSampleIndex = 0;
bool s_smoothedCompassBufferInitialised = false;

bool smoothedCompassCalcInProgress = false;


uint32_t s_lastCompassNotSmoothedDisplayRefresh = 0;
const uint32_t s_compassNotSmoothedHeadingUpdateRate = 100; // Reduced from 200ms to 150ms for more responsive updates

uint32_t s_lastSendTestSerialBytesTest = 0;
const uint32_t s_sendTestSerialBytesPeriodMs = 1000;

uint32_t s_lastTempHumidityDisplayRefresh = 0;
const uint32_t s_tempHumidityUpdatePeriod = 250; // time between each humidity and depth update to screen

uint32_t s_lastCallToAcquireSensorData = 0;
const uint32_t s_acquireSensorDataUpdatePeriod = 100; // allow the compass to update ok even if there are not Messages being received from the surface.

const uint8_t  BUTTON_GOPRO_TOP_GPIO = 25;
const uint8_t  BUTTON_GOPRO_SIDE_GPIO = 0;    // can't use this at startup - strapping pin
const uint8_t  M5_BUTTON_A_PIN = BUTTON_A_PIN;
const uint8_t  M5_BUTTON_B_PIN = BUTTON_B_PIN;
// GPIO 38 has no internal pull-up resistor so needs to have external pull-up to 3.3V. That's why not working.
const uint8_t  REED_SWITCH_GPIO = 38;      // input triggers with finger proximity - not now used - new input only input - on white wire of Mako
const uint32_t MERCATOR_DEBOUNCE_MS = 0;

const uint8_t HAT_GPS_RX_GPIO = 26;
const uint8_t HAT_GPS_TX_GPIO = 2;
const uint8_t IR_LED_GPS_TX_GPIO = 9;

const bool switchActionInverted = true;
Button BtnGoProTop = Button(BUTTON_GOPRO_TOP_GPIO, switchActionInverted, MERCATOR_DEBOUNCE_MS);
Button BtnGoProSide = Button(BUTTON_GOPRO_SIDE_GPIO, switchActionInverted, MERCATOR_DEBOUNCE_MS); 
// There is no internal pull-up on GPIO 38 so the Button implementation that uses INPUT_PULLUP will be 
// ignored or you need to override and set it to INPUT instead.
// Button ReedGoProBottomRight = Button(REED_SWITCH_GPIO, switchActionInverted, MERCATOR_DEBOUNCE_MS);
uint16_t sideCount = 0, topCount = 0;

bool topGoProButtonActiveAtStartup = false;
bool sideGoProButtonActiveAtStartup = false;
bool goProReedActiveAtStartup = false;

Button* p_primaryButton = NULL;
Button* p_secondButton = NULL;
void updateButtons();

const uint32_t CLEARED_FIX_TIME_STAMP = 0xF0000000;
uint32_t latestFixTimeStamp = CLEARED_FIX_TIME_STAMP;
uint32_t latestFixTimeStampStreamOk = 0;

uint32_t latestNoFixTimeStamp = 0;
uint32_t newNoFixCount = 0;
uint32_t latestGPSMessageTimeStamp = 0;

// Magnetic heading calculation functions
template <typename T> double calculateTiltCompensatedHeading(vec<T> from);
template <typename Ta, typename Tb, typename To> void vector_cross(const vec<Ta> *a, const vec<Tb> *b, vec<To> *out);
template <typename Ta, typename Tb> float vector_dot(const vec<Ta> *a, const vec<Tb> *b);
void vector_normalize(vec<double> *a);
bool useGetDepthAsync = true;

bool enableButtonTestMode = false;

float hallOffset = 0;  // Store the initial value of magnetic force
const float magnetHallReadingForReset = -50;

void updateButtons()
{
  p_primaryButton->read();
  p_secondButton->read();
}

bool isTopGoProButtonPressed() { // Direct GPIO Read Bypass button press code
  return digitalRead(BUTTON_GOPRO_TOP_GPIO) == false;
}

bool isSideGoProButtonPressed() { // Direct GPIO Read Bypass button press code
  return digitalRead(BUTTON_GOPRO_SIDE_GPIO) == false;
}

bool isButtonAPressed() { // Direct GPIO Read Bypass button press code
  return digitalRead(M5_BUTTON_A_PIN) == false;
}

bool isButtonBPressed() { // Direct GPIO Read Bypass button press code
  return digitalRead(M5_BUTTON_B_PIN) == false;
}

void setPrimaryControls(const bool useGoProButtons)
{
  if (useGoProButtons)
  {
    p_primaryButton = &BtnGoProTop;
    p_secondButton = &BtnGoProSide;
  }
  else
  {
    p_primaryButton = &M5.BtnA;
    p_secondButton = &M5.BtnB;
  }
}

char rxQueueItemBuffer[256];
const uint8_t queueLength=4;

void dumpHeapUsage(const char* msg)
{  
  if (writeLogToSerial)
  {
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
    USB_SERIAL_PRINTF("\n%s : free heap bytes: %i  largest free heap block: %i min free ever: %i\n",  msg, info.total_free_bytes, info.largest_free_block, info.minimum_free_bytes);
  }
}

bool haltAllProcessingDuringOTAUpload = false;

bool forceLoopInitialOTAEnablement = false;

const char* buildTimestamp = __DATE__ " " __TIME__;

void readPreferencesFromEEPROM()
{
  // Initialize preferences and load skipDiagnosticDisplays from EEPROM
  persistedPreferences.begin("mako_config", false);
  skipDiagnosticDisplays = persistedPreferences.getBool("skipDiag", true);
  latestConnectedWiFiSSID = persistedPreferences.getString("lastSSID", "");
}


void lemonRxTask(void *arg)
{
  LemonDataPacket packet;
  for (;;)
  {
    if (!haltAllProcessingDuringOTAUpload)
    {
      int bytesRead = uart_read_bytes(UART_NUMBER_LEMON_FLOAT, packet.data, sizeof(packet.data), LEMON_RX_TIMEOUT);
      if (bytesRead > 0)
      {
        packet.length = bytesRead;

        // Send packet to main loop via FreeRTOS queue (don't block if queue is full)
        if (xQueueSend(lemonRxQueue, &packet, 0) != pdTRUE)
        {
            // Queue full - increment dropped packet counter
            // Could add statistics tracking here
        }
      }
    }
    else
    {
      delay(100);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// PROTECTED - DO NOT ADD CODE IN THE BELOW PROTECTED AREA - RISK OF OTA FAILURE
////////////////////////////////////////////////////////////////////////////////////////////////////
bool systemStartupAndCheckForOTADemand()
{
  delay(1000);
  M5.begin(/* LCD Enable */ true, /* Power Enable */ true,/* Serial Enable */ false, /* Buzzer Enable */ false);

  readPreferencesFromEEPROM();

  pinMode(RED_LED_GPIO, OUTPUT); // Red LED - the interior LED to M5 Stick
  setRedLEDOff();

  ssid_connected = ssid_not_connected;

  // Always initialise Serial over the USB connection - whether writeLogToSerial is enabled or not
  Serial.begin(115200);

  // wait one second - terminate wait if a button is held
  uint32_t start = millis();
  while(millis() < start + 1000 && !disableESPNowIfSideButtonHeld() &&  !enableOTAAtStartupIfTopButtonHeld());

  if (topGoProButtonActiveAtStartup)
  {
    // OTA to be enabled
    haltAllProcessingDuringOTAUpload = true;
    forceLoopInitialOTAEnablement = true;
    disableFeaturesForOTA(); 
  }

  return haltAllProcessingDuringOTAUpload;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// PROTECTED - DO NOT ADD CODE IN THE ABOVE PROTECTED AREA - RISK OF OTA FAILURE
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  //////// PROTECTED - DO NOT ADD CODE BEFORE THE OTA DEMAND CHECK  - RISK OF OTA FAILURE
  if (systemStartupAndCheckForOTADemand())
    return;     // OTA Required, skip rest of setup.
  //////// PROTECTED - DO NOT ADD CODE BEFORE THE OTA DEMAND CHECK  - RISK OF OTA FAILURE

  espNowMsgsReceivedQueue = xQueueCreate(queueLength,sizeof(rxQueueItemBuffer));

  lemonRxQueue = xQueueCreate(LEMON_RX_QUEUE_SIZE, sizeof(LemonDataPacket));

  switchDivePlan();   // initialise to dive one

  M5.Lcd.setRotation(0);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);

  M5.Axp.ScreenBreath(ScreenBrightness);

  setPrimaryControls(goProButtonsPrimaryControl);

  if (enableESPNowAtStartup)
    toggleESPNowActive();

  lemon_float_serial.setRxBufferSize(512); // was 256 - must set before begin called

  // Had to solder to GPIO2 on the main board and cut the header pin to the HAT mezzenine board as there 
  // were some components there preventing the UART from working. Also was cutting power to I2C.
  const bool invert = false;
  lemon_float_serial.begin(UPLINK_BAUD_RATE, SERIAL_8N2, HAT_GPS_RX_GPIO, HAT_GPS_TX_GPIO, invert);   // pin 26=rx, 9=tx specifies the HAT pin for Rx and the IR LED for Tx (not used)                

  // Create Lemon RS485 receive task on Core 1
  xTaskCreatePinnedToCore(lemonRxTask,
                          "lemonRxTask",
                          4096,    // stack size
                          nullptr, // user parameters to pass to task
                          7,       // task priority
                          &lemonTaskHandle, // task handle
                          1);      // core id

  initSensors();

  acquireAllSensorReadings(); // compass, IMU, Depth, Temp, Humidity, Pressure

  delay(1000);

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setCursor(0, 0);
}

uint32_t OTAUploadFlashLEDTimer = 0;
const uint32_t OTAUploadFlashAwaitLEDPeriodicity = 500;
const uint32_t OTAUploadFlashInProgressLEDPeriodicity = 250;

uint32_t OTAUploadFlashCurrentLEDPeriodicity = OTAUploadFlashAwaitLEDPeriodicity;
uint32_t recoveryScreenStartTime = 0;
bool recoveryScreenShown = false;

bool cutShortLoopOnOTADemand()
{
  // DO WE NEED TO ADD TEAR DOWN ESP NOW IF IT IS ACTIVE HERE?
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////// PROTECTED - DO NOT ADD CODE IN THE BELOW PROTECTED AREA - RISK OF OTA FAILURE
  ////////////////////////////////////////////////////////////////////////////////////////////////////  
  if (haltAllProcessingDuringOTAUpload)
  {
    // Handle OTA restart if scheduled
    if (restartForGoodOTAScheduled && millis() >= restartAfterGoodOTAUpdateAt) {
        ESP.restart();
    }

    if (forceLoopInitialOTAEnablement)
    {
      forceLoopInitialOTAEnablement = false;
      const bool wifiOnly = false;
      const int maxWifiScanAttempts = 3;
      otaActive = connectToWiFiAndInitOTA(wifiOnly,maxWifiScanAttempts);
    }

    if (!recoveryScreenShown) 
    {
      showOTARecoveryScreen();
      recoveryScreenStartTime = millis();
      recoveryScreenShown = true;
    }

    if (millis() > OTAUploadFlashLEDTimer)
    {
      OTAUploadFlashLEDTimer += OTAUploadFlashCurrentLEDPeriodicity;
      toggleRedLED();
    }

    // After 5 seconds of recovery screen, allow restart if any button is pressed 
    if (recoveryScreenShown && (millis() - recoveryScreenStartTime > 5000)) 
    {            
      // check if either button is pressed once 5 seconds has passed since ota screen was shown
      bool topPressed = digitalRead(BUTTON_GOPRO_TOP_GPIO) == false;
      bool sidePressed = digitalRead(BUTTON_GOPRO_SIDE_GPIO) == false;
      
      if (topPressed || sidePressed) {
        M5.Lcd.fillScreen(TFT_GREEN);
        M5.Lcd.setCursor(0,10);
        M5.Lcd.setTextSize(3);
        M5.Lcd.println(" ########### ");
        M5.Lcd.println("#           #");
        M5.Lcd.println("# Rebooting #");
        M5.Lcd.println("#           #");
        M5.Lcd.println(" ########### ");
        delay(1000);
        esp_restart();
      }
    }
  }
  return haltAllProcessingDuringOTAUpload;

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////// PROTECTED - DO NOT ADD CODE IN THE ABOVE PROTECTED AREA - RISK OF OTA FAILURE
  ////////////////////////////////////////////////////////////////////////////////////////////////////
}

/////////////// EVENT LOOP
void loop()
{
  //////// PROTECTED - DO NOT ADD CODE BEFORE OR WITHIN THE OTA DEMAND CHECK BELOW  - RISK OF OTA FAILURE
  if (cutShortLoopOnOTADemand())
    return;
  ///////////////////////////////////////////////////////////////////////////////////////
  
  uint32_t now = millis();

  updateButtons();

  // check for incoming messages
  processIncomingESPNowMessages();
   
  if (useGetDepthAsync)
    getDepthAsync(depth, water_temperature, water_pressure, depth_altitude);
  
  processGPSMessageIfAvailable();

  if  (s_lastCallToAcquireSensorData + s_acquireSensorDataUpdatePeriod <= now)
  {
    acquireAllSensorReadings(); // compass, IMU, Depth, Temp, Humidity, Pressure
    s_lastCallToAcquireSensorData = now;
    requestDisplayRefresh = true;
  }

  if (sendBrightLightEventToTiger)
    publishToTigerBrightLightEvent();
    
  if (sendLightLevelToOceanic)
    publishToOceanicLightLevel(currentLightLevel);
  
  if (requestDisplayRefresh || now > nextDisplayRefreshAt)
  {
    checkDivingDepthForTimer(depth);
    refreshDisplay();

    requestDisplayRefresh = false;
    nextDisplayRefreshAt = now + displayScreenRefreshMinimumInterval;
  }

  if (requestMapScreenRefresh || now > nextMapScreenRefresh)
  {
    // UPDATED NEEDED HERE TO USE MASTER NAV WAYPOINTS CODE /////
    publishToTigerAndOceanicLocationAndTarget(nextWaypoint->_m5label);

    requestMapScreenRefresh = false;
    nextMapScreenRefresh = now + map_screen_refresh_minimum_interval;
  }  

//  refreshGlobalStatusDisplay();     // I think needs some debugging

  refreshDiveTimer();

  checkForButtonPresses();
}

void scanI2C()
{
  M5.Lcd.println("Scan I2C...");
  /*
    Wire1: 1st I2C Bus (pins 32 and 33 - see /.platformio/packages/framework-arduinoespressif32/variants/m5stick_c/pins_arduino.h

    MS5837 Depth:                     0x76
    BME280 Temp/Humid/Baro:           0x77
    MPU6886 M5 IMU:                   0x68
    LIS2MDL Magnetometer:             0x1E
    LSM303  Accelerometer:            0x19
    APDS9960 Colour/Proximity/Gesture 0x39

    Wire1: 2nd I2C Bus (pins 21 and 22 - internal to M5)

    AXP192 M5 Power Mgmt IC:          0x34
  */
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t rc = Wire.endTransmission();
    if (rc == 0) M5.Lcd.printf("Found 0x%02X\n", addr);
  }
}

bool isGPSStreamOk()
{
  return (millis() - latestGPSMessageTimeStamp < minimumExpectedTimeBetweenFix);
}

bool isLatestGPSMsgFix()
{
  return  (latestFixTimeStampStreamOk > latestNoFixTimeStamp);
}

bool isGPSTargetShortTimedOut()
{
  return (millis() - latestGPSMessageTimeStamp >= gpsTargetYellowTimeout);
}

bool isInternetUploadOk()
{
  return internetUploadStatusGood;
}

bool processGPSMessageIfAvailable()
{ 
  LemonDataPacket lemonPacket;

  bool result = false;

  if (xQueueReceive(lemonRxQueue, &lemonPacket, 0) == pdTRUE)
  {
    result = true;

    for (int i = 0; i < lemonPacket.length; i++)
    {
      if (gps.encode(lemonPacket.data[i]))
      {
        // Process GPS messages regardless of whether location is valid (handles both FIX and NO FIX)
        uint32_t newFixCount = gps.sentencesWithFix();
        uint32_t newNoFixCount = gps.sentencesWithNoFix();
        newPassedChecksum = gps.passedChecksum();
        newFailedChecksum = gps.failedChecksum();
        
        bool messageCountChanged = false;
        bool hasValidLocation = gps.location.isValid();
        
        if (newFixCount > fixCount)
        {
          fixCount = newFixCount;
          messageCountChanged = true;

          USB_SERIAL_PRINTF("\nFix: %lu Good Msg: %lu Bad Msg: %lu", fixCount, newPassedChecksum, gps.failedChecksum());

          latestFixTimeStampStreamOk = latestFixTimeStamp = millis();
          latestGPSMessageTimeStamp = millis();  // Update fix message timestamp only
        }
        else if (newNoFixCount > noFixCount)
        {
          noFixCount = newNoFixCount;
          messageCountChanged = true;
          latestNoFixTimeStamp = millis();
          // Don't update latestGPSMessageTimeStamp for no-fix messages
          
          USB_SERIAL_PRINTF("\nNo Fix: %lu Good Msg: %lu Bad Msg: %lu", noFixCount, newPassedChecksum, gps.failedChecksum());
        }
        
        if (messageCountChanged)
        {
          if (power_up_no_fix_byte_loop_count > -1)
          {
            // clear the onscreen counter that increments whilst attempting to get first valid location
            power_up_no_fix_byte_loop_count = -1;
            M5.Lcd.fillScreen(TFT_BLACK);
          }

          if (newPassedChecksum <= passedChecksumCount)
          {
            // incomplete message received, continue reading bytes, don't update display.
            // continue reading bytes
            result = false;
          }
          else
          {
            passedChecksumCount = newPassedChecksum;

            if (gps.isSentenceGGA())      // only send uplink message back for GGA messages
            {
              // Respond to both FIX and NO FIX GGA messages to maintain communication with Lemon
              if (gps.isSentenceContainingValidFix())
              {
                // Valid GPS fix - update positional data and respond normally
                refreshAndCalculatePositionalAttributes();
                hasEverReceivedGPSFix = true;  // Mark that we've received a valid GPS fix
              }
              // Always respond with sensor data regardless of GPS fix status
              performUplinkTasks();

              acquireAllSensorReadings(); // compass, IMU, Depth, Temp, Humidity, Pressure
      
              checkDivingDepthForTimer(depth);
    
              refreshDisplay();
      
              checkForButtonPresses();
                
              result = true;
            }
            else if (gps.isSentenceRMC())
            {
              if (gps.date.year() == 2000)      // fake data received from float for No GPS.
              {
                GPS_status = GPS_NO_GPS_LIVE_IN_FLOAT;
              }
              else if (gps.date.year() == 2012) // fake data received from float for No Fix.
              {
                GPS_status = GPS_NO_FIX_FROM_FLOAT;
              }
              else
              {
                GPS_status = GPS_FIX_FROM_FLOAT;
                hasEverReceivedGPSFix = true;   // Mark that we've received a fix
              }
              result = false;
            }
          }
        }
        else
        {
          // get location invalid if there is no new fix to read before 1 second is up.
          if (power_up_no_fix_byte_loop_count > -1)
          {
            // Bytes are being received but no valid location fix has been seen since startup
            // Increment byte count shown until first fix received.
            M5.Lcd.setCursor(50, 90);
            M5.Lcd.printf("%d", power_up_no_fix_byte_loop_count++);
          }
          result = false;
        }
      }
      else
      {
        // no byte received.
        result = false;
      }
    }
  }

  return result;
}

void saveToEEPROMSkipDiagnosticDisplays() 
{
  persistedPreferences.putBool("skipDiag", skipDiagnosticDisplays);
}

#define BUILD_INCLUDE_MAIN_BUTTON_PRESS_CODE
#include "main_button_press_code.cpp"

#define BUILD_INCLUDE_MAIN_TELEMETRY_CODE
#include "main_telemetry_code.cpp"

#define BUILD_INCLUDE_MAIN_SENSOR_CODE
#include "main_sensor_code.cpp"

#define BUILD_INCLUDE_MAIN_DISPLAY_CODE
#include "main_display_code.cpp"

#define BUILD_INCLUDE_MAIN_ESP_NOW_MESSAGES_CODE
#include "main_esp_now_messages_code.cpp"

#define BUILD_INCLUDE_MAIN_NETWORK_CODE
#include "main_network_code.cpp"

#define BUILD_INCLUDE_MAIN_ESP_NOW_CODE
#include "main_esp_now_code.cpp"

#define BUILD_INCLUDE_MAIN_TEST_CODE
#include "main_test_code.cpp"
