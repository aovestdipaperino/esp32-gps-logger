/**************************************************************
 *
 * NB-IOT/CAT-M SIM7080G based Azure IOTHub GPS logger
 * (c) 2024 - Enzo Lombardi, Juna Salviati
 **************************************************************/

#define TINY_GSM_MODEM_SIM7080
#define SerialMon Serial

// #define DUMP_AT_COMMANDS

// Set serial for AT commands (to the module)
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
#include <Arduino.h>
#include <ArduinoHttpClient.h>
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"
#include "iot_configs.h"
#include <TinyGsmClient.h>
#include <math.h>
#include <TimeLib.h>
#include "UUID.h"
#include <CircularBuffer.h>
#if __has_include("test_iot_configs.h")
#include "test_iot_configs.h"
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

XPowersPMU PMU;
#define R 6371000
#define TO_RAD (3.1415926536 / 180)
#define MAX_PAYLOAD_SIZE 200 

char payload[MAX_PAYLOAD_SIZE];
const String EMPTY = "";

double haversine(double lat1, double lon1, double lat2, double lon2)
{
  double dx, dy, dz;
  lon1 -= lon2;
  lon1 *= TO_RAD, lat1 *= TO_RAD, lat2 *= TO_RAD;

  dz = sin(lat1) - sin(lat2);
  dx = cos(lon1) * cos(lat1) - cos(lat2);
  dy = sin(lon1) * cos(lat1);
  return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;
}

float _lat = 0;
float _lon = 0;
float _speed = 0;
float _alt = 0;
int _vsat = 0;
int _usat = 0;
float _accuracy = 0;
int _year = 0;
int _month = 0;
int _day = 0;
int _hour = 0;
int _mins = 0;
int _sec = 0;

bool refresh_gps()
{
  return modem.getGPS(&_lat, &_lon, &_speed, &_alt, &_vsat, &_usat, &_accuracy,
                      &_year, &_month, &_day, &_hour, &_mins, &_sec);
}

enum MovementStatus
{
  STOPPED,
  ON_THE_MOVE
};
MovementStatus currentMovementStatus = MovementStatus::STOPPED;
time_t lastMovement = 0;
UUID uuid;

String generateTelemetryPayload()
{
  static float prev_lon;
  static float prev_lat;

  if (refresh_gps())
  {
    double dist = haversine(_lat, _lon, prev_lat, prev_lon);
    DBG("Distance (m): " + String(dist, 4) + " curr(" + String(_lat, 6) + ", " + String(_lon, 6) + ") prev(" + String(prev_lat, 6) + ", " + String(prev_lon, 6) + ") speed: " + String(_speed, 4));
    prev_lon = _lon;
    prev_lat = _lat;
    if (dist > 1.0)
    {
      uint8_t chargeState = -99;
      int8_t percent = -99;
      uint16_t milliVolts = -9999;
      modem.getBattStats(chargeState, percent, milliVolts);
      tmElements_t t = {_sec, _mins, _hour, _day, _month, _year};
      time_t timestamp = makeTime(t);
      if (_speed > 0.0)
      {
        lastMovement = timestamp;
        if (currentMovementStatus == MovementStatus::STOPPED)
        {
          currentMovementStatus = MovementStatus::ON_THE_MOVE;
          uuid.generate();
        }
      }
      else
      {
        time_t currentTime = timestamp;
        if (((currentTime - lastMovement) > 5 * 60) && (currentMovementStatus == MovementStatus::ON_THE_MOVE))
        {
          currentMovementStatus = MovementStatus::STOPPED;
          uuid.generate();
        }
      }
      snprintf(payload, MAX_PAYLOAD_SIZE - 1, "{ \"timestamp\": %d%02d%02d%02d%02d%02d, \"long\": %.6f, \"lat\": %.6f, \"speed\": %f, \"batt\": %d, \"id\": \"%s\", \"st\": %d }",
               _year,
               _month,
               _day,
               _hour,
               _mins,
               _sec,
               _lon,
               _lat,
               _speed,
               percent,
               uuid.toCharArray(),
               (int)currentMovementStatus);

      // Creates a copy
      return String(payload);
    }
    DBG("Skipping measurement. Too close.");
  }
  return EMPTY;
}
void restartRadio()
{
  // Pull down PWRKEY for more than 1 second according to manual requirements
  DBG("Restarting SIMCOM radio");
  digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
}

void setup()
{
  uuid.generate();
  // Set console baud rate
  Serial.begin(115200);

  /*********************************
   *  step 1 : Initialize power chip,
   *  turn on modem and gps antenna power channel
   ***********************************/
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL))
  {
    Serial.println("Failed to initialize power.....");
    while (1)
    {
      delay(5000);
    }
  }
  // Set the working voltage of the modem, please do not modify the parameters
  PMU.setDC3Voltage(3300); // SIM7080 Modem main power channel 2700~ 3400V
  PMU.enableDC3();

  // Modem GPS Power channel
  PMU.setBLDO2Voltage(3300);
  PMU.enableBLDO2(); // The antenna power must be turned on to use the GPS function

  // TS Pin detection must be disable, otherwise it cannot be charged
  PMU.disableTSPinMeasure();

  /*********************************
   * step 2 : start modem
   ***********************************/

  Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);

  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_DTR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_RI_PIN, INPUT);

  int retry = 0;
  while (!modem.testAT(1000))
  {
    Serial.print(".");
    if (retry++ > 6)
    {
      restartRadio();
      retry = 0;
      Serial.println("Retry start modem .");
    }
  }
  Serial.println();
  Serial.print("Modem started!");
  DBG("Wait...");
  delay(6000);

  // Set GSM module baud rate
  modem.setBaud(115200);

  // When configuring GNSS, you need to stop GPS first
  modem.disableGPS();
  delay(1500);
  modem.sendAT("+CGNSMOD=1,1,0,0,0");
  modem.waitResponse();

  modem.sendAT("+SGNSCMD=2,1000,0,0");
  modem.waitResponse();

  // Turn off GNSS.
  modem.sendAT("+SGNSCMD=0");
  modem.waitResponse();
  DBG("GPS is now setup");
  delay(500);
  // SerialAT.begin(9600);
}

enum ModemStatus
{
  SUCCESS = 0,
  SIM_NOT_WORKING,
  CANNOT_RESTART_MODEM,
  NETWORK_CONNECT_FAIL,
  APN_CONNECT_FAIL,
  GPRS_CONNECT_FAIL,
  MESSAGE_PUBLISH_FAIL
};

const char *ModemStatusStrings[] = {
    "Modem started",
    "SIM Error",
    "Failed to restart modem",
    "Can't connect to network",
    "Can't connect to APN",
    "Can't connect to GPRS",
    "IOTHUB publish fail",
};

bool sendMessage(const char *aServer, const char *aToken, String deviceName, String postData)
{

  TinyGsmClientSecure secureClient(modem, 1);

  const int securePort = 443;
  DBG("Connecting securely to", aServer);
  TinyGsmClientSecure client(modem);
  HttpClient http(client, aServer, securePort);
  http.beginRequest();
  http.connectionKeepAlive();
  http.post("/devices/" + deviceName + "/messages/events?api-version=2020-03-13");
  http.sendHeader("Content-Length", postData.length());
  http.sendHeader("Authorization: " + String(aToken));
  http.beginBody();
  http.print(postData);
  http.endRequest();

  int responseCode = http.responseStatusCode();
  DBG("Response Code", responseCode);

  return responseCode == 204;
}

#define SIM_ERROR_RETRIES 5
int simErrorRetry = SIM_ERROR_RETRIES;

ModemStatus enableModem()
{
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DBG("Initializing modem...");
  if (modem.getSimStatus(5000) != SimStatus::SIM_READY)
  {
    --simErrorRetry;
    if (simErrorRetry == 1)
    {
      restartRadio();
    }
    if (simErrorRetry == 0)
    {
      esp_restart();
    }
    return ModemStatus::SIM_NOT_WORKING;
  }
  simErrorRetry = SIM_ERROR_RETRIES;
  if (!modem.restart())
  {
    DBG("Failed to restart modem, delaying 10s and retrying");
    return ModemStatus::CANNOT_RESTART_MODEM;
  }

  DBG("Waiting for network...");
  if (!modem.waitForNetwork(30000L, true))
  {
    return ModemStatus::NETWORK_CONNECT_FAIL;
  }

  if (modem.isNetworkConnected())
  {
    DBG("Network connected");
  }

  DBG("Connecting to", APN);
  if (!modem.gprsConnect(APN))
  {
    return ModemStatus::APN_CONNECT_FAIL;
  }

  if (modem.isGprsConnected())
  {
    DBG("GPRS status: connected");
    IPAddress local = modem.localIP();
    DBG("Local IP:", local);
    int csq = modem.getSignalQuality();
    DBG("Signal quality:", csq);
    return ModemStatus::SUCCESS;
  }
  DBG("GPRS status: not connected");
  return ModemStatus::GPRS_CONNECT_FAIL;
}

void wait_for_gps_lock(int retries)
{
  while (!refresh_gps() && retries--)
  {
    delay(1000);
  }
}

#define MAX_GPS_RETRIES 10
int gpsAttempts = MAX_GPS_RETRIES;
CircularBuffer messageBuffer;
void loop()
{
  static long lastTelemetryEvent = 0;
  if (modem.enableGPS())
  {
    DBG("Waiting for GPS to lock in position.");
    PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
    // If the buffer is empty, the GPS can keep trying to lock.
    // Otherwise return after 2 mins.
    wait_for_gps_lock(messageBuffer.empty() ? 1000000 : 120);
    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
    String payload = generateTelemetryPayload();
    if (!payload.isEmpty())
    {
      if (messageBuffer.isFull())
      {
        // discard the oldest message
        messageBuffer.pop();
      }
      // Empty means discard the measurement for heuristics (distance, speed, etc.)
      messageBuffer.push(payload);
    }
    modem.disableGPS(); // enable
  }
  else
  {
    modem.disableGPS();
    if (messageBuffer.empty())
    {
      gpsAttempts--;
      if (gpsAttempts == 1)
      {
        restartRadio();
      }
      if (gpsAttempts == 0)
      {
        esp_restart();
      }
      DBG("Enable GPS failed, reattempting later");
      delay(3000);
      return;
    }
  }
  gpsAttempts = MAX_GPS_RETRIES;
  if (messageBuffer.empty())
  {
    DBG("Nothing to upload. Sleeping.");
    delay(3000);
    return;
  }

  // If we are here there is something to upload
  PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
  ModemStatus status = enableModem();
  // if modem doesn't connect, add the payload to the Q.
  if (status != ModemStatus::SUCCESS)
  {
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    DBG(ModemStatusStrings[status]);
    delay(10000);
    return;
  }
  PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
  while (!messageBuffer.empty())
  {
    String telemetry = messageBuffer.pop();
    DBG("Sending: ", telemetry);
    if (!sendMessage(ENDPOINT, DEVICE_SAS_TOKEN, IOT_CONFIG_DEVICE_ID, telemetry))
    {
      DBG("Error with telemetry");
    }
  }
  PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);

  modem.gprsDisconnect();
  delay(5000L);
  DBG(modem.isGprsConnected() ? "GPRS disconnect: Failed." : "GPRS disconnected");
  delay(TELEMETRY_FREQUENCY_MILLISECS);
}
