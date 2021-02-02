/*
“latitude longitude weatherCode temperature himidity airQuality speed direction”
*/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <dht11.h>

#include "RTWL_constants.h"
#include "functions.h"

/* Define all functions */
void (*resetFunc)(void) = 0;
void checkIntervalToSendData(bool forceSend = false);
void sendState(int state, int value = NULL);
void GPSReceive();
void SendWithBluetooth(char *msg);
void GetTempAndHumidity();
int GetWeather();
bool CheckValidity(int weather, int temp, int hum, int air);

//SoftwareSerial Bluetooth(PIN_BT_RX, PIN_BT_TX); // RX | TX
SoftwareSerial serialGPS(PIN_GPS_RX, PIN_GPS_TX);
TinyGPS gps;
dht11 DHT11; // The temperature and humidity module

/* Variables for checking functionality of sensors */
unsigned int gpsFailures;
bool proxymityFailure; // Assigned true when receiving a bluetooth command (can't detect sensor defection)
unsigned int airFailures;

/* GPS variabiles */
bool sendGPSCoordinates = false;
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;
unsigned long GPS_read_delay; // for counting 1 second - parsing gps data
unsigned long timeDelayForGpsRead;

/* For proxymity sensor - wiper frequency */
unsigned long lastTimeWiper = 0;
bool firstWiperMoveDetected = false;
double period = 0;

/* Wind sensor variables */
int windSensorValue = 0;

/* Strings for Bluetooth communication */
char *msg_received; // receive data from bluetooth device
char *msg_to_send;  // final message to send

char *coords;          // coordinates received from GPS module
char *lastCoordinates; // store the last valid coordinates
int weatherCode = -1;  /* result of the function that uses some sensors to determinate the weather; (1:4) -> (sun, rain, wind, snowfall)*/
int *lastDTHValues;    // store the last valid temperature and humidity
int air = -1;          // air quality sensor; -1 is an invalid value (0:100)%
int speed = -1;        // speed of the system - if the GPS module is installed in a mobile system
char *direction;       // the GPS mobile direction (ex: N, NW, NWN, NE, SW)

unsigned int dhtSensorIsWorkingWell; // count failed measures of sensor: >=3 means error
unsigned long timeDelaySensorsRead;  // time delay between sensors reading
bool readyForNewCommand;             // prevent receiving command from bluetooth device until the current command is done

void setup()
{
  // Initialize pins
  pinMode(PIN_AIR_QUALITY, INPUT);
  pinMode(PIN_PROXY_SENSOR, INPUT);
  pinMode(PIN_WIND_SENSOR, INPUT);

  // Assign memory to arrays
  msg_received = (char *)malloc(SIZE_RECEIVING_STRING);
  msg_to_send = (char *)malloc(SIZE_MESSAGE_TO_SEND);
  coords = (char *)malloc(SIZE_COORDINATES);
  lastCoordinates = (char *)malloc(SIZE_COORDINATES);
  lastDTHValues = (int *)malloc(2);
  direction = (char *)malloc(4);
  strcpy(msg_received, "");
  strcpy(msg_to_send, "");
  strcpy(coords, "");
  strcpy(lastCoordinates, "");
  strcpy(direction, "U"); // the unknown direction is represented by "U"

  // Set failures variables with fail values
  // When the program starts, the sensors values are not known
  // If all sensors are working, these values will be overwritten
  gpsFailures = FAILURES_GPS;
  proxymityFailure = false; /* this sensor can't verified - only the user can notify the failure */
  airFailures = FAILURES_AIR_QUALITY;

  // Initialize variabiles with invalid values (to detect errors)
  lastDTHValues[0] = LIMIT_MAX_TEMPERATURE + 1;
  lastDTHValues[1] = LIMIT_MAX_HUMIDITY + 1;
  air = LIMIT_MAX_AIR + 1;
  dhtSensorIsWorkingWell = 3;

  GPS_read_delay = lastTimeWiper = timeDelaySensorsRead = timeDelayForGpsRead = millis();
  readyForNewCommand = true;

  // Read [counter] times the sensors to check the functionality
  int counter = NUMBER_OF_READINGS_STARTUP;
  while (counter--)
  {
    checkIntervalToSendData();
    delay(MAXIMUM_TIME_READINGS_STARTUP / NUMBER_OF_READINGS_STARTUP);
  }

  // Initialize serial communication
  Bluetooth.begin(BT_BAUD_RATE);
  serialGPS.begin(GPS_BAUD_RATE);

  // Clear the buffer - all the data received before the program started
  while (serialGPS.available())
    serialGPS.read();
  while (Bluetooth.available())
    Bluetooth.read();

  // Everything is ready to start
  Bluetooth.print(STATE_START);
}

void loop()
{
  BluetoothReceive(); //check if a command was send through Bluetooth
  GPSReceive();       //clear buffer and update current location

  calculatePeriod(); //detect proxy sensor for wiper frequency
  checkIntervalToSendData();

  delay(10);
}

/* Receive data from Bluetooth module through serial port RX. */
void BluetoothReceive()
{
  // Check if the program is ready for new command - the last command ended
  // Check if there are any data received
  if (!readyForNewCommand || Bluetooth.available() == 0)
    return;
  // Clear variables to start the reading
  readyForNewCommand = false;
  int index = 0;
  strcpy(msg_received, "");
  delay(10);
  while (Bluetooth.available())
  {
    // Check if the command is too long
    if (index >= SIZE_RECEIVING_STRING - 1)
    {
      // The command is too long
      index = 0;
      strcpy(msg_received, "");
    }
    char ch = Bluetooth.read(); // reading char by char the data received
    // Check if the current char received is a number or a letter
    if (isNumberOrLetter(ch))
    {
      msg_received[index++] = ch;
    }
  }
  msg_received[index] = '\0'; // add the string terminator
  // Make the command uppercase
  // This allow the user to write the commands in any way (ex: Command, CoMMand, command, CoMmaNd)
  toUppercase(index, msg_received);
  // Perform the received command
  performCommands();
  readyForNewCommand = true;
}

/* Send to Bluetooth module the program state (ex: errors, sensors failure).
  The state can have values (ex: sensors value).*/
void sendState(char state, int value = NULL)
{
  Bluetooth.print(state); // send the state latter
  Bluetooth.print(":");   // send the separator
  if (value != NULL)
  {
    // Send the state value if exists
    Bluetooth.print(state);
  }
  Bluetooth.print(SEND_MESSAGE_DELIMITER);
}

/* Call all functions for reading all sensors */
void getAllData()
{
  weatherCode = GetWeather();
  strcpy(msg_to_send, lastCoordinates);
  timeDelayForGpsRead = millis();
  GetTempAndHumidity();
  air = map(analogRead(PIN_AIR_QUALITY), 0, 1024, 0, 100);
}

/* Read the sensors and modules every amount of time.
  If all data is valid, the function will send the data to Bluetooth module */
void checkIntervalToSendData(bool force = false)
{
  if (force || millis() - timeDelaySensorsRead > READING_DELAY)
  {
    // Call all functions for reading all sensors
    getAllData();
    if (CheckValidity())
    {
      setMessageToSend();
      for (int i = 0; i < strlen(msg_to_send); i++)
        Bluetooth.print(msg_to_send[i]);
      Bluetooth.print(SEND_MESSAGE_DELIMITER);
    }
    timeDelaySensorsRead = millis();
  }
}

/* Set the message using the format to send to Bluetooth module.
  Format is: <GPS_coordinates> <weather_code> <temperature> <humidity> <airQuality> <speed> <direction> */
void setMessageToSend()
{
  char tmp[30];
  // Add the weather code
  strcpy(tmp, "");
  itoa(weatherCode, tmp, 10);
  strcat(msg_to_send, " ");
  strcat(msg_to_send, tmp);
  // Add the temperature
  itoa(lastDTHValues[0], tmp, 10);
  strcat(msg_to_send, " ");
  strcat(msg_to_send, tmp);
  // Add the humidity
  itoa(lastDTHValues[1], tmp, 10);
  strcat(msg_to_send, " ");
  strcat(msg_to_send, tmp);
  // Add the air quality
  itoa(air, tmp, 10);
  strcat(msg_to_send, " ");
  strcat(msg_to_send, tmp);
  // Add the speed
  itoa(speed, tmp, 10);
  strcat(msg_to_send, " ");
  strcat(msg_to_send, tmp);
  // Add the direction
  strcat(msg_to_send, " ");
  strcat(msg_to_send, direction);
}

/* Send the GPS coordinates */
void sendCoordinates(float flat, float flon)
{
  char *buff;
  if (sendGPSCoordinates)
  {
    buff = (char *)malloc(10);
    dtostrf(flat, 10, 6, buff);
    Bluetooth.print(buff);
    Bluetooth.print(" ");
    dtostrf(flon, 10, 6, buff);
    Bluetooth.print(buff);
    Bluetooth.print(SEND_MESSAGE_DELIMITER);
  }
}

/* Receive the GPS data. */
void GPSReceive()
{
  static char *result = (char *)malloc(SIZE_COORDINATES);
  static char *tmp = (char *)malloc(5);

  // Wait at least a second between GPS data reading
  if (millis() - GPS_read_delay > ONE_SECOND) // for counting 1 second - parsing gps data
  {
    while (serialGPS.available())
    {
      // Read the incoming data char by char
      char c = serialGPS.read();
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  // GPS module sent the whole sentence
  if (newData)
  {
    newData = false;
    float flat, flon;
    unsigned long age;

    // Get the GPS latitude, longitude and age
    gps.f_get_position(&flat, &flon, &age);

    if (flat == TinyGPS::GPS_INVALID_F_ANGLE || flon == TinyGPS::GPS_INVALID_F_ANGLE)
    {
      gpsFailures++; // count the GPS failures
      sendState(STATE_GPS_INVALID_DATA);
      return;
    }

    // Get the direction
    strcpy(direction, degrees2direction(gps.f_course()));
    // Get the speed
    speed = (int)gps.f_speed_kmph();

    // Create the coordinates string
    dtostrf(flat, 5, 2, tmp);
    strcpy(result, tmp);
    strcat(result, " ");
    dtostrf(flon, 5, 2, tmp);
    strcat(result, tmp);
    strcat(result, "\0");
    lastCoordinates = result;

    // Send the GPS coordinates
    sendCoordinates(flat, flon);

    gpsFailures = 0;
    GPS_read_delay = millis();
  }
  else
  {
    gpsFailures++;
  }
}

/* Get the temperature and humidity from the DHT11 module.
  Store the values if are valid and in the limits. */
void GetTempAndHumidity()
{
  int chk = DHT11.read(PIN_DHT11);
  int tmp = DHT11.temperature;
  int hum = DHT11.humidity;

  if (chk == -2 || tmp > 100 || hum > 100 || hum < 0)
  {
    // When this variable becomes greater than 2 -> the sensor have a problem
    dhtSensorIsWorkingWell++;
    return;
  }
  else if (tmp >= LIMIT_MIN_TEMPERATURE && tmp <= LIMIT_MAX_TEMPERATURE && hum >= LIMIT_MIN_HUMIDITY && hum <= LIMIT_MAX_HUMIDITY)
  {
    dhtSensorIsWorkingWell = 0;
    lastDTHValues[0] = DHT11.temperature;
    lastDTHValues[1] = DHT11.humidity;
  }
}

/* Calculate the period between two detections, reading the proximity sensor. */
void calculatePeriod()
{
  // Read proximity sensor
  // Check if this is the first detection of the sensor
  if (!firstWiperMoveDetected)
  {
    // This is the first detection
    if ((!digitalRead(PIN_PROXY_SENSOR)) && millis() - lastTimeWiper > 1200)
    {
      firstWiperMoveDetected = true;
      proxymityFailure = false; // proxy sensor working again (if it was declared as not working)
      // Calculate the period of time from the last detection
      period = millis() - lastTimeWiper;
      lastTimeWiper = millis();
    }
  }
  else
  {
    // This is not the first detection
    if ((!digitalRead(PIN_PROXY_SENSOR)) && millis() - lastTimeWiper > 300)
    {
      firstWiperMoveDetected = false;
    }
  }
  // Check if there was detections in the last seconds
  if (millis() - lastTimeWiper > 10000)
  {
    period = 0; // wiper not moved for 10 seconds -> 0 Hz
    lastTimeWiper = millis();
  }
}

/* Get the weather code using the sensors values */
int GetWeather()
{
  int temp = lastDTHValues[0];
  int hum = lastDTHValues[1];
  //calculate weather code
  // 1xx: sun, 2xx: rain, 3xx: wind: 4xx: snowfall
  // xx = 00:99 (intensity)
  int sun_weather = map(temp, 15, LIMIT_MAX_TEMPERATURE, 100, 199); //sunny, sun or heat weather

  // It's SUN
  if (period == 0 && windSensorValue <= 200 && temp >= 15)
    return sun_weather;
  // It's RAIN - wipers are moving (period > 0)
  else if (period > 0 && temp <= 15 && hum >= 50)
  {
    if (period < 3333.33)
      return map(period, 1666.66, 3333.33, 200, 233);
    else if (period < 1666.66)
      return map(period, 1111.11, 1666.66, 234, 266);
    else if (period < 1111.11)
      return map(period, 0, 1111.11, 267, 299);
    else
      return sun_weather;
  }
  // It's WIND
  else if (windSensorValue > 0 && hum < 30 && temp < 15)
    return map(windSensorValue, 0, 1024, 300, 399);
  // It's WINTER - wipers are active for snowfall
  else if (period > 0 && temp < 5 && hum > 50)
    if (period < 0.90f)
      return map(period, 0.01f, 0.90f, 400, 499);

  return sun_weather;
}

/* Check if the sensors values are valid and in limits */
bool CheckValidity()
{
  bool valid = true;
  // Check if the GPS failed a couples of times
  if (gpsFailures >= FAILURES_GPS)
  {
    // GPS not working
    strcpy(lastCoordinates, "0.0 0.0");
  }
  /* Check if the weather code is correct */
  if (weatherCode < LIMIT_MIN_WEATHER_CODE || weatherCode > LIMIT_MAX_WEATHER_CODE)
  {
    sendState(STATE_WEATHER_CODE_INVALID, weatherCode);
    valid = false;
  }
  /* Check if the DHT11 sensor is working */
  if (dhtSensorIsWorkingWell > 2)
  {
    // After 2 failed measured -> sensor problem
    sendState(STATE_TEMP_HUM_SENSOR_NOT_WORKING);
  }
  /* Check if the temperature is valid */
  if (lastDTHValues[0] < LIMIT_MIN_TEMPERATURE || lastDTHValues[0] > LIMIT_MAX_TEMPERATURE)
  {
    sendState(STATE_TEMP_INVALID_DATA, lastDTHValues[0]);
    valid = false;
  }
  /* Check if the humidity is valid */
  if (lastDTHValues[1] < LIMIT_MIN_HUMIDITY || lastDTHValues[1] > LIMIT_MAX_HUMIDITY)
  {
    sendState(STATE_HUM_INVALID_DATA, lastDTHValues[1]);
    valid = false;
  }
  // Check if the air quality is valid
  if (air < LIMIT_MIN_AIR || air > LIMIT_MAX_AIR)
  {
    airFailures++;
    if (airFailures >= FAILURES_AIR_QUALITY)
      sendState(STATE_AIR_SENSOR_NOT_WORKING);
    sendState(STATE_AIR_SENSOR_INVALID_DATA, air);
    valid = false;
  }
  airFailures = 0;
  return valid;
}

/* Perform the commands received from Bluetooth module. */
void performCommands()
{
  if (strlen(msg_received) == 0)
    return;
  if (strcmp(msg_received, COMMAND_RESET_INTEGER) == 0 || strcmp(msg_received, COMMAND_RESET_SHORT) == 0 || strcmp(msg_received, COMMAND_RESET_LONG) == 0)
    resetFunc();
  else if (strcmp(msg_received, COMMAND_GET_RESPONSE_INTEGER) == 0 || strcmp(msg_received, COMMAND_GET_RESPONSE_SHORT) == 0 || strcmp(msg_received, COMMAND_GET_RESPONSE_LONG) == 0)
  {
    Bluetooth.print("PONG!");
    Bluetooth.print(SEND_MESSAGE_DELIMITER);
  }
  else if (strcmp(msg_received, COMMAND_GET_DATA_INTEGER) == 0 || strcmp(msg_received, COMMAND_GET_DATA_SHORT) == 0 || strcmp(msg_received, COMMAND_GET_DATA_LONG) == 0)
    checkIntervalToSendData(true);
  else if (strcmp(msg_received, COMMAND_PROXIMITY_SENSOR_DEFECTION_INTEGER) == 0 || strcmp(msg_received, COMMAND_PROXIMITY_SENSOR_DEFECTION_SHORT) == 0 || strcmp(msg_received, COMMAND_PROXIMITY_SENSOR_DEFECTION_LONG) == 0)
  {
    proxymityFailure = true; //user detected that proxy sensor is not working
    sendState(STATE_PROXY_SENSOR_NOT_WORKING);
  }
  else if (strcmp(msg_received, COMMAND_GET_GPS_COORDINATES_INTEGER) == 0 || strcmp(msg_received, COMMAND_GET_GPS_COORDINATES_SHORT) == 0 || strcmp(msg_received, COMMAND_GET_GPS_COORDINATES_LONG) == 0)
  {
    sendGPSCoordinates = true;
    sendState(STATE_GET_GPS_COORDINATES_ENABLED);
  }
  else if (strcmp(msg_received, COMMAND_DISABLE_GET_GPS_COORDINATES_INTEGER) == 0 || strcmp(msg_received, COMMAND_DISABLE_GET_GPS_COORDINATES_SHORT) == 0 || strcmp(msg_received, COMMAND_DISABLE_GET_GPS_COORDINATES_LONG) == 0)
  {
    sendGPSCoordinates = false;
    sendState(STATE_GET_GPS_COORDINATES_DISABLED);
  }
  else
  {
    Bluetooth.print(UNKNOWN_COMMAND);
    Bluetooth.print(SEND_MESSAGE_DELIMITER);
  }
}
