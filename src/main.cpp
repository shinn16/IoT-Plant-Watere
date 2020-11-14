#include <Arduino.h>
#include <FFUpdates.h>
#include <MQTT.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "settings.hpp"

// GPIO pins
#define trigPin D1
#define echoPin D2
#define relay D4
#define soilSatPin A0

// whether or not debug statements will print
#define testing True

// define the debug statement to print only when testing is defined
#ifdef testing
  #define DEBUG(statement) Serial.println(statement)
  #define DEBUG_ONELINE(statement) Serial.print(statement)
#else
  #define DEBUG(statment) 
  #define DEBUG_ONELINE(statement)
#endif

// Function Prototypes
inline void stopPump();
inline void startPump();
inline int min(int a, int b);
inline int max(int a, int b);
int getWaterLevel();
int getAverageWaterLevel(int samples);
float getSaturation();
void settingsUpdate(String &topic, String &payload);
//----- Global Variables -----
int waterLevel          = 0;
float saturation        = 0.0;
// reservoir settings
float reservoirVolume   = 0.0;
float reservoir2D       = 0.0;
// soil sensor settings
float minimumSaturation = 20.0;
int dry                 = 850;
int saturated           = 445;
int maxValue            = dry - saturated;
// ultrasonic sensor settings
float sensorHeight      = 19.5;
int minimumWaterLevel   = 20;
int maxSanityFails      = 100;
int ultrasonicSensorResolution = 0.03;   

MQTTClient mqtt;
WiFiClient network;

/**
 * Setup function, runs once upon system startup or when waking up from deepsleep.
 * 
 * @returns void.
 */ 
void setup() {
  float reservoirWidth  = 12.7; // default width in cm
  float reservoirLength = 12.7; // default length in cm
  float reservoirHeight = 17.5; // default height of reservoir in cm

  reservoir2D     = reservoirWidth * reservoirLength;
  reservoirVolume = reservoir2D * reservoirHeight;

  DEBUG_ONELINE("Resevoir Volume: ");
  DEBUG(reservoirVolume);

  #ifdef testing
    Serial.begin(9600);
  #endif
  DEBUG("Hello world");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relay, OUTPUT);
  pinMode(soilSatPin, INPUT);

  // disable the pump at startup
  stopPump();
  delay(500);

  // connect to network
  WiFi.begin(ssid, pk);
  while(WiFi.status() != WL_CONNECTED) delay(100);
  DEBUG_ONELINE("Connected to network, IP: ");
  DEBUG(WiFi.localIP());

  // connect to MQTT
}


/**
 * Main function loop.
 * 
 * @returns void.
 */ 
void loop() {
  waterLevel = getWaterLevel();
  saturation = getSaturation();
  DEBUG_ONELINE("Saturation: ");
  DEBUG_ONELINE(saturation);
  DEBUG("%");
  DEBUG_ONELINE("Water Level: ");
  DEBUG_ONELINE(waterLevel);
  DEBUG("%");

  while(saturation <  minimumSaturation && waterLevel > minimumWaterLevel){
    startPump();
    waterLevel = getWaterLevel();
    saturation = getSaturation();
    DEBUG_ONELINE("Saturation: ");
    DEBUG_ONELINE(saturation);
    DEBUG("%");
    DEBUG_ONELINE("Water Level: ");
    DEBUG_ONELINE(waterLevel);
    DEBUG("%");
    delay(500);
  }
  
  stopPump();
  int time = 80;
  while(time > 0){
    waterLevel = getAverageWaterLevel(10);
    saturation = getSaturation();
    DEBUG_ONELINE("Saturation: ");
    DEBUG_ONELINE(saturation);
    DEBUG("%");
    DEBUG_ONELINE("Water Level: ");
    DEBUG_ONELINE(waterLevel);
    DEBUG("%");
    time --;
    delay(2000);
  }
}

/**
 * Starts the pump. Inline convience function.
 * 
 * @returns void.
 */ 
inline void startPump(){
  digitalWrite(relay, LOW);
}


/**
 * Stops the pump. Inline convience function.
 * 
 * @returns void.
 */ 
inline void stopPump(){
  digitalWrite(relay, HIGH);
}


/**
 * Gets the water level by percent left in the reservoir as
 * read by the ultrasonic sensor.
 * 
 * @returns the distance read in centimeters
 */ 
int getWaterLevel(){
  static float lastLevel = -1.0;
  static float lastSaneLevel = 0.0;
  static float sanityFails = 0;

  long duration = 0;
  float level = 0;

  // get the sensor data
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  delayMicroseconds(2);

  // calculate distance
  level = (float(duration) / 29.0)/ 2.0;  // get the height of the water left in cm
  level = sensorHeight - level;      // get current height of the water         
  level *= reservoir2D;              // convert to total volume left
  level /= reservoirVolume;          // get the percent left of the original volume
  
  if (lastLevel != -1 && fabs(lastLevel - level) > ultrasonicSensorResolution){ // prevent random spikes based on bad sensor readings
    sanityFails ++;
    DEBUG("Sanity fail");
    if( sanityFails > 1 && sanityFails <= maxSanityFails) { // if we are have repeated sanity failues, start taking a weighted averaged based on the number of fails
      lastLevel = (lastSaneLevel * (maxSanityFails - float(sanityFails)/float(maxSanityFails))) + (level *  (float(sanityFails)/float(maxSanityFails)));
      lastLevel /= 100.0; // keeps the range within the proper decimal place
      DEBUG_ONELINE("last level: ");
      DEBUG(lastLevel);
      DEBUG_ONELINE("current level: ");
      DEBUG(level);
    }
    else if(sanityFails > maxSanityFails){ // if we surpass 100 fails, accept the current level as the last sane level
      lastSaneLevel = lastLevel = level;
    }

    return int(lastSaneLevel * 100.0); 
  }
  lastSaneLevel = lastLevel = level;
  sanityFails = 0;
  return int(level * 100.0);
}

/**
 * A more stable version of getWaterLevel. This fucntion makes multiple calls to getWaterLevel
 * and averages the reported value in an attempt to smooth the sensor errors.
 * Should be used for reporting telemetry. This is a simple moving average.
 * 
 * @param samples number of samples to take for averaging
 * @return the average value of the water level
 */
int getAverageWaterLevel(int samples){
  int waterLevel = 0;
  for(int i = 0; i < samples; i++){
     waterLevel += getWaterLevel();
     delay(150);
  }
  return waterLevel/samples;
}

/**
 * Get the current soil saturation as a percentage
 * 
 * @returns the saturation value of the soil as a percentage 
 */ 
float getSaturation(){
  int rawReading = 0;

  rawReading = analogRead(soilSatPin);
  rawReading -= saturated;                // subract the lowest value it should ever be
  rawReading = max(rawReading, 0);        // if somehow we get something lower, rewrite to 0
  rawReading = min(rawReading, maxValue); // if the value is greater than the max, make it the max
  return float(100.0 - ((float(rawReading)/float(maxValue)) * 100.0)); // return the saturation as a percent
}

/**
 * Determins the min of two numbers
 * 
 * @param a first value to compare
 * @param b second value to compare
 * @returns minimum of the compared values
 */ 
inline int min(int a, int b) {
  return (a > b? b : a);
}


/**
 * Determines the max of two numbers
 * 
 * @param a first value to compare
 * @param b second value to compare
 * @returns maximum of the compared values
 */
inline int max(int a, int b){
  return (a > b? a : b);
}

/**
 * Updates device settings based on provided mqtt changes
 * 
 * @param topic the MQTT topic that fired this event
 * @param payload the payload coming from the MQTT topic
 * @returns void. 
 */ 
void settingsUpdate(String &topic, String &payload) {
  float resevoirWidth   = 0.0;
  float reservoirlength = 0.0;
  float reservoirHeight = 0.0;
  StaticJsonDocument<sizeof(payload)> json;

  // deserialize our settings json string, if this fails, exit and return to main loop
  DeserializationError err = deserializeJson(json, payload);
  if(err){
    DEBUG_ONELINE("Deserialization Error: ");
    DEBUG(err.c_str());
    return;
  }
  Serial.println("incoming: " + topic + " - " + payload);
}

/**
 * MQTT connection setup.
 * 
 * @param willTopic MQTT will topic
 * @param broker MQTT broker IP/Hostname
 * @param clientID unique ID for this device
 * @param mqttUser MQTT username
 * @param mqttPass MQTT password
 * @returns void.
 */ 
void mqtt_connect(char* willTopic, char* broker, char* clientID, char* mqttUser, char* mqttPass){
  mqtt.setWill(willTopic, "offline", true, 0);
  mqtt.setOptions(60, true, 6000);
  mqtt.begin(broker, network);
  DEBUG_ONELINE("Connecting to MQTT Broker: ");
  DEBUG(broker);
  while(!mqtt.connect(clientID, mqttUser, mqttPass)){
    delay(500);
  }
  mqtt.publish(willTopic, "online", true, 0); // birth message
  DEBUG("Connected to MQTT!");
}