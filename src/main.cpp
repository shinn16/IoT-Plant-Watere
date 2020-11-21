#include <Arduino.h>
#include <FFUpdates.h>
#include <MQTT.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "settings.hpp"

// GPIO pins
#define trigPin 5
#define echoPin 4
#define relay 2
#define soilSatPin A0

// whether or not debug statements will print
#define testing

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
void publishTelemetry();
void network_connect();
uint32_t calculateCRC32(const uint8_t *data, size_t length);


//----- Global Variables -----
int waterLevel          = 0;
float saturation        = 0.0;

struct {
  // crc for data validation on reset
  uint32_t crc32;   
  // reservoir settings
  float reservoirVolume;
  float reservoir2D; 
  // soil sensor settings
  float minimumSaturation;
  int dry;
  int saturated;
  int maxValue;
  // ultrasonic sensor settings
  float sensorHeight;
  int minimumWaterLevel;
} settings;
 

MQTTClient mqtt(256);
WiFiClient network;

/**
 * Setup function, runs once upon system startup or when waking up from deepsleep.
 * 
 * @returns void.
 */ 
void setup() {

  #ifdef testing
    Serial.begin(9600);
  #endif
  DEBUG("Hello world");

  if(ESP.rtcUserMemoryRead(0, (uint32_t*) &settings, sizeof(settings))) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*) &settings) + 4, sizeof(settings) - 4 );
    if(crc == settings.crc32) {
      DEBUG("Restored settings");
    }else{
      /**
       * Here is where you should put your default settings.
       */ 
      DEBUG("Failed to restore settings");
      float reservoirWidth  = 12.7; // default width in cm
      float reservoirLength = 12.7; // default length in cm
      float reservoirHeight = 17.5; // default height of reservoir in cm

      settings.reservoir2D     = reservoirWidth * reservoirLength;
      settings.reservoirVolume = settings.reservoir2D * reservoirHeight;
      
      // soil sensor settings
      settings.minimumSaturation   = 20.0;
      settings.dry                 = 850;
      settings.saturated           = 445;
      settings.maxValue            = settings.dry - settings.saturated;

      // ultra sonic sensor water level settings
      settings.sensorHeight        = 19.5;
      settings.minimumWaterLevel   = 20;
    }
  }
  // cacluate checksum. The =/- 4 skips the checksum in the struct
  settings.crc32 = calculateCRC32(((uint8_t*) &settings) + 4, sizeof(settings) - 4); 
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &settings, sizeof(settings));
  DEBUG("==========================================");
  DEBUG("Applied settings:");
  DEBUG_ONELINE("\tReservoir Volume: ");
  DEBUG(settings.reservoirVolume);
  DEBUG("\tSoil Sat Sensor:");
  DEBUG_ONELINE("\t\tminimumSaturation: ");
  DEBUG(settings.minimumSaturation);
  DEBUG_ONELINE("\t\tdry: ");
  DEBUG(settings.dry);
  DEBUG_ONELINE("\t\tsaturated: ");
  DEBUG(settings.saturated);
  DEBUG_ONELINE("\t\tmaxValue: ");
  DEBUG(settings.maxValue);
  DEBUG("\tUltrasonic Sensor:");
  DEBUG_ONELINE("\t\tsensorHeight: ");
  DEBUG(settings.sensorHeight);
  DEBUG_ONELINE("\t\tminimumWaterLevel: ");
  DEBUG(settings.minimumWaterLevel);
  DEBUG("==========================================");


  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relay, OUTPUT);
  pinMode(soilSatPin, INPUT);

  // disable the pump at startup
  stopPump();
  delay(500);

  // connect to WiFi MQTT
  mqtt.setOptions(60, true, 6000);
  mqtt.begin(broker, network);
  mqtt.onMessage(settingsUpdate);
  network_connect();
}


/**
 * Main function loop.
 * 
 * @returns void.
 */ 
void loop() {
  mqtt.loop();
  delay(10);  

  if (!mqtt.connected()) {
    network_connect();
  }
  waterLevel = getWaterLevel();
  saturation = getSaturation();
  DEBUG_ONELINE("Saturation: ");
  DEBUG_ONELINE(saturation);
  DEBUG("%");
  DEBUG_ONELINE("Water Level: ");
  DEBUG_ONELINE(waterLevel);
  DEBUG("%");

  while(saturation <  settings.minimumSaturation && waterLevel > settings.minimumWaterLevel){
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
  waterLevel = getAverageWaterLevel(10);
  saturation = getSaturation();
  DEBUG_ONELINE("Saturation: ");
  DEBUG_ONELINE(saturation);
  DEBUG("%");
  DEBUG_ONELINE("Water Level: ");
  DEBUG_ONELINE(waterLevel);
  DEBUG("%");
  publishTelemetry();
  delay(sample_rate);
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
  level = settings.sensorHeight - level;      // get current height of the water         
  level *= settings.reservoir2D;              // convert to total volume left
  level /= settings.reservoirVolume;          // get the percent left of the original volume
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
  static int sanityFails = 0;
  float data[samples];
  float outlierThreshold = 3;
  float average = 0;
  for(int i =0; i < samples; i ++){
    float level = -1.0;
    while(level < 0){ // case in which the sensor reading is a fluke
      level = getWaterLevel();
    }
    data[i] = level;
    delay(500);
  }

  // now we need to error check the data and ensure we eliminate any outliers the best we can.
  bool outlier = true;
  while(outlier){
    outlier = false;
    for(int i=0; i < samples; i ++) average += data[i];
    average /= samples; // averaging

    DEBUG("Outlier detection value: " + String(average));
    for(int i=0; i < samples; i ++){
      if(abs(data[i] - average) > outlierThreshold){
        DEBUG("outlier detected: " + String(data[i]));
        data[i] = average;
        outlier = true;
      }
    }
     average = 0;  // reseting average start value
  }
  // final summing of the data and averaging it.
  for(int i=0; i < samples; i ++) average += data[i];
  average /= samples; // average the data.
  DEBUG("Final Distance: " + String(average));
  if(abs(waterLevel - average) > 15 && sanityFails < 1){
    DEBUG("Failed sanity check");
    sanityFails ++;
    return waterLevel;
  }
  sanityFails = 0;
  return round(average);
}

/**
 * Get the current soil saturation as a percentage
 * 
 * @returns the saturation value of the soil as a percentage 
 */ 
float getSaturation(){
  int rawReading = 0;

  rawReading = analogRead(soilSatPin);
  rawReading -= settings.saturated;                // subract the lowest value it should ever be
  rawReading = max(rawReading, 0);        // if somehow we get something lower, rewrite to 0
  rawReading = min(rawReading, settings.maxValue); // if the value is greater than the max, make it the max
  return float(100.0 - ((float(rawReading)/float(settings.maxValue)) * 100.0)); // return the saturation as a percent
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
  float reservoirLength = 0.0;
  float reservoirHeight = 0.0;
  const size_t capacity = JSON_OBJECT_SIZE(8) + 30;
  DynamicJsonDocument json(capacity);

  // deserialize our settings json string, if this fails, exit and return to main loop
  DeserializationError err = deserializeJson(json, payload);
  if(err){
    DEBUG_ONELINE("Deserialization Error: ");
    DEBUG(err.c_str());
    return;
  }
  DEBUG("incoming: " + topic + " - " + payload);

  // apply settings
  resevoirWidth              = json["rw"];
  reservoirLength            = json["rl"];
  settings.reservoir2D       = resevoirWidth * reservoirLength;
  reservoirHeight            = json["rh"];
  settings.reservoirVolume   = settings.reservoir2D * reservoirHeight;
  settings.minimumSaturation = json["ms"];
  settings.sensorHeight      = json["sh"];
  settings.minimumWaterLevel = json["ml"];
  settings.dry               = json["dr"];
  settings.saturated         = json["st"];
  settings.maxValue          = settings.dry - settings.saturated;

  settings.crc32 = calculateCRC32(((uint8_t*) &settings) + 4, sizeof(settings) - 4); 
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &settings, sizeof(settings));
}

/**
 * WiFi/MQTT connection setup.
 * 
 * @returns void.
 */ 
void network_connect(){

  if(WiFi.status() != WL_CONNECTED){
    WiFi.begin(ssid, pk);
    while(WiFi.status() != WL_CONNECTED) delay(100);
    DEBUG_ONELINE("Connected to network, IP: ");
    DEBUG(WiFi.localIP());
  }
  
  mqtt.setWill(stateTopic, "Offline", true, 0);
  DEBUG_ONELINE("Connecting to MQTT Broker: ");
  DEBUG(broker);
  while(!mqtt.connect(mqttID, mqttUser, mqttPass)){
    delay(500);
  }

  DEBUG("Connected to MQTT!");
  mqtt.subscribe(settingsTopic);
  mqtt.publish(stateTopic, String("Online"), true, 2);
}

/**\
 * Published telemetry over MQTT.
 * 
 * @returns void.
 */
void publishTelemetry(){
  char buffer[32];
  String publish;
  sprintf(buffer, "{\"wtr_lvl\": %d, \"sat\": %d}", waterLevel, int(saturation));
  publish = String(buffer);
  DEBUG_ONELINE("JSON to publish: ");
  DEBUG(publish);
  mqtt.publish(attributesTopic, publish, true, 2);
}

/**
 * Calculates the crc value of the settings..
 * 
 * @param data data to perform crc check on
 * @param length size of the data
 * @returns crc32
 */
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while(length--) {
    uint8_t c = *data++;
    for(uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if(c & i) {
        bit = !bit;
      }

      crc <<= 1;
      if(bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}