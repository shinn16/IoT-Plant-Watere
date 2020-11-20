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
 

MQTTClient mqtt(256);
WiFiClient network;

/**
 * Setup function, runs once upon system startup or when waking up from deepsleep.
 * 
 * @returns void.
 */ 
void setup() {
  // TODO read from flash for default values

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
  level = sensorHeight - level;      // get current height of the water         
  level *= reservoir2D;              // convert to total volume left
  level /= reservoirVolume;          // get the percent left of the original volume
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
  Serial.println("incoming: " + topic + " - " + payload);

  // apply settings
  resevoirWidth     = json["rw"];
  reservoirLength   = json["rl"];
  reservoir2D       = resevoirWidth * reservoirLength;
  reservoirHeight   = json["rh"];
  reservoirVolume   = reservoir2D * reservoirHeight;
  minimumSaturation = json["ms"];
  sensorHeight      = json["sh"];
  minimumWaterLevel = json["ml"];
  dry               = json["dr"];
  saturated         = json["st"];

  // TODO store these values in flash for reload on POR
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

void publishTelemetry(){
  char buffer[32];
  String publish;
  sprintf(buffer, "{\"wtr_lvl\": %d, \"sat\": %d}", waterLevel, int(saturation));
  publish = String(buffer);
  DEBUG_ONELINE("JSON to publish: ");
  DEBUG(publish);
  mqtt.publish(attributesTopic, publish, true, 2);
}