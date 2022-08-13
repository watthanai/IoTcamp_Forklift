// Read 3-axis accelerometer by NodeMCU ESP8266
#define Switch_PIN  27
#define LEDG_PIN 32
#define Buzzer_PIN 33
#define LEDB_PIN 12
#define LEDR_PIN 14
#define LED_IN 2




// MQTT Broker
const char *mqtt_broker = "xxxxx";  
const char *topic = "xxxxxx";
const char *mqtt_username = "";        //
const char *mqtt_password = "";
const int mqtt_port = 1883;


extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <WiFi.h>
#include <Wire.h>
#include "src/ArduinoJson.h"
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

WiFiClient espClient;
PubSubClient client(espClient);

long previousMillis = 0;
long interval = 500;



////////////////////////
// Config Wifi//
////////////////////////
typedef struct {

  char ssid[64] = "xxxxx";
  char pass[64] = "xxxxxx";
} config_wf;
config_wf cfg_wifi;
TimerHandle_t wifiReconnectTimer;

float velocity = 0;
typedef struct {
  //  uint8_t numReadings = 10;
  double readings[10];
  float total = 0;
  float Average = 0;
  uint8_t index = 0;
  float value = 0;
} accleration_t;
accleration_t accleration;


typedef struct {
  bool LED_G = 0;
  bool LED_R = 0;
  bool LED_SW = 0;
  bool Buzzer = 0;
} Alarm_t;
Alarm_t Alarm;


//******WIFI***********//
void wifi_setup() {
  Serial.println("WIFI: SETUP");
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(wifi_connect));
  wifi_connect();
}
void wifi_connect() {
  Serial.println("WIFI: Connecting...");
  WiFi.begin(cfg_wifi.ssid, cfg_wifi.pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);

    digitalWrite(LED_IN, !digitalRead(LED_IN));
  }
  digitalWrite(LED_IN, 1);
  Serial.println("WIFI: Connected");
  Serial.print("WIFI: IP address=");
  Serial.println(WiFi.localIP());
}
//*******************//


void mqtt_setup() {
  client.setServer(mqtt_broker, mqtt_port);
  //  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public emqx mqtt broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }


}

TimerHandle_t sensorTimer;

void sensor_setup() {
  sensorTimer = xTimerCreate("sensorTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(sensor_update));
  xTimerStart(sensorTimer, 0);

}

void sensor_update() {



  if (accleration.Average >= 1.1) {
    Alarm.LED_G = 0;
    Alarm.LED_R = 1;
    Alarm.Buzzer = 1;
    //    digitalWrite(LEDB_PIN, Alarm.LED_SW);
    digitalWrite(Buzzer_PIN, 1);
    digitalWrite(LEDR_PIN, 1);
    digitalWrite(LEDG_PIN, 0);




  }

  else {
    Alarm.LED_G = 1;
    Alarm.LED_R = 0;
    Alarm.Buzzer = 0;
    digitalWrite(LEDB_PIN, Alarm.LED_SW);
    digitalWrite(Buzzer_PIN, 0);
    digitalWrite(LEDR_PIN, 0);
    digitalWrite(LEDG_PIN, 1);


  }
  uplink_start();



}

void uplink_start() {
  Serial.println("UPLINK: START");
  StaticJsonDocument<200> doc;

  doc["LED_Green"] = Alarm.LED_G;
  doc["LED_RED"] = Alarm.LED_R;
  doc["LED_SW"] = Alarm.LED_SW;
  doc["velocity"] = velocity;
  char json[512];
  serializeJson(doc, json); // print to client
  client.publish(topic, json);
  Serial.println(json);

}




void setup() {

  Serial.begin(9600);

  pinMode(LEDG_PIN, OUTPUT);
  pinMode(LEDR_PIN, OUTPUT);
  pinMode(LEDB_PIN, OUTPUT);
  pinMode(Buzzer_PIN, OUTPUT);
  pinMode(Switch_PIN, INPUT_PULLUP);

  Serial.println("Accelerometer Test");

  Serial.println("");

  if (!accel.begin()) {

    Serial.println("Oops, no ADXL345 detected.");

    while (1);

  }
  accel.setRange(ADXL345_RANGE_16_G);
  wifi_setup();
  mqtt_setup();
  sensor_setup();


}



void loop() {

  Alarm.LED_SW = digitalRead(Switch_PIN);
  unsigned long currentMillis = millis();
  //   client.loop();
  switch (Alarm.LED_SW) {
    case true :


      if (currentMillis - previousMillis > interval) {

        previousMillis = currentMillis;

        if (Alarm.LED_SW == 0)
        {
          Alarm.LED_SW = 1;


        }
        else
        {
          Alarm.LED_SW = 0;

        }

        digitalWrite(LEDB_PIN, Alarm.LED_SW);

      }
      digitalWrite(Buzzer_PIN, 0);
      digitalWrite(LEDR_PIN, 1);
      digitalWrite(LEDG_PIN, 0);
      Alarm.LED_G = 0;
      Alarm.LED_R = 1;
      Alarm.Buzzer = 0;
      break;


    case false :

      sensors_event_t event;
      accel.getEvent(&event);
      accleration.value = (event.acceleration.y - 0.24);
      accleration.value = accleration.value < 0 ? -accleration.value : accleration.value;
      accleration.total = accleration.total - accleration.readings[accleration.index];             // subtract the last TEMP reading:
      accleration.readings[accleration.index] = accleration.value;                       // read TEMP value:
      accleration.total = accleration.total + accleration.readings[accleration.index];            // add the TEMP reading to the total:
      accleration.index = accleration.index + 1;
      if (accleration.index >= 10)
        accleration.index = 0;
      accleration.Average = accleration.total / 10;
      velocity=accleration.Average ;// m/s^2 * 1s
//      Serial.println("Avg " + String(accleration.Average) +  "m/s^2");
      Serial.println("velocity" + String(velocity) +  "m/s");  
      delay(100);
  }


}
