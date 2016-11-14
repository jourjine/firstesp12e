#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <ESP8266mDNS.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoOTA.h>
#include <PubSubClient.h>


  WiFiManager wifiManager;

  IPAddress server(192, 168, 1, ***); // вставить свой IP

  void callback(char* topic, byte* payload, unsigned int length) {
    // handle message arrived
  }

  WiFiClient espClient;

  PubSubClient client(server, 1883, callback, espClient);

  float vdd;
  char buffer[10];
  long currentTime, loopTime;

  extern "C" {
  #include "user_interface.h"
  }

  ADC_MODE(ADC_VCC);
char *dtostrf(double val, signed char width, unsigned char prec, char *s);

  void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (client.connect("ESP8266Client")) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        client.publish("outTopic", "hello world");
        // ... and resubscribe
        client.subscribe("inTopic");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }


void setup() {
  Serial.begin(115200);

  wifiManager.autoConnect("ESP-wifi", "wifimag");

  ArduinoOTA.onStart([]() {
      Serial.println("Start");
    });
    ArduinoOTA.onEnd([]() {
     Serial.println("\nEnd");
   });
   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
   });
   ArduinoOTA.onError([](ota_error_t error) {
     Serial.printf("Error[%u]: ", error);
     if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
     else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
     else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
     else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
     else if (error == OTA_END_ERROR) Serial.println("End Failed");
   });

   ArduinoOTA.begin();

   if (client.connect("espClient", "login", "password")) {  // Вставить свои логин и пароль для mqtt
    client.publish("outTopic","hello world");
    client.subscribe("inTopic");
   }

void vddpub() {
  vdd = system_get_vdd33()/1000.0*1.063; //*1.051;
  dtostrf(vdd, 4, 2, buffer);
  client.publish("/esp/voltage", buffer);
}

void loop() {

  currentTime = millis();                           // считываем время, прошедшее с момента запуска программы
  if(currentTime >= (loopTime + 2000) ){              // сравниваем текущий таймер с переменной loopTime + 1 секунда
  vddpub();
  loopTime = currentTime;                         // в loopTime записываем новое значение
  }

ArduinoOTA.handle();
client.loop();

}
