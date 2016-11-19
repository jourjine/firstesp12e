//#include <FS.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <ESP8266mDNS.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
//#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
//#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFunBME280.h>
//#include <ArduinoJson.h>
#include "secrets.txt"

unsigned long currentTime;
unsigned long loopTime;
double totaltime;

// Possible values for client.state()
#define MQTT_CONNECTION_TIMEOUT     -4
#define MQTT_CONNECTION_LOST        -3
#define MQTT_CONNECT_FAILED         -2
#define MQTT_DISCONNECTED           -1
#define MQTT_CONNECTED               0
#define MQTT_CONNECT_BAD_PROTOCOL    1
#define MQTT_CONNECT_BAD_CLIENT_ID   2
#define MQTT_CONNECT_UNAVAILABLE     3
#define MQTT_CONNECT_BAD_CREDENTIALS 4
#define MQTT_CONNECT_UNAUTHORIZED    5

#define DISABLESLEEP 12
int EnableOTA = 0;

int mystatus;

// **************************
// HW Pins setup
// **************************

int rssistr;
double my_double;
float my_float;
int my_int;
bool my_bool;

// const char* subcribe_topic[] ={"/esp/bme280/reset",
//                                "/esp/bme280/wificonf",
//                                "/esp/bme280/sleep",
//                                "/esp/bme280/disableSleep"
//                             };

float vdd;
extern "C" {
#include "user_interface.h"
}

ADC_MODE(ADC_VCC);

#define SCL 13
#define SDA 4

int lastReconnectAttempt = 0;
bool shouldSaveConfig = false;

// void saveConfigCallback () {
//   Serial.println("Should save config");
//   shouldSaveConfig = true;
// }

char buffer[10];                        // буффер для MQTT
char data[40];                          // mqtt буффер
char temperatureFString[6];             // для www
char dpString[6];                       // для www
char humidityString[6];                 // для www
char pressureString[7];                 // для www
float bmet, bmep, bmea, bmeh, bmepraw;      // bme280

char *dtostrf(double val, signed char width, unsigned char prec, char *s); //конвертируем числа в строку для MQTT

BME280 bme280;                  // датчик BME280
WiFiClient espClient;               // для MQTT
PubSubClient mqttclient(espClient);

void deepSleep() {

    double nowtotaltime = millis();
    totaltime = (nowtotaltime - totaltime) / 1000.0;
    dtostrf(totaltime, 5, 3, buffer);
    mqttclient.publish("/esp/bme280/status", buffer);
    mqttclient.loop();
    ESP.deepSleep(600 * 1000 * 1000); // 600 sec (10 min) deepsleep
}

void callback(char* topic, byte* payload, unsigned int length) {

    // int i = 0;
    // for (i = 0; i < length; i++) {
    //   data[i] = payload[i];
    // }
    // data[i] = '\0';
    // const char *p_payload = data;
    // char *end;
    // my_double = strtod(p_payload, &end);
    // if (end == p_payload) {
    //
    // } else {
    //     my_int = atoi(p_payload);
    //     my_float = atof(p_payload);
    //     if (my_int < 0 || my_int > 0) my_bool = true;
    //     else my_bool = false;
    //   }

      // if (strcmp(topic, "/esp/bme280/reset") == 0 ) {
      //   //  int int_payload = atoi(p_payload);
      //   if (my_bool) {
      //     byte* p = (byte*)malloc(length);
      //     memcpy(p,payload,length);
      //     mqttclient.publish("/esp/bme280/status", "Перегрузка");
      //     free(p);
      //     mqttclient.loop();
      //     delay(3000);
      //     ESP.reset();
      //   }
      // }

      // if (strcmp(topic, "/esp/bme280/sleep") == 0 ) {
      //   //  int int_payload = atoi(p_payload);
      //   if (my_bool) {
      //     byte* p = (byte*)malloc(length);
      //     memcpy(p,payload,length);
      //     mqttclient.publish("/esp/bme280/status", "засыпаем");
      //     free(p);
      //     mqttclient.loop();
      //     delay(1000);
      //
      //     p = (byte*)malloc(length);
      //     memcpy(p,payload,length);
      //     mqttclient.publish("/esp/bme280/status", "Сплю!!!");
      //     mqttclient.loop();
      //     free(p);
      //     mqttclient.loop();
      //     ESP.deepSleep(10 * 1000000); // 10 sec deepsleep
      //   }
      // }
      // memset(data,0,sizeof(data));
      // handle message arrived
    }

// void broker_subcribe() {
//    for (int i = 0; i < (sizeof(subcribe_topic)/sizeof(int)); i++){
//      mqttclient.subscribe(subcribe_topic[i]);
//      mqttclient.loop();
//   }
// }

boolean reconnect() {
    mqttclient.connect(MQTTCLIENTID, mqtt_username, mqtt_password);
    mqttclient.publish("/esp/bme280/status", "Соеденение установленно!!!");
    return mqttclient.connected();
}

void bme280sensorsetup() {
    bme280.settings.commInterface = I2C_MODE;
  	bme280.settings.I2CAddress = 0x76;

   //***Operation settings*****************************//

  	//renMode can be:
  	//  0, Sleep mode
  	//  1 or 2, Forced mode
  	//  3, Normal mode
  	bme280.settings.runMode = 2; //Normal mode

  	//tStandby can be:
  	//  0, 0.5ms
  	//  1, 62.5ms
  	//  2, 125ms
  	//  3, 250ms
  	//  4, 500ms
  	//  5, 1000ms
  	//  6, 10ms
  	//  7, 20ms
  	bme280.settings.tStandby = 3;

  	//filter can be off or number of FIR coefficients to use:
  	//  0, filter off
  	//  1, coefficients = 2
  	//  2, coefficients = 4
  	//  3, coefficients = 8
  	//  4, coefficients = 16
  	bme280.settings.filter = 3;

  	//tempOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  	bme280.settings.tempOverSample = 4;

  	//pressOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
      bme280.settings.pressOverSample = 4;

  	//humidOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  	bme280.settings.humidOverSample = 1;
    bme280.begin();
  }



void setup() {
  totaltime = millis();
  pinMode(DISABLESLEEP, INPUT_PULLUP);
  pinMode(1, OUTPUT);
  pinMode(3, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    }
  bme280sensorsetup();
  // **************************
  // OTA inizialice
  // **************************
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

    mqttclient.setServer(mqtt_server, 1883);
    mqttclient.setCallback(callback);
    lastReconnectAttempt = 0;

      if (digitalRead(DISABLESLEEP) == 0) {
        EnableOTA = 1;
        digitalWrite(1, HIGH);
        // mountspiffs();
        // wifimanagerSetup();
       }
}

void mainloop() {
  vdd = system_get_vdd33()/1000.0*1.051;
  dtostrf(vdd, 4, 2, buffer);
  mqttclient.publish("/esp/bme280/voltage", buffer);

  bmet = bme280.readTempC();
  bmep = bme280.readFloatPressure()/100.0/1.3332239;
  bmeh = bme280.readFloatHumidity();
  dtostrf(bmet, 4, 2, buffer);
  mqttclient.publish("/esp/bme280/bmet", buffer);
  dtostrf(bmep, 4, 2, buffer);
  mqttclient.publish("/esp/bme280/bmep", buffer);
  dtostrf(bmeh, 4, 2, buffer);
  mqttclient.publish("/esp/bme280/bmeh", buffer);

  rssistr = WiFi.RSSI();
  dtostrf(rssistr, 4, 0, buffer);
  mqttclient.publish("/esp/bme280/rssi", buffer);
}

void loop() {

 if (EnableOTA == 1) {

   ArduinoOTA.handle();

   if (!mqttclient.connected()) {
       long now = millis();
       if (now - lastReconnectAttempt > 5000) {
         lastReconnectAttempt = now;
         // Attempt to reconnect
         if (reconnect()) {
           lastReconnectAttempt = 0;
           mqttclient.publish("/esp/bme280/status", "Режим прошивки");
        //  broker_subcribe();
         }
       }
     } else mqttclient.loop();

   } else {
      if (!mqttclient.connected()) {
          long now = millis();
          if (now - lastReconnectAttempt > 1000) {
            lastReconnectAttempt = now;
            if (reconnect()) {
              lastReconnectAttempt = 0;
              //broker_subcribe();
            }
          }
      } else {
         mainloop();
         mqttclient.loop();
         deepSleep();
        }
     }
}
