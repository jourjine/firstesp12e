#include <FS.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <ESP8266mDNS.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <ArduinoJson.h>
#include "secrets.txt"

unsigned long currentTime;
unsigned long loopTime;
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

#define DISABLESLEEP 0
int dsbsleep = 0;

int mystatus;

bool DEBUG =  false;
bool wificonf = false;

// **************************
// HW Pins setup
// **************************

int rssistr;
double my_double;
float my_float;
int my_int;
bool my_bool;

const char* subcribe_topic[] ={"/esp/bme280/reset",
                               "/esp/bme280/wificonf",
                               "/esp/bme280/sleep",
                               "/esp/bme280/disableSleep"
                            };

float vdd;
extern "C" {

#include "user_interface.h"
}
ADC_MODE(ADC_VCC);

#define SCL 5
#define SDA 4
#define TRIGGER_PIN 0
int lastReconnectAttempt = 0;
bool shouldSaveConfig = false;
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

char buffer[10];                        // буффер для MQTT
char data[40];                          // mqtt буффер
char temperatureFString[6];             // для www
char dpString[6];                       // для www
char humidityString[6];                 // для www
char pressureString[7];                 // для www
float bmet, bmep, bmea, bmeh, bmepraw;      // bme280

BME280 bme280;                  // датчик BME280

WiFiManager wifiManager;        // инициализация WiFi mAnager
char *dtostrf(double val, signed char width, unsigned char prec, char *s); //конвертируем числа в строку для MQTT

WiFiClient espClient;               // для MQTT
WiFiServer wwwserver(80);       // инициализация Web Server


double EEPROM_readDouble(int address) {
   unsigned long value = 0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

void EEPROM_writeDouble(int address, unsigned long value) {
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
   delay(100);
   EEPROM.commit();
   //EEPROM.end();
}


void mountspiffs() {
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(blynk_token, json["blynk_token"]);

        } else {
        }
      }
    }
  }
  //end read
}
PubSubClient mqttclient(espClient);


void deepSleep() {
  // terminal.println("Засыпаем на 5 min.");
  // terminal.flush();
    mqttclient.publish("esp/bme280/status", "Сплю!!!");
    mqttclient.loop();

//  i++;
//  EEPROM_writeDouble(1,i);
  ESP.deepSleep(10 * 1000000); // 10 sec deepsleep
}

void callback(char* topic, byte* payload, unsigned int length) {


   int i = 0;
    for (i = 0; i < length; i++) {
     data[i] = payload[i];
     }
     data[i] = '\0';
     const char *p_payload = data;

     char *end;

     my_double = strtod(p_payload, &end);
     if(end == p_payload) {

    } else {
      my_int = atoi(p_payload);
      my_float = atof(p_payload);
      if (my_int < 0 || my_int > 0) my_bool = true;
      else my_bool = false;
     }





  if (strcmp(topic, "/esp/bme280/reset") == 0 ) {
  //  int int_payload = atoi(p_payload);
      if (my_bool) {
        byte* p = (byte*)malloc(length);
        memcpy(p,payload,length);
        mqttclient.publish("/esp/bme280/status", "Перегрузка");
        free(p);
          mqttclient.loop();
        delay(3000);
        ESP.reset();
      }
 }
 if (strcmp(topic, "/esp/bme280/sleep") == 0 ) {
 //  int int_payload = atoi(p_payload);
     if (my_bool) {
       byte* p = (byte*)malloc(length);
       memcpy(p,payload,length);
       mqttclient.publish("/esp/bme280/status", "засыпаем");
       free(p);
         mqttclient.loop();
       delay(1000);

       p = (byte*)malloc(length);
       memcpy(p,payload,length);
       mqttclient.publish("/esp/bme280/status", "Сплю!!!");
       mqttclient.loop();
       free(p);
       mqttclient.loop();
   //  i++;
   //  EEPROM_writeDouble(1,i);
     ESP.deepSleep(10 * 1000000); // 10 sec deepsleep
       //deepSleep();
     }
}

if (strcmp(topic, "/esp/bme280/disableSleep") == 0 ) {
//  int int_payload = atoi(p_payload);
    if (my_bool) {
      byte* p = (byte*)malloc(length);
      memcpy(p,payload,length);
      mqttclient.publish("/esp/bme280/sleepstatus", "Сон Запрещен");
      free(p);
      dsbsleep = 1;
      EEPROM_writeDouble(1,dsbsleep);
      EEPROM.commit();

    } else {
      byte* p = (byte*)malloc(length);
      memcpy(p,payload,length);
      mqttclient.publish("/esp/bme280/sleepstatus", "Сон Разрешен");
      dsbsleep = 0;
      free(p);
      dsbsleep = EEPROM_readDouble(1);

    }
}



  if (strcmp(topic, "/esp/bmo280/wificonf") == 0 ) {
      if (my_bool) wificonf = true;
 }
    memset(data,0,sizeof(data));
    // handle message arrived
}


//mqtt_server, 1883, callback,

void broker_subcribe() {
   for (int i = 0; i < (sizeof(subcribe_topic)/sizeof(int)); i++){
  mqttclient.subscribe(subcribe_topic[i]);
  mqttclient.loop();
  }
}

boolean reconnect() {
    mqttclient.connect(MQTTCLIENTID, mqtt_username, mqtt_password);
    //switch:
    //mystatus = "Режим работы: ";
    //mystatus += mqttclient.state();
    //dtostrf(mqttclient.state(), 4, 0, buffer);

    mqttclient.publish("/esp/bme280/status", "Соеденение установленно!!!");
    delay(1000);
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
  	bme280.settings.runMode = 3; //Normal mode

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



void setup()
{
pinMode(DISABLESLEEP, INPUT);
//digitalWrite(RED_LED, LOW);
EEPROM.begin(1);

  mountspiffs();

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);

  //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    //set static ip
    wifiManager.setSTAStaticIPConfig(IPAddress(192,168,1,202), IPAddress(192,168,1,1), IPAddress(255,255,255,0));

    //add all your parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_blynk_token);

    if (!wifiManager.autoConnect("ESP-wifi", "12345678")) {
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
      }

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

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

bme280sensorsetup();

wwwserver.begin();
mqttclient.setServer(mqtt_server, 1883);
mqttclient.setCallback(callback);
lastReconnectAttempt = 0;


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

  dsbsleep = digitalRead(DISABLESLEEP);
  dtostrf(dsbsleep, 2, 0, buffer);
  mqttclient.publish("/esp/bme280/sleepstatus", buffer);


}

void getWeather() {
dtostrf(bmet, 5, 1, temperatureFString);
dtostrf(bmeh, 5, 1, humidityString);
dtostrf(bmep, 6, 1, pressureString);
}

void wwwloop() {
  WiFiClient wwwclient = wwwserver.available();

 if (wwwclient) {
   // bolean to locate when the http request ends
   boolean blank_line = true;
   while (wwwclient.connected()) {
     if (wwwclient.available()) {
       char c = wwwclient.read();

       if (c == '\n' && blank_line) {
           getWeather();
           wwwclient.println("HTTP/1.1 200 OK");
           wwwclient.println("Content-Type: text/html");
           wwwclient.println("Connection: close");
           wwwclient.println();
           // your actual web page that displays temperature
           wwwclient.println("<!DOCTYPE HTML>");
           wwwclient.println("<html>");
           wwwclient.println("<head><META HTTP-EQUIV=\"refresh\" CONTENT=\"15\">");
           wwwclient.println("</head>");
           wwwclient.println("<body><h1>ESP8266 Погодный типа сервак</h1>");
           wwwclient.println("<table border=\"2\" width=\"456\" cellpadding=\"10\"><tbody><tr><td>");
           wwwclient.println("<h3>Температура = ");
           wwwclient.println(temperatureFString);
           wwwclient.println("&deg;C</h3><h3>Влажность = ");
           wwwclient.println(humidityString);
           wwwclient.println("%</h3><h3>Давление = ");
           wwwclient.println(pressureString);
           wwwclient.println("mmHg");
           break;
       }
       if (c == '\n') {
         // when starts reading a new line
         blank_line = true;
       }
       else if (c != '\r') {
         // when finds a character on the current line
         blank_line = false;
       }
     }
   }
   // closing the wwwclient connection
   delay(1);
   wwwclient.stop();
   if (sensordebug == true) {

 }
 }
}


void loop()
{
if (!dsbsleep) {

ArduinoOTA.handle();

if (!mqttclient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
        broker_subcribe();
      }
    }
   } else {

     currentTime = millis();                           // считываем время, прошедшее с момента запуска программы
     if (currentTime >= (loopTime + 10000)){              // сравниваем текущий таймер с переменной loopTime + 1 секунда
     mainloop() ;
     loopTime = currentTime;                         // в loopTime записываем новое значение
     }

     mqttclient.loop();

     wwwloop();
   }


   } else {
   if (!mqttclient.connected()) {
     long now = millis();
     if (now - lastReconnectAttempt > 5000) {
       lastReconnectAttempt = now;
       // Attempt to reconnect
       if (reconnect()) {
         lastReconnectAttempt = 0;
         broker_subcribe();
       }
     }
    } else {
     mainloop();
     mqttclient.loop();
     deepSleep();
    }
 }

}
