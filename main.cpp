#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <ESP8266mDNS.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SPI.h>
//#include <LedControl.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <BH1750.h>
#include <SparkFunHTU21D.h>
#include <Adafruit_BMP085.h>
#include "secrets.txt"

// настройка выходов
#define RLED 15
#define GLED 16
#define BLED 0
#define SSR 12
#define BUZZER 13
#define SPIDATAIN 1
#define SPICLK 2
#define SPICS 3
#define SCL 4
#define SDA 5


int connecttry = 0;

char buffer[10];                        // буффер для MQTT
char temperatureFString[6];             // для www
char dpString[6];                       // для www
char humidityString[6];                 // для www
char pressureString[7];                 // для www
char pressureInchString[6];             // для www

float bmet, bmep, bmea, bmeh, bmepraw;      // bme280
uint16_t lux;                               // bh1750
float bmp180t, bmp180p, bmp180a, bmp180ra;  // bmp180180
float htut, htuh;                           // htu21d

float vdd;                                  // напряжение питания ESP

long currentTime, loopTime;                  // переменные для временной задержки

extern "C" {                                  // необходимо для переключения АДС для измерения питания ESP
#include "user_interface.h"
}

ADC_MODE(ADC_VCC);                            // переключаем ADC на измерение напряжения питания

char *dtostrf(double val, signed char width, unsigned char prec, char *s); //конвертируем числа в строку для MQTT

unsigned long delaytime=250;                // Задержка для 7219
long lastReconnectAttempt = 0;


/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn
 pin 11 is connected to the CLK
 pin 10 is connected to LOAD
 We have only a single MAX72XX.
 */
 //clk: 2
//data:1
//cs:3
//LedControl lc=LedControl(SPIDATAIN,SPICLK,SPICS,2);

BME280 bme280;                  // датчик BME280
BH1750 lightMeter(0x23);        // датчик BH1750
HTU21D htu21;                   // датчик HTU21D
Adafruit_BMP085 bmp180;         // датчик BMP180
WiFiManager wifiManager;        // инициализация WiFi mAnager
WiFiServer wwwserver(80);       // инициализация Web Server
//IPAddress mqttserver(192, 168, 1, 131);// инициализация для MQTT


WiFiClient espClient;               // для MQTT


/*
Обработка подписанных топиков MQTT
*/
  void callback(char* topic, byte* payload, unsigned int length) {
    // handle message arrived
  }

PubSubClient mqttclient(mqtt_server, 1883, callback, espClient);


  void bme280sensorsetup() {
    //***Driver settings********************************//
  	//commInterface can be I2C_MODE or SPI_MODE
  	//specify chipSelectPin using arduino pin names
  	//specify I2C address.  Can be 0x77(default) or 0x76

  	//For I2C, enable the following and disable the SPI section
  	bme280.settings.commInterface = I2C_MODE;
  	bme280.settings.I2CAddress = 0x76;

  	//For SPI enable the following and dissable the I2C section
  	//bme280.settings.commInterface = SPI_MODE;
  	//bme280.settings.chipSelectPin = 10;


  	//***Operation settings*****************************//

  	//renMode can be:
  	//  0, Sleep mode
  	//  1 or 2, Forced mode
  	//  3, Normal mode
  	bme280.settings.runMode = 1; //Normal mode

  	//tStandby can be:
  	//  0, 0.5ms
  	//  1, 62.5ms
  	//  2, 125ms
  	//  3, 250ms
  	//  4, 500ms
  	//  5, 1000ms
  	//  6, 10ms
  	//  7, 20ms
  	bme280.settings.tStandby = 0;

  	//filter can be off or number of FIR coefficients to use:
  	//  0, filter off
  	//  1, coefficients = 2
  	//  2, coefficients = 4
  	//  3, coefficients = 8
  	//  4, coefficients = 16
  	bme280.settings.filter = 2;

  	//tempOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  	bme280.settings.tempOverSample = 3;

  	//pressOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
      bme280.settings.pressOverSample = 3;

  	//humidOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  	bme280.settings.humidOverSample = 1;
if (sensordebug == true) {
  	//Serial.begin(57600);
  	Serial.print("Program Started\n");
  	Serial.print("Starting BME280... result of .begin(): 0x");

  	//Calling .begin() causes the settings to be loaded
  	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  	Serial.println(bme280.begin(), HEX);

  	Serial.print("Displaying ID, reset and ctrl regs\n");

  	Serial.print("ID(0xD0): 0x");
  	Serial.println(bme280.readRegister(BME280_CHIP_ID_REG), HEX);
  	Serial.print("Reset register(0xE0): 0x");
  	Serial.println(bme280.readRegister(BME280_RST_REG), HEX);
  	Serial.print("ctrl_meas(0xF4): 0x");
  	Serial.println(bme280.readRegister(BME280_CTRL_MEAS_REG), HEX);
  	Serial.print("ctrl_hum(0xF2): 0x");
  	Serial.println(bme280.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

  	Serial.print("\n\n");

  	Serial.print("Displaying all regs\n");
  	uint8_t memCounter = 0x80;
  	uint8_t tempReadData;
  	for(int rowi = 8; rowi < 16; rowi++ )
  	{
  		Serial.print("0x");
  		Serial.print(rowi, HEX);
  		Serial.print("0:");
  		for(int coli = 0; coli < 16; coli++ )
  		{
  			tempReadData = bme280.readRegister(memCounter);
  			Serial.print((tempReadData >> 4) & 0x0F, HEX);//Print first hex nibble
  			Serial.print(tempReadData & 0x0F, HEX);//Print second hex nibble
  			Serial.print(" ");
  			memCounter++;
  		}
  		Serial.print("\n");
  	}


  	Serial.print("\n\n");

  	Serial.print("Displaying concatenated calibration words\n");
  	Serial.print("dig_T1, uint16: ");
  	Serial.println(bme280.calibration.dig_T1);
  	Serial.print("dig_T2, int16: ");
  	Serial.println(bme280.calibration.dig_T2);
  	Serial.print("dig_T3, int16: ");
  	Serial.println(bme280.calibration.dig_T3);

  	Serial.print("dig_P1, uint16: ");
  	Serial.println(bme280.calibration.dig_P1);
  	Serial.print("dig_P2, int16: ");
  	Serial.println(bme280.calibration.dig_P2);
  	Serial.print("dig_P3, int16: ");
  	Serial.println(bme280.calibration.dig_P3);
  	Serial.print("dig_P4, int16: ");
  	Serial.println(bme280.calibration.dig_P4);
  	Serial.print("dig_P5, int16: ");
  	Serial.println(bme280.calibration.dig_P5);
  	Serial.print("dig_P6, int16: ");
  	Serial.println(bme280.calibration.dig_P6);
  	Serial.print("dig_P7, int16: ");
  	Serial.println(bme280.calibration.dig_P7);
  	Serial.print("dig_P8, int16: ");
  	Serial.println(bme280.calibration.dig_P8);
  	Serial.print("dig_P9, int16: ");
  	Serial.println(bme280.calibration.dig_P9);

  	Serial.print("dig_H1, uint8: ");
  	Serial.println(bme280.calibration.dig_H1);
  	Serial.print("dig_H2, int16: ");
  	Serial.println(bme280.calibration.dig_H2);
  	Serial.print("dig_H3, uint8: ");
  	Serial.println(bme280.calibration.dig_H3);
  	Serial.print("dig_H4, int16: ");
  	Serial.println(bme280.calibration.dig_H4);
  	Serial.print("dig_H5, int16: ");
  	Serial.println(bme280.calibration.dig_H5);
  	Serial.print("dig_H6, uint8: ");
  	Serial.println(bme280.calibration.dig_H6);

  	Serial.println();
  } else bme280.begin();
  }

void wwwsetup() {
  wwwserver.begin();
if (sensordebug == true) {
  Serial.println("Web server running. Waiting for the ESP IP...");
  delay(1000);
}
}


boolean reconnect() {
    Serial.println("Attempting MQTT connection...");   // Attempt to connect
      int ret = mqttclient.connect(MQTTCLIENTID, mqtt_username, mqtt_password);
            switch (ret) {
    	      case 2: Serial.println("Wrong protocol"); break;
    	      case 3: Serial.println("ID rejected"); break;
    	      case 4: Serial.println("Server unavailable");  break;
    	      case 5: Serial.println("Bad user/password");  break;
    	      case 6: Serial.println("Not authenticated");  break;
    	      case 7: Serial.println("Failed to subscribe");  break;
    	      default: Serial.print("Couldn't connect to server, code: ");
    	               Serial.println(ret);
    	               break;
            }
            Serial.print("Current MQTT state is ");

            Serial.println(mqttclient.state());
            Serial.print("var ret is ");
            Serial.println(ret);
return mqttclient.connected();
}


void reconnectold2() {

  while (!mqttclient.connected()) {
    connecttry++;
    if (connecttry <= 10) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    int ret;
    while ((ret = mqttclient.connect(MQTTCLIENTID)) != 0) { // connect will return 0 for connected
    	    switch (ret) {
    	      case 1: Serial.println("Wrong protocol"); break;
    	      case 2: Serial.println("ID rejected"); break;
    	      case 3: Serial.println("Server unavailable"); break;
    	      case 4: Serial.println("Bad user/password"); break;
    	      case 5: Serial.println("Not authenticated"); break;
    	      case 6: Serial.println("Failed to subscribe"); break;
    	      default: Serial.print("Couldn't connect to server, code: ");
    	        Serial.println(ret);
    	        break;
            }
            delay(5000);
          }
        } else {
          Serial.print("Couldn't connect to server, mqqt stopped");
          break;
            }
       }
    }



  void reconnectold() {
    // Loop until we're reconnected

    while (!mqttclient.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (mqttclient.connect("ESP8266Client")) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        mqttclient.publish("outTopic", "hello world");
        // ... and resubscribe
        mqttclient.subscribe("inTopic");
      }
      else {
        Serial.print("failed, rc=");
        Serial.print(mqttclient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }


void setup() {
  pinMode(RLED, OUTPUT);
  digitalWrite(RLED, LOW);
  pinMode(GLED, OUTPUT);
  digitalWrite(GLED, LOW);
  pinMode(BLED, OUTPUT);
  digitalWrite(BLED, LOW);
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, LOW);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);




  Serial.begin(115200);

  wifiManager.autoConnect("ESP-wifi", "12345678");

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

  //  if (mqttclient.connect("espClient", "Jourjine", "Gjxnj1dsQ")) {
  //   mqttclient.publish("outTopic","hello world");
  //   mqttclient.subscribe("inTopic");
  // } else digitalWrite(BLED, HIGH);
  mqttclient.setServer(mqtt_server, 1883);
  mqttclient.setCallback(callback);
 lastReconnectAttempt = 0;



bme280sensorsetup();

lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);

  htu21.begin();
  bmp180.begin();
  wwwsetup();

}

void vddpub() {
  vdd = system_get_vdd33()/1000.0*1.063; //*1.051;
  dtostrf(vdd, 4, 2, buffer);
  mqttclient.publish("/esp/voltage", buffer);
  // lc.setDigit(0,6,buffer[0],false);
  // lc.setDigit(0,5,buffer[1],false);
  // lc.setDigit(0,4,buffer[2],true);
  // lc.setDigit(0,3,buffer[3],false);
  // lc.setChar(0,2,buffer[4],false);
  // lc.setChar(0,1,buffer[5],false);
  //lc.setChar(0,0,'O',false);

//  printNumber(vdd);
}



void bme280loop() {
  //Each loop, take a reading.
	//Start with temperature, as that data is needed for accurate compensation.
	//Reading the temperature updates the compensators of the other functions
	//in the background.
bmet = bme280.readTempC();
bmep = bme280.readFloatPressure()/100.0/1.3332239;
bmepraw = bme280.readFloatPressure()/100;

bmea = bme280.readFloatAltitudeMeters();
bmeh = bme280.readFloatHumidity();

dtostrf(bmet, 4, 2, buffer);
mqttclient.publish("/esp/bmet", buffer);
dtostrf(bmep, 4, 2, buffer);
mqttclient.publish("/esp/bmep", buffer);
dtostrf(bmea, 4, 2, buffer);
mqttclient.publish("/esp/bmea", buffer);
dtostrf(bmeh, 4, 2, buffer);
mqttclient.publish("/esp/bmeh", buffer);


if (sensordebug == true) {

  Serial.println("-----------BME280-------------");
	Serial.print("Temperature: ");
	Serial.print(bmet, 2);
	Serial.println(" degrees C");

	Serial.print("Pressure: ");
	Serial.print(bmep, 2);
	Serial.println(" Pa");

	Serial.print("Altitude: ");
	Serial.print(bmea, 2);
	Serial.println("m");

	Serial.print("%RH: ");
	Serial.print(bmeh, 2);
	Serial.println(" %");

//	Serial.println();
}
}

void bh1750loop() {
  lux = lightMeter.readLightLevel();
  if (sensordebug == true) {
  Serial.println("---------BH1750--------------");
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
}
  dtostrf(lux, 4, 0, buffer);
  mqttclient.publish("/esp/lux", buffer);

  //Serial.println();
//  delay(1000);
}

void htu21dloop() {
   htuh = htu21.readHumidity();
   htut = htu21.readTemperature();
if (sensordebug == true) {
  // Serial.print("Time:");
  // Serial.print(millis());
Serial.println("-------------HTU21D------------");
  Serial.print(" Temperature:");
  Serial.print(htut, 1);
  Serial.println("C");
  Serial.print(" Humidity:");
  Serial.print(htuh, 1);
  Serial.println("%");
}
  dtostrf(htuh, 4, 2, buffer);
  mqttclient.publish("/esp/htuh", buffer);
  dtostrf(htut, 4, 2, buffer);
  mqttclient.publish("/esp/htut", buffer);
//  Serial.println();
//  delay(1000);
}

void bmp180loop() {

   bmp180t = bmp180.readTemperature();
   bmp180p = bmp180.readPressure()/100/1.3332239;
   bmp180a = bmp180.readAltitude();
   bmp180ra = bmp180.readAltitude(101500);

  dtostrf(bmp180t, 4, 2, buffer);
  mqttclient.publish("/esp/bmpt", buffer);
  dtostrf(bmp180p, 4, 2, buffer);
  mqttclient.publish("/esp/bmpp", buffer);
  dtostrf(bmp180a, 4, 2, buffer);
  mqttclient.publish("/esp/bmpa", buffer);
  dtostrf(bmp180ra, 4, 2, buffer);
  mqttclient.publish("/esp/bmpra", buffer);

  if (sensordebug == true) {
  Serial.println("------------BMP180------------");
  Serial.print("Temperature = ");


    Serial.print(bmp180t);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp180p);
    Serial.println(" Pa");

    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp180a);
    Serial.println(" meters");


  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp180ra);
    Serial.println(" meters");

    Serial.println();
  //  delay(500);
}
}


void getWeather() {

    //float dp = 17.27*bmet/(237.7+bmet)+log((bmeh/100));
    //bmet-0.36*(100.0-bmeh);

float dp = 18.4*log(101.325/(bmepraw/100));
    dtostrf(bmet, 5, 1, temperatureFString);
    dtostrf(bmeh, 5, 1, humidityString);
    dtostrf(bmep, 6, 1, pressureString);
    dtostrf(dp, 5, 1, dpString);
  //  delay(100);

}
void wwwloop() {
  WiFiClient wwwclient = wwwserver.available();

 if (wwwclient) {
   if (sensordebug == true) {

   Serial.println("New wwwclient");
}
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
           wwwclient.println("%</h3><h3>Высота = ");
           wwwclient.println(dpString);
           wwwclient.println("м.</h3><h3>Давление = ");
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
   Serial.println("Client disconnected.");
 }
 }
}

void loop() {

  currentTime = millis();                           // считываем время, прошедшее с момента запуска программы
  if(currentTime >= (loopTime + 2000) ){              // сравниваем текущий таймер с переменной loopTime + 1 секунда
  vddpub();
  bme280loop();
  bh1750loop();
  htu21dloop();
  bmp180loop();
  wwwloop();
  loopTime = currentTime;                         // в loopTime записываем новое значение
  }

ArduinoOTA.handle();

if (!mqttclient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    mqttclient.loop();
  }

}
