#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <ESP8266mDNS.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoOTA.h>
#include <PubSubClient.h>
//#include <MD_MAX72xx.h>
#include <SPI.h>
#include <LedControl.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <BH1750.h>
#include <SparkFunHTU21D.h>
#include <Adafruit_BMP085.h>


char buffer[10];
char temperatureFString[6];
char dpString[6];
char humidityString[6];
char pressureString[7];
char pressureInchString[6];

float bmet, bmep, bmea, bmeh;   // bme280
uint16_t lux;                   // bh1750
float bmpt, bmpp, bmpa, bmpra;  // bmp180
float htut, htuh;               // htu21d

float vdd;

long currentTime, loopTime;

extern "C" {
#include "user_interface.h"
}

ADC_MODE(ADC_VCC);
char *dtostrf(double val, signed char width, unsigned char prec, char *s);

unsigned long delaytime=250;





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
LedControl lc=LedControl(1,2,3,2);
BME280 mySensor;
BH1750 lightMeter(0x23);
HTU21D myHumidity;
Adafruit_BMP085 bmp;
WiFiManager wifiManager;
WiFiServer wwwserver(80);
  IPAddress server(192, 168, 1, ***);   // Заменить на свой IP

  void callback(char* topic, byte* payload, unsigned int length) {
    // handle message arrived
  }

  WiFiClient espClient;

  PubSubClient client(server, 1883, callback, espClient);


  void bme280sensorsetup() {
    //***Driver settings********************************//
  	//commInterface can be I2C_MODE or SPI_MODE
  	//specify chipSelectPin using arduino pin names
  	//specify I2C address.  Can be 0x77(default) or 0x76

  	//For I2C, enable the following and disable the SPI section
  	mySensor.settings.commInterface = I2C_MODE;
  	mySensor.settings.I2CAddress = 0x76;

  	//For SPI enable the following and dissable the I2C section
  	//mySensor.settings.commInterface = SPI_MODE;
  	//mySensor.settings.chipSelectPin = 10;


  	//***Operation settings*****************************//

  	//renMode can be:
  	//  0, Sleep mode
  	//  1 or 2, Forced mode
  	//  3, Normal mode
  	mySensor.settings.runMode = 3; //Normal mode

  	//tStandby can be:
  	//  0, 0.5ms
  	//  1, 62.5ms
  	//  2, 125ms
  	//  3, 250ms
  	//  4, 500ms
  	//  5, 1000ms
  	//  6, 10ms
  	//  7, 20ms
  	mySensor.settings.tStandby = 0;

  	//filter can be off or number of FIR coefficients to use:
  	//  0, filter off
  	//  1, coefficients = 2
  	//  2, coefficients = 4
  	//  3, coefficients = 8
  	//  4, coefficients = 16
  	mySensor.settings.filter = 0;

  	//tempOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  	mySensor.settings.tempOverSample = 1;

  	//pressOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
      mySensor.settings.pressOverSample = 1;

  	//humidOverSample can be:
  	//  0, skipped
  	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  	mySensor.settings.humidOverSample = 1;

  	//Serial.begin(57600);
  	Serial.print("Program Started\n");
  	Serial.print("Starting BME280... result of .begin(): 0x");

  	//Calling .begin() causes the settings to be loaded
  	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  	Serial.println(mySensor.begin(), HEX);

  	Serial.print("Displaying ID, reset and ctrl regs\n");

  	Serial.print("ID(0xD0): 0x");
  	Serial.println(mySensor.readRegister(BME280_CHIP_ID_REG), HEX);
  	Serial.print("Reset register(0xE0): 0x");
  	Serial.println(mySensor.readRegister(BME280_RST_REG), HEX);
  	Serial.print("ctrl_meas(0xF4): 0x");
  	Serial.println(mySensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
  	Serial.print("ctrl_hum(0xF2): 0x");
  	Serial.println(mySensor.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

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
  			tempReadData = mySensor.readRegister(memCounter);
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
  	Serial.println(mySensor.calibration.dig_T1);
  	Serial.print("dig_T2, int16: ");
  	Serial.println(mySensor.calibration.dig_T2);
  	Serial.print("dig_T3, int16: ");
  	Serial.println(mySensor.calibration.dig_T3);

  	Serial.print("dig_P1, uint16: ");
  	Serial.println(mySensor.calibration.dig_P1);
  	Serial.print("dig_P2, int16: ");
  	Serial.println(mySensor.calibration.dig_P2);
  	Serial.print("dig_P3, int16: ");
  	Serial.println(mySensor.calibration.dig_P3);
  	Serial.print("dig_P4, int16: ");
  	Serial.println(mySensor.calibration.dig_P4);
  	Serial.print("dig_P5, int16: ");
  	Serial.println(mySensor.calibration.dig_P5);
  	Serial.print("dig_P6, int16: ");
  	Serial.println(mySensor.calibration.dig_P6);
  	Serial.print("dig_P7, int16: ");
  	Serial.println(mySensor.calibration.dig_P7);
  	Serial.print("dig_P8, int16: ");
  	Serial.println(mySensor.calibration.dig_P8);
  	Serial.print("dig_P9, int16: ");
  	Serial.println(mySensor.calibration.dig_P9);

  	Serial.print("dig_H1, uint8: ");
  	Serial.println(mySensor.calibration.dig_H1);
  	Serial.print("dig_H2, int16: ");
  	Serial.println(mySensor.calibration.dig_H2);
  	Serial.print("dig_H3, uint8: ");
  	Serial.println(mySensor.calibration.dig_H3);
  	Serial.print("dig_H4, int16: ");
  	Serial.println(mySensor.calibration.dig_H4);
  	Serial.print("dig_H5, int16: ");
  	Serial.println(mySensor.calibration.dig_H5);
  	Serial.print("dig_H6, uint8: ");
  	Serial.println(mySensor.calibration.dig_H6);

  	Serial.println();
  }

void wwwsetup() {
  wwwserver.begin();
  Serial.println("Web server running. Waiting for the ESP IP...");
  delay(1000);
}
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

  wifiManager.autoConnect("ESP-wifi", "admin");  

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

   if (client.connect("espClient", "*****", "******")) { // логин и пароль на mqqt сервер
    client.publish("outTopic","hello world");
    client.subscribe("inTopic");
   }



   /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
//  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
//  lc.setIntensity(0,8);
  /* and clear the display */
//  lc.clearDisplay(0);

bme280sensorsetup();

lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
 Serial.println(F("BH1750 Test"));

 Serial.println("HTU21D Example!");

  myHumidity.begin();
  bmp.begin();
  wwwsetup();

}


/*
 This method will display the characters for the
 word "Arduino" one after the other on digit 0.
 */
void writeArduinoOn7Segment() {
  lc.setChar(0,0,'a',false);
  delay(delaytime);
  lc.setRow(0,0,0x05);
  delay(delaytime);
  lc.setChar(0,0,'d',false);
  delay(delaytime);
  lc.setRow(0,0,0x1c);
  delay(delaytime);
  lc.setRow(0,0,B00010000);
  delay(delaytime);
  lc.setRow(0,0,0x15);
  delay(delaytime);
  lc.setRow(0,0,0x1D);
  delay(delaytime);
  lc.clearDisplay(0);
  delay(delaytime);
}

void printNumber(int v) {
    int ones;
    int tens;
    int hundreds;

    boolean negative=false;

    if(v < -9999 || v > 9999)
        return;
    if(v<0) {
        negative=true;
        v=v*-1;
    }
    ones=v%10;
    v=v/10;
    tens=v%10;
    v=v/10; hundreds=v;
    if(negative) {
        //print character '-' in the leftmost column
        lc.setChar(0,3,'-',false);  }
    else {
        //print a blank in the sign column
        lc.setChar(0,3,' ',false);
    }
    //Now print the number digit by digit
    lc.setDigit(0,2,(byte)hundreds,false);
    lc.setDigit(0,2,(byte)hundreds,false);
    lc.setDigit(0,1,(byte)tens,false);
    lc.setDigit(0,0,(byte)ones,false);
}

void vddpub() {
  vdd = system_get_vdd33()/1000.0*1.063; //*1.051;
  dtostrf(vdd, 4, 2, buffer);
  client.publish("/esp/voltage", buffer);
  lc.setDigit(0,6,1,false);
  lc.setDigit(0,5,4,false);
  lc.setDigit(0,4,2,true);
  lc.setDigit(0,3,6,false);
  lc.setChar(0,2,'O',false);
  lc.setChar(0,1,'C',false);
  //lc.setChar(0,0,'O',false);

//  printNumber(vdd);
}

/*
  This method will scroll all the hexa-decimal
 numbers and letters on the display. You will need at least
 four 7-Segment digits. otherwise it won't really look that good.
 */
void scrollDigits() {
  for(int i=0;i<13;i++) {
    lc.setDigit(0,3,i,false);
    lc.setDigit(0,2,i+1,false);
    lc.setDigit(0,1,i+2,false);
    lc.setDigit(0,0,i+3,false);
    delay(delaytime);
  }
  lc.clearDisplay(0);
  delay(delaytime);
}

void bme280loop() {
  //Each loop, take a reading.
	//Start with temperature, as that data is needed for accurate compensation.
	//Reading the temperature updates the compensators of the other functions
	//in the background.
bmet = mySensor.readTempC();
bmep = mySensor.readFloatPressure()/100/1.3332239;
bmea = mySensor.readFloatAltitudeMeters();
bmeh = mySensor.readFloatHumidity();

dtostrf(bmet, 4, 2, buffer);
client.publish("/esp/bmet", buffer);
dtostrf(bmep, 4, 2, buffer);
client.publish("/esp/bmep", buffer);
dtostrf(bmea, 4, 2, buffer);
client.publish("/esp/bmea", buffer);
dtostrf(bmeh, 4, 2, buffer);
client.publish("/esp/bmeh", buffer);




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

void bh1750loop() {
  lux = lightMeter.readLightLevel();
  Serial.println("---------BH1750--------------");
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  dtostrf(lux, 4, 0, buffer);
  client.publish("/esp/lux", buffer);

  //Serial.println();
//  delay(1000);
}

void htu21dloop() {
   htuh = myHumidity.readHumidity();
   htut = myHumidity.readTemperature();

  // Serial.print("Time:");
  // Serial.print(millis());
Serial.println("-------------HTU21D------------");
  Serial.print(" Temperature:");
  Serial.print(htut, 1);
  Serial.println("C");
  Serial.print(" Humidity:");
  Serial.print(htuh, 1);
  Serial.println("%");
  dtostrf(htuh, 4, 2, buffer);
  client.publish("/esp/htuh", buffer);
  dtostrf(htut, 4, 2, buffer);
  client.publish("/esp/htut", buffer);
//  Serial.println();
//  delay(1000);
}

void bmp180sloop() {
  Serial.println("------------BMP180------------");
  Serial.print("Temperature = ");
   bmpt = bmp.readTemperature();
   bmpp = bmp.readPressure()/100/1.3332239;
   bmpa = bmp.readAltitude();
   bmpra = bmp.readAltitude(101500);

  dtostrf(bmpt, 4, 2, buffer);
  client.publish("/esp/bmpt", buffer);
  dtostrf(bmpp, 4, 2, buffer);
  client.publish("/esp/bmpp", buffer);
  dtostrf(bmpa, 4, 2, buffer);
  client.publish("/esp/bmpa", buffer);
  dtostrf(bmpra, 4, 2, buffer);
  client.publish("/esp/bmpra", buffer);



    Serial.print(bmpt);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmpp);
    Serial.println(" Pa");

    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmpa);
    Serial.println(" meters");


  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmpra);
    Serial.println(" meters");

    Serial.println();
  //  delay(500);
}


void getWeather() {

    float dp = bmet-0.36*(100.0-bmeh);


    dtostrf(bmet, 5, 1, temperatureFString);
    dtostrf(bmeh, 5, 1, humidityString);
    dtostrf(bmep, 6, 1, pressureString);
    dtostrf(dp, 5, 1, dpString);
    delay(100);

}
void wwwloop() {
  WiFiClient client = wwwserver.available();

 if (client) {
   Serial.println("New client");
   // bolean to locate when the http request ends
   boolean blank_line = true;
   while (client.connected()) {
     if (client.available()) {
       char c = client.read();

       if (c == '\n' && blank_line) {
           getWeather();
           client.println("HTTP/1.1 200 OK");
           client.println("Content-Type: text/html");
           client.println("Connection: close");
           client.println();
           // your actual web page that displays temperature
           client.println("<!DOCTYPE HTML>");
           client.println("<html>");
           client.println("<head><META HTTP-EQUIV=\"refresh\" CONTENT=\"15\">");
           client.println("</head>");
           client.println("<body><h1>ESP8266 Погодный типа сервак</h1>");
           client.println("<table border=\"2\" width=\"456\" cellpadding=\"10\"><tbody><tr><td>");
           client.println("<h3>Температура = ");
           client.println(temperatureFString);
           client.println("&deg;C</h3><h3>Влажность = ");
           client.println(humidityString);
           client.println("%</h3><h3>Точка росы = ");
           client.println(dpString);
           client.println("&deg;C</h3><h3>Давление = ");
           client.println(pressureString);
           client.println("mmHg");
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
   // closing the client connection
   delay(1);
   client.stop();
   Serial.println("Client disconnected.");
 }
}

void loop() {

  currentTime = millis();                           // считываем время, прошедшее с момента запуска программы
  if(currentTime >= (loopTime + 2000) ){              // сравниваем текущий таймер с переменной loopTime + 1 секунда
  vddpub();
  // uint16_t lux = lightMeter.readLightLevel();
  // Serial.print("Light: ");
  // Serial.print(lux);
  // Serial.println(" lx");
  bme280loop();
  bh1750loop();
  htu21dloop();
  //bmp180loop();
  bmp180sloop();
  wwwloop();
  loopTime = currentTime;                         // в loopTime записываем новое значение
  }

ArduinoOTA.handle();
client.loop();


//writeArduinoOn7Segment();
//  scrollDigits();

}
