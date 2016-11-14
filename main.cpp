#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
//#include <mdns.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoOTA.h>
#include <PubSubClient.h>
//#include <MD_MAX72xx.h>
#include <SPI.h>

#include <LedControl.h>
//#include "brzo_i2c\brzo_i2c.h"
//#include <Wire.h>
//#include <BH1750.h>


/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn
 pin 11 is connected to the CLK
 pin 10 is connected to LOAD
 We have only a single MAX72XX.
 */
 LedControl lc=LedControl(14,13,12,1);

  unsigned long delaytime=250;


  WiFiManager wifiManager;

  IPAddress server(192, 168, 1, 131);

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

  wifiManager.autoConnect("ESP-wifi", "GjxnjdsQ");

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

   if (client.connect("espClient", "Jourjine", "Gjxnj1dsQ")) {
    client.publish("outTopic","hello world");
    client.subscribe("inTopic");
   }



   /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,8);
  /* and clear the display */
  lc.clearDisplay(0);


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


void loop() {

  currentTime = millis();                           // считываем время, прошедшее с момента запуска программы
  if(currentTime >= (loopTime + 2000) ){              // сравниваем текущий таймер с переменной loopTime + 1 секунда
  vddpub();
  // uint16_t lux = lightMeter.readLightLevel();
  // Serial.print("Light: ");
  // Serial.print(lux);
  // Serial.println(" lx");


  loopTime = currentTime;                         // в loopTime записываем новое значение
  }

ArduinoOTA.handle();
client.loop();

//writeArduinoOn7Segment();
//  scrollDigits();

}
