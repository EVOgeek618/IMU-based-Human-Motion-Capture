#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>


#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

const char* ssid = "SAD";
const char* password = "hellodarkness";

const uint16_t port = 8090;
//const char * host = "192.168.0.167";  //Main-pc
const char * host = "192.168.0.184";  //ubuntu notebook

void setup() {
  
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setPort(3232);           // Port defaults to 3232

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Wire.begin();

  delay(100); //  Wait for BNO to boot
  // Start i2c and BNO080
  Wire.flush();   // Reset I2C
  myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
  Wire.begin(4, 5); 
  //Wire.setClockStretchLimit(4000);

  Wire.setClock(400000);                                //Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(50);                       //Send data update every 50ms

 
}

void loop() {
  
  ArduinoOTA.handle();

  WiFiClient client;

   if (!client.connect(host, port)) {
 
        Serial.println("Connection to host failed");
 
        delay(1000);
        return;
    }
  Serial.println("Connected to server successful!");
  
  if (myIMU.dataAvailable() == true){
        String quatI = String(myIMU.getQuatI(), 4);
        String quatJ = String(myIMU.getQuatJ(), 4);
        String quatK = String(myIMU.getQuatK(), 4);
        String quatReal = String(myIMU.getQuatReal(), 4);
        String quatRadianAccuracy = String(myIMU.getQuatRadianAccuracy(), 4);
        //String DATA = String( quatI + ", " + quatJ + ", " + quatK + ", " + quatReal + ", " + quatRadianAccuracy + ", " + millis());
        String DATA = String( quatI + ", " + quatJ + ", " + quatK + ", " + quatReal + ", " + quatRadianAccuracy);
//        client.println("I:  " + quatI);
//        client.println("J:  " + quatJ);
//        client.println("K:  " + quatK);
//        client.println("W:  " + quatReal);
//        client.println("Acc:  " + quatRadianAccuracy);
          client.println(DATA);
//        client.stop();
          delay(50);
    }
   //client.print(String(myIMU.dataAvailable())+" Hello from ESP32!");
}
