#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

#include "I2Cdev.h"
/* =========================================================================
                            !!!   IMPORTANT   !!!
                 To make it work on ESP32 you have to make changes
                           like that in the link below:
                   https://github.com/jrowberg/i2cdevlib/pull/367
 * ========================================================================= */

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5/3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the ESP32 GPIO15
   pin.
 * ========================================================================= */
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

#define INTERRUPT_PIN 15 // use pin 15 on ESP32

const char* ssid = "SAD";
const char* password = "hellodarkness";

const uint16_t port = 8090;
//const char * host = "192.168.0.167";  //Main-pc
const char * host = "192.168.0.184";  //ubuntu notebook

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
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
//        WiFiClient client;
//        client.connect(host, port);
        String quatI = String(myIMU.getQuatI(), 4);
        String quatJ = String(myIMU.getQuatJ(), 4);
        String quatK = String(myIMU.getQuatK(), 4);
        String quatReal = String(myIMU.getQuatReal(), 4);
        String quatRadianAccuracy = String(myIMU.getQuatRadianAccuracy(), 4);
        String BNO_DATA = String("#BNO080 "+ quatI + " " + quatJ + " " + quatK + " " + quatReal+ " ;");
//        char* bno_data;
//        BNO_DATA.toCharArray(bno_data, 64);
//        client.println("I:  " + quatI);
//        client.println("J:  " + quatJ);
//        client.println("K:  " + quatK);
//        client.println("W:  " + quatReal);
//        client.println("Acc:  " + quatRadianAccuracy);
//          client.write(bno_data, 64);
          client.println(BNO_DATA);
//          client.stop();
//          delay(50);
    }
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
//    WiFiClient client;
//    client.connect(host, port);
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    String quatI = String(q.x, 4);
    String quatJ = String(q.y, 4);
    String quatK = String(q.z, 4);
    String quatReal = String(q.w, 4);
    String MPU_DATA = String("#MPU6050 "+ quatI + " " + quatJ + " " + quatK + " " + quatReal+ " ;");
 //   char* mpu_data;
//    MPU_DATA.toCharArray(mpu_data, 64);
//    client.write(mpu_data, 64);
    client.println(MPU_DATA);
//    client.stop();
  }

//  client.println(BNO_DATA);
//  client.println(MPU_DATA);
//  client.stop();
  delay(10);

}
