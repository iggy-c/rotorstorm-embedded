#include <Wire.h> //enables I2C
#include <SPI.h> //enables SPI
#include <Adafruit_Sensor.h> //used for adafruit stuff
#include "Adafruit_BMP3XX.h" //enables BMP
#include <Adafruit_BNO08x.h> //enables BNO
#include <SD.h> // enables sd communication
#include <Adafruit_INA260.h> // enables INA
#include <SparkFun_u-blox_GNSS_v3.h>
#include <ctime> // time library
#include <Servo.h>
// #include "Timer.h"
#include "functions.h"
// #include "time.h"
// #include <arduino-timer.h>

#define BMP390_ADDRESS 0x77  // Try 0x76 if this fails
#define BNO08x_ADDRESS 0x4A // or 0x4B if DI pin is pulled high to VCC
#define INA260_ADDRESS 0x40 // my shins hurt
#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required

#define servo_wire 11 
#define BMP_SDA 14 //SDA for I2C stuff for bmp
#define BMP_SCL 15 //SCL for I2C stuff for bmp
#define BNO08X_SDA 14 //SDA for I2C stuff for bno
#define BNO08X_SCL 15 //SCL for I2C stuff for bno
#define INA260_SDA 14
#define INA260_SCL 15
#define GNSS_SDA 14
#define GNSS_SCL 15

#define SEALEVELPRESSURE_HPA (1013.25) 

// Timer timer;
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x;
Adafruit_INA260 ina260;
SFE_UBLOX_GNSS myGNSS;
Servo release_servo;

const int UARTRX = 1;
const int UARTTX = 0;

int packetCount = 0;
String echoStr = "None";
int servoPos = 0;
sh2_SensorValue_t sensorValue; // does something to help bno work

// auto timer = timer_create_default();

// Timer<16, millis, const char *> t_timer;

// bool sendpacketandusetimer(void *)
// {
// String message = (
//     "3194," //TEAM_ID
//   + String(1) + "," //MISSION_TIME
//   + String(packetCount) + "," //PACKET_COUNT
//   + "mode,state," //MODESTATE
//   + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "," //ALTITUDE
//   + bmp.readTemperature() + "," //TEMPERATURE
//   + bmp.readPressure() + "," //PRESSURE
//   + String(ina260.readBusVoltage() / 1000) + "," //VOLTAGE
//   + String(sensorValue.un.rawGyroscope.x) + "," 
//   + String(sensorValue.un.rawGyroscope.y) + ","
//   + String(sensorValue.un.rawGyroscope.z) + ","
//   + String(sensorValue.un.linearAcceleration.x) + ","
//   + String(sensorValue.un.linearAcceleration.y) + ","
//   + String(sensorValue.un.linearAcceleration.z) + ","
//   + "1,2,3,"
//   + "rotate_rate,"
//   + String(timer.read()) + ","
//   + String(myGNSS.getAltitudeMSL()) + ","
//   + String(myGNSS.getLatitude()) + ","
//   + String(myGNSS.getLongitude()) + ","
//   + "gpssats,echo"
//   );

//   packetCount++;
//   return true;
// }

long lastTime = 0;

void setup() {

  Serial.begin(9600);
  // timer.start();

  // initialize XBee
  Serial1.setRX(UARTRX);
  Serial1.setTX(UARTTX);
  Serial1.begin(9600);

  // Initialize I2C1 (Wire1) for BMP3XX
  Wire1.setSDA(BMP_SDA);
  Wire1.setSCL(BMP_SCL);
  Wire1.begin();

  // Initialize I2C1 (Wire1) for BNO
  Wire1.setSDA(BNO08X_SDA);
  Wire1.setSCL(BNO08X_SCL);
  Wire1.begin();

  // Initialize I2C1 (Wire1) for INA
  Wire1.setSDA(INA260_SDA);
  Wire1.setSCL(INA260_SCL);
  Wire1.begin();

  // Initialize I2C1 (Wire1) for GNSS
  Wire1.setSDA(GNSS_SDA);
  Wire1.setSCL(GNSS_SCL);
  Wire1.begin();

  Serial.println("Starting XBee API packet transmission...");
  delay(1000);  // Allow XBee to initialize

  // Scan I2C1 Bus
  Serial.println("Scanning I2C Bus...");
  for (uint8_t address = 1; address < 127; address++) {
      Wire1.beginTransmission(address);
      if (Wire1.endTransmission() == 0) {
          Serial.print("Device found at 0x");
          Serial.println(address, HEX);
      }
  }
  Serial.println("I2C Scan Complete.");

  // Initialize BMP3XX sensor using I2C1
  if (!bmp.begin_I2C(BMP390_ADDRESS, &Wire1)) {  //this is important to initialize the i2c bus
      Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
      while (1);  // Halt execution
  }

  Serial.println("BMP3XX Sensor Initialized!");

  // Intialize BNO sensor
  if (!bno08x.begin_I2C(BNO08x_ADDRESS, &Wire1)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  // Initialize INA sensor
  if (!ina260.begin(INA260_ADDRESS, &Wire1)) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");

  while (myGNSS.begin(Wire1, gnssAddress) == false) //Connect to the u-blox module using our custom port and address
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(100);

  release_servo.attach(servo_wire, 400, 2600);
  release_servo.write(60);

  //Timer setup
  // timer.every(1000, sendpacketandusetimer);
}

void loop() {

// timer.tick(); // tick the timer

  if (!bno08x.getSensorEvent(&sensorValue)) {
    setReports(bno08x);
    return;
  }

  // Serial.println("timer: ");
  // Serial.println(timer.read());
  servoPos = 0;
  String str = "";
  int unused = 0;

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  if (bno08x.wasReset()) {
      Serial.println("BNO08x was reset, reinitializing reports...");
      setReports(bno08x);
      delay(100);  // Allow time for reinitialization
  }


  if (myGNSS.getPVT() == true)
  {
    int32_t latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    int32_t longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
//beginning of section to start getting time
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    
    Serial.println();
    // Serial.print(myGNSS.getYear());
    // Serial.print("-");
    // Serial.print(myGNSS.getMonth());
    // Serial.print("-");
    // Serial.print(myGNSS.getDay());
    Serial.print(" ");
    Serial.print(myGNSS.getHour());
    Serial.print(":");
    Serial.print(myGNSS.getMinute());
    Serial.print(":");
    Serial.print(myGNSS.getSecond());
    Serial.print(" ");

  }

  /*
  GYRO_R, GYRO_P, GYRO_Y, ACCEL_R,ACCEL_P, ACCEL_Y,
  MAG_R, MAG_P, MAG_Y, AUTO_GYRO_ROTATION_RATE,
  GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
  CMD_ECHO [,,OPTIONAL_DATA]
  */
  String message = (
    "3194," //TEAM_ID
  + String(1) + "," //MISSION_TIME
  + String(packetCount) + "," //PACKET_COUNT
  + "mode,state," //MODESTATE
  + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "," //ALTITUDE
  + bmp.readTemperature() + "," //TEMPERATURE
  + bmp.readPressure() + "," //PRESSURE
  + String(ina260.readBusVoltage() / 1000) + "," //VOLTAGE
  + String(sensorValue.un.rawGyroscope.x) + "," 
  + String(sensorValue.un.rawGyroscope.y) + ","
  + String(sensorValue.un.rawGyroscope.z) + ","
  + String(sensorValue.un.linearAcceleration.x) + ","
  + String(sensorValue.un.linearAcceleration.y) + ","
  + String(sensorValue.un.linearAcceleration.z) + ","
  + "1,2,3,"
  + "rotate_rate,"
  + String("placementfortime") + ","
  + String(myGNSS.getAltitudeMSL()) + ","
  + String(myGNSS.getLatitude()) + ","
  + String(myGNSS.getLongitude()) + ","
  + "gpssats" + ","
  + String(echoStr)
  );

  packetCount++;
  send_xbee(message);

  //Handle received messages from XBee
  while(Serial1.available()){
    int first_byte = Serial1.read();
    if (first_byte == 126) {
      // int length = Serial1.read() >> 8 & 0xFF;
      Serial.println(Serial1.read());
      int length = Serial1.read();
      Serial.println(length);
      int packetType = Serial1.read();
 
      if (packetType == 139) { //If packet type is Transmit Status (0x8B)
        for(int i = 0; i < length; i++){
          unused = Serial1.read();
        }
        Serial.println("Packet received by ground station");
      }

      else if (packetType == 144) { //If packet type is Receive Packet (0x90)
        Serial.println("something received");
        for (int i = 0; i < 11; i++){
          // unused = Serial1.read();
          Serial.print(char(Serial1.read()));
          Serial.print(" ");
        }
        Serial.println("\na;lskdfhja;dlfkj");
        delay(1000);
        for(int i = 0; i <= Serial1.available(); i++) {
          Serial.println(char(Serial1.read()));
          // str += char(Serial1.read());

        }
        Serial.println("jhg ");
        // Serial.println(str);
        if (str.substring(0,3).equals("MEC")) {
          Serial.print("2 ");
          if (str.substring( str.indexOf(',') + 1, str.indexOf(',') + 3).equals("RE")) {
            Serial.print("3 ");
            if (str.substring( str.lastIndexOf(',') + 1, str.lastIndexOf(',' + 3) ).equals("ON")) {
              Serial.print("4 ");
              release_servo.write(60);
            }
            else if (str.substring( str.lastIndexOf(',') + 1, str.lastIndexOf(',' + 3) ).equals("OF")) {
              Serial.print("5 ");
              release_servo.write(85);
            }
          }
        }
        echoStr = str;
        // servoPos = str.toInt();
        // release_servo.write(servoPos);
        unused = Serial1.read();
      }
    }
  }
  Serial1.flush();

  delay(1000);
}