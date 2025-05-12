#include <Wire.h>                                  //enables I2C
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // enables GNSS
#include <Adafruit_BNO08x.h>                       // enables BNO085
#include "Adafruit_BMP3XX.h"                       //enables BMP
#include <Adafruit_Sensor.h>
#include <Servo.h>             // enables servo
#include <SoftwareSerial.h>    // openlog bout to pull another allnighter aleady tired gonna die soon

#define BMP390_ADDRESS 0x77    // Try 0x76 if this fails
#define gnssAddress 0x42       // The default I2C address for u-blox modules is 0x42. Change this if required
#define BNO08x_ADDRESS 0x4A

#define I2C_SDA 12
#define I2C_SCL 13
#define BNO08X_SDA 12
#define BNO08X_SCL 13
#define servo_wire 11

const int UARTTX = 0;
const int UARTRX = 1;
const int UARTTXUPCAM0 = 8;  //upcam tx
const int UARTRXUPCAM0 = 9;  // upcam rx
int packetCount = 1;
const int analogPin = 26;

#define SEALEVELPRESSURE_HPA (1013.25)
#define START_DELIMITER 0x7E  // XBee API start byte

// uint16_t packetCounter = 1;  // Start from packet #1
// Servo myServo;
Servo release_servo;
int servoPos = 0;
SFE_UBLOX_GNSS myGNSS;
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

String packet;  

int32_t latitude;
int32_t longitude;
int32_t gps_altitude;
byte SIV;

void send_xbee(String message) {

  uint8_t payloadLength = message.length();
  uint8_t frame[20 + payloadLength];
  uint8_t frameLength = 14 + payloadLength;
  uint8_t checksum = 0;

  frame[0] = START_DELIMITER;
  frame[1] = (frameLength >> 8) & 0xFF;
  frame[2] = frameLength & 0xFF;
  frame[3] = 0x10;
  frame[4] = 0x01;

  frame[5] = 0x00;
  frame[6] = 0x13;
  frame[7] = 0xA2;
  frame[8] = 0x00;
  frame[9] = 0x42;
  frame[10] = 0x5B;
  frame[11] = 0xD6;
  frame[12] = 0x18;

  frame[13] = 0xFF;
  frame[14] = 0xFE;
  frame[15] = 0x00;
  frame[16] = 0x01;

  // Copy the message into the payload
  for (uint8_t i = 0; i < payloadLength; i++) {
    frame[17 + i] = message[i];
    checksum += message[i];
  }

  checksum += 0x10 + 0x01;
  checksum += 0x00 + 0x13 + 0xA2 + 0x00 + 0x42 + 0x5B + 0xD6 + 0x18;
  checksum += 0xFF + 0xFE + 0x00 + 0x01;
  checksum = 0xFF - checksum;

  frame[17 + payloadLength] = checksum;

  Serial1.write(frame, 18 + payloadLength);

  Serial.print("XBee API frame sent: ");
  Serial.println(message);
}
//end initializing xbee packet setup stuff
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
long lastTime = 0;  //Simple local timer. Limits amount if I2C traffic to u-blox module.
float currentaltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
float truemaximumaltitude = 0;
float intPress;
bool launch_pad = true;
bool ascent = false;
bool apogee = false;
bool descent = false;
bool probe_release = false;

float maxaltitude = 0;
//  float currentaltitude = 0;
float previousaltitude = 0;
float ppreviousaltitude = 0;
float pppreviousaltitude = 0;
float ppppreviousaltitude = 0;
float pppppreviousaltitude = 0;
float maximumaltitude = 0;
float bmaltitude = 0;

void setup() {

  analogReadResolution(12);
  Serial.begin(9600);
  Serial1.setRX(UARTRX);
  Serial1.setTX(UARTTX);
  Serial1.begin(9600);
  while (!Serial);

  Serial.println("Starting XBee API packet transmission...");

  delay(500);  // Allow XBee to initialize

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(BNO08X_SDA);
  Wire.setSCL(BNO08X_SCL);
  Wire.begin();
  
  delay(500);

  if (!bmp.begin_I2C()) {  //this is important to initialize the i2c bus
    Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
    while (1);  // Halt execution
  }

  if (!bno08x.begin_I2C(BNO08x_ADDRESS, &Wire)) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }
  delay(10);

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // question
  delay(100);
  int n = 2;
  while (n < 10)  // check this number to see whats working and whats not
  {
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    intPress = (bmp.pressure / 100.0);
    Serial.print(intPress);
    Serial.println(" hPa");
    n++;
  }
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  intPress = (bmp.pressure / 100);


  //beginning of GNSS setup
  delay(500);

  while (myGNSS.begin(Wire, gnssAddress) == false) 
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay(1000);
  }

  release_servo.attach(servo_wire, 400, 2600);
  setReports();
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY)) {
    Serial.println("Could not enable gravity vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
    Serial.println("Could not enable geomagnetic rotation vector");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector");
  }
  if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
    Serial.println("Could not enable step counter");
  }
  if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
    Serial.println("Could not enable stability classifier");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
  if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
    Serial.println("Could not enable shake detector");
  }
  if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
    Serial.println("Could not enable personal activity classifier");
  }
}

void loop() {
  String sp = "";
  int unused = 0;
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading on BMP :(");
  }
  // beginning of GNSS stuff
  if (myGNSS.getPVT() == true) {
    latitude = myGNSS.getLatitude();
    Serial.print(myGNSS.getLatitude());
    longitude = myGNSS.getLongitude();
    Serial.print(myGNSS.getLongitude());
    gps_altitude = myGNSS.getAltitudeMSL();  // Altitude above Mean Sea Level
    Serial.println(gps_altitude);

    SIV = myGNSS.getSIV();
    Serial.print(SIV);

  }
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  (!bno08x.getSensorEvent(&sensorValue));
Serial.println(float(analogRead(analogPin)));
  Serial.println(float(analogRead(analogPin)) * (3.3 / 4095.0) / (680.0 / 2180));

  //Handle received messages from XBee
  while (Serial1.available()) {
    int first_byte = Serial1.read();
    if (first_byte == 126) {
      int length = Serial1.read() << 8 & 0xFF;
      length += Serial1.read() & 0xFF;

      int packetType = Serial1.read();

      if (packetType == 139) {  //If packet type is Transmit Status (0x8B)
        for (int i = 0; i < length; i++) {
          unused = Serial1.read();
        }
        Serial.println("Packet received by ground station");
      }

      else if (packetType == 144) {  //If packet type is Receive Packet (0x90)
        Serial.println("something received");
        for (int i = 0; i < 11; i++) {
          unused = Serial1.read();
        }
        for (int i = 0; i <= Serial1.available(); i++) {
          sp += char(Serial1.read());
        }
        if (sp.equals("cal")) {
          //do later
        } else if (sp.equals("ron")) {
          Serial.println("releasing");
          release_servo.write(60);
        } else if (sp.equals("rof")) {
          Serial.println("priming");
          release_servo.write(83);
        }

        unused = Serial1.read();
      }
    }
  }
  Serial.println(sp);
  Serial1.flush(); //beware of this dont know what it does exaclty


  currentaltitude = bmp.readAltitude(intPress);

  String flightState = statemachine(currentaltitude, previousaltitude, ppreviousaltitude, pppreviousaltitude, ppppreviousaltitude, pppppreviousaltitude, maximumaltitude, bmaltitude);
  bmaltitude = maximumaltitude;
  maximumaltitude = pppppreviousaltitude;
  pppppreviousaltitude = ppppreviousaltitude;
  ppppreviousaltitude = pppreviousaltitude;
  pppreviousaltitude = ppreviousaltitude;
  ppreviousaltitude = previousaltitude;
  previousaltitude = currentaltitude;
  Serial.println("flightstate: ");
  Serial.println(flightState);

  String packet =
    ("3194,"                                        //TEAM_ID
     + String(1) + ","                              //MISSION_TIME
     + String(packetCount) + ","                    //PACKET_COUNT
     + "mode,"                                      //MODE
     + String(flightState) + ","                    //STATE
     + String(bmp.readAltitude(intPress)) + ","     //ALTITUDE
     + bmp.readTemperature() + ","                  //TEMPERATURE
     + bmp.readPressure() + ","                     //PRESSURE
     + String(float(analogRead(analogPin)) * (3.3 / 4095.0) / (680.0 / 1500.0)) + ","                        //VOLTAGE
     + String(sensorValue.un.gyroscope.x) + ","  //GYRO X
     + String(sensorValue.un.gyroscope.y) + ","  //GYRO Y
     + String(sensorValue.un.gyroscope.z) + ","  //GYRO Z
     // + String(gyro)
     + ",,,,,,"
     // + String(String(&angVelocityData)) + ","
     // + String(String(&angVelocityData)) + ","
     // + String(sensorValue.un.linearAcceleration.x) + ","
     // + String(sensorValue.un.linearAcceleration.y) + ","
     // + String(sensorValue.un.linearAcceleration.z) + ","
     + ",,,rotate_rate,"
     + "gpstime,"
     + String(gps_altitude) + ","
     + String(latitude) + ","
     + String(longitude) + ","
     + String(SIV) + ","
     + "echo" + "z");

  packetCount++;

  Serial.println(packet);

  send_xbee(packet);
  delay(250);
}