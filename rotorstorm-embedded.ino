#include <Wire.h>                                  //enables I2C
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // enables GNSS
// #include <Adafruit_BNO08x.h>                       // enables BNO085
#include "Adafruit_BMP3XX.h"                       //enables BMP
#include <Adafruit_Sensor.h>                       //enables adafruit sensors
#include <Servo.h>                                 // enables servo
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BMP390_ADDRESS 0x77  // Try 0x76 if this fails
#define gnssAddress 0x42     // The default I2C address for u-blox modules is 0x42. Change this if required
// #define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed
#define TACHOMETER_ADRESS 0x12
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // go from wire to wire as needed

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; // this could cause timing issues
#define I2C_SDA 12
#define I2C_SCL 13
#define SERVO_CTRL 11
#define VOLT_PIN 26
#define SER1_TX 0
#define SER1_RX 1
#define CAM_TX 8
#define CAM_RX 9
#define TACH_SDA 14
#define TACH_SCL 15

#define SEALEVELPRESSURE_HPA (1013.25)
#define START_DELIMITER 0x7E  // XBee API start byte

Servo release_servo;
SFE_UBLOX_GNSS myGNSS;
Adafruit_BMP3XX bmp;
// Adafruit_BNO08x bno08x;
// BNO08x myIMU;
// sh2_SensorValue_t sensorValue;
bool telemetry = true;
String packet;
String echo;
int packetCount = 1;

float calibrated_pressure;
int32_t latitude;
int32_t longitude;
int32_t gps_altitude;
uint16_t rpm;
byte SIV;

bool DEBUG_MODE = false;


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



void setup() {

  // initializing wires
  analogReadResolution(12);
  Serial.begin(9600);
  Serial1.setRX(SER1_RX);
  Serial1.setTX(SER1_TX);
  Serial1.begin(9600);
  while (DEBUG_MODE && !Serial);

  Serial.println("Serial Started");
  delay(10);
  Wire.setSDA(I2C_SDA); // i2c setpup
  Wire.setSCL(I2C_SCL); // i2c setup
  Wire.begin();
  Wire1.setSDA(TACH_SDA);
  Wire1.setSCL(TACH_SCL);
  Wire1.begin();

  delay(100);
  // checking BMP
  if (!bmp.begin_I2C()) {  //this is important to initialize the i2c bus
    Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
    while (DEBUG_MODE)
      ;  // Halt execution
  }
if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  // Checking BNO
// if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
//     Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
//   }
//   Serial.println("BNO08x found!");

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(100);

  for (int i = 0; i < 10; i++) {
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
    }

    calibrated_pressure = bmp.pressure / 100.0;

    if (DEBUG_MODE) {
      Serial.print("Temperature: ");
      Serial.println(bmp.temperature);

      Serial.print("Pressure: ");
      Serial.println(calibrated_pressure);
    }
  }

  //beginning of GNSS setup
  delay(100);

  while (myGNSS.begin(Wire, gnssAddress) == false) {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay(1000);
  }

  // release_servo.attach(SERVO_CTRL, 400, 2600);
  // release_servo.write(70);

  delay(100);
}

void loop() {

sensors_event_t linearAccelData, magnetometerData, accelerometerData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);

  String packet_payload = "";
  int unused = 0;
  
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading on BMP :(");
  }
  // beginning of GNSS stuff
  if (myGNSS.getPVT() == true) {
    latitude = myGNSS.getLatitude();
    // Serial.print(myGNSS.getLatitude());
    longitude = myGNSS.getLongitude();
    // Serial.print(myGNSS.getLongitude());
    gps_altitude = myGNSS.getAltitudeMSL();  // Altitude above Mean Sea Level
    // Serial.println(gps_altitude);

    lastTime = millis(); //Update the timer
    SIV = myGNSS.getSIV();
    // Serial.print(SIV);
  }
  

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
        // Serial.print(unused);
      }

      else if (packetType == 144) {  //If packet type is Receive Packet (0x90)
        Serial.println("something received");
        for (int i = 0; i < 11; i++) {
          unused = Serial1.read();
        }
        for (int i = 0; i <= Serial1.available(); i++) {
          packet_payload += char(Serial1.read());
        }
        if (packet_payload.equals("cal")) {
          Serial.println("calibrating");
          calibrated_pressure = bmp.pressure / 100;
        } else if (packet_payload.equals("rel")) {
          Serial.println("releasing");
          release_servo.write(65);
          echo = "";
        } else if (packet_payload.equals("clo")) {
          Serial.println("priming");
          release_servo.write(80);
          echo = "";
        } else if (packet_payload.equals("con")) {
          Serial.println("telemetry on");
          telemetry = true;
          echo = "CXON";
        } else if (packet_payload.equals("cof")) {
          Serial.println("telemetry off");
          telemetry = false;
          echo = "CXOFF";
        } else {
          Serial.print("unrecognized command: ");
          Serial.println(packet_payload);
        }

        unused = Serial1.read();
      }
    }
  }
  // Serial.println(packet_payload);
  Serial1.flush();


  currentaltitude = bmp.readAltitude(intPress);

  String flightState = statemachine(currentaltitude, previousaltitude, ppreviousaltitude, pppreviousaltitude, ppppreviousaltitude, pppppreviousaltitude, maximumaltitude, bmaltitude);
  bmaltitude = maximumaltitude;
  maximumaltitude = pppppreviousaltitude;
  pppppreviousaltitude = ppppreviousaltitude;
  ppppreviousaltitude = pppreviousaltitude;
  pppreviousaltitude = ppreviousaltitude;
  ppreviousaltitude = previousaltitude;
  previousaltitude = currentaltitude;
  // Serial.println("flightstate: ");
  // Serial.println(flightState);

  Wire1.requestFrom(TACHOMETER_ADRESS, 2);
  if (Wire1.available() == 2) {                // If 2 bytes are available
    byte lowByte = Wire1.read();               // Read low byte
    byte highByte = Wire1.read();              // Read high byte
    uint16_t rpm = (highByte << 8) | lowByte;  // Combine the two bytes into a 16-bit integer
    // Print RPM value
    Serial.println("******");
    Serial.println(rpm);
    Serial.println("******");
  } else {
    Serial.println("incomplete data wire1");
  }

  if (telemetry) {
    

    String packet =
      ("3194,"                                                                          //TEAM_ID
       + String(myGNSS.getHour()) + ":" + String(myGNSS.getMinute()) + ":" + String(myGNSS.getSecond()) + ","                                                                //MISSION_TIME
       + String(packetCount) + ","                                                      //PACKET_COUNT
       + "F,"                                                                        //MODE
       + String(flightState) + ","                                                      //STATE
       + String(float(bmp.readAltitude(calibrated_pressure)), 1) + ","                            //ALTITUDE
       + String(float(bmp.readTemperature()), 1) + ","                                                    //TEMPERATURE
       + String(float(bmp.readPressure()/1000), 1) + ","                                                       //PRESSURE
       + String(float((analogRead(VOLT_PIN)) * (3.3 / 4095.0) / (600.0 / 1985.5)), 1) + ","  //VOLTAGE
       + String(float(linearAccelData.acceleration.x)) + ","                                       //GYRO X
       + String(float(linearAccelData.acceleration.y)) + ","                                       //GYRO Y
       + String(float(linearAccelData.acceleration.z)) + ","                                       //GYRO Z
       + String(float(accelerometerData.acceleration.x)) + ","
       + String(float(accelerometerData.acceleration.y)) + ","
       + String(float(accelerometerData.acceleration.z)) + ","
       + String(float(magnetometerData.magnetic.x/100)) + ","
       + String(float(magnetometerData.magnetic.y/100)) + ","
       + String(float(magnetometerData.magnetic.z/100)) + ","
       + String(rpm) + ","
       + String(myGNSS.getHour()) + ":" + String(myGNSS.getMinute()) + ":" + String(myGNSS.getSecond()) + ","
       + String(float(gps_altitude/1000.0), 1) + ","
       + String(float(latitude/10000000.0), 4)+ ","
       + String(float(longitude/10000000.0), 4) + ","
       + String(SIV) + ","
       + String(echo))
       + ",,,";

    packetCount++;

    // Serial.println(packet);
    send_xbee(packet);
  }
  delay(1);
}

void 

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }
  delay(1000);
}