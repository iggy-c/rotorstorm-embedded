#include <Wire.h> //enables I2C
#include <SPI.h> //enables SPI
#include <Adafruit_Sensor.h> //used for adafruit stuff
#include "Adafruit_BMP3XX.h" //enables BMP
#include <Adafruit_BNO08x.h> //enables BNO
#include <SD.h> // enables sd communication
#include <Adafruit_INA260.h> // enables INA
#include <ctime> // time library
#include <Servo.h>

#define BMP390_ADDRESS 0x77  // Try 0x76 if this fails
#define BNO08x_ADDRESS 0x4A // or 0x4B if DI pin is pulled high to VCC
#define INA260_ADDRESS 0x40 // my shins hurt
#define servo_wire 11 
#define BMP_SDA 14 //SDA for I2C stuff for bmp
#define BMP_SCL 15 //SCL for I2C stuff for bmp
#define BNO08X_SDA 14 //SDA for I2C stuff for bno
#define BNO08X_SCL 15 //SCL for I2C stuff for bno
#define INA260_SDA 14
#define INA260_SCL 15

const int UARTRX = 1;
const int UARTTX = 0;

Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x;
Adafruit_INA260 ina260;
Servo release_servo;

#define SEALEVELPRESSURE_HPA (1013.25) //calculation for setting sea level pressure from bmp i think

int packetCount = 0;

// String S_state
// String G_state

sh2_SensorValue_t sensorValue; // does something to help bno work


void send_xbee(String message) {
    /*
    this is default command explained
    */

    uint8_t payloadLength = message.length();
    uint8_t frame[20 + payloadLength];  
    uint8_t frameLength = 14 + payloadLength; 
    uint8_t checksum = 0;

    frame[0] = 0x7E; //start delimiter
    frame[1] = (frameLength >> 8) & 0xFF; //length
    frame[2] = frameLength & 0xFF; //length
    frame[3] = 0x10; //frame type
    frame[4] = 0x01; //frame id

    frame[5]  = 0x00; //64-bit destination address
    frame[6]  = 0x13;  
    frame[7]  = 0xA2;  
    frame[8]  = 0x00;  
    frame[9]  = 0x42;  
    frame[10] = 0x5B;  
    frame[11] = 0xD6;  
    frame[12] = 0x18;  

    frame[13] = 0xFF; //16-bit destination address
    frame[14] = 0xFE;

    frame[15] = 0x00; //broadcast radius
    frame[16] = 0x00; //options

    // Copy the message into the payload
    for (uint8_t i = 0; i < payloadLength; i++) {
        frame[17 + i] = message[i];
        checksum += message[i];
    }

    checksum += 0x10 + 0x01;
    checksum += 0x00 + 0x13 + 0xA2 + 0x00 + 0x42 + 0x5B + 0xD6 + 0x18;
    checksum += 0xFF + 0xFE + 0x00 + 0x00;
    checksum = 0xFF - checksum;

    frame[17 + payloadLength] = checksum;

    Serial1.write(frame, 18 + payloadLength);
    
    Serial.print("XBee API frame sent:"); 
    Serial.println(message);
}

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

void setup() {

  Serial.begin(9600);

  Serial1.setRX(UARTRX);
  Serial1.setTX(UARTTX);

  Serial1.begin(9600);//initialize UART
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

  Serial1.setTX(0);  //xbee TX
  Serial1.setRX(1);  //xbee RX

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

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  // setReports();

  // Serial.println("Reading events");
  delay(100);

  release_servo.attach(servo_wire, 400, 2600);
  release_servo.write(180);
  delay(1000);
  release_servo.write(90);
  delay(1000);
  release_servo.write(180);
}


void loop() {

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

if (bno08x.wasReset()) {
    Serial.println("BNO08x was reset, reinitializing reports...");
    setReports();
    delay(100);  // Allow time for reinitialization
}

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
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
  + String(sensorValue.un.accelerometer.x) + ","
  + String(sensorValue.un.accelerometer.y) + ","
  + String(sensorValue.un.accelerometer.z) + ","
  + "1,2,3,rotate_rate,gpstime,gpsaltitude,1,2,gpssats,echo"
  );

  packetCount++;
  send_xbee(message);
  
  int incomingByte = 0;
  if (Serial.available() > 0) {

    // read the incoming byte:

    incomingByte = Serial.read();

    // say what you got:

    Serial1.print("I received: ");
    Serial1.println(incomingByte, DEC);

  }

  delay(1000);
}