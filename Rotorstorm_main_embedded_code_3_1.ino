#include <Wire.h> //enables I2C
#include <SPI.h> //enables SPI
#include <Adafruit_Sensor.h> //used for adafruit stuff
#include "Adafruit_BMP3XX.h" //enables BMP
#include <Adafruit_BNO08x.h> //enables BNO
#include <SD.h> // enables sd connumication
#include <Adafruit_INA260.h> // enables INA

#define BMP390_ADDRESS 0x77  // Try 0x76 if this fails
#define BNO08x_ADDRESS 0x4A // or 0x4B if DI pin is pulled high to VCC
#define INA260_ADDRESS 0x40 // my shins hurt
#define BMP_SDA 14 //SDA for I2C stuff for bmp
#define BMP_SCL 15 //SCL for I2C stuff for bmp
#define BNO08X_SDA 14 //SDA for I2C stuff for bno
#define BNO08X_SCL 15 //SCL for I2C stuff for bno
#define INA260_SDA 14
#define INA260_SCL 15

#define SEALEVELPRESSURE_HPA (1013.25) //calculation for setting sea level prussure fro bmp i think
#define START_DELIMITER 0x7E  // XBee API start byte

const int UARTRX = 1;
const int UARTTX = 0;


int packetCount = 0;
uint16_t packetCounter = 1;  // Start from packet #1
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x;
Adafruit_INA260 ina260;

// String S_state
// String G_state


sh2_SensorValue_t sensorValue; // does something to help bno work

// begin intializing the xbee packet stuff
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

    frame[5]  = 0x00;  
    frame[6]  = 0x13;  
    frame[7]  = 0xA2;  
    frame[8]  = 0x00;  
    frame[9]  = 0x42;  
    frame[10] = 0x5B;  
    frame[11] = 0xD6;  
    frame[12] = 0x18;  

    frame[13] = 0xFF;
    frame[14] = 0xFE;
    frame[15] = 0x00;
    frame[16] = 0x00;

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
    
    Serial.print("XBee API frame sent: ");
    Serial.println(message);
}
//end initializing xbee packet setup stuff

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
      while (!Serial);  // Wait for Serial Monitor to open

      Serial.println("Starting BMP3XX Sensor Test...");

  Serial1.setRX(UARTRX);
  Serial1.setTX(UARTTX);
      Serial1.begin(9600);//initailize UART
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

      delay(2000);  // Allow XBee to initialize
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
      // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
      // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
      Serial.println("Failed to find BNO08x chip");
      while (1) {
        delay(10);
      }
    }
    Serial.println("BNO08x Found!");

    // Initailize INA sensor
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
  //beginning for some bno print initialization
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }


  setReports();

  Serial.println("Reading events");
  delay(100);
}
  //end for some bno print initialization


  // beginning of where you define the sensor outputs you want to receive for bno
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
void printActivity(uint8_t activity_id) {
  switch (activity_id) {
  case PAC_UNKNOWN:
    Serial.print("Unknown");
    break;
  case PAC_IN_VEHICLE:
    Serial.print("In Vehicle");
    break;
  case PAC_ON_BICYCLE:
    Serial.print("On Bicycle");
    break;
  case PAC_ON_FOOT:
    Serial.print("On Foot");
    break;
  case PAC_STILL:
    Serial.print("Still");
    break;
  case PAC_TILTING:
    Serial.print("Tilting");
    break;
  case PAC_WALKING:
    Serial.print("Walking");
    break;
  case PAC_RUNNING:
    Serial.print("Running");
    break;
  case PAC_ON_STAIRS:
    Serial.print("On Stairs");
    break;
  default:
    Serial.print("NOT LISTED");
  }
  Serial.print(" (");
  Serial.print(activity_id);
  Serial.print(")");
  // end of where you define the sensor outputs you want to recive for bno
}

void loop() {
  // put your main code here, to run repeatedly:
if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }
//beginning of bmp sensor data
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();
//end of bmp sensor data

//beginning of bno sensor data
delay(10);

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

  case SH2_ACCELEROMETER:
    Serial.print("Accelerometer - x: ");
    Serial.print(sensorValue.un.accelerometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.accelerometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.accelerometer.z);
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    Serial.print("Gyro - x: ");
    Serial.print(sensorValue.un.gyroscope.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gyroscope.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gyroscope.z);
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    Serial.print("Magnetic Field - x: ");
    Serial.print(sensorValue.un.magneticField.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.magneticField.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.magneticField.z);
    break;
  case SH2_LINEAR_ACCELERATION:
    Serial.print("Linear Acceration - x: ");
    Serial.print(sensorValue.un.linearAcceleration.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.linearAcceleration.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.linearAcceleration.z);
    break;
  case SH2_GRAVITY:
    Serial.print("Gravity - x: ");
    Serial.print(sensorValue.un.gravity.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gravity.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gravity.z);
    break;
  case SH2_ROTATION_VECTOR:
    Serial.print("Rotation Vector - r: ");
    Serial.print(sensorValue.un.rotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.rotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.rotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.rotationVector.k);
    break;
  case SH2_GEOMAGNETIC_ROTATION_VECTOR:
    Serial.print("Geo-Magnetic Rotation Vector - r: ");
    Serial.print(sensorValue.un.geoMagRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.geoMagRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.geoMagRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.geoMagRotationVector.k);
    break;

  case SH2_GAME_ROTATION_VECTOR:
    Serial.print("Game Rotation Vector - r: ");
    Serial.print(sensorValue.un.gameRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.gameRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.gameRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.gameRotationVector.k);
    break;

  case SH2_STEP_COUNTER:
    Serial.print("Step Counter - steps: ");
    Serial.print(sensorValue.un.stepCounter.steps);
    Serial.print(" latency: ");
    Serial.println(sensorValue.un.stepCounter.latency);
    break;

  case SH2_STABILITY_CLASSIFIER: {
    Serial.print("Stability Classification: ");
    sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
    switch (stability.classification) {
    case STABILITY_CLASSIFIER_UNKNOWN:
      Serial.println("Unknown");
      break;
    case STABILITY_CLASSIFIER_ON_TABLE:
      Serial.println("On Table");
      break;
    case STABILITY_CLASSIFIER_STATIONARY:
      Serial.println("Stationary");
      break;
    case STABILITY_CLASSIFIER_STABLE:
      Serial.println("Stable");
      break;
    case STABILITY_CLASSIFIER_MOTION:
      Serial.println("In Motion");
      break;
    }
    break;
  }

  case SH2_RAW_ACCELEROMETER:
    Serial.print("Raw Accelerometer - x: ");
    Serial.print(sensorValue.un.rawAccelerometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.rawAccelerometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.rawAccelerometer.z);
    break;
  case SH2_RAW_GYROSCOPE:
    Serial.print("Raw Gyro - x: ");
    Serial.print(sensorValue.un.rawGyroscope.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.rawGyroscope.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.rawGyroscope.z);
    break;
  case SH2_RAW_MAGNETOMETER:
    Serial.print("Raw Magnetic Field - x: ");
    Serial.print(sensorValue.un.rawMagnetometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.rawMagnetometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.rawMagnetometer.z);
    break;

  case SH2_SHAKE_DETECTOR: {
    Serial.print("Shake Detector - shake detected on axis: ");
    sh2_ShakeDetector_t detection = sensorValue.un.shakeDetector;
    switch (detection.shake) {
    case SHAKE_X:
      Serial.println("X");
      break;
    case SHAKE_Y:
      Serial.println("Y");
      break;
    case SHAKE_Z:
      Serial.println("Z");
      break;
    default:
      Serial.println("None");
      break;
    }
  }

//end of bno sensor data

  // begining of INA sensor data
  Serial.print("Current: ");
  Serial.print(ina260.readCurrent());
  Serial.println(" mA");

  Serial.print("Bus Voltage: ");
  Serial.print(ina260.readBusVoltage());
  Serial.println(" mV");

  Serial.print("Power: ");
  Serial.print(ina260.readPower());
  Serial.println(" mW");

  Serial.println();
  delay(1000);
  // end of INA sensor data
  }

  // float pressure = bmp.pressure / 100.0;
  // float (comment hrere so you can probobly just use the actaul value for the strings to send)

  String message = ("Pressure: " + String(bmp.pressure / 100.0) + "Altitude: " + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "Temperature: " + String(bmp.temperature) + "Voltage: " + String(ina260.readBusVoltage()) + "Accelerometer_x: " + String(sensorValue.un.rawAccelerometer.x) + "Acceleromter_y: " + String(sensorValue.un.rawAccelerometer.y) + "Accelerometer_z: ");
  message = message + ("Gyroscope_x: " + String(sensorValue.un.rawGyroscope.x) + "Gyroscope_y: " + String(sensorValue.un.rawGyroscope.y) +)
  send_xbee(message);
}