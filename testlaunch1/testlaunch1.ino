#include <Wire.h> //enables I2C
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // enables GNSS
#include <Adafruit_INA260.h> // enables INA260
#include <Adafruit_BNO055.h> // enables BNO055
#include "Adafruit_BMP3XX.h" //enables BMP
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h> // BNO055 math
#include <Servo.h> // enables servo
#include <SoftwareSerial.h> // openlog bout to pull another allnighter aleady tired gonna die soon
#define BMP_SDA 14 //SDA for I2C stuff for bmp
#define BMP_SCL 15 //SCL for I2C stuff for bmp
#define _I2CSDA 14 // SDA for I2C stuff for BNO055
#define _I2CSCL 15 // SCL for I2C stuff for BNO055
#define INA260_SDA 14 // SDA for I2C stuff for INA
#define INA260_SCL 15 // SCL for I2C stuff for INA
#define GNSS_SDA 14 // SDA for I2C stuff for SDA
#define GNSS_SCL 15 // SCL for I2C stuff for SCL
#define BMP390_ADDRESS 0x77  // Try 0x76 if this fails
#define INA260_ADDRESS 0x40 // INA address
#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required
#define SEALEVELPRESSURE_HPA (1013.25) 
#define START_DELIMITER 0x7E  // XBee API start byte
#define servo_wire 11 
const int UARTRX = 1;
const int UARTTX = 0;
const int UARTTXUPCAM0 = 8; //upcam tx
const int UARTRXUPCAM0 = 9; // upcam rx
int packetCount = 1;
// uint16_t packetCounter = 1;  // Start from packet #1
// Servo myServo;
Servo release_servo;
int servoPos = 0;
SFE_UBLOX_GNSS myGNSS; 
Adafruit_INA260 ina260; //= Adafruit_INA260();
Adafruit_BMP3XX bmp;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // adress for BNO055
/* Set the delay between fresh samples */
//this delay might mess up timers so keep this in mind
// Servo release_servo;
// int servoPos = 0; // initializes servo pos
// // begin intializing the xbee packet stuff
String message; // this may mess things up so be wary
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
// //end initializing xbee packet setup stuff
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
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
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  // statemachine(state); only put this here if u need to initialize it for some reason why this no work idk
  Serial1.setRX(UARTRX);
  Serial1.setTX(UARTTX);
  Serial1.begin(9600);//initailize UART
  Serial1.setTX(0);  //xbee TX
  Serial1.setRX(1);  //xbee RX

  Serial.println("Starting XBee API packet transmission...");

  delay(2000);  // Allow XBee to initialize
  // Initialize I2C1 (Wire1) for BMP3XX
  Wire1.setSDA(BMP_SDA);
  Wire1.setSCL(BMP_SCL);
  Wire1.begin();
  // Initialize BMP3XX sensor using I2C1
  if (!bmp.begin_I2C(BMP390_ADDRESS, &Wire1)) {  //this is important to initialize the i2c bus
      Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
      while (1);  // Halt execution
  }

  Serial.println("BMP3XX Sensor Initialized!");


   // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // question
  delay(100);
  int n=2;
  while(n<10) // check this number to see whats working and whats not
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
    Serial.println(" hPa");
    n++;

  }
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
    }
  intPress = (bmp.pressure/100);

// beginning of INA steup
// Wait until serial port is opened
  // while (!Serial) { delay(10); }

  Serial.println("Adafruit INA260 Test");

  Wire1.setSDA(INA260_SDA);
  Wire1.setSCL(INA260_SCL);
  Wire1.begin();

  if (!ina260.begin(INA260_ADDRESS, &Wire1)) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
  // end of INA setup

  //beginning of GNSS setup
  delay(1000); 
  Serial.println("SparkFun u-blox Example");

  //myWire.begin(); // Start I2C maybe kinda iff idk man 
  Wire1.setSDA(GNSS_SDA);
  Wire1.setSCL(GNSS_SCL);
  Wire1.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myGNSS.begin(Wire1, gnssAddress) == false) //Connect to the u-blox module using our custom port and address
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }
  // end of GNSS setup

  // myServo.attach(11);
  // release_servo.attach(servo_wire, 400, 2600); // attaches, min threshold, max threshold
  // release_servo.write(60); // unlocked position
  release_servo.attach(servo_wire, 400, 2600);
  release_servo.write(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  String sp = "";
  int unused = 0;
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(intPress));
  Serial.println(" m");

  Serial.println();

  //beginning of INA stuff
  Serial.print("Bus Voltage: ");
  Serial.print(ina260.readBusVoltage());
  Serial.println(" mV");
  // end of INA stuff

  // beginning of GNSS stuff
  if (myGNSS.getPVT() == true) {
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

  // if (millis() - lastTime > 1000) {
  //   lastTime = millis(); //Update the timer

  //   long latitude = myGNSS.getLatitude();
  //   Serial.print(F("Lat: "));
  //   Serial.print(latitude);

  //   long longitude = myGNSS.getLongitude();
  //   Serial.print(F(" Long: "));
  //   Serial.print(longitude);
  //   Serial.print(F(" (degrees * 10^-7)"));

  //   long altitude = myGNSS.getAltitude();
  //   Serial.print(F(" Alt: "));
  //   Serial.print(altitude);
  //   Serial.print(F(" (mm)"));

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

  // }
  // end of GNSS stuff

  //beginning of BNO055 loop code
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  // sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // printEvent(&orientationData);
  // printEvent(&angVelocityData);
  // printEvent(&linearAccelData);
  // printEvent(&magnetometerData);
  // printEvent(&accelerometerData);
  // printEvent(&gravityData);


  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  // Serial.println("--");
  // delay(BNO055_SAMPLERATE_DELAY_MS);
  //end of BNO055 loop code

  //Handle received messages from XBee
  while(Serial1.available()){
    int first_byte = Serial1.read();
    if (first_byte == 126) {
      int length = Serial1.read() << 8 & 0xFF;
      length += Serial1.read() & 0xFF;

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
          unused = Serial1.read();
        }
        for(int i = 0; i <= Serial1.available(); i++){
          sp += char(Serial1.read());
        }
        if(sp.equals("cal")){
          //do later
        }
        else if(sp.equals("ron")){
          Serial.println("releasing");
          release_servo.write(60);
        }
        else if(sp.equals("rof")){
          Serial.println("priming");
          release_servo.write(83);
        }
      
        unused = Serial1.read();
      }
    }
  }
  Serial.println(sp);
  Serial1.flush();



// String message = ("Team_ID: " + "3194"
// + "Mission_time: " + "mission_timeplaceholder"
// + "Packet_Count: " + packetCount
// + "Mode: " + "modeplaceholder"
// // + "State: " + String(flightState)
// + "Altitude: " + String(bmp.readAltitude(SEALEVELPRESSURE_HPA))
// + "Temperature: " + String(bmp.temperature)
// + "Pressure: " + String(bmp.pressure / 100.0)
// + "Voltage: " + String(ina260.readBusVoltage()
// // + "Gyro_R: " + String(&angVelocityData)
// // + "Gyro_P: " + String(&angVelocityData)
// // + "Gyro_Y: " + String(&angVelocityData)
// // + "Accel_R: " + String(&linearAccelData)
// // + "Accel_P: " + String(&linearAccelData)
// // + "Accel_Y: " + String(&linearAccelData)
// // + "Mag_R: " + String(&magnetometerData)
// // + "Mag_P: " + String(&magnetometerData)
// // + "Mag_Y: " + String(&magnetometerData)
// + "Auto_Gyro_Rotation_Rate: " + "placeholder_tachometerdata"
// + "GPS_Time: " + String( myGNSS.getHour() + myGNSS.getMinute() + myGNSS.getSecond() )
// + "GPS_Altitude: " + String(myGNSS.getAltitude())
// + "GPS_Latitude: " + String(myGNSS.getLatitude())
// + "GPS_Longitude: " + String(myGNSS.getLongitude())
// + "GPS_Sats: " + String(myGNSS.getSIV())
// + "CMD_ECHO: " + "echoplaceholder")
  // String message = ("Team_ID: " + "3194" + "Mission_time: " + "mission_timeplaceholder" + "Packet_Count: " + packetCount + "Mode: " + "modeplaceholder" + "State: " + String(flightState) + "Altitude: " + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "Temperature: " + String(bmp.temperature) + "Pressure: " + String(bmp.pressure / 100.0) + "Voltage: " + String(ina260.readBusVoltage());
  // message += ("Gyro_R: " + String(&angVelocityData) + "Gyro_P: " + String(&angVelocityData) + "Gyro_Y: " + String(&angVelocityData) + "Accel_R: " + String(&linearAccelData) + "Accel_P: " + String(&linearAccelData) + "Accel_Y: " + String(&linearAccelData) + "Mag_R: " + String(&magnetometerData) + "Mag_P: " + String(&magnetometerData) + "Mag_Y: " + String(&magnetometerData) + "Auto_Gyro_Rotation_Rate: " + "placeholder_tachometerdata");
  // message += ("GPS_Time: " + String( myGNSS.getHour() + myGNSS.getMinute() + myGNSS.getSecond() ) + "GPS_Altitude: " + String(myGNSS.getAltitude()) + "GPS_Latitude: " + String(myGNSS.getLatitude()) + "GPS_Longitude: " + String(myGNSS.getLongitude()) + "GPS_Sats: " + String(myGNSS.getSIV()) + "CMD_ECHO: " + "echoplaceholder");
  currentaltitude = bmp.readAltitude(intPress);

  // Serial1.print(message); // write to openlog and xbee

  String flightState = statemachine(currentaltitude, previousaltitude, ppreviousaltitude, pppreviousaltitude, ppppreviousaltitude, pppppreviousaltitude, maximumaltitude, bmaltitude);
  bmaltitude = maximumaltitude;
  maximumaltitude = pppppreviousaltitude;
  pppppreviousaltitude = ppppreviousaltitude;
  ppppreviousaltitude = pppreviousaltitude;
  pppreviousaltitude = ppreviousaltitude;
  ppreviousaltitude = previousaltitude;
  previousaltitude = currentaltitude;
  Serial.println("flightstate: " );
  Serial.println(flightState);

  String message = (
    "3194," //TEAM_ID
  + String(1) + "," //MISSION_TIME
  + String(packetCount) + "," //PACKET_COUNT
  + "mode,"
  + String(flightState) + "," 
  + String(bmp.readAltitude(intPress)) + "," //ALTITUDE
  + bmp.readTemperature() + "," //TEMPERATURE
  + bmp.readPressure() + "," //PRESSURE
  + String(ina260.readBusVoltage() / 1000) + "," //VOLTAGE
  // + String(gyro)
  + ",,,,,,"
  // + String(String(&angVelocityData)) + ","
  // + String(String(&angVelocityData)) + ","
  // + String(sensorValue.un.linearAcceleration.x) + ","
  // + String(sensorValue.un.linearAcceleration.y) + ","
  // + String(sensorValue.un.linearAcceleration.z) + ","
  + "1,2,3,rotate_rate,gpstime,gpsaltitude,1,2,gpssats,echoz"
  );

  packetCount++;

  send_xbee(message);
  delay(250);
}

// void printEvent(sensors_event_t* event) // prints data from BNO055
//  {
//   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//     Serial.print("Accl:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else if (event->type == SENSOR_TYPE_ORIENTATION) {
//     Serial.print("Orient:");
//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//   }
//   else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
//     Serial.print("Mag:");
//     x = event->magnetic.x;
//     y = event->magnetic.y;
//     z = event->magnetic.z;
//   }
//   else if (event->type == SENSOR_TYPE_GYROSCOPE) {
//     Serial.print("Gyro:");
//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//   }
//   else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
//     Serial.print("Rot:");
//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//   }
//   else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//     Serial.print("Linear:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }

//   else {
//     Serial.print("Unk:");
//   }

//   Serial.print("\tx= ");
//   Serial.print(x);
//   Serial.print(" |\ty= ");
//   Serial.print(y);
//   Serial.print(" |\tz= ");
//   Serial.println(z);

// }