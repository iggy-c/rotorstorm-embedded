#include <Wire.h>                     // enables I2C
#include <SparkFun_u-blox_GNSS_v3.h>  // enables GNSS
#include "Adafruit_BMP3XX.h"          // enables BMP
#include <Adafruit_Sensor.h>          // enables adafruit sensors
#include <Servo.h>                    // enables servo
#include <Adafruit_BNO055.h>          // enables BNO055
#include <utility/imumaths.h>
#include <arduino-timer.h>

#define gnssAddress 0x42
#define TACHOMETER_ADRESS 0x12

#define I2C_SDA 12
#define I2C_SCL 13
#define SERVO_CTRL 11
#define VOLT_PIN 26
#define SER1_TX 0
#define SER1_RX 1
#define TACH_SDA 14
#define TACH_SCL 15

#define SEALEVELPRESSURE_PA (101325)

bool telemetry = true;
bool DEBUG_MODE = false;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
bool using_bno055 = true;


auto timer = timer_create_default();
Servo release_servo;
SFE_UBLOX_GNSS myGNSS;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno055 = Adafruit_BNO055(55, 0x28);

int packetCount = 1;
String packet = "";
String echo;
String buffer = "";
String command = "";
String mission_time = "";
bool sentApo = false;
String flightState = "LAUNCH_PAD";
String flightMode = "F";
bool sim_enable = false;
float sim_last_commanded_altitude;

float truemaximumaltitude = 0;
float calibrated_pressure;
int32_t latitude;
int32_t longitude;
int32_t gps_altitude;
uint16_t rpm;
byte SIV;


bool launch_pad = true;
bool ascent = false;
bool apogee = false;
bool descent = false;
bool probe_release = false;


float maxaltitude = 0;
float currentaltitude = 0;
float previousaltitude = 0;
float ppreviousaltitude = 0;
float pppreviousaltitude = 0;
float ppppreviousaltitude = 0;
float pppppreviousaltitude = 0;
float maximumaltitude = 0;
float bmaltitude = 0;

sensors_event_t linearAccelData, magnetometerData, accelerometerData;

bool send_api(void *) {
  uint8_t payloadLength = packet.length();
  uint8_t frame[20 + payloadLength];
  uint8_t frameLength = 14 + payloadLength;
  uint8_t checksum = 0;

  if (sentApo == false && (flightState == "DESCENT" || flightState == "APOGEE")) {
    sentApo = true;
  }

  frame[0] = 0x7E;                       //start delimiter
  frame[1] = (frameLength >> 8) & 0xFF;  //length
  frame[2] = frameLength & 0xFF;         //length
  frame[3] = 0x10;                       //frame type
  frame[4] = 0x01;                       //frame id

  frame[5] = 0x00;  //64-bit destination address
  frame[6] = 0x13;
  frame[7] = 0xA2;
  frame[8] = 0x00;
  frame[9] = 0x42;
  frame[10] = 0x5B;
  frame[11] = 0xD6;
  frame[12] = 0x18;

  frame[13] = 0xFF;  //16-bit destination address
  frame[14] = 0xFE;

  frame[15] = 0x00;  //broadcast radius
  frame[16] = 0x00;  //options

  // Copy the message into the payload
  for (uint8_t i = 0; i < payloadLength; i++) {
    frame[17 + i] = packet[i];
    checksum += packet[i];
  }

  checksum += 0x10 + 0x01;
  checksum += 0x00 + 0x13 + 0xA2 + 0x00 + 0x42 + 0x5B + 0xD6 + 0x18;
  checksum += 0xFF + 0xFE + 0x00 + 0x00;
  checksum = 0xFF - checksum;

  frame[17 + payloadLength] = checksum;

  Serial1.write(frame, 18 + payloadLength);

  Serial.print("XBee API frame sent: ");
  Serial.println(packet);

  return true;
}

bool send_transparent(void *) {
  if (telemetry) {
    Serial1.print("[" + packet + "]");
    Serial.print("sent packet: ");
    Serial.println(packet);
    packetCount++;
  }
  return true;
}

String cur_time() {
  unsigned long runMillis = millis();
  unsigned long allSeconds = millis() / 1000;
  int h = allSeconds / 3600;
  int secsRemaining = allSeconds % 3600;
  int m = secsRemaining / 60;
  int s = secsRemaining % 60;
  int ms = runMillis % 100;
  char time[5];
  sprintf(time, "%02d:%02d", m, s);
  return String(time);
}

void setup() {

  analogReadResolution(12);
  Serial.begin(9600);
  Serial1.setRX(SER1_RX);
  Serial1.setTX(SER1_TX);
  Serial1.begin(9600);
  while (DEBUG_MODE && !Serial);
  Wire.setTimeout(25, true); // fixes i2c communication problems between bno & bmp
  Serial.println("Serial Started");
  delay(100);

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire1.setSDA(TACH_SDA);
  Wire1.setSCL(TACH_SCL);
  Wire1.begin();
  Serial.println("wire started");
  delay(100);

  // checking BMP
  if (!bmp.begin_I2C(0x76)) {
    if (!bmp.begin_I2C(0x77)) {
      Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
      while (DEBUG_MODE)
        ;
    }
  }

  if (using_bno055 && !bno055.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  bno055.setExtCrystalUse(true);

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(100);

  // read bmp 10 times
  for (int i = 0; i < 10; i++) {
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading on bmp :(");
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
  if (myGNSS.begin(Wire, gnssAddress) == false) {
    Serial.println(F("u-blox GNSS not detected"));
    while (DEBUG_MODE);
  }

  myGNSS.setNavigationFrequency(4); 
  myGNSS.setAutoPVT(true);

  release_servo.attach(SERVO_CTRL, 400, 2600);
  release_servo.write(80);

  timer.every(985, send_transparent);
}

void loop() {
  timer.tick();

  if (Wire.getTimeoutFlag()) {
    Serial.println("Wire Error Flagged");
    Wire.clearTimeoutFlag();
  }

  while (Serial1.available()) {
    buffer += char(Serial1.read());
  }

  if (buffer.indexOf("{") > -1 && buffer.indexOf("}") > -1) {
    command = buffer.substring(buffer.indexOf("{") + 1, buffer.indexOf("}"));
    Serial.print("cmd received: ");
    Serial.println(command);
    buffer = buffer.substring(buffer.indexOf("}") + 1);

    echo = command;
    echo.replace(",", "");

    if (command.equals("CMD,3194,CAL")) {
    } else if (command.equals("CMD,3194,MEC,RELEASE,OFF")) {
      release_servo.write(100);
    } else if (command.equals("CMD,3194,MEC,RELEASE,ON")) {
      release_servo.write(80);
    } else if (command.equals("CMD,3194,CX,ON")) {
      telemetry = true;
    } else if (command.equals("CMD,3194,CX,OFF")) {
      telemetry = false;
    } else if (command.equals("CMD,3194,ST,GPS")) {
      mission_time = String(myGNSS.getHour()) + ":" + String(myGNSS.getMinute()) + ":" + String(myGNSS.getSecond());
    } else if (command.substring(0, 12).equals("CMD,3194,ST,")) {
      mission_time = command.substring(12);
    } else if (command.equals("CMD,3194,SIM,ENABLE")) {
      sim_enable = true;
    } else if (command.equals("CMD,3194,SIM,ACTIVATE")) {
      Serial.println("asdfasf");
      if (sim_enable) {
        flightMode = "S";
      }
      sim_enable = false;
    } else if (command.equals("CMD,3194,SIM,DISABLE")) {
      flightMode = "F";
      sim_enable = false;

      bmaltitude = 0;
      maximumaltitude = 0;
      pppppreviousaltitude = 0;
      ppppreviousaltitude = 0;
      pppreviousaltitude = 0;
      ppreviousaltitude = 0;
      previousaltitude = 0;
    } else if (command.substring(0, 13).equals("CMD,3194,SIMP") && flightMode.equals("S")) {
      sim_last_commanded_altitude = 44330.0 * (1.0 - pow((command.substring(command.lastIndexOf(",") + 1).toFloat()) / SEALEVELPRESSURE_PA, 0.1903));
      Serial.print("SIMP: ");
      Serial.println(sim_last_commanded_altitude);

      bmaltitude = 0;
      maximumaltitude = 0;
      pppppreviousaltitude = 0;
      ppppreviousaltitude = 0;
      pppreviousaltitude = 0;
      ppreviousaltitude = 0;
      previousaltitude = 0;
    }
  }
  // if(using_bno055){
  sensors_event_t event;
  bno055.getEvent(&event);

  sensors_event_t linearAccelData, magnetometerData, accelerometerData;
  bno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading on BMP");
  }

  if (myGNSS.getPVT() == true) {
    latitude = myGNSS.getLatitude();
    longitude = myGNSS.getLongitude();
    gps_altitude = myGNSS.getAltitudeMSL();  // Altitude above Mean Sea Level
    SIV = myGNSS.getSIV();
  }

  if (flightMode.equals("F")) {
    currentaltitude = bmp.readAltitude(calibrated_pressure);
  } else if (flightMode.equals("S")) {
    currentaltitude = sim_last_commanded_altitude;
  }

  flightState = statemachine(currentaltitude, previousaltitude, ppreviousaltitude, pppreviousaltitude, ppppreviousaltitude, pppppreviousaltitude, maximumaltitude, bmaltitude);
  bmaltitude = maximumaltitude;
  maximumaltitude = pppppreviousaltitude;
  pppppreviousaltitude = ppppreviousaltitude;
  ppppreviousaltitude = pppreviousaltitude;
  pppreviousaltitude = ppreviousaltitude;
  ppreviousaltitude = previousaltitude;
  previousaltitude = currentaltitude;


  Wire1.requestFrom(TACHOMETER_ADRESS, 2);
  if (Wire1.available() == 2) {                // If 2 bytes are available
    byte lowByte = Wire1.read();               // Read low byte
    byte highByte = Wire1.read();              // Read high byte
    uint16_t rpm = (highByte << 8) | lowByte;  // Combine the two bytes into a 16-bit integer
  } 

  if (telemetry) {
    packet =
      ("3194,"                      //TEAM_ID
       + mission_time + ","         //MISSION TIME
       + String(packetCount) + ","  //PACKET COUNT
       + flightMode + ","           //MODE
      ); 

    if (sentApo == false && (flightState == "DESCENT" || flightState == "APOGEE")) {
      packet += "APOGEE";
      packet += ",";
    } else {
      packet += flightState;
      packet += ",";
    }

    packet += ( 
      String(currentaltitude) + ","                                                                 //ALTITUDE
      + String(float(bmp.readTemperature()), 1) + ","                                               //TEMPERATURE
      + String(float(bmp.readPressure() / 1000), 1) + ","                                           //PRESSURE
      + String(float((analogRead(VOLT_PIN)) * (3.3 / 4095.0) / (680.0 / 1500.0)) + 1.8, 1) + ","    //VOLTAGE
    );  
    
      if (using_bno055) {
        packet += (String(float(linearAccelData.acceleration.x)) + ","   //GYRO ROLL
               + String(float(linearAccelData.acceleration.y)) + ","     //GYRO PITCH
               + String(float(linearAccelData.acceleration.z)) + ","     //GYRO YAW
               + String(float(accelerometerData.acceleration.x)) + ","   //ACCELERATION ROLL
               + String(float(accelerometerData.acceleration.y)) + ","   //ACCELERATION PITCH
               + String(float(accelerometerData.acceleration.z)) + ","   //ACCELERATION YAW
               + String(float(magnetometerData.magnetic.x / 100)) + ","  //MAGNETISM ROLL
               + String(float(magnetometerData.magnetic.y / 100)) + ","  //MAGNETISM PITCH
               + String(float(magnetometerData.magnetic.z / 100)) + ","  //MAGNETISM YAW
              );
    }
    packet += (String(rpm) + ","
             + String(myGNSS.getHour()) + ":" + String(myGNSS.getMinute()) + ":" + String(myGNSS.getSecond()) + "," //GPS TIME
             + String(float(gps_altitude / 1000.0), 1) + ","                                                        //GPS ALTITUDE
             + String(float(latitude / 10000000.0), 4) + ","                                                        //GPS LATITUDE
             + String(float(longitude / 10000000.0), 4) + ","                                                       //LONGITUDE
             + String(SIV) + ","                                                                                    //SATELLITES IN VIEW
             + String(echo) + ","                                                                                   //ECHO (LAST COMMAND RECEIVED)
             + "ON," 
             + cur_time());


  } else {
    packet = "";
  }
}
