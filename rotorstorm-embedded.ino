alright this marks the begninning of libraires i might use

BMP:
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
end of BMP with note that sealevelpressure is not right unit and use I2C not SPI NO SPIb

INA:
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_INA260.h"

void setup() {
  // put your setup code here, to run once:
  //test

}

void loop() {
  // put your main code here, to run repeatedly:
 test
}
