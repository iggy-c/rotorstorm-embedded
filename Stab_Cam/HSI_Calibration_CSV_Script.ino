#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20948 icm;

#define ICM_ADDRESS 0x69
#define I2C_SDA 6 
#define I2C_SCL 7 

void setup() {
  Serial.begin(115200);
  while (!Serial);


  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  if (!icm.begin_I2C(ICM_ADDRESS, &Wire)) {
    Serial.println("Failed to connect to ICM20948");
    while (1) delay(10);
  }
  Serial.println("ICM20948 found");
}

void loop() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);

  Serial.print(mag.magnetic.x);
  Serial.print(", ");
  Serial.print(mag.magnetic.y);
  Serial.print(", ");
  Serial.println(mag.magnetic.z);

  delay(50);
}
