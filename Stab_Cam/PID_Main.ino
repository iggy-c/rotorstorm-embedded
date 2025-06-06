#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>
#include <Servo.h>
#include <math.h>

Adafruit_ICM20948 icm;
Servo cameraServo;

const int servoPin = 5; // change as needed
const int stopPWM = 90;
const int maxPWM = 130;
const int minPWM = 50;

// PID variables
double setpoint = 90;
double input = 0;
double output = 0;
double lastError = 0;
double integral = 0;
unsigned long lastTime;

double Kp = 3.5; // adjust
double Ki = 0.0;
double Kd = 0.5;

// Calibration variables
#define calibration_sample 300
float magX[calibration_sample], magY[calibration_sample], magZ[calibration_sample];
float offset[3] = {0, 0, 0};
float scale[3] = {1, 1, 1};

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!icm.begin_I2C()) {
  Serial.println("Failed to connect to ICM20948");
  while (1) delay(10);
}


  Serial.println("ICM found");

  cameraServo.attach(servoPin);
  cameraServo.write(stopPWM);

  performCalibration();
  lastTime = millis();
}

void loop() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);

  float mx = (mag.magnetic.x - offset[0]) * scale[0];
  float my = (mag.magnetic.y - offset[1]) * scale[1];

  float heading = atan2(my, mx) * 180.0 / PI;
  if (heading < 0) heading += 360;

  double error = setpoint - heading;
  if (error < -180) error += 360;
  if (error > 180) error -= 360;

  input = -error;
  output = computePID(input);

  int pwmSignal = stopPWM + output;
  pwmSignal = constrain(pwmSignal, minPWM, maxPWM);
  if (abs(error) < 5) pwmSignal = stopPWM;

  cameraServo.write(pwmSignal);

  delay(50);
}

double computePID(double error) {
  unsigned long now = millis();
  double deltaTime = (now - lastTime) / 1000.0;
  lastTime = now;

  integral += error * deltaTime;
  double derivative = (error - lastError) / deltaTime;
  lastError = error;

  return Kp * error + Ki * integral + Kd * derivative;
}

void performCalibration() {
  Serial.println("Starting magnetometer calibration...");
  int sampleIndex = 0;

  for (int i = 0; i < 5; i++) {
    for (int angle = minPWM; angle <= maxPWM && sampleIndex < calibration_sample; angle += 3) {
      cameraServo.write(angle);
      delay(100);
      readMagnetometer(sampleIndex++);
    }
    for (int angle = maxPWM; angle >= minPWM && sampleIndex < calibration_sample; angle -= 3) {
      cameraServo.write(angle);
      delay(100);
      readMagnetometer(sampleIndex++);
    }
  }

  cameraServo.write(stopPWM);
  computeHardSoftIron(sampleIndex);
  Serial.println("Calibration complete.");
}

void readMagnetometer(int index) {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);
  magX[index] = mag.magnetic.x;
  magY[index] = mag.magnetic.y;
  magZ[index] = mag.magnetic.z;
}

void computeHardSoftIron(int count) {
  float minX = magX[0], maxX = magX[0];
  float minY = magY[0], maxY = magY[0];
  float minZ = magZ[0], maxZ = magZ[0];

  for (int i = 1; i < count; i++) {
    if (magX[i] < minX) minX = magX[i];
    if (magX[i] > maxX) maxX = magX[i];
    if (magY[i] < minY) minY = magY[i];
    if (magY[i] > maxY) maxY = magY[i];
    if (magZ[i] < minZ) minZ = magZ[i];
    if (magZ[i] > maxZ) maxZ = magZ[i];
  }

  offset[0] = (maxX + minX) / 2.0;
  offset[1] = (maxY + minY) / 2.0;
  offset[2] = (maxZ + minZ) / 2.0;

  float deltaX = (maxX - minX) / 2.0;
  float deltaY = (maxY - minY) / 2.0;
  float deltaZ = (maxZ - minZ) / 2.0;
  float avgDelta = (deltaX + deltaY + deltaZ) / 3.0;

  scale[0] = avgDelta / deltaX;
  scale[1] = avgDelta / deltaY;
  scale[2] = avgDelta / deltaZ;

  Serial.print("Offsets: ");
  Serial.print(offset[0]); Serial.print(", ");
  Serial.print(offset[1]); Serial.print(", ");
  Serial.println(offset[2]);

  Serial.print("Scales: ");
  Serial.print(scale[0]); Serial.print(", ");
  Serial.print(scale[1]); Serial.print(", ");
  Serial.println(scale[2]);
}
