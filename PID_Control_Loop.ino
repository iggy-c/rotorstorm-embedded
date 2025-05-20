#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <EEPROM.h>  // EEPROM library for RP2040 flash logging

/* {current system uses custom PID loop based on principles from these papers:
https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
https://eng.libretexts.org/.../9.03%3A_PID_Tuning_via_Classical_Methods

Basis for PID loop:
Output = K_p * e(t) + K_i * ∫e(t)dt + K_d * de(t)/dt
Or alternatively:
u(t) = K_c * [e(t) + (1/τ_i) * ∫e(t')dt' + τ_d * de(t)/dt] + b
}
*/

// Struct for logging
struct PIDLog {
  unsigned long timestamp;
  float heading;
  float error;
  float output;
  int pwm;
};

const int EEPROM_SIZE = 2048;
const int LOG_CAPACITY = EEPROM_SIZE / sizeof(PIDLog);
int logIndex = 0;

Adafruit_ICM20948 icm;
Servo cameraServo;

// Servo parameters
const int servoPin = 5;
const int stopPWM = 90;
const int maxPWM = 130;
const int minPWM = 50;

// PID parameters
double setpoint = 0;
double input = 0;
double output = 0;
double lastError = 0;
double integral = 0;
unsigned long lastTime;

// PID tuning
double Kp = 3.5;
double Ki = 0.0;
double Kd = 0.5;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!icm.begin_I2C()) {
    Serial.println("Failed to connect to ICM");
    while (1) delay(10);
  }
  Serial.println("ICM found");

  cameraServo.attach(servoPin);
  cameraServo.write(stopPWM);

  lastTime = millis();

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);


  /*Flashlog reset uncomment to clear flash data
  for (int i = 0; i < EEPROM_SIZE; i++) {
   EEPROM.write(i, 0);
}
   EEPROM.commit();
  */

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

void loop() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);

  float heading = atan2(mag.magnetic.y, mag.magnetic.x) * 100 / PI;
  if (heading < 0) heading += 360;

  float error = heading - setpoint;

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  input = -error;
  output = computePID(input);

  int pwmSignal = stopPWM + output;
  pwmSignal = constrain(pwmSignal, minPWM, maxPWM);

  if (abs(error) < 5) pwmSignal = stopPWM;

  cameraServo.write(pwmSignal);

  // Store log entry in EEPROM
  if (logIndex < LOG_CAPACITY) {
    PIDLog log = {
      millis(),
      heading,
      error,
      (float)output,
      pwmSignal
    };
    EEPROM.put(logIndex * sizeof(PIDLog), log);
    EEPROM.commit();
    logIndex++;
  }

  delay(50);
}






