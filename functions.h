void send_xbee(String message) {
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

void setReports(Adafruit_BNO08x bno08x) {
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