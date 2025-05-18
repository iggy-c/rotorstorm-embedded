bool send(void *msg_ptr) {
  String message = (*(String*)msg_ptr);
  uint8_t payloadLength = message.length();
  uint8_t frame[20 + payloadLength];
  uint8_t frameLength = 14 + payloadLength;
  uint8_t checksum = 0;

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

  return true;
}
