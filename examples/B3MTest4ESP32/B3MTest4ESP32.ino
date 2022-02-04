/*
    B3M4Arduino
    Copyright (c) 2022 aomushi020
    https://github.com/aomushi020/B3M4Arduino
    This library is released under the MIT license.
*/

#include <B3M.h>
HardwareSerial RS485(2);
B3M B3M(&RS485, 16, 17, 18, 115200, 1000);

void setup(void){
    Serial.begin(115200);
    B3M.begin();
    B3M.reset();
    uint8_t sendData[2] = {0x02, 0x00};
    // Free
    B3M.write(B3M_BROADCAST_ADDR, sendData, 0x01, B3M_SERVO_MODE_SET);
    delay(1000);
    sendData[0] = 0x00;
    // Set Even
    B3M.write(B3M_BROADCAST_ADDR, sendData, 0x01, B3M_SERVO_CONTROL_TYPE);
    delay(1000);
    // Start control
    B3M.write(B3M_BROADCAST_ADDR, sendData, 0x01, B3M_SERVO_MODE_SET);
    delay(1000);
    // set position 0
    B3M.position(B3M_BROADCAST_ADDR, 0x0000);

}

void loop(void){

}