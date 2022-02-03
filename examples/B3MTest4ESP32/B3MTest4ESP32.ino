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
    uint8_t sendData[2] = {0x00, 0x00};
    B3M.write(0xFF, sendData, 0x01, 0x28);
    delay(1000);
    B3M.write(0xFF, sendData, 0x01, 0x29);
    delay(1000);
    B3M.position(0xFF, 0x0000);

}

void loop(void){

}