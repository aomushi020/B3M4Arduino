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
    B3M.begin();
    Serial.begin(115200);
    B3M.reset(0xFF, 0x00, 0x00);
    uint8_t sendData[2] = {0x00, 0x00};
    Serial.println(B3M.write(0xFF, 0x00, sendData, 0x01, 0x28), DEC);
    delay(1000);
    uint8_t sendData2[2] = {0x00, 0x00};
    Serial.println(B3M.write(0xFF, 0x00, sendData2, 0x01, 0x29), DEC);
    delay(1000);
    Serial.println(B3M.position(0xFF, 0x00, 0x0000, 0x0000), DEC);

}

void loop(void){

}