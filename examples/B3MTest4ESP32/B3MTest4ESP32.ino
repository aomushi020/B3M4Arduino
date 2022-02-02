/*
    B3M4Arduino
    Copyright (c) 2022 aomushi020
    https://github.com/aomushi020/B3M4Arduino
    This library is released under the MIT license.
*/
#include <B3M.h>

B3M B3M($Serial2, 16, 17, 18, 115200, 100);

void setup(void){
    B3M.begin();
    B3M.reset(0xFF, 0x80, 0x00);
    
}

void loop(void){

}