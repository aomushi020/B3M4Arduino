/*
    B3M4Arduino
    Copyright (c) 2022 aomushi020
    https://github.com/aomushi020/B3M4Arduino
    This library is released under the MIT license.
*/
#include<Arduino.h>
#include"B3M.h"

// public members
B3M::B3M(HardwareSerial* b3mSerialPointer_, uint8_t enPin_){
    b3mSerial_ = b3mSerialPointer_;
    b3mEnPin_ = enPin_;
    b3mTxPin_ = 0;
    b3mRxPin_ = 0;
    b3mBaudrate_ = 0;
    b3mTimeout_ = 0;
}

B3M::B3M(HardwareSerial* b3mSerialPointer_, uint8_t enPin_, uint32_t baudrate_, uint32_t timeout_){
    b3mSerial_ = b3mSerialPointer_;
    b3mEnPin_ = enPin_;
    b3mTxPin_ = 0;
    b3mRxPin_ = 0;
    b3mBaudrate_ = baudrate_;
    b3mTimeout_ = timeout_;
}

B3M::B3M(HardwareSerial* b3mSerialPointer_, uint8_t enPin_, uint8_t rxPin_, uint8_t txPin_, uint32_t baudrate_, uint32_t timeout_){
    b3mSerial_ = b3mSerialPointer_;
    b3mEnPin_ = enPin_;
    b3mTxPin_ = txPin_;
    b3mRxPin_ = rxPin_;
    b3mBaudrate_ = baudrate_;
    b3mTimeout_ = timeout_;
}

void B3M::begin(void){
    if(b3mTxPin_ && b3mRxPin_){
        b3mSerial_->begin(b3mBaudrate_, SERIAL_8N1, b3mRxPin_, b3mTxPin_);
        b3mSerial_->setTimeout(b3mTimeout_);
    }else if(b3mBaudrate_ && b3mTimeout_){
        b3mSerial_->begin(b3mBaudrate_, SERIAL_8N1);
        b3mSerial_->setTimeout(b3mTimeout_);
    }else{
        b3mSerial_->begin(B3M_DEFAULT_BAUDRATE, SERIAL_8N1);
    }
    pinMode(b3mEnPin_, OUTPUT);
}


// protected members
HardwareSerial *b3mSerial_;
uint8_t b3mEnPin_, b3mTxPin_, b3mRxPin_;
uint32_t b3mBaudrate_, b3mTimeout_;