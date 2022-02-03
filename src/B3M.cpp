/*
    B3M4Arduino
    Copyright (c) 2022 aomushi020
    https://github.com/aomushi020/B3M4Arduino
    This library is released under the MIT license.
*/
#include "Arduino.h"
#include "B3M.h"

B3M::B3M(HardwareSerial *serialPointer_, uint8_t enPin_) {
    b3mSerial_ = serialPointer_;
    b3mEnPin_ = enPin_;
    b3mTxPin_ = 0;
    b3mRxPin_ = 0;
    b3mBaudrate_ = 0;
    b3mTimeout_ = 0;
}
B3M::B3M(HardwareSerial *serialPointer_, uint8_t enPin_, uint32_t baudrate_, uint32_t timeout_) {
    b3mSerial_ = serialPointer_;
    b3mEnPin_ = enPin_;
    b3mTxPin_ = 0;
    b3mRxPin_ = 0;
    b3mBaudrate_ = baudrate_;
    b3mTimeout_ = timeout_;
}
B3M::B3M(HardwareSerial *serialPointer_, uint8_t enPin_, uint8_t rxPin_, uint8_t txPin_, uint32_t baudrate_, uint32_t timeout_) {
    b3mSerial_ = serialPointer_;
    b3mEnPin_ = enPin_;
    b3mTxPin_ = txPin_;
    b3mRxPin_ = rxPin_;
    b3mBaudrate_ = baudrate_;
    b3mTimeout_ = timeout_;
}

void B3M::begin(void) {
    if (b3mTxPin_ || b3mRxPin_) {
        b3mSerial_->begin(b3mBaudrate_, SERIAL_8N1, b3mRxPin_, b3mTxPin_);
        b3mSerial_->setTimeout(b3mTimeout_);
    } else if (b3mBaudrate_ || b3mTimeout_) {
        b3mSerial_->begin(b3mBaudrate_, SERIAL_8N1);
        b3mSerial_->setTimeout(b3mTimeout_);
    } else {
        b3mSerial_->begin(B3M_DEFAULT_BAUDRATE, SERIAL_8N1);
    }
    pinMode(b3mEnPin_, OUTPUT);
    digitalWrite(b3mEnPin_, LOW);
}

// uint8_t B3M::load(uint8_t id_, uint8_t option_){
//     return 0;
// }

// void B3M::load(uint8_t *id_, uint8_t option_, uint8_t length_){

// }
// uint8_t B3M::save(uint8_t id_, uint8_t option_){
//     return 0;
// }
// void B3M::save(uint8_t *id_, uint8_t option_, uint8_t length_){

// }
// uint8_t B3M::read(uint8_t id_, uint8_t option_, uint8_t address_, uint8_t length_){
//     return 0;
// }

uint8_t B3M::write(uint8_t id_, uint8_t option_, uint8_t *data_, uint8_t bytes_, uint8_t address_) {
    uint8_t b3mFormat[7 + bytes_], b3m_i;
    b3mFormat[0] = 7 + bytes_;
    b3mFormat[1] = B3M_WRITE;
    b3mFormat[2] = option_;
    b3mFormat[3] = id_;
    for (b3m_i = 4; b3m_i < (4 + bytes_); b3m_i++) {
        b3mFormat[b3m_i] = *data_;
        data_++;
    }
    b3mFormat[b3m_i] = address_;
    b3mFormat[b3m_i + 1] = bytes_;
    b3mFormat[b3m_i + 2] = b3mCheckSum_(b3mFormat, 6 + bytes_);
    b3mSend_(b3mFormat, 7 + bytes_);
    return b3mFormat[b3m_i + 2];
}
// void B3M::write(uint8_t *id_, uint8_t option_, uint8_t *data_, uint8_t bytes_, uint8_t address_, uint8_t length_){

// }

void B3M::reset(void){
    reset(0xFF);
}
void B3M::reset(uint8_t id_){
    reset(id_, 0x80, 0x00);
}
void B3M::reset(uint8_t id_, uint8_t option_, uint8_t time_) {
    uint8_t b3mFormat[6];
    b3mFormat[0] = 0x06;
    b3mFormat[1] = B3M_RESET;
    b3mFormat[2] = option_;
    b3mFormat[3] = id_;
    b3mFormat[4] = time_;
    b3mFormat[5] = b3mCheckSum_(b3mFormat, 0x05);
    b3mSend_(b3mFormat, 0x06);
}
void B3M::reset(uint8_t *id_, uint8_t option_, uint8_t time_, uint8_t length_) {
    uint8_t b3mFormat[length_ + 5], b3m_i;
    b3mFormat[0] = length_ + 5;
    b3mFormat[1] = B3M_RESET;
    b3mFormat[2] = option_;
    for (b3m_i = 3; b3m_i < (length_ + 3); b3m_i++) {
        b3mFormat[b3m_i] = *id_;
        id_++;
    }
    b3mFormat[b3m_i] = time_;
    b3mFormat[b3m_i + 1] = b3mCheckSum_(b3mFormat, b3m_i + 2);
    b3mSend_(b3mFormat, b3m_i + 3);
}

uint8_t B3M::position(uint8_t id_, uint16_t position_){
    return position(id_, 0x80, position_, 0x00);
}

uint8_t B3M::position(uint8_t id_, uint8_t option_, uint16_t position_, uint16_t time_) {
    uint8_t b3mFormat[9];
    b3mFormat[0] = 0x09;
    b3mFormat[1] = B3M_POSITION;
    b3mFormat[2] = option_;
    b3mFormat[3] = id_;
    b3mFormat[4] = lowByte(position_);
    b3mFormat[5] = highByte(position_);
    b3mFormat[6] = lowByte(time_);
    b3mFormat[7] = highByte(time_);
    b3mFormat[8] = b3mCheckSum_(b3mFormat, 0x08);
    b3mSend_(b3mFormat, 0x09);
    return b3mFormat[8];
}
void B3M::position(uint8_t *id_, uint16_t *position_, uint8_t length_){
    position(id_, 0x80, position_, 0x00, length_);
}
void B3M::position(uint8_t *id_, uint8_t option_, uint16_t *position_, uint16_t time_, uint8_t length_) {
    uint8_t b3mFormat[(length_ * 3) + 6], b3m_i;
    b3mFormat[0] = (length_ * 3) + 6;
    b3mFormat[1] = B3M_POSITION;
    b3mFormat[2] = option_;
    for (b3m_i = 3; b3m_i < (length_ * 3 + 3); b3m_i += 3) {
        b3mFormat[b3m_i] = *id_;
        b3mFormat[b3m_i + 1] = lowByte(*position_);
        b3mFormat[b3m_i + 2] = highByte(*position_);
        id_++;
        position_++;
    }
    b3mFormat[b3m_i + 1] = lowByte(time_);
    b3mFormat[b3m_i + 2] = highByte(time_);
    b3mFormat[b3m_i + 3] = b3mCheckSum_(b3mFormat, b3m_i + 4);
    b3mSend_(b3mFormat, b3m_i + 5);
}

// protected members
HardwareSerial *b3mSerial_;
uint8_t b3mEnPin_, b3mTxPin_, b3mRxPin_;
uint32_t b3mBaudrate_, b3mTimeout_;

uint8_t B3M::b3mCheckSum_(uint8_t *send_formats_, uint8_t bytes_) {
    uint8_t checkSum = 0x00, b3m_i = 0;
    for (b3m_i = 0; b3m_i < bytes_; b3m_i++) {
        checkSum += *send_formats_;
        send_formats_++;
    }
    return checkSum;
}

void B3M::b3mSend_(uint8_t *send_formats_, uint8_t bytes_) {
    uint8_t b3m_i;
    digitalWrite(b3mEnPin_, HIGH);
    delay(10);
    // for (b3m_i = 0; b3m_i < bytes_; b3m_i++) {
    b3mSerial_->write(send_formats_, bytes_);
        // send_formats_++;
        // delayMicroseconds(220);
    // }
    delay(10);
    digitalWrite(b3mEnPin_, LOW);
}

uint8_t B3M::b3mRead_(uint8_t bytes_) {
    return 0;
}