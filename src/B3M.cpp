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

uint8_t B3M::load(uint8_t id_) {
    return load(id_, B3M_GET_ERROR);
}
uint8_t B3M::load(uint8_t id_, uint8_t option_) {
    uint8_t b3mFormat_[5];
    b3mFormat_[0] = 0x05;
    b3mFormat_[1] = B3M_LOAD;
    b3mFormat_[2] = option_;
    b3mFormat_[3] = id_;
    b3mFormat_[4] = b3mCheckSum_(b3mFormat_, 4);
    b3mSend_(b3mFormat_, 5);
    return b3mRead_(readBuffer);
}
void B3M::load(uint8_t *id_, uint8_t option_, uint8_t length_) {
    uint8_t b3mFormat_[4 + length_], b3m_i_;
    b3mFormat_[0] = 0x05;
    b3mFormat_[1] = B3M_LOAD;
    b3mFormat_[2] = option_;
    for (b3m_i_ = 3; b3m_i_ < 3 + length_; b3m_i_++) {
        b3mFormat_[b3m_i_] = *id_;
        id_++;
    }
    b3mFormat_[b3m_i_] = b3mCheckSum_(b3mFormat_, 3 + length_);
    b3mSend_(b3mFormat_, 4 + length_);
}

uint8_t B3M::save(uint8_t id_){
    return save(id_, B3M_GET_ERROR);
}
uint8_t B3M::save(uint8_t id_, uint8_t option_){
    uint8_t b3mFormat_[5];
    b3mFormat_[0] = 0x05;
    b3mFormat_[1] = B3M_SAVE;
    b3mFormat_[2] = option_;
    b3mFormat_[3] = id_;
    b3mFormat_[4] = b3mCheckSum_(b3mFormat_, 4);
    b3mSend_(b3mFormat_, 5);
    return b3mRead_(readBuffer);
}
void B3M::save(uint8_t *id_, uint8_t option_, uint8_t length_){
    uint8_t b3mFormat_[4 + length_], b3m_i_;
    b3mFormat_[0] = 0x05;
    b3mFormat_[1] = B3M_SAVE;
    b3mFormat_[2] = option_;
    for (b3m_i_ = 3; b3m_i_ < 3 + length_; b3m_i_++) {
        b3mFormat_[b3m_i_] = *id_;
        id_++;
    }
    b3mFormat_[b3m_i_] = b3mCheckSum_(b3mFormat_, 3 + length_);
    b3mSend_(b3mFormat_, 4 + length_);
}

uint8_t B3M::read(uint8_t id_, uint8_t option_, uint8_t address_, uint8_t length_){
    uint8_t b3mFormat_[7];
    b3mFormat_[0] = 0x07;
    b3mFormat_[1] = B3M_READ;
    b3mFormat_[2] = option_;
    b3mFormat_[3] = id_;
    b3mFormat_[4] = address_;
    b3mFormat_[5] = length_;
    b3mFormat_[6] = b3mCheckSum_(b3mFormat_, 4);
    b3mSend_(b3mFormat_, 7);
    return b3mRead_(readBuffer);
}

uint8_t B3M::write(uint8_t id_, uint8_t *data_, uint8_t bytes_, uint8_t address_) {
    return write(id_, B3M_GET_ERROR, data_, bytes_, address_);
}
uint8_t B3M::write(uint8_t id_, uint8_t option_, uint8_t *data_, uint8_t bytes_, uint8_t address_) {
    uint8_t b3mFormat_[7 + bytes_], b3m_i_;
    b3mFormat_[0] = 7 + bytes_;
    b3mFormat_[1] = B3M_WRITE;
    b3mFormat_[2] = option_;
    b3mFormat_[3] = id_;
    for (b3m_i_ = 4; b3m_i_ < (4 + bytes_); b3m_i_ += bytes_) {
        b3mFormat_[b3m_i_] = *data_;
        data_++;
    }
    b3mFormat_[b3m_i_] = address_;
    b3mFormat_[b3m_i_ + 1] = bytes_;
    b3mFormat_[b3m_i_ + 2] = b3mCheckSum_(b3mFormat_, 6 + bytes_);
    b3mSend_(b3mFormat_, 7 + bytes_);
    return b3mRead_(readBuffer);
}
// void B3M::write(uint8_t *id_, uint8_t option_, uint8_t *data_, uint8_t bytes_, uint8_t address_, uint8_t length_){

// }

void B3M::reset(void) {
    reset(B3M_BROADCAST_ADDR);
}
void B3M::reset(uint8_t id_) {
    reset(id_, B3M_GET_ERROR, 0x00);
}
void B3M::reset(uint8_t id_, uint8_t option_, uint8_t time_) {
    uint8_t b3mFormat_[6];
    b3mFormat_[0] = 0x06;
    b3mFormat_[1] = B3M_RESET;
    b3mFormat_[2] = option_;
    b3mFormat_[3] = id_;
    b3mFormat_[4] = time_;
    b3mFormat_[5] = b3mCheckSum_(b3mFormat_, 0x05);
    b3mSend_(b3mFormat_, 0x06);
}
void B3M::reset(uint8_t *id_, uint8_t length_) {
    reset(id_, B3M_GET_ERROR, 0x00, length_);
}
void B3M::reset(uint8_t *id_, uint8_t option_, uint8_t time_, uint8_t length_) {
    uint8_t b3mFormat_[length_ + 5], b3m_i_;
    b3mFormat_[0] = length_ + 5;
    b3mFormat_[1] = B3M_RESET;
    b3mFormat_[2] = option_;
    for (b3m_i_ = 3; b3m_i_ < (length_ + 3); b3m_i_++) {
        b3mFormat_[b3m_i_] = *id_;
        id_++;
    }
    b3mFormat_[b3m_i_] = time_;
    b3mFormat_[b3m_i_ + 1] = b3mCheckSum_(b3mFormat_, b3m_i_ + 2);
    b3mSend_(b3mFormat_, b3m_i_ + 3);
}

uint8_t B3M::position(uint8_t id_, int16_t position_) {
    return position(id_, B3M_GET_ERROR, position_, 0x00);
}
uint8_t B3M::position(uint8_t id_, uint8_t option_, int16_t position_, uint16_t time_) {
    uint8_t b3mFormat_[9];
    b3mFormat_[0] = 0x09;
    b3mFormat_[1] = B3M_POSITION;
    b3mFormat_[2] = option_;
    b3mFormat_[3] = id_;
    position_ = constrain(position_, B3M_MIN_POSITION, B3M_MAX_POSITION);
    b3mFormat_[4] = lowByte(position_);
    b3mFormat_[5] = highByte(position_);
    b3mFormat_[6] = lowByte(time_);
    b3mFormat_[7] = highByte(time_);
    b3mFormat_[8] = b3mCheckSum_(b3mFormat_, 0x08);
    b3mSend_(b3mFormat_, 0x09);
    return b3mRead_(readBuffer);
}
void B3M::position(uint8_t *id_, int16_t *position_, uint8_t length_) {
    position(id_, B3M_GET_ERROR, position_, 0x00, length_);
}
void B3M::position(uint8_t *id_, uint8_t option_, int16_t *position_, uint16_t time_, uint8_t length_) {
    uint8_t b3mFormat_[(length_ * 3) + 6], b3m_i_;
    b3mFormat_[0] = (length_ * 3) + 6;
    b3mFormat_[1] = B3M_POSITION;
    b3mFormat_[2] = option_;
    for (b3m_i_ = 3; b3m_i_ < (length_ * 3 + 3); b3m_i_ += 3) {
        b3mFormat_[b3m_i_] = *id_;
        *position_ = constrain(*position_, B3M_MIN_POSITION, B3M_MAX_POSITION);
        b3mFormat_[b3m_i_ + 1] = lowByte(*position_);
        b3mFormat_[b3m_i_ + 2] = highByte(*position_);
        id_++;
        position_++;
    }
    b3mFormat_[b3m_i_ + 1] = lowByte(time_);
    b3mFormat_[b3m_i_ + 2] = highByte(time_);
    b3mFormat_[b3m_i_ + 3] = b3mCheckSum_(b3mFormat_, b3m_i_ + 4);
    b3mSend_(b3mFormat_, b3m_i_ + 5);
}

int16_t B3M::deg2Pos(float deg_) {
    return constrain(int(deg_ * 100), B3M_MIN_POSITION, B3M_MAX_POSITION);
}
uint8_t B3M::deg2Pos(float *deg_, uint8_t length_) {
    uint8_t b3m_i_;
    for (b3m_i_ = 0; b3m_i_ < length_; b3m_i_++) {
        *deg_ = deg2Pos(*deg_);
        deg_++;
    }
    return length_;
}

float B3M::pos2Deg(int16_t position_) {
    return (position_ / 100.0);
}

// protected members
HardwareSerial *b3mSerial_;
uint8_t b3mEnPin_, b3mTxPin_, b3mRxPin_;
uint32_t b3mBaudrate_, b3mTimeout_;

uint8_t B3M::b3mCheckSum_(uint8_t *send_formats_, uint8_t bytes_) {
    uint8_t checkSum_ = 0x00, b3m_i_ = 0;
    for (b3m_i_ = 0; b3m_i_ < bytes_; b3m_i_++) {
        checkSum_ += *send_formats_;
        send_formats_++;
    }
    return checkSum_;
}

void B3M::b3mSend_(uint8_t *send_formats_, uint8_t bytes_) {
    uint8_t b3m_i_;
    digitalWrite(b3mEnPin_, HIGH);
    delay(10);
    b3mSerial_->write(send_formats_, bytes_);
    delay(10);
    digitalWrite(b3mEnPin_, LOW);
}

uint8_t B3M::b3mRead_(uint8_t *returnBuffer_) {
    uint8_t *b3m_length_ = returnBuffer_;
    while (b3mSerial_->available()) {
        *returnBuffer_ = b3mSerial_->read();
        returnBuffer_++;
    }
    return *b3m_length_;
}