/*
    B3M4Arduino
    Copyright (c) 2022 aomushi020
    https://github.com/aomushi020/B3M4Arduino
    This library is released under the MIT license.
*/

#ifndef B3M_h
#define B3M_h

// B3M Commands
#define B3M_LOAD        0x01
#define B3M_SAVE        0x02
#define B3M_READ        0x03
#define B3M_WRITE       0x04
#define B3M_RESET       0x05
#define B3M_POSITION    0x06

// Option Commands
#define B3M_CLR_ERROR   0x00
#define B3M_CLR_SYSTEM  0x01
#define B3M_CLR_MOTOR   0x02
#define B3M_CLR_UART    0x03
#define B3M_CLR_COMMAND 0x04
#define B3M_GET_ERROR   0x80
#define B3M_GET_SYSTEM  0x81
#define B3M_GET_MOTOR   0x82
#define B3M_GET_UART    0x83
#define B3M_GET_COMMAND 0x84

// B3M Registor Map


class B3M{
    public:
        B3M(HardwareSerial* b3mSerialPointer_, uint8_t enPin_);
        B3M(HardwareSerial* b3mSerialPointer_, uint8_t enPin_, uint32_t baudrate_, uint32_t timeout_);
        void begin(void);
        // Single Mode
        uint8_t
        // Multi Mode

    protected:
        uint8_t singleSend_(uint8_t *send_format_);
        uint8_t multiSend_(uint8_t *send_formats_, uint8_t bytes_);
        HardwareSerial *b3mSerial_
        uint8_t checkSum_(uint8_t *send_format_);
}

#endif