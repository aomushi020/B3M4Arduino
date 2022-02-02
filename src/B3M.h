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

// B3M Registor Addresses
// SYSTEM
#define B3M_SYS_ID                          0x00
#define B3M_SYS_BAUDRATE                    0x01
#define B3M_SYS_MIN_POSITION_LIMIT          0x05
#define B3M_SYS_MAX_POSITION_LIMIT          0x07
#define B3M_SYS_CENTER_OFFSET               0x09
#define B3M_SYS_MCU_TEMP_OFFSET             0x0B
#define B3M_SYS_PWM_LIMIT_RATE_AT_MCU       0x0D
#define B3M_SYS_MOTOR_TEMP_LIMIT            0x0E
#define B3M_SYS_PWM_LIMIT_RATE_AT_MOTOR     0x10
#define B3M_SYS_CURRENT_LIMIT               0x11
#define B3M_SYS_PWM_LIMIT_RATE_AT_CURRENT   0x13
#define B3M_SYS_MOTORLOCK_DELAY             0x14
#define B3M_SYS_MOTORLOCK_RATE              0x15
#define B3M_SYS_LOCKED_LOSS_RATE            0x16
#define B3M_SYS_MIN_INPUT_VOLTAGE           0x17
#define B3M_SYS_MAX_INPUT_VOLTAGE           0x19
#define B3M_SYS_PWM_LIMIT                   0x1B
#define B3M_SYS_DEADBAND_WIDTH              0x1C
// #define B3M_SYS_RESERVED                    0x1E
// #define B3M_SYS_RESERVED                    0x20
#define B3M_SYS_MOTOR_CW_ROTATE_RATE        0x22
#define B3M_SYS_MOTOR_CCW_ROTATE_RATE       0x23
// Servo
#define B3M_SRV_OPTION              0x27
#define B3M_SRV_MODE_CHANGE         0x28
#define B3M_SRV_CONTROL_TYPE        0x29
#define B3M_SRV_TARGET_POSITION     0x2A
#define B3M_SRV_CURRENT_POSITION    0x2C
#define B3M_SRV_LAST_POSITION       0x2E
#define B3M_SRV_TARGET_SPEED        0x30
#define B3M_SRV_CURRENT_SPEED       0x32
#define B3M_SRV_LAST_SPEED          0x34
#define B3M_SRV_TARGET_TIME         0x36
#define B3M_SRV_RUNNING_TIME        0x38
#define B3M_SRV_UPTIME              0x3A
#define B3M_SRV_TARGET_TORQUE       0x3C
#define B3M_SRV_SYSTEM_CLOCK        0x3E
#define B3M_SRV_SAMPLING_TIME       0x42
#define B3M_SRV_MCU_TEMP            0x44
#define B3M_SRV_MOTOR_TEMP          0x46
#define B3M_SRV_CURRENT             0x48
#define B3M_SRV_VOLTAGE             0x4A
#define B3M_SRV_PWM_DUTY_RATE       0x4C
#define B3M_SRV_PWM_CYCLE           0x4E
#define B3M_SRV_ENCORDER_CURRENT    0x50
#define B3M_SRV_ENCORDER_TOTAL      0x52
#define B3M_SRV_HALL_IC             0x56
// Control
#define B3M_CTR_PID_GAIN_NO                     0x5C
// #define B3M_CTR_RESERVED                        0x5D
#define B3M_CTR_P_GAIN_0                        0x5E
#define B3M_CTR_I_GAIN_0                        0x62
#define B3M_CTR_D_GAIN_0                        0x66
#define B3M_CTR_STATIC_FRICTION_COEFFICIENT_0   0x6A
#define B3M_CTR_DYNAMIC_FRICTION_COEFFICIENT_0  0x6C
#define B3M_CTR_P_GAIN_1                        0x6E
#define B3M_CTR_I_GAIN_1                        0x72
#define B3M_CTR_D_GAIN_1                        0x76
#define B3M_CTR_STATIC_FRICTION_COEFFICIENT_1   0x7A
#define B3M_CTR_DYNAMIC_FRICTION_COEFFICIENT_1  0x7C
#define B3M_CTR_P_GAIN_2                        0x7E
#define B3M_CTR_I_GAIN_2                        0x82
#define B3M_CTR_D_GAIN_2                        0x86
#define B3M_CTR_STATIC_FRICTION_COEFFICIENT_2   0x8A
#define B3M_CTR_DYNAMIC_FRICTION_COEFFICIENT_2  0x8C
// Status
#define B3M_STS_ERROR           0x9D
#define B3M_STS_SYSTEM_ERROR    0x9E
#define B3M_STS_MOTOR_ERROR     0x9F
#define B3M_STS_UART_ERROR      0xA0
#define B3M_STS_COMMAND_ERROR   0xA1
// Version
#define B3M_VER_VOLTAGE     0xA2
#define B3M_VER_MODEL       0xA3
#define B3M_VER_TORQUE      0xA4
#define B3M_VER_CASE        0xA5
#define B3M_VER_MOTOR       0xA8
#define B3M_VER_DEVICE      0xA9
#define B3M_VER_BUILD       0xAA
#define B3M_VER_REVISION    0xAB
#define B3M_VER_MINOR       0xAC
#define B3M_VER_MAJOR       0xAD

class B3M{
    public:
        B3M(HardwareSerial* b3mSerialPointer_, uint8_t enPin_);
        B3M(HardwareSerial* b3mSerialPointer_, uint8_t enPin_, uint32_t baudrate_, uint32_t timeout_);
        void begin(void);
        // Single Mode
        // uint8_t
        // Multi Mode

    protected:
        uint8_t singleSend_(uint8_t *send_format_);
        uint8_t multiSend_(uint8_t *send_formats_, uint8_t bytes_);
        HardwareSerial *b3mSerial_;
        uint8_t checkSum_(uint8_t *send_format_);
}

#endif