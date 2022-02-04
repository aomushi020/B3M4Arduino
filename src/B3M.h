/*
    B3M4Arduino
    Copyright (c) 2022 aomushi020
    https://github.com/aomushi020/B3M4Arduino
    This library is released under the MIT license.
*/

#ifndef B3M_h
#define B3M_h

// B3M Default Value
#define B3M_DEFAULT_BAUDRATE 115200
#define B3M_MIN_POSITION    -35000
#define B3M_MAX_POSITION    35000
#define B3M_BROADCAST_ADDR  0xFF

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

// Servo Parameter Value
// Servo Option 0x27
#define B3M_SERVO_OPTION_NORMAL     0x00
#define B3M_SERVO_OPTION_CLONE      0x40
#define B3M_SERVO_OPTION_REVERSE    0xC0
// Mode Select  0x28
// Move Mode
#define B3M_MOVE_MODE_NORMAL        0x00
#define B3M_MOVE_MODE_FREE          0x02
#define B3M_MOVE_MODE_HOLD          0x03
// Control Mode
#define B3M_CONTROL_MODE_POSITION   0x00
#define B3M_CONTROL_MODE_SPEED      0x04
#define B3M_CONTROL_MODE_TORQUE     0x08
#define B3M_CONTROL_MODE_FFC        0x0C
// Control Type 0x29
#define B3M_CONTROL_TYPE_LINEAR     0x00
#define B3M_CONTROL_TYPE_EVEN       0x01
#define B3M_CONTROL_TYPE_CUBIC      0x03
#define B3M_CONTROL_TYPE_QUARTIC    0x04
#define B3M_CONTROL_TYPE_FIFTH      0x05

// B3M Registor Addresses
// SYSTEM
#define B3M_SYSTEM_ID                          0x00
#define B3M_SYSTEM_BAUDRATE                    0x01
#define B3M_SYSTEM_MIN_POSITION_LIMIT          0x05
#define B3M_SYSTEM_MAX_POSITION_LIMIT          0x07
#define B3M_SYSTEM_CENTER_OFFSET               0x09
#define B3M_SYSTEM_MCU_TEMP_OFFSET             0x0B
#define B3M_SYSTEM_PWM_LIMIT_RATE_AT_MCU       0x0D
#define B3M_SYSTEM_MOTOR_TEMP_LIMIT            0x0E
#define B3M_SYSTEM_PWM_LIMIT_RATE_AT_MOTOR     0x10
#define B3M_SYSTEM_CURRENT_LIMIT               0x11
#define B3M_SYSTEM_PWM_LIMIT_RATE_AT_CURRENT   0x13
#define B3M_SYSTEM_MOTORLOCK_DELAY             0x14
#define B3M_SYSTEM_MOTORLOCK_RATE              0x15
#define B3M_SYSTEM_LOCKED_LOSS_RATE            0x16
#define B3M_SYSTEM_MIN_INPUT_VOLTAGE           0x17
#define B3M_SYSTEM_MAX_INPUT_VOLTAGE           0x19
#define B3M_SYSTEM_PWM_LIMIT                   0x1B
#define B3M_SYSTEM_DEADBAND_WIDTH              0x1C
// #define B3M_SYSTEM_RESERVED                    0x1E
// #define B3M_SYSTEM_RESERVED                    0x20
#define B3M_SYSTEM_MOTOR_CW_ROTATE_RATE        0x22
#define B3M_SYSTEM_MOTOR_CCW_ROTATE_RATE       0x23
// Servo
#define B3M_SERVO_OPTION              0x27
#define B3M_SERVO_MODE_SET            0x28
#define B3M_SERVO_CONTROL_TYPE        0x29
#define B3M_SERVO_TARGET_POSITION     0x2A
#define B3M_SERVO_CURRENT_POSITION    0x2C
#define B3M_SERVO_LAST_POSITION       0x2E
#define B3M_SERVO_TARGET_SPEED        0x30
#define B3M_SERVO_CURRENT_SPEED       0x32
#define B3M_SERVO_LAST_SPEED          0x34
#define B3M_SERVO_TARGET_TIME         0x36
#define B3M_SERVO_RUNNING_TIME        0x38
#define B3M_SERVO_UPTIME              0x3A
#define B3M_SERVO_TARGET_TORQUE       0x3C
#define B3M_SERVO_SYSTEM_CLOCK        0x3E
#define B3M_SERVO_SAMPLING_TIME       0x42
#define B3M_SERVO_MCU_TEMP            0x44
#define B3M_SERVO_MOTOR_TEMP          0x46
#define B3M_SERVO_CURRENT             0x48
#define B3M_SERVO_VOLTAGE             0x4A
#define B3M_SERVO_PWM_DUTY_RATE       0x4C
#define B3M_SERVO_PWM_CYCLE           0x4E
#define B3M_SERVO_ENCORDER_CURRENT    0x50
#define B3M_SERVO_ENCORDER_TOTAL      0x52
#define B3M_SERVO_HALL_IC             0x56
// Control
#define B3M_CONTROL_PID_GAIN_NO                     0x5C
// #define B3M_CONTROL_RESERVED                        0x5D
#define B3M_CONTROL_P_GAIN_0                        0x5E
#define B3M_CONTROL_I_GAIN_0                        0x62
#define B3M_CONTROL_D_GAIN_0                        0x66
#define B3M_CONTROL_STATIC_FRICTION_COEFFICIENT_0   0x6A
#define B3M_CONTROL_DYNAMIC_FRICTION_COEFFICIENT_0  0x6C
#define B3M_CONTROL_P_GAIN_1                        0x6E
#define B3M_CONTROL_I_GAIN_1                        0x72
#define B3M_CONTROL_D_GAIN_1                        0x76
#define B3M_CONTROL_STATIC_FRICTION_COEFFICIENT_1   0x7A
#define B3M_CONTROL_DYNAMIC_FRICTION_COEFFICIENT_1  0x7C
#define B3M_CONTROL_P_GAIN_2                        0x7E
#define B3M_CONTROL_I_GAIN_2                        0x82
#define B3M_CONTROL_D_GAIN_2                        0x86
#define B3M_CONTROL_STATIC_FRICTION_COEFFICIENT_2   0x8A
#define B3M_CONTROL_DYNAMIC_FRICTION_COEFFICIENT_2  0x8C
// Status
#define B3M_STATUS_ERROR           0x9D
#define B3M_STATUS_SYSTEM_ERROR    0x9E
#define B3M_STATUS_MOTOR_ERROR     0x9F
#define B3M_STATUS_UART_ERROR      0xA0
#define B3M_STATUS_COMMAND_ERROR   0xA1
// Version
#define B3M_VERSION_VOLTAGE     0xA2
#define B3M_VERSION_MODEL       0xA3
#define B3M_VERSION_TORQUE      0xA4
#define B3M_VERSION_CASE        0xA5
#define B3M_VERSION_MOTOR       0xA8
#define B3M_VERSION_DEVICE      0xA9
#define B3M_VERSION_BUILD       0xAA
#define B3M_VERSION_REVISION    0xAB
#define B3M_VERSION_MINOR       0xAC
#define B3M_VERSION_MAJOR       0xAD

class B3M{
    public:
        uint8_t readBuffer[16];
        B3M(HardwareSerial *serialPointer_, uint8_t enPin_);
        B3M(HardwareSerial *serialPointer_, uint8_t enPin_, uint32_t baudrate_, uint32_t timeout_);
        B3M(HardwareSerial *serialPointer_, uint8_t enPin_, uint8_t rxPin_, uint8_t txPin_, uint32_t baudrate_, uint32_t timeout_);
        void begin(void);
        // Basic Commands
        uint8_t load(uint8_t id_);
        uint8_t load(uint8_t id_, uint8_t option_);
        void load(uint8_t *id_, uint8_t option_, uint8_t length_);
        uint8_t save(uint8_t id_);
        uint8_t save(uint8_t id_, uint8_t option_);
        void save(uint8_t *id_, uint8_t option_, uint8_t length_);
        uint8_t read(uint8_t id_, uint8_t option_, uint8_t address_, uint8_t bytes_);
        uint8_t write(uint8_t id_, uint8_t *data_, uint8_t bytes_, uint8_t address_);
        uint8_t write(uint8_t id_, uint8_t option_, uint8_t *data_, uint8_t bytes_, uint8_t address_);
        void write(uint8_t *id_, uint8_t option_, uint8_t *data_, uint8_t bytes_, uint8_t address_, uint8_t length_);
        void reset(void);
        void reset(uint8_t id_);
        void reset(uint8_t id_, uint8_t option_, uint8_t time_);
        void reset(uint8_t *id_, uint8_t length_);
        void reset(uint8_t *id_, uint8_t option_, uint8_t time_, uint8_t length_);
        uint8_t position(uint8_t id_, int16_t position_);
        uint8_t position(uint8_t id_, uint8_t option_, int16_t position_, uint16_t time_);
        void position(uint8_t *id_, int16_t *position_, uint8_t length_);
        void position(uint8_t *id_, uint8_t option_, int16_t *position_, uint16_t time_, uint8_t length_);
        // Advance Commands
        int16_t deg2Pos(float deg_);
        // uint8_t deg2Pos(float *deg, uint8_t length_);
        float pos2Deg(int16_t position_);
    protected:
        HardwareSerial *b3mSerial_;
        uint8_t b3mEnPin_, b3mTxPin_, b3mRxPin_;
        uint32_t b3mBaudrate_, b3mTimeout_;
        uint8_t b3mCheckSum_(uint8_t *send_formats_, uint8_t bytes_);
        void b3mSend_(uint8_t *send_formats_, uint8_t bytes_);
        uint8_t b3mRead_(uint8_t *returnBuffer_);
};

#endif