/*
 * Class for Arduino IDE implementing many methods for  interaction with
 * Hall effect based rotational position sensor MT6701 from MagnTek
 * (http://www.magntek.com.cn/en/index.htm)
 *
 * Documentation for the sensor:
 *  http://www.magntek.com.cn/en/list/177/559.htm
 *  http://www.magntek.com.cn/upload/MT6701_Rev.1.8.pdf
 *
 * Contacts:
 *  GitHub - https://github.com/S-LABc
 *  Gmail - romansklyar15@gmail.com
 *
 * Copyright (C) 2022. v1.2 / License MIT / Sklyar Roman S-LAB
*/

#pragma once
#include "Arduino.h"
#include "Wire.h"

/*=== Sensor I2C bus settings ===*/
const uint32_t MT6701_I2C_CLOCK_100KHZ = 100000;
const uint32_t MT6701_I2C_CLOCK_400KHZ = 400000;
const uint32_t MT6701_I2C_CLOCK_1MHZ   = 1000000;
const uint8_t MT6701_I2C_ADDRESS = 0x06;

/*=== MODE pins on different boards (depending on the core) ===*/
#define STM32_MT6701_MODE_PIN   PC13
#define ESP8266_MT6701_MODE_PIN 2
#define ESP32_MT6701_MODE_PIN   4
#define ARDUINO_MT6701_MODE_PIN 3

/*=== Sensor register addresses ===*/
// Angle Data Register
const uint8_t MT6701_I2C_ANGLE_DATA_REG_H = 0x03;
const uint8_t MT6701_I2C_ANGLE_DATA_REG_L = 0x04;
// UVW_MUX (on QFN package only)
const uint8_t MT6701_I2C_EEPROM_UVW_MUX_REG = 0x25;
const uint8_t MT6701_I2C_EEPROM_UVW_MUX_BIT = 7;
// ABZ_MUX
const uint8_t MT6701_I2C_EEPROM_ABZ_MUX_REG = 0x29;
const uint8_t MT6701_I2C_EEPROM_ABZ_MUX_BIT = 6;
// DIR
const uint8_t MT6701_I2C_EEPROM_DIR_REG = 0x29;
const uint8_t MT6701_I2C_EEPROM_DIR_BIT = 1;
// UVW_RES
const uint8_t MT6701_I2C_EEPROM_UVW_RES_REG   = 0x30;
const uint8_t MT6701_I2C_EEPROM_UVW_MUX_BIT_S = 4;
// ABZ_RES
const uint8_t MT6701_I2C_EEPROM_ABZ_RES_REG_H = 0x30;
const uint8_t MT6701_I2C_EEPROM_ABZ_RES_REG_L = 0x31;
const uint8_t MT6701_I2C_EEPROM_ABZ_MUX_BIT_S = 0;
// HYST
const uint8_t MT6701_I2C_EEPROM_HYST_REG_H = 0x32;
const uint8_t MT6701_I2C_EEPROM_HYST_REG_L = 0x34;
// Z_PULSE_WIDTH
const uint8_t MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG   = 0x32;
const uint8_t MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S = 4;
// ZERO
const uint8_t MT6701_I2C_EEPROM_ZERO_REG_H = 0x32;
const uint8_t MT6701_I2C_EEPROM_ZERO_REG_L = 0x33;
// PWM_FREQ
const uint8_t MT6701_I2C_EEPROM_PWM_FREQ_REG = 0x38;
const uint8_t MT6701_I2C_EEPROM_PWM_FREQ_BIT = 7;
// PWM_POL
const uint8_t MT6701_I2C_EEPROM_PWM_POL_REG = 0x38;
const uint8_t MT6701_I2C_EEPROM_PWM_POL_BIT = 6;
// OUT_MODE
const uint8_t MT6701_I2C_EEPROM_OUT_MODE_REG = 0x38;
const uint8_t MT6701_I2C_EEPROM_OUT_MODE_BIT = 5;
// A_START
const uint8_t MT6701_I2C_EEPROM_A_START_REG_H = 0x3E;
const uint8_t MT6701_I2C_EEPROM_A_START_REG_L = 0x3F;
// A_STOP
const uint8_t MT6701_I2C_EEPROM_A_STOP_REG_H = 0x3E;
const uint8_t MT6701_I2C_EEPROM_A_STOP_REG_L = 0x40;
const uint8_t MT6701_I2C_EEPROM_A_STOP_BIT_S = 4;
// 7.2 EEPROM Programming
const uint8_t MT6701_I2C_EEPROM_PROG_KEY_REG   = 0x09;
const uint8_t MT6701_I2C_EEPROM_PROG_KEY_VALUE = 0xB3;
const uint8_t MT6701_I2C_EEPROM_PROG_CMD_REG   = 0x0A;
const uint8_t MT6701_I2C_EEPROM_PROG_CMD_VALUE = 0x05;

/*=== Auxiliary values ===*/
// Output type configuration (QFN package only)
enum MT6701I2CConfigurationOutputType {
  MT6701I2_CONFIG_OUTPUT_TYPE_UVW,
  MT6701I2_CONFIG_OUTPUT_TYPE_A_B_Z,
};
// Output interface type
enum MT6701I2COutputType {
  MT6701I2_OUTPUT_TYPE_ABZ,
  MT6701I2_OUTPUT_TYPE_UVW,
};
// Positive direction of rotation
enum MT6701I2CDirection {
  MT6701I2_DIRECTION_COUNTERCLOCKWISE, // Counterclockwise
  MT6701I2_DIRECTION_CLOCKWISE, // Clockwise
};
// Pulse width Z
enum MT6701I2CZPulseWidth {
  MT6701I2_Z_PULSE_WIDTH_1LSB,
  MT6701I2_Z_PULSE_WIDTH_2LSB,
  MT6701I2_Z_PULSE_WIDTH_4LSB,
  MT6701I2_Z_PULSE_WIDTH_8LSB,
  MT6701I2_Z_PULSE_WIDTH_12LSB,
  MT6701I2_Z_PULSE_WIDTH_16LSB,
  MT6701I2_Z_PULSE_WIDTH_180DEG,
  MT6701I2_Z_PULSE_WIDTH_1LSB_2,
};
// PWM frequency
enum MT6701I2CFrequencyPWM {
  MT6701I2_PWM_FREQUENCY_9944,
  MT6701I2_PWM_FREQUENCY_4972,
};
// PWM polarity
enum MT6701I2CPolarityPWM {
  MT6701I2_PWM_POLARITY_HIGH,
  MT6701I2_PWM_POLARITY_LOW,
};
// Output mode
enum MT6701I2COutputMode {
  MT6701I2_OUTPUT_MODE_ANALOG,
  MT6701I2_OUTPUT_MODE_PWM,
};
// Standard success/error return values
const uint8_t MT6701I2C_DEFAULT_REPORT_ERROR = 0;
const uint8_t MT6701I2C_DEFAULT_REPORT_OK    = 1;
// Sensor interface selector values
const uint8_t MT6701I2C_MODE_I2C_SSI = 0;
const uint8_t MT6701I2C_MODE_UVW_ABZ = 1;


class MT6701I2C {
  private:
    TwoWire* _wire_; // I2C TwoWire object
    int8_t _pin_mode_ = -1; // GPIO pin of the microcontroller to which the MODE pin of the sensor is connected

  protected:
    uint8_t MT_RequestSingleRegister(uint8_t _reg_addr); // Request one byte register value
    void MT_WriteOneByte(uint8_t _reg_addr, uint8_t _payload); // Write one byte to a register

  public:
    MT6701I2C(TwoWire* _twi); // Constructor using only I2C interface

    void begin(void); // Calls Wire.begin()
#if defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_STM32)
    void begin(int8_t _sda_pin, int8_t _scl_pin); // Calla Wire.begin(SDA, SCL) specifying pins used for I2C
#endif
    void setClock(uint32_t _clock = MT6701_I2C_CLOCK_400KHZ); // I2C frequency setting. Possible values: 100kHz, 400kHz, 1MHz, or custom value (default 400kHz)

    void saveNewValues(void); // Manufacturer's method for storing values in EEPROM memory. EEPROM programming requires a supply voltage of 4.5V to 5.5V

    bool isConnected(void); // Checks for the device on the I2C bus using a standard algorithm

    void attachModePin(byte _pin_mode); // Assign a microcontroller pin to control the interface mode
    void detachModePin(void); // Release the assigned microcontroller pin controlling the interface mode

    void enableI2CorSSI(void); // Enable interface I2C/SSI. MT6701I2C_MODE_I2C_SSI
    void enableUVWorABZ(void); //  Enable interface UVW/ABZ. MT6701I2C_MODE_UVW_ABZ

    word getRawAngle(void); // Get raw angle value. 0 - 16383
    float getDegreesAngle(void); // Get angle in degrees. 0.00 - 359.98
    float getRadiansAngle(void); // Get angle in radians. 0.00 - 6.28 (0 to 2pi)

    MT6701I2CConfigurationOutputType getConfigurationOutputType(void); // Get output interface configuration type (QFN package only). MT6701I2_CONFIG_OUTPUT_TYPE_UVW, MT6701I2_CONFIG_OUTPUT_TYPE_A_B_Z
    void setConfigurationOutputTypeABZ(void); // Set output interface configuration type to -A-B-Z (QFN package only)
    bool setConfigurationOutputTypeABZVerify(void); // Same as above, but with confirmation (QFN package only)
    void setConfigurationOutputTypeUVW(void); // Set output interface configuration type to UVW (QFN package only)
    bool setConfigurationOutputTypeUVWVerify(void); // Same as above, but with confirmation (QFN package only)

    MT6701I2COutputType getOutputType(void); // Get output interface type. MT6701I2_OUTPUT_TYPE_ABZ, MT6701I2_OUTPUT_TYPE_UVW
    void setOutputTypeABZ(void); // Set output interface type to ABZ
    bool setOutputTypeABZVerify(void); // Same as above, but with confirmation
    void setOutputTypeUVW(void); // Set output interface type to UVW
    bool setOutputTypeUVWVerify(void); // Same as above, but with confirmation

    MT6701I2CDirection getOutputRotationDirection(void); // Get direction of rotation. MT6701I2_DIRECTION_COUNTERCLOCKWISE, MT6701I2_DIRECTION_CLOCKWISE
    void setOutputRotationDirectionCounterclockwise(void); // Set rotation direction counterclockwise
    bool setOutputRotationDirectionCounterclockwiseVerify(void); // Same as above, but with confirmation
    void setOutputRotationDirectionClockwise(void); // Set rotation direction clockwise
    bool setOutputRotationDirectionClockwiseVerify(void); // Same as above, but with confirmation

    byte getOutputResolutionUVW(void); // Get output resolution in UVW mode. 1 - 16
    void setOutputResolutionUVW(byte _resolution); // Set output resolution in UVW mode. 1 - 16
    bool setOutputResolutionUVWVerify(byte _resolution); // Same as above, but with confirmation

    word getOutputResolutionABZ(void); // Get output resolution in ABZ mode. 1 - 1024
    void setOutputResolutionABZ(word _resolution); // Set output resolution in ABZ mode. 1 - 1024
    bool setOutputResolutionABZVerify(word _resolution); // Same as above, but with confirmation

    MT6701I2CZPulseWidth getZPulseWidth(void); // Get the pulse width value at pin Z in ABZ mode. MT6701I2_Z_PULSE_WIDTH_1LSB, MT6701I2_Z_PULSE_WIDTH_2LSB,
    // MT6701I2_Z_PULSE_WIDTH_4LSB, MT6701I2_Z_PULSE_WIDTH_8LSB, MT6701I2_Z_PULSE_WIDTH_12LSB, MT6701I2_Z_PULSE_WIDTH_16LSB, MT6701I2_Z_PULSE_WIDTH_180DEG, MT6701I2_Z_PULSE_WIDTH_1LSB_2,
    void setZPulseWidth1LSB(void); // Set pulse width 1LSB
    bool setZPulseWidth1LSBVerify(void); // Same as above, but with confirmation
    void setZPulseWidth2LSB(void); // Set pulse width 2LSB
    bool setZPulseWidth2LSBVerify(void); // Same as above, but with confirmation
    void setZPulseWidth4LSB(void); // Set pulse width 4LSB
    bool setZPulseWidth4LSBVerify(void); // Same as above, but with confirmation
    void setZPulseWidth8LSB(void); // Set pulse width 8LSB
    bool setZPulseWidth8LSBVerify(void); // Same as above, but with confirmation
    void setZPulseWidth12LSB(void); // Set pulse width 12LSB
    bool setZPulseWidth12LSBVerify(void); // Same as above, but with confirmation
    void setZPulseWidth16LSB(void); // Set pulse width 16LSB
    bool setZPulseWidth16LSBVerify(void); // Same as above, but with confirmation
    void setZPulseWidth180DEG(void); // Set pulse width to 180 degrees
    bool setZPulseWidth180DEGVerify(void); // Same as above, but with confirmation

    word getZeroDegreePositionData(void); // Get the zero position value. See the table in the datasheet page 30. 0x000 - 0xFFF
    void setZeroDegreePositionData(word _zero_position_data); // Set the zero position value. See the table in the documentation
    bool setZeroDegreePositionDataVerify(word _zero_position_data); // Same as above, but with confirmation

    MT6701I2CFrequencyPWM getFrequencyPWM(void); // Получить значение частоты ШИМ. MT6701I2_PWM_FREQUENCY_9944, MT6701I2_PWM_FREQUENCY_4972
    void setFrequencyPWM9944(void); // Set the PWM frequency to 994.4Hz
    bool setFrequencyPWM9944Verify(void); // Same as above, but with confirmation
    void setFrequencyPWM4972(void); // Set the PWM frequency to 497.2Hz
    bool setFrequencyPWM4972Verify(void); // Same as above, but with confirmation

    MT6701I2CPolarityPWM getPolarityPWM(void); // Get PWM polarity. MT6701I2_PWM_POLARITY_HIGH, MT6701I2_PWM_POLARITY_LOW
    void setPolarityPWMHigh(void); // Set PWM polarity HIGH
    bool setPolarityPWMHighVerify(void); // Same as above, but with confirmation
    void setPolarityPWMLow(void); // Set PWM polarity  LOW
    bool setPolarityPWMLowVerify(void); // Same as above, but with confirmation

    MT6701I2COutputMode getOutputMode(void); // Get output mode. MT6701I2_OUTPUT_MODE_ANALOG, MT6701I2_OUTPUT_MODE_PWM
    void setOutputModeAnalog(void); // Set analog output mode
    bool setOutputModeAnalogVerify(void); // Same as above, but with confirmation
    void setOutputModePWM(void); // Set PWM output mode
    bool setOutputModePWMVerify(void); // Same as above, but with confirmation
};
