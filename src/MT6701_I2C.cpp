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

#include "MT6701_I2C.h"

// ########## CONSTRUCTOR ##########
/*
 * @brief: assign the I2C interface to use (default &Wire)
 * @param _twi: Wire object to be used
 */
MT6701I2C::MT6701I2C(TwoWire* _twi) : _wire_(_twi ? _twi : &Wire) {
  // Nothing more to be done 
}

/* 
 * @brief: request one byte of data from the buffer
 * @param _reg_addr: 1 byte register address
 * @return: the byte value from the register that was requested
 * @note: use for a single register, such as 0x29
 */
uint8_t MT6701I2C::MT_RequestSingleRegister(uint8_t _reg_addr) {
  uint8_t single_byte = 0;

  // Start transfer to address
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  // Send register address byte
  _wire_->write(_reg_addr);
  // End connection
  _wire_->endTransmission();
  
  // Request a byte of data at MT6701 address
  _wire_->requestFrom(MT6701_I2C_ADDRESS, (uint8_t)1);
  // Read data from buffer
  if (_wire_->available() >= 1 ) {
    single_byte = _wire_->read();
  }
  // End connection
  _wire_->endTransmission();

  return single_byte;
}

/*
 * @brief: write a 1-byte value to a 1-byte arbitrary register
 * @param _reg_addr: 1 byte register address
 * @param _payload: 1 byte payload
 */
void MT6701I2C::MT_WriteOneByte(uint8_t _reg_addr, uint8_t _payload) {
  // Start transfer to MT6701 address
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  _wire_->write(_reg_addr);
  _wire_->write(_payload);
  // End connection
  _wire_->endTransmission();
}

// ########## PUBLIC ##########
/* 
 * @brief: call Wire.begin() method
 * @note: use if action has not been performed before
 */
void MT6701I2C::begin(void) {
  _wire_->begin();
}

/* 
 * @brief: call the Wire.begin(SDA, SCL) method specifying the pins
 * @param_sda_pin: SDA user pin
 * @param_scl_pin: custom SCL pin
 * @note: Use if the action has not been performed previously.
 * Applicable for ESP8266, ESP32 and STM32 based platforms which might
 * have several I2C controllers
 */
#if defined(ESP8266) || defined(ESP32)
void MT6701I2C::begin(int8_t _sda_pin, int8_t _scl_pin) {
  _wire_->begin(_sda_pin, _scl_pin);
}
#endif

#if defined(ARDUINO_ARCH_STM32)
void MT6701I2C::begin(int8_t _sda_pin, int8_t _scl_pin) {
  _wire_->setSDA(_sda_pin);
  _wire_->setSCL(_scl_pin);
  _wire_->begin();
}
#endif

/* 
 * @brief: setting a custom I2C bus frequency (default 400kHz)
 * @note: use if the bus frequency changes due to different devices
 */
void MT6701I2C::setClock(uint32_t _clock) {
  _wire_->setClock(_clock);
}

/*
 * @brief: saves data to sensor EEPROM memory
 * @note: the purpose of each command is not described in the documentation, the order of the commands is described in 7.2 EEPROM Programming
 * For programming the EEPROM a supply voltage of 4.5V to 5.5V is needed
 */
void MT6701I2C::saveNewValues(void) {
  // Start transfer to address
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  // Send 0x09
  _wire_->write(MT6701_I2C_EEPROM_PROG_KEY_REG);
  // Send 0xB3
  _wire_->write(MT6701_I2C_EEPROM_PROG_KEY_VALUE);
  // End connection
  _wire_->endTransmission();
  
  // Start transfer to address
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  // Send 0x0A
  _wire_->write(MT6701_I2C_EEPROM_PROG_CMD_REG);
  // Send 0x05
  _wire_->write(MT6701_I2C_EEPROM_PROG_CMD_VALUE);
  // End connection
  _wire_->endTransmission();
}

/*
 * @brief: find out if the sensor is connected to the I2C bus
 * @note: the standard search algorithm for devices on the I2C bus is used
 * @return:
 * MT6701I2C_DEFAULT_REPORT_ERROR - not connected
 * MT6701I2C_DEFAULT_REPORT_OK - connected
 */
bool MT6701I2C::isConnected(void) {
  // Start transfer to address
  _wire_->beginTransmission(MT6701_I2C_ADDRESS);
  return (!_wire_->endTransmission(MT6701_I2C_ADDRESS)) ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: assign microcontroller pin to control interface mode
 * @param_pin_mode: pin of the microcontroller to which the MODE pin of the sensor is connected
 */
void MT6701I2C::attachModePin(byte _pin_mode) {
  _pin_mode_ = _pin_mode;
  pinMode(_pin_mode_, OUTPUT);
}

/*
 * @brief: release the assigned microcontroller pin to control the interface mod
 */
void MT6701I2C::detachModePin(void) {
  pinMode(_pin_mode_, INPUT);
  _pin_mode_ = -1;
}

/* 
 * @brief: enable interface I2C/SSI
 */
void MT6701I2C::enableI2CorSSI(void) {
  digitalWrite(_pin_mode_, MT6701I2C_MODE_I2C_SSI);
}

/* 
 * @brief: enable interface UVW/ABZ
 */
void MT6701I2C::enableUVWorABZ(void) {
  digitalWrite(_pin_mode_, MT6701I2C_MODE_UVW_ABZ);
}

/* 
 * @brief: get raw angle value from Angle Data Register(13:0)
 * @return:
 *  0 - 16383
 */
word MT6701I2C::getRawAngle(void) {
  uint8_t high_byte = MT_RequestSingleRegister(MT6701_I2C_ANGLE_DATA_REG_H);
  uint8_t low_byte = MT_RequestSingleRegister(MT6701_I2C_ANGLE_DATA_REG_L);
  return (word)(high_byte << 6) | (low_byte >> 2);
}
/* 
 * @brief: get the angle value in degrees
 * @return:
 *  0.00 - 359.98
 */
float MT6701I2C::getDegreesAngle(void) {
  return ((float)getRawAngle() * 360) / 16384;

}
/* 
 * @brief: get the angle value in radians
 * @return:
 *  0.00 - 6.28319 (0 - 2pi)
 */
float MT6701I2C::getRadiansAngle(void) {
  return (getDegreesAngle() * M_PI) / 180;
}

/* 
 * @brief: get output interface type configuration
 * @note: QFN package only
 * @return:
 *  MT6701I2_OUTPUT_TYPE_A_B_Z
 *  MT6701I2_OUTPUT_TYPE_UVW
 */
MT6701I2CConfigurationOutputType MT6701I2C::getConfigurationOutputType(void) {
  return (MT6701I2CConfigurationOutputType)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG) >> MT6701_I2C_EEPROM_UVW_MUX_BIT) & 0x01);
}

/* 
 * @brief: set output interface type
 * @note: QFN package only
 */
void MT6701I2C::setConfigurationOutputTypeABZ(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_UVW_MUX_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_UVW_MUX_REG, bkup);
}

/* 
 * @brief: set output interface configuration type to -A-B-Z with check
 * @note: QFN package only
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setConfigurationOutputTypeABZVerify(void) {
  setConfigurationOutputTypeABZ();
  return getConfigurationOutputType() ? MT6701I2C_DEFAULT_REPORT_ERROR : MT6701I2C_DEFAULT_REPORT_OK;
}

/* 
 * @brief: Set output interface configuration type to UVW
 * @note: QFN package only
 */
void MT6701I2C::setConfigurationOutputTypeUVW(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_MUX_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_UVW_MUX_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_UVW_MUX_REG, bkup);
}

/* 
 * @brief: set output interface configuration type UVW with check
 * @note: QFN package only
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setConfigurationOutputTypeUVWVerify(void) {
  setConfigurationOutputTypeUVW();
  return getConfigurationOutputType() ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: get the output interface type
 * @return:
 *  MT6701I2_OUTPUT_TYPE_ABZ
 *  MT6701I2_OUTPUT_TYPE_UVW
 */
MT6701I2COutputType MT6701I2C::getOutputType(void) {
  return (MT6701I2COutputType)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_MUX_REG) >> MT6701_I2C_EEPROM_ABZ_MUX_BIT) & 0x01);
}

/* 
 * @brief: set output interface type ABZ
 */
void MT6701I2C::setOutputTypeABZ(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_MUX_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_ABZ_MUX_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_MUX_REG, bkup);
}

/* 
 * @brief: set output interface type ABZ with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputTypeABZVerify(void) {
  setOutputTypeABZ();
  return getOutputType() ? MT6701I2C_DEFAULT_REPORT_ERROR : MT6701I2C_DEFAULT_REPORT_OK;
}

/* 
 * @brief: set the output interface type to UVW
 */
void MT6701I2C::setOutputTypeUVW(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_MUX_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_ABZ_MUX_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_MUX_REG, bkup);
}

/* 
 * @brief: set output interface type UVW with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputTypeUVWVerify(void) {
  setOutputTypeUVW();
  return getOutputType() ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: get the value of the positive direction of rotation
 * @return:
 *  MT6701I2_DIRECTION_COUNTERCLOCKWISE
 *  MT6701I2_DIRECTION_CLOCKWISE
 */
MT6701I2CDirection MT6701I2C::getOutputRotationDirection(void) {
  return (MT6701I2CDirection)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_DIR_REG) >> MT6701_I2C_EEPROM_DIR_BIT) & 0x01);
}

/* 
 * @brief: set positive direction of rotation counterclockwise
 */
void MT6701I2C::setOutputRotationDirectionCounterclockwise(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_DIR_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_DIR_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_DIR_REG, bkup);
}

/* 
 * @brief: set the positive direction of rotation counterclockwise and check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputRotationDirectionCounterclockwiseVerify(void) {
  setOutputRotationDirectionCounterclockwise();
  return getOutputRotationDirection() ? MT6701I2C_DEFAULT_REPORT_ERROR : MT6701I2C_DEFAULT_REPORT_OK;
}
/* 
 * @brief: set positive direction of rotation clockwise
 */
void MT6701I2C::setOutputRotationDirectionClockwise(void) {
  uint8_t bkup =MT_RequestSingleRegister(MT6701_I2C_EEPROM_DIR_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_DIR_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_DIR_REG, bkup);
}

/* 
 * @brief: set the positive direction of rotation clockwise and check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputRotationDirectionClockwiseVerify(void) {
  setOutputRotationDirectionClockwise();
  return getOutputRotationDirection() ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: пget output resolution value in UVW mode
 * @return:
 *  1 - 16
 */
byte MT6701I2C::getOutputResolutionUVW(void) {
  return ((MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_RES_REG) >> MT6701_I2C_EEPROM_UVW_MUX_BIT_S) & 0x0F) + 1; // 0x0F = 0b00001111, +1 to shift into range 1-16
}

/* 
 * @brief: set output resolution value in UVW mode
 * @param _resolution:
 *  1 - 16
 */
void MT6701I2C::setOutputResolutionUVW(byte _resolution) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_UVW_RES_REG);
  bkup |= (_resolution - 1) << MT6701_I2C_EEPROM_UVW_MUX_BIT_S; // -1 shifts into range 0-15
  MT_WriteOneByte(MT6701_I2C_EEPROM_UVW_RES_REG, bkup);
}

/* 
 * @brief: set output resolution value in UVW mode with check
 * @param _resolution:
 *  1 - 16
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputResolutionUVWVerify(byte _resolution) {
  setOutputResolutionUVW(_resolution);
  return getOutputResolutionUVW() == _resolution ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: get output resolution value in ABZ mode
 * @return:
 *  1 - 1024
 */
word MT6701I2C::getOutputResolutionABZ(void) {
  uint8_t reg_h = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_RES_REG_H) & 0x03; // 0x03 = 0b00000011
  return (word)((reg_h << 8) | MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_RES_REG_L)) + 1; // +1 to shift into range 1-1024
}

/* 
 * @brief: set the output resolution value in ABZ mode
 * @param _resolution:
 *  1 - 1024
 */
void MT6701I2C::setOutputResolutionABZ(word _resolution) {
  uint8_t reg_l = (_resolution - 1) & 0xFF;
  uint8_t reg_h = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ABZ_RES_REG_H);
  reg_h |= ((_resolution - 1) >> 8) & 0x03;
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_RES_REG_H, reg_h);
  MT_WriteOneByte(MT6701_I2C_EEPROM_ABZ_RES_REG_L, reg_l);
}

/* 
 * @brief: set the output resolution value in ABZ mode with check
 * @param _resolution:
 *  1 - 1024
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputResolutionABZVerify(word _resolution) {
  setOutputResolutionABZ(_resolution);
  return getOutputResolutionABZ() == _resolution ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}


/* 
 * @brief: get pulse width Z value in ABZ mode
 * @return:
 *  MT6701I2_Z_PULSE_WIDTH_1LSB
 *  MT6701I2_Z_PULSE_WIDTH_2LSB
 *  MT6701I2_Z_PULSE_WIDTH_4LSB
 *  MT6701I2_Z_PULSE_WIDTH_8LSB
 *  MT6701I2_Z_PULSE_WIDTH_12LSB
 *  MT6701I2_Z_PULSE_WIDTH_16LSB
 *  MT6701I2_Z_PULSE_WIDTH_180DEG
 *  MT6701I2_Z_PULSE_WIDTH_1LSB_2
 */
MT6701I2CZPulseWidth MT6701I2C::getZPulseWidth(void) {
  return (MT6701I2CZPulseWidth)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG) >> MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S) & 0x07); // 0x07 = 0b00000111
}

/*
 * @brief: set pulse width Z 1LSB
 */
void MT6701I2C::setZPulseWidth1LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  bkup |= MT6701I2_Z_PULSE_WIDTH_1LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}

/*
 * @brief: set pulse width Z 1LSB with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZPulseWidth1LSBVerify(void) {
  setZPulseWidth1LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_1LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: set pulse width Z 2LSB
 */
void MT6701I2C::setZPulseWidth2LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  bkup |= MT6701I2_Z_PULSE_WIDTH_2LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}

/*
 * @brief: set pulse width Z 2LSB with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZPulseWidth2LSBVerify(void) {
  setZPulseWidth2LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_2LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: set pulse width Z LSB
 */
void MT6701I2C::setZPulseWidth4LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  bkup |= MT6701I2_Z_PULSE_WIDTH_4LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}

/*
 * @brief: set pulse width Z 4LSB with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZPulseWidth4LSBVerify(void) {
  setZPulseWidth1LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_4LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: set pulse width Z 8LSB
 */
void MT6701I2C::setZPulseWidth8LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  bkup |= MT6701I2_Z_PULSE_WIDTH_8LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}

/*
 * @brief: set pulse width Z 8LSB with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZPulseWidth8LSBVerify(void) {
  setZPulseWidth8LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_8LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: set pulse width Z 12LSB
 */
void MT6701I2C::setZPulseWidth12LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  bkup |= MT6701I2_Z_PULSE_WIDTH_12LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}

/*
 * @brief: set pulse width Z 12LSB with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZPulseWidth12LSBVerify(void) {
  setZPulseWidth12LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_12LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: set pulse width Z 16LSB
 */
void MT6701I2C::setZPulseWidth16LSB(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  bkup |= MT6701I2_Z_PULSE_WIDTH_16LSB << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}

/*
 * @brief: set pulse width Z 16LSB with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZPulseWidth16LSBVerify(void) {
  setZPulseWidth16LSB();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_16LSB ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: set pulse width Z 180 degrees
 */
void MT6701I2C::setZPulseWidth180DEG(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG);
  bkup |= MT6701I2_Z_PULSE_WIDTH_180DEG << MT6701_I2C_EEPROM_Z_PULSE_WIDTH_BIT_S;
  MT_WriteOneByte(MT6701_I2C_EEPROM_Z_PULSE_WIDTH_REG, bkup);
}

/*
 * @brief: set pulse width Z 180 degrees with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZPulseWidth180DEGVerify(void) {
  setZPulseWidth180DEG();
  return getZPulseWidth() == MT6701I2_Z_PULSE_WIDTH_180DEG ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: get the value of the null position
 * @note: SEE TABLE IN DOCUMENTATION
 * @return:
 *  0 - 4095
 */
word MT6701I2C::getZeroDegreePositionData(void) {
  uint8_t reg_h = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ZERO_REG_H) & 0x0F; // 0x0F = 0b00001111
  return (word)((reg_h << 8) | MT_RequestSingleRegister(MT6701_I2C_EEPROM_ZERO_REG_L));
}

/* 
 * @brief: set the null position value
 * @note: SEE TABLE IN DOCUMENTATION
 * @param _zero_position_data:
 *  0 - 4095
 */
void MT6701I2C::setZeroDegreePositionData(word _zero_position_data) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_ZERO_REG_H) & 0xF0; // 0xF0 = 0b11110000
  uint8_t reg_l = _zero_position_data & 0xFF;
  bkup |= _zero_position_data >> 8;
  MT_WriteOneByte(MT6701_I2C_EEPROM_ZERO_REG_H, bkup);
  MT_WriteOneByte(MT6701_I2C_EEPROM_ZERO_REG_L, reg_l);
}

/* 
 * @brief: set null position value with check
 * @note: SEE TABLE IN DOCUMENTATION
 * @param _zero_position_data:
 *  0 - 4095
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setZeroDegreePositionDataVerify(word _zero_position_data) {
  setZeroDegreePositionData(_zero_position_data);
  return getZeroDegreePositionData() == _zero_position_data ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: get the PWM frequency value
 * @return:
 *  MT6701I2_PWM_FREQUENCY_9944
 *  MT6701I2_PWM_FREQUENCY_4972
 */
MT6701I2CFrequencyPWM MT6701I2C::getFrequencyPWM(void) {
  return (MT6701I2CFrequencyPWM)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_FREQ_REG) >> MT6701_I2C_EEPROM_PWM_FREQ_BIT) & 0x01);
}

/* 
 * @brief: set the PWM frequency value to 994.4Hz
 */
void MT6701I2C::setFrequencyPWM9944(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_FREQ_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_PWM_FREQ_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_FREQ_REG, bkup);
}

/* 
 * @brief: set the PWM frequency value to 994.4Hz with verification
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setFrequencyPWM9944Verify(void) {
  setFrequencyPWM9944();
  return getFrequencyPWM() == MT6701I2_PWM_FREQUENCY_9944 ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: set the PWM frequency value to 497.2Hz
 */
void MT6701I2C::setFrequencyPWM4972(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_FREQ_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_PWM_FREQ_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_FREQ_REG, bkup);
}

/* 
 * @brief: set the PWM frequency value to 497.2Hz with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setFrequencyPWM4972Verify(void) {
  setFrequencyPWM4972();
  return getFrequencyPWM() == MT6701I2_PWM_FREQUENCY_4972 ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: get PWM polarity value
 * @return:
 *  MT6701I2_PWM_POLARITY_HIGH
 *  MT6701I2_PWM_POLARITY_LOW
 */
MT6701I2CPolarityPWM MT6701I2C::getPolarityPWM(void) {
  return (MT6701I2CPolarityPWM)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_POL_REG) >> MT6701_I2C_EEPROM_PWM_POL_BIT) & 0x01);
}

/* 
 * @brief: set the PWM polarity value to HIGH with check
 */
void MT6701I2C::setPolarityPWMHigh(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_POL_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_PWM_POL_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_POL_REG, bkup);
}

/* 
 * @brief: set the PWM polarity value to HIGH with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setPolarityPWMHighVerify(void) {
  setPolarityPWMHigh();
  return getPolarityPWM() == MT6701I2_PWM_POLARITY_HIGH ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: set the PWM polarity value to LOW
 */
void MT6701I2C::setPolarityPWMLow(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_PWM_POL_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_PWM_POL_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_PWM_POL_REG, bkup);
}

/* 
 * @brief: set the PWM polarity value to LOW with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setPolarityPWMLowVerify(void) {
  setPolarityPWMLow();
  return getPolarityPWM() == MT6701I2_PWM_POLARITY_LOW ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: get output mode
 * @return:
 *  MT6701I2_OUTPUT_MODE_ANALOG
 *  MT6701I2_OUTPUT_MODE_PWM
 */
MT6701I2COutputMode MT6701I2C::getOutputMode(void) {
  return (MT6701I2COutputMode)((MT_RequestSingleRegister(MT6701_I2C_EEPROM_OUT_MODE_REG) >> MT6701_I2C_EEPROM_OUT_MODE_BIT) & 0x01);
}

/* 
 * @brief: set output mode analog
 */
void MT6701I2C::setOutputModeAnalog(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_OUT_MODE_REG);
  bkup &= ~(1 << MT6701_I2C_EEPROM_OUT_MODE_BIT);
  MT_WriteOneByte(MT6701_I2C_EEPROM_OUT_MODE_REG, bkup);
}

/* 
 * @brief: set output mode analog with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputModeAnalogVerify(void) {
  setOutputModeAnalog();
  return getOutputMode() == MT6701I2_OUTPUT_MODE_ANALOG ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

/* 
 * @brief: set PWM output mode
 */
void MT6701I2C::setOutputModePWM(void) {
  uint8_t bkup = MT_RequestSingleRegister(MT6701_I2C_EEPROM_OUT_MODE_REG);
  bkup |= 1 << MT6701_I2C_EEPROM_OUT_MODE_BIT;
  MT_WriteOneByte(MT6701_I2C_EEPROM_OUT_MODE_REG, bkup);
}

/* 
 * @brief: set PWM output mode with check
 * @return:
 *  MT6701I2C_DEFAULT_REPORT_ERROR - failure
 *  MT6701I2C_DEFAULT_REPORT_OK - success
 */
bool MT6701I2C::setOutputModePWMVerify(void) {
  setOutputModePWM();
  return getOutputMode() == MT6701I2_OUTPUT_MODE_PWM ? MT6701I2C_DEFAULT_REPORT_OK : MT6701I2C_DEFAULT_REPORT_ERROR;
}

