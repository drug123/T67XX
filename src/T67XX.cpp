/***************************************************
  This is a library for the Telaire T6700 Series Miniature CO2 Sensor Module.

  These sensors use UART and I2C to communicate, this library intended for I2C
  communication option, 2 pins are required to interface.

  To enable I2C communication, connect CTRL pin (pin 6 on the module) to the
 GND.

  Copyright (c) 2020 Yaroslav Osadchyy (drug123@gmail.com)
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "T67XX.h"

T67XX::T67XX() {}

bool T67XX::begin(void) { return this->begin(T67XX_I2C_ADDR); };

bool T67XX::begin(uint8_t Address)
{
  _address = Address;
#ifdef ESP8266
  Wire.begin();
  Wire.setClockStretchLimit(500);
  uint8_t status = Wire.status();
#endif
#ifdef ESP32
  bool status = Wire.begin();
#endif
#ifdef AVR
  Wire.begin();
  bool status = true;
#endif

#ifdef T67XX_DEBUG
  Serial.println("T67XX: Initiate I2C communication");
  Serial.print("T67XX: Wire reported status: ");
  Serial.println(status);
#endif

#ifdef ESP8266
  return status == 0;
#else
  return status;
#endif
};

#if defined(ESP8266) || defined(ESP32)
bool T67XX::begin(uint8_t sda, uint8_t scl)
{
  return this->begin(sda, scl, T67XX_I2C_ADDR);
}

bool T67XX::begin(uint8_t sda, uint8_t scl, uint8_t Address)
{
  _address = Address;
#ifdef ESP8266
  Wire.begin(sda, scl);
  Wire.setClockStretchLimit(500);
  uint8_t status = Wire.status();
#else
  bool status = Wire.begin(sda, scl);
#endif

#ifdef T67XX_DEBUG
  Serial.println("T67XX: Initiate I2C communication");
  Serial.print("T67XX: Wire reported status: ");
  Serial.println(status);
#endif

#ifdef ESP8266
  return status == 0;
#else
  return status;
#endif
}
#endif


uint16_t T67XX::readPPM(void)
{
#ifdef T67XX_DEBUG
  Serial.println("T67XX: Requesting CO2 PPM value");
#endif
  return this->read16(T67XX_REG_PPM);
};

uint16_t T67XX::getStatus(void)
{
#ifdef T67XX_DEBUG
  Serial.println("T67XX: Requesting sensor status");
#endif

  uint16_t _sta = this->read16(T67XX_REG_STATUS);
  _status.set(_sta);

#ifdef T67XX_DEBUG
  Serial.print("T67XX: Status of the sensor: ");
  Serial.println(_sta);
  Serial.print("T67XX: ");
  Serial.println("FEDCBA9876543210");
  String sStatus = String("0000000000000000") + String(_sta, BIN);
  sStatus.remove(0, sStatus.length() - 16);
  Serial.print("T67XX: ");
  Serial.println(sStatus);
  Serial.print("T67XX: Status flags: ");
  Serial.println(getStatusMsg(_sta));
#endif

  return _sta;
};

uint16_t T67XX::getFirmwareVersion(void)
{
#ifdef T67XX_DEBUG
  Serial.println("T67XX: Requesting firmware version");
#endif
  return this->read16(T67XX_REG_FIRMWARE);
};

void T67XX::reset(void)
{
#ifdef T67XX_DEBUG
  Serial.println("T67XX: Performing reset");
#endif
  this->write16(T67XX_REG_RESET, T67XX_REG_VAL_ENABLE);
};

void T67XX::setABCMode(bool Enabled)
{
#ifdef T67XX_DEBUG
  Serial.print("T67XX: Setting ABC self-calibration to ");
  Serial.println(Enabled ? "Enabled" : "Disabled");
#endif
  this->write16(T67XX_REG_ABC_LOGIC,
                Enabled ? T67XX_REG_VAL_ENABLE : T67XX_REG_VAL_DISABLE);
};

#ifdef T67XX_DEBUG
void T67XX::setSlaveAddress(uint8_t Address)
{
  Serial.print("T67XX: Setting new I2C address 0x");
  Serial.println(Address, HEX);
};
#endif

void T67XX::flashUpdate(void)
{
#ifdef T67XX_DEBUG
  Serial.println("T67XX: Updating flash with settings in RAM");
#endif
  this->write16(T67XX_REG_FLASH_UPDATE, T67XX_REG_VAL_ENABLE);
}

void T67XX::beginCalibration(void) { beginCalibration(false); };

void T67XX::beginCalibration(bool waitForCompletion)
{
#ifdef T67XX_DEBUG
  Serial.print("T67XX: Begin single point calibration and ");
  Serial.println(waitForCompletion ? "wait for copletion"
                                   : "do not wait for completion");
#endif
  this->write16(T67XX_REG_SPCAL, T67XX_REG_VAL_ENABLE);
  do
  {
    _status.set(this->getStatus());
    delay(100);
  } while (waitForCompletion && _status.SINGLE_POINT_CAL);
};

void T67XX::endCalibration(void)
{
#ifdef T67XX_DEBUG
  Serial.println("T67XX: Terminating single point calibration");
#endif
  this->write16(T67XX_REG_SPCAL, T67XX_REG_VAL_DISABLE);
};

/*** Private Section ***/
uint16_t T67XX::read16(uint16_t addr)
{
  Wire.beginTransmission(_address);
  Wire.write(0x04);
  Wire.write(byte(addr >> 8));
  Wire.write(byte(addr & 0xFF));
  Wire.write(0x00);
  Wire.write(0x01);
  Wire.endTransmission();

  delay(T67XX_READ_DELAY);

  Wire.requestFrom(int(_address), 4);
  _data[0] = Wire.read();
  _data[1] = Wire.read();
  _data[2] = Wire.read();
  _data[3] = Wire.read();
#ifdef T67XX_DEBUG
  Serial.print("T67XX: Returned function = ");
  Serial.println(_data[0]);
  Serial.print("T67XX: Returned byte count = ");
  Serial.println(_data[1]);
  Serial.print("T67XX: Returned Value MSB = ");
  Serial.println(_data[2]);
  Serial.print("T67XX: Returned Value LSB = ");
  Serial.println(_data[3]);
#endif
  return ((_data[2] << 8) | _data[3]);
}

void T67XX::write8(uint16_t addr, uint8_t data)
{
  Wire.beginTransmission(_address);
  Wire.write(0x06);
  Wire.write(byte(addr >> 8));
  Wire.write(byte(addr & 0xFF));
  Wire.write(0x00);
  Wire.write(data);
  Wire.endTransmission();

  delay(T67XX_READ_DELAY);

  Wire.requestFrom(int(_address), 5);
  _data[0] = Wire.read();
  _data[1] = Wire.read();
  _data[2] = Wire.read();
  _data[3] = Wire.read();
  _data[4] = Wire.read();
#ifdef T67XX_DEBUG
  Serial.print("T67XX: Returned function = ");
  Serial.println(_data[0]);
  Serial.print("T67XX: Returned Address MSB = ");
  Serial.println(_data[1]);
  Serial.print("T67XX: Returned Address LSB = ");
  Serial.println(_data[2]);
  Serial.print("T67XX: Returned Value MSB = ");
  Serial.println(_data[3]);
  Serial.print("T67XX: Returned Value LSB = ");
  Serial.println(_data[4]);
#endif
}

void T67XX::write16(uint16_t addr, uint16_t data)
{
  Wire.beginTransmission(_address);
  Wire.write(0x05);
  Wire.write(byte(addr >> 8));
  Wire.write(byte(addr & 0xFF));
  Wire.write(byte(data >> 8));
  Wire.write(byte(data & 0xFF));
  Wire.endTransmission();

  delay(T67XX_READ_DELAY);

  Wire.requestFrom(int(_address), 5);
  _data[0] = Wire.read();
  _data[1] = Wire.read();
  _data[2] = Wire.read();
  _data[3] = Wire.read();
  _data[4] = Wire.read();
#ifdef T67XX_DEBUG
  Serial.print("T67XX: Returned function = ");
  Serial.println(_data[0]);
  Serial.print("T67XX: Returned Address MSB = ");
  Serial.println(_data[1]);
  Serial.print("T67XX: Returned Address LSB = ");
  Serial.println(_data[2]);
  Serial.print("T67XX: Returned Value MSB = ");
  Serial.println(_data[3]);
  Serial.print("T67XX: Returned Value LSB = ");
  Serial.println(_data[4]);
#endif
}

String T67XX::getStatusMsg(uint16_t sensorStatus)
{
  T67XX::status statusStruct;
  String statusString = "";

  statusStruct.set(sensorStatus);

  if (sensorStatus)
  {
    if (statusStruct.ERROR)
      statusString.concat("GENERAL ERROR; ");
    if (statusStruct.CALIBRATION_ERROR)
      statusString.concat("CALIBRATION ERROR; ");
    if (statusStruct.FLASH_ERROR)
      statusString.concat("FLASH ERROR; ");
    if (statusStruct.REBOOT)
      statusString.concat("REBOOT; ");
    if (statusStruct.WARMUP)
      statusString.concat("WARMUP; ");
    if (statusStruct.SINGLE_POINT_CAL)
      statusString.concat("CALIBRATING;");
  }
  else
    statusString = "OK";

  return statusString;
}