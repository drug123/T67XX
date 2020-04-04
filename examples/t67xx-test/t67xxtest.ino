/***************************************************
  This is an example for the Telaire T6700 Series Miniature CO2 Sensor Module.
  -->
 https://www.amphenol-sensors.com/en/telaire/co2/525-co2-sensor-modules/3215-t6700

  These sensors use UART and I2C to communicate, this library intended for I2C
  communication option, 2 pins are required to interface.

  To enable I2C communication, connect CTRL pin (pin 6 on the module) to the
 GND.

  Copyright (c) 2020 Yaroslav Osadchyy (drug123@gmail.com)
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <T67XX.h>
#include <Wire.h>

#define USE_OLED 1

#if (USE_OLED == 1)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32);
#endif

T67XX co2sensor;
const int buttonPin = 0;

void setup()
{
#if (USE_OLED == 1)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();

  delay(5000);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.clearDisplay();
  display.println("INIT");
  display.display();
#endif

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(112500);

  while (!co2sensor.begin())
  {
    Serial.println("Could not find a valid T67XX sensor, please check wiring");
    delay(2000);
  }
  Serial.println("Connection with T67XX sensor established");

  co2sensor.reset();

  uint16_t sensorStatus = co2sensor.getStatus();
  while (sensorStatus)
  {
    Serial.print("Status: ");
    Serial.println(sensorStatus);
    Serial.println("FEDCBA9876543210");
    String strBinStatus =
        String("0000000000000000") + String(sensorStatus, BIN);
    strBinStatus.remove(0, strBinStatus.length() - 16);
    Serial.println(strBinStatus);

#if (USE_OLED == 1)
    display.setCursor(0, 0);
    display.clearDisplay();
    display.print("Status: ");
    display.println(sensorStatus);
    display.println("FEDCBA9876543210");
    display.println(strBinStatus);
    display.print("Status flags: ");
    display.println(getStatusMsg(sensorStatus));
    display.display();
#endif

    delay(T67XX_MEASURE_DELAY);
    sensorStatus = co2sensor.getStatus();
  }

  Serial.print("Sensor firmware version: ");
  Serial.println(co2sensor.getFirmwareVersion());
  Serial.println("Enabling ABC self-calibration");
  co2sensor.setABCMode(true);
  Serial.println("Saving settings to flash");
  co2sensor.flashUpdate();
  Serial.println("Send capital C to begin one-point calibration");
}

uint16_t _ppm, _sensorStatus;
long prevMillis;
int _pinStatus;

void loop()
{
  _ppm = co2sensor.readPPM();
  Serial.print("Current CO2 PPM vlaue: ");
  Serial.println(_ppm);
  _sensorStatus = co2sensor.getStatus();
  Serial.print("Sensor status: ");
  Serial.print(_sensorStatus);
  Serial.print(" (");
  Serial.print(getStatusMsg(_sensorStatus));
  Serial.println(")");

  prevMillis = millis();
  while (millis() - prevMillis < 5000)
  {
    delay(T67XX_MEASURE_DELAY);
    _sensorStatus = co2sensor.getStatus();
#if (USE_OLED == 1)
    display.setCursor(0, 0);
    display.clearDisplay();
    display.print("CO2 PPM: ");
    display.println(_ppm);
    display.print("Status: ");
    display.println(getStatusMsg(_sensorStatus));
    display.display();
#endif
  }
}

String getStatusMsg(uint16_t sensorStatus)
{
  T67XX::status statusStruct;
  String statusString = "";

  statusStruct.set(sensorStatus);

  if (sensorStatus)
  {
    if (statusStruct.ERROR)
      statusString.concat("ERROR ");
    if (statusStruct.CALIBRATION_ERROR)
      statusString.concat("ERRCALIB ");
    if (statusStruct.FLASH_ERROR)
      statusString.concat("ERRFLASH ");
    if (statusStruct.REBOOT)
      statusString.concat("REBOOT ");
    if (statusStruct.WARMUP)
      statusString.concat("WARMUP ");
    if (statusStruct.SINGLE_POINT_CAL)
      statusString.concat("CALIBRATING");
  }
  else
    statusString = "OK";

  statusString.trim();
  return statusString;
}