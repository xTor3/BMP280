#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define BMP280_ADDRESS 0x76
#define TEMP_XLSB 0xFC
#define TEMP_LSB 0xFB
#define TEMP_MSB 0xFA
#define PRESS_XLSB 0xF9
#define PRESS_LSB 0xF8
#define PRESS_MSB 0xF7
#define CONFIG 0xF5
#define CTRL_MEAS 0xF4
#define STATUS 0xF3
#define RESET 0xE0
#define ID 0xD0
#define CALIB25 0xA1
/*
      .
      .
      .
      .
*/
#define CALIB00 0x88

#define TEMPERATURE_AT_SEA_LEVEL 20.0
#define STANDARD_TEMPERATURE_LAPSE_RATE -0.00825
#define PRESSURE_AT_SEA_LEVEL 101325.0
#define UNIVERSAL_GAS_CONSTANT 8.31432
#define GRAVITATIONAL_ACCELERATION_CONSTANT 9.80665
#define MOLAR_MASS_OF_EARTH_AIR 0.0289644

class BMP280 {
   private:
      uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
      uint8_t readByte(uint8_t address, uint8_t subAddress);
      uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest); 
   public:
      BMP280();
      ~BMP280();

      uint8_t getId();
      void reset();
      uint8_t isMeasuring();
      uint8_t isUpdate();
      void setOversamplingTemp(uint8_t osrs_t);
      uint8_t getOversamplingTemp();
      void setOversamplingPress(uint8_t osrs_p);
      uint8_t getOversamplingPress();
      void setMode(uint8_t mode);
      uint8_t getMode();
      void setTStandby(uint8_t t_sb);
      void setFilter(uint8_t filter);
      void set3WireSPI(uint8_t enable);
      void readPressRegisters(uint8_t* data);
      void readTempRegisters(uint8_t* data);
      void readMeasurementRegister(uint8_t* data);
      void getAdcMeasurement(uint32_t* data);
      uint16_t getDigX1(uint8_t t_or_p);
      int16_t getDigXX(uint8_t t_or_p, uint8_t n);
      void getMeasurements(double* data);

      double getRealAltitude(double p);
};

#endif