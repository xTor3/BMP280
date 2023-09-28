#include "BMP280.h"

BMP280::BMP280() {}

BMP280::~BMP280() {}


uint8_t BMP280::getId() {
   return readByte(BMP280_ADDRESS, ID);
}

void BMP280::reset() {
   writeByte(BMP280_ADDRESS, RESET, 0xB6);
}

uint8_t BMP280::isMeasuring() {
   uint8_t c = readByte(BMP280_ADDRESS, STATUS);

   return ((c & 0x08) >> 3);
}

uint8_t BMP280::isUpdate() {
   uint8_t c = readByte(BMP280_ADDRESS, STATUS);

   return (c & 0x01);
}

/*
0: temperature measurements are skipped; 1: x1; 2: x2; 3: x4; 4: x8; 5,6 or 7: x16
*/
void BMP280::setOversamplingTemp(uint8_t osrs_t) {
   uint8_t c = readByte(BMP280_ADDRESS, CTRL_MEAS);
   writeByte(BMP280_ADDRESS, CTRL_MEAS, (c & ~(0xE0)) | (osrs_t << 5));
   #ifdef DEBUG
      Serial.print("CTRL_MEAS: 0x"); Serial.println(readByte(BMP280_ADDRESS, CTRL_MEAS), HEX);
   #endif
}

uint8_t BMP280::getOversamplingTemp() {
   return (readByte(BMP280_ADDRESS, CTRL_MEAS) & (0xE0)) >> 5;
}

/*
0: pressure measurements are skipped; 1: x1; 2: x2; 3: x4; 4: x8; 5,6 or 7: x16
*/
void BMP280::setOversamplingPress(uint8_t osrs_p) {
   uint8_t c = readByte(BMP280_ADDRESS, CTRL_MEAS);
   writeByte(BMP280_ADDRESS, CTRL_MEAS, (c & ~(0x1C)) | (osrs_p << 2));
   #ifdef DEBUG
      Serial.print("CTRL_MEAS: 0x"); Serial.println(readByte(BMP280_ADDRESS, CTRL_MEAS), HEX);
   #endif
}

uint8_t BMP280::getOversamplingPress() {
   return (readByte(BMP280_ADDRESS, CTRL_MEAS) & (0x1C)) >> 2;
}

/*
0: Sleep Mode, 1 or 2: Forced Mode, 3: Normal Mode
*/
void BMP280::setMode(uint8_t mode) {
   uint8_t c = readByte(BMP280_ADDRESS, CTRL_MEAS);
   writeByte(BMP280_ADDRESS, CTRL_MEAS, (c & ~(0x03)) | mode);
   #ifdef DEBUG
      Serial.print("CTRL_MEAS: 0x"); Serial.println(readByte(BMP280_ADDRESS, CTRL_MEAS), HEX);
   #endif
}

uint8_t BMP280::getMode() {
   return (readByte(BMP280_ADDRESS, CTRL_MEAS) & (0x03));
}
/*
0: 0.5ms; 1: 62.5ms; 2: 125ms; 3: 250ms; 4: 500ms; 5: 1000ms; 6: 2000ms; 7: 4000ms
*/
void BMP280::setTStandby(uint8_t t_sb) {
   uint8_t c = readByte(BMP280_ADDRESS, BMP280_CONFIG);
   writeByte(BMP280_ADDRESS, BMP280_CONFIG, (c & ~(0xE0)) | (t_sb << 5));
   #ifdef DEBUG
      Serial.print("BMP280_CONFIG: 0x"); Serial.println(readByte(BMP280_ADDRESS, BMP280_CONFIG), HEX);
   #endif
}

/*
Filter Coefficient: Samples to reach >= 75% of step response
1: 1; 2: 2; 4: 5; 8: 11; 16: 22
*/
void BMP280::setFilter(uint8_t filter) {
   uint8_t c = readByte(BMP280_ADDRESS, BMP280_CONFIG);
   writeByte(BMP280_ADDRESS, BMP280_CONFIG, (c & ~(0x1C)) | (filter << 2));
   #ifdef DEBUG
      Serial.print("BMP280_CONFIG: 0x"); Serial.println(readByte(BMP280_ADDRESS, BMP280_CONFIG), HEX);
   #endif
}

void BMP280::set3WireSPI(uint8_t enable) {
   writeByte(BMP280_ADDRESS, BMP280_CONFIG, enable);
   #ifdef DEBUG
      Serial.print("BMP280_CONFIG: 0x"); Serial.println(readByte(BMP280_ADDRESS, BMP280_CONFIG), HEX);
   #endif
}

/*
data must be an array with 3 indices
*/
void BMP280::readPressRegisters(uint8_t* data) {
   readBytes(BMP280_ADDRESS, PRESS_MSB, 3, data);
}

/*
data must be an array with 3 indices
*/
void BMP280::readTempRegisters(uint8_t* data) {
   readBytes(BMP280_ADDRESS, TEMP_MSB, 3, data);
}

/*
data must be an array with 6 indices
*/
void BMP280::readMeasurementRegister(uint8_t* data) {
   readBytes(BMP280_ADDRESS, PRESS_MSB, 6, data);
}

uint32_t BMP280::getAdcPressMeasurement() {
   uint8_t raw[3];
   BMP280::readPressRegisters(raw);
   return (((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 3) | ((uint32_t)raw[2]));
}

uint32_t BMP280::getAdcTempMeasurement() {
   uint8_t raw[3];
   BMP280::readTempRegisters(raw);
   return (((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 3) | ((uint32_t)raw[2]));
}

/*
data must be an array with 2 indices
*/
void BMP280::getAdcMeasurement(uint32_t* data) {
   uint8_t raw[6];
   BMP280::readMeasurementRegister(raw);
   data[0] = (((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 3) | ((uint32_t)raw[2]));
   data[1] = (((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 3) | ((uint32_t)raw[5]));
}

/*
t_or_p: 0 = Temperature, 1 = Pressure
*/
uint16_t BMP280::getDigX1(uint8_t t_or_p) {
   uint8_t data[2];
   readBytes(BMP280_ADDRESS, (CALIB00 + t_or_p*6), 2, data);
   return (((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/*
t_or_p: 0 = Temperature, 1 = Pressure; n >= 2
*/
int16_t BMP280::getDigXX(uint8_t t_or_p, uint8_t n) {
   uint8_t data[2];
   readBytes(BMP280_ADDRESS, (CALIB00 + t_or_p*6 + (n-1)*2), (uint8_t)2, data);
   return (((int16_t)data[1] << 8) | (int16_t)data[0]);
}

/*
data must be an array with 2 indices
*/
void BMP280::getMeasurements(double* data) {
   uint32_t adc_data[2];
   BMP280::getAdcMeasurement(adc_data);

   double var1, var2;
   var1 = (((double)adc_data[1])/16384.0 - ((double)BMP280::getDigX1(0))/1024.0) * ((double)BMP280::getDigXX(0, 2));
   var2 = ((((double)adc_data[1])/131072.0 - ((double)BMP280::getDigX1(0))/8192.0) * ((double)adc_data[1])/131072.0 - ((double)BMP280::getDigX1(0))/8192.0) * ((double)BMP280::getDigXX(0, 2));
   int32_t t_fine = (int32_t)(var1 + var2);
   data[1] = (var1 + var2) / 5120.0;

   var1 = ((double)t_fine/2.0) - 64000.0;
   var2 = var1 * var1 * ((double)BMP280::getDigXX(1, 6)) / 32768.0;
   var2 = var2 + var1 * ((double)BMP280::getDigXX(1, 5)) * 2.0;
   var2 = (var2/4.0)+(((double)BMP280::getDigXX(1, 4)) * 65536.0);
   var1 = (((double)BMP280::getDigXX(1, 3)) * var1 * var1 / 524288.0 + ((double)BMP280::getDigXX(1, 2)) * var1) / 524288.0;
   var1 = (1.0 + var1 / 32768.0)*((double)BMP280::getDigX1(1));
   if (var1 == 0.0) {
      data[0] = 0;
      return;
   }
   data[0] = 1048576.0 - (double)adc_data[0];
   data[0] = (data[0] - (var2 / 4096.0)) * 6250.0 / var1;
   var1 = ((double)BMP280::getDigXX(1, 9)) * data[0] * data[0] / 2147483648.0;
   var2 = data[0] * ((double)BMP280::getDigXX(1, 8)) / 32768.0;
   data[0] = data[0] + (var1 + var2 + ((double)BMP280::getDigXX(1, 7))) / 16.0;
}

/*
p must be the pressure for which you want to know the height above sea level
*/
double BMP280::getRealAltitude(double p) {
   double h = ((TEMPERATURE_AT_SEA_LEVEL+273.15)/STANDARD_TEMPERATURE_LAPSE_RATE)*(pow(p/PRESSURE_AT_SEA_LEVEL, ((-UNIVERSAL_GAS_CONSTANT)*STANDARD_TEMPERATURE_LAPSE_RATE)/(GRAVITATIONAL_ACCELERATION_CONSTANT*MOLAR_MASS_OF_EARTH_AIR))-1);
   return h;
}


/*
   ********************************************************************
*/
uint8_t BMP280::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
   Wire.beginTransmission(address);
   Wire.write(subAddress);
   Wire.write(data);
   return (Wire.endTransmission());
}

uint8_t BMP280::readByte(uint8_t address, uint8_t subAddress) {
   uint8_t data;
   Wire.beginTransmission(address);
   Wire.write(subAddress);
   Wire.endTransmission(false);
   Wire.requestFrom(address, (uint8_t) 1);
   return Wire.read();
}

uint8_t BMP280::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
   Wire.beginTransmission(address);   // Initialize the Tx buffer
   Wire.write(subAddress);            // Put slave register address in Tx buffer
   int result = Wire.endTransmission(false);
   if(result)       // Send the Tx buffer, but send a restart to keep connection alive
      return result;
   uint8_t i = 0;
   Wire.requestFrom(address, count);  // Read bytes from slave register address
   while (Wire.available())
      dest[i++] = Wire.read();        // Put read results in the Rx buffer
   return 0;
}