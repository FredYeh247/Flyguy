/**


**/
#include "embARC.h"
#include "embARC_debug.h"

#ifndef BMP180_h
#define BMP180_h

// includes


// nice
typedef uint8_t byte;

// i2c address
#define BMP180_I2C_ADDRESS							0x77

// register
#define BMP180_MEASURE_VALUE_XLSB					0xF8
#define BMP180_MEASURE_VALUE_LSB					0xF7
#define BMP180_MEASURE_VALUE_MSB					0xF6
#define BMP180_CONTROL_REGISTER						0xF4
#define BMP180_SOFT_RESET_REGISTER					0xE0
#define BMP180_CHIP_ID_REGISTER						0xD0

// values
#define BMP180_SOFT_RESET							0xB6
#define BMP180_MEASURE_TEMPERATURE					0x2E
#define BMP180_MEASURE_PRESSURE						0x34
#define BMP180_CHIP_ID								0x55

// resolutions
#define BMP180_OVERSAMPLING_ULTRA_LOW_POWER			0x00
#define BMP180_OVERSAMPLING_STANDARD				0x01
#define BMP180_OVERSAMPLING_HIGH_RESOLUTION			0x02
#define BMP180_OVERSAMPLING_ULTRA_HIGH_RESOLUTION	0x03

// calibration data
#define BMP180_CALIBRATION_DATA_AC1					0xAA
#define BMP180_CALIBRATION_DATA_AC2					0xAC
#define BMP180_CALIBRATION_DATA_AC3					0xAE
#define BMP180_CALIBRATION_DATA_AC4					0xB0
#define BMP180_CALIBRATION_DATA_AC5					0xB2
#define BMP180_CALIBRATION_DATA_AC6					0xB4
#define BMP180_CALIBRATION_DATA_B1					0xB6
#define BMP180_CALIBRATION_DATA_B2					0xB8
#define BMP180_CALIBRATION_DATA_MB					0xBA
#define BMP180_CALIBRATION_DATA_MC					0xBC
#define BMP180_CALIBRATION_DATA_MD					0xBE

#define seaLevelPressure 101325

DEV_IIC_PTR bmp_i2c_ptr;
static uint8_t string_buffer[50] = {0};


void measureTemperature();
void readCalibrationData();
long calculateB5(long UT);
long compensateTemperature(uint8_t * data);
float getTemperature(uint8_t * data);
float getPressure(uint8_t * data, int samplingMode);
void measurePressure(int samplingMode);
void getAltitude_measure_temperature(void);
void getAltitude_measure_pressure(void);
float getAltitude_measure_altitude(void);

#endif