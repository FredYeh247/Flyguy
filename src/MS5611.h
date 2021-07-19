/*
ms5611.h
Library for barometric pressure sensor MS5611-01BA on I2C with arduino
by Petr Gronat@2014
*/

// Include guard token - prevents to include header file twice
#ifndef MS5611_h
#define MS5611_h 	//create token

// Include libraries in replace of those of Arduino
#include "embARC.h"
#include "embARC_debug.h"
#include "stdio.h"
#include "dev_iic.h"
/*
MS5611.h
Library for barometric pressure sensor MS5611-01BA on I2C with arduino
by Petr Gronat@2014
*/
#define OSR 					3		// 0-3
#define CMD_RESET 				0x1E
#define CMD_ADC_READ			0x00
#define CMD_CONV_D1_BASE 		0x40
#define CMD_CONV_D2_BASE 		0x50
#define CONV_REG_SIZE 			0x02
#define CMD_PROM_READ_BASE		0xA2
#define PROM_REG_SIZE			0x02
#define NBYTES_CONV 			3
#define NBYTES_PROM 			2

// Temperature sampling period threshold [milliseconds]
// Kindly read the comment bellow in getPressure() method
#define T_THR					1000
/*
TODO:
1) Separate OSR for temperature and pressure
2) Timedelay empirical formula for temperature oversampling
3) Precidion pressure celibration for high temperature
4) Default and optional OSR
5) Documentation
*/

#define N_PROM_PARAMS 6


// address of the device MS5611
#define ADD_MS5611 0x77 	// can be 0x76 if CSB pin is connected to GND

#define altitude_resolution 30

void        altitude_measure(void);
void 		MS5611_begin();
void        getRawTemperature_cmd();
uint32_t 	getRawTemperature_read();
int32_t 	getTemperature();
void 	    getRawPressure_cmd();
uint32_t 	getRawPressure_read();
int32_t 	getPressure();
void 		readCalibration();
void 		getCalibration(uint16_t *);
void 		sendCommand(uint8_t);
uint32_t 	readnBytes(uint8_t);

void 		reset();

int32_t 	_P;
int32_t  	_T;
int32_t 	_dT;
uint16_t 	_C[N_PROM_PARAMS];
uint32_t 	_lastTime;
float       pressure;
volatile int altitude;
DEV_IIC_PTR ms5611_i2c_ptr;

#endif