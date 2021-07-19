/* ------------------------------------------
 * Copyright (c) 2017, Synopsys, Inc. All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1) Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.

 * 3) Neither the name of the Synopsys, Inc., nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
--------------------------------------------- */

#include "embARC.h"
#include "embARC_debug.h"
#include "stdio.h"
#include "BMP_180.h"
#include <math.h>
#include "dev_iic.h"


#define seaLevelPressure 101325

//static MPU9250_DEFINE(mpu9250_sensor, MPU9250_IIC_ID, MPU9250_IIC_ADDRESS);
/*
uint8_t string_buffer[50] = {0};

void readCalibrationData();
void measureTemperature(uint8_t * data);
long calculateB5(long UT);
long compensateTemperature(uint8_t * data);
float getTemperature(uint8_t * data);
*/
int16_t _AC1;
int16_t _AC2;
int16_t _AC3;
int16_t _B1;
int16_t _B2;
int16_t _MB;
int16_t _MC;
int16_t _MD;
uint16_t _AC4;
uint16_t _AC5;
uint16_t _AC6;
long _B5;
static float pressure;

int32_t writeByte_bmp(uint8_t address, uint8_t subAddress, uint8_t data)
{
	int32_t ercd = E_PAR;

	uint8_t data_write[2];
	data_write[0] = subAddress;
	data_write[1] = data;

	bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_TAR_ADDR, CONV2VOID(address));

	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_RESTART));
	
	ercd = bmp_i2c_ptr->iic_write(&data_write[0], 1);
	
	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_STOP));
	ercd = bmp_i2c_ptr->iic_write(&data_write[1], 1);

	if (ercd == 1) {
		ercd = E_OK;
	}


	return ercd;
}

int32_t readByte_bmp(uint8_t address, uint8_t subAddress , uint8_t * data)
{  
	int32_t ercd = E_PAR;
	uint8_t data_write[1];
	
	data_write[0] = subAddress;

	bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_TAR_ADDR, CONV2VOID(address));

	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_RESTART));
	ercd = bmp_i2c_ptr->iic_write(&data_write[0], 1);
	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_STOP));

	ercd = bmp_i2c_ptr->iic_read(&data[0], 1);

	if (ercd == 1) 
	{
		ercd = E_OK;
	}

	return ercd; 
}


int32_t writeBytes_bmp(uint8_t address, uint8_t subAddress , uint8_t count, uint8_t * data)
{
	int32_t ercd = E_PAR;

	uint8_t data_write[1];
	data_write[0] = subAddress;

	bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_TAR_ADDR, CONV2VOID(address));

	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_RESTART));
	ercd = bmp_i2c_ptr->iic_write(&data_write[0], 1);
	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_STOP));
	ercd = bmp_i2c_ptr->iic_write(&data[0], count);

	if (ercd == count) {
		ercd = E_OK;
	}

	return ercd;
}

int32_t readBytes_bmp(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * data)
{  
	//EMBARC_PRINTF("READBYTES\r\n");
	int32_t ercd = E_PAR;	
	uint8_t data_write[1];

	data_write[0] = subAddress;

	bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_TAR_ADDR, CONV2VOID(address));

	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_RESTART));
	//EMBARC_PRINTF("READBYTES: %x\r\n", ercd);
	ercd = bmp_i2c_ptr->iic_write(&data_write[0], 1);
	//EMBARC_PRINTF("READBYTES: %x\r\n", ercd);
	ercd = bmp_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_STOP));
	//EMBARC_PRINTF("READBYTES: %x\r\n", ercd);
	ercd = bmp_i2c_ptr->iic_read(&data[0], count);
	//EMBARC_PRINTF("READBYTES: %x\r\n", ercd);

	if (ercd == count) 
	{
		ercd = E_OK;
	}

	return ercd;
} 



void measureTemperature() {
	int32_t ercd = E_PAR;
	ercd = writeByte_bmp(BMP180_I2C_ADDRESS, BMP180_CONTROL_REGISTER, BMP180_MEASURE_TEMPERATURE);


	return;
}
void readTemperature(uint8_t * data) {
	int32_t ercd = E_PAR;
	ercd = readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_MEASURE_VALUE_MSB, 2, data);
	//EMBARC_PRINTF("%x %x\r\n", data[0], data[1]);

	return;
}

void measurePressure(int samplingMode) {
	int32_t ercd = E_PAR;
	uint8_t addr = BMP180_MEASURE_PRESSURE | (samplingMode << 6);
	ercd = writeByte_bmp(BMP180_I2C_ADDRESS, BMP180_CONTROL_REGISTER, addr);

	return;
}
void readPressure(uint8_t * data) {
	int32_t ercd = E_PAR;
	ercd = readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_MEASURE_VALUE_MSB, 3, data);
	//EMBARC_PRINTF("%x %x %x\r\n", data[0], data[1],data[2]);

	return;
}

void readCalibrationData() {
	uint8_t data[2];
	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_AC1, 2, data);
	_AC1 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_AC2, 2, data);
	_AC2 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_AC3, 2, data);
	_AC3 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_AC4, 2, data);
	_AC4 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_AC5, 2, data);
	_AC5 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_AC6, 2, data);
	_AC6 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_B1, 2, data);
	_B1 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_B2, 2, data);
	_B2 = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_MB, 2, data);
	_MB = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_MC, 2, data);
	_MC = (int16_t)((data[0] << 8) + data[1]);

	readBytes_bmp(BMP180_I2C_ADDRESS, BMP180_CALIBRATION_DATA_MD, 2, data);
	_MD = (int16_t)((data[0] << 8) + data[1]);

	return;
}

/**
 * Calculate B5
 */
long calculateB5(long UT) {

	long X1 = (UT - (long)_AC6) * (long)_AC5 >> 15;
	long X2 = ((long)_MC << 11) / (X1 + (long)_MD);

	return X1 + X2;
}

/**
 * Compensate the measured temperature with the calibration data.
 */
long compensateTemperature(uint8_t * data) {
	long UT;

	UT = (long)((data[0] << 8) + data[1]);
	_B5 = calculateB5(UT);

	return (_B5 + 8) >> 4;
}
long compensatePressure(uint8_t * data, int oversampling) {
	long UP;
	long B6, X1, X2, X3, B3, p;
	unsigned long B4, B7;
	UP = (long)((data[0] << 16) + (data[1]<<8) + data[2]);
	UP = UP>>(8 - oversampling);
	B6 = _B5 - 4000;
	X1 = (_B2 * (B6 * B6 >> 12)) >> 11;
	X2 = (_AC2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = (((_AC1 * 4 + X3) << oversampling) + 2) >> 2;
	X1 = _AC3 * B6 >> 13;
	X2 = (_B1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = _AC4 * (unsigned long)(X3 + 32768) >> 15;
	B7 = ((unsigned long)UP - B3) * (50000 >> oversampling);
	if (B7 < 0x80000000)
		p = (B7 * 2) / B4;
	else
		p = (B7 / B4) * 2;
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	p = p + ((X1 + X2 + 3791) >> 4);
	return p;
}

/**
 * Get the temperature.
 */
float getTemperature(uint8_t * data) {
	readTemperature(data);
	long t = compensateTemperature(data);
	return (float)t/10.0;
}

float getPressure(uint8_t * data, int samplingMode) {
	readPressure(data);
	long p = compensatePressure(data,samplingMode);
	return (float)p/100.0;
}

void getAltitude_measure_temperature(void) {
	uint8_t data[3];
	int32_t ercd = E_OK;

	/* Setting up iic communication:
	 * If connection is successful,
	 * monitor will print "I2C0 = Start"
	 */
	bmp_i2c_ptr = iic_get_dev(0);

	ercd = bmp_i2c_ptr->iic_open(DEV_MASTER_MODE, IIC_SPEED_STANDARD);
	if ((ercd == E_OK) || (ercd == E_OPNED))
	{
		readCalibrationData();
		measureTemperature();
	}

	return;
}

void getAltitude_measure_pressure(void) {
	uint8_t data[3];
	int32_t ercd = E_OK;
	float temp;
	int samplingMode=3;

	/* Setting up iic communication:
	 * If connection is successful,
	 * monitor will print "I2C0 = Start"
	 */

	if ((ercd == E_OK) || (ercd == E_OPNED))
	{
		temp = getTemperature(data);
		//printf("Temperature = %.3f\r\n", temp);
		measurePressure(samplingMode);
	}

	return;
}

float getAltitude_measure_altitude(void) {
	uint8_t data[3];
	int32_t ercd = E_OK;
	int samplingMode = 3;
	float altitude;

	/* Setting up iic communication:
	 * If connection is successful,
	 * monitor will print "I2C0 = Start"
	 */
	bmp_i2c_ptr = iic_get_dev(0);

	ercd = bmp_i2c_ptr->iic_open(DEV_MASTER_MODE, IIC_SPEED_STANDARD);
	if ((ercd == E_OK) || (ercd == E_OPNED))
	{
		pressure = getPressure(data,samplingMode);
		//printf("pressure = %f\r\n", pressure);
		altitude = 44330 * (1.0 - pow(pressure*100 / seaLevelPressure, 0.1903));
		//printf("altitude = %f\r\n", altitude);
	}

	return altitude;
}
