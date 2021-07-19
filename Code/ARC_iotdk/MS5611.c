#include "embARC.h"
#include "embARC_debug.h"
#include "MS5611.h"
#include "stdio.h"
#include <math.h>

#define seaLevelPressure 101325

void altitude_measure(void)
{
	static int cnt = 0;
	int cnt_num = 20;
	static int state = 0;
	static float tmp = 0;
	int32_t temp;

	state++;
	if (state == 1)
	{
		getRawTemperature_cmd();
	}
	else if (state == 10)
	{
		temp = getRawTemperature_read();
		getRawPressure_cmd();
		//printf("temp: %ld\r\n", temp);
	}
	else if (state == 20)
	{
		pressure = getRawPressure_read();
		//printf("pressure: %f\r\n", pressure);
		tmp += 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
		state = 0;
		cnt++;
	}

	if (cnt == cnt_num)
	{
		altitude = tmp * 100 / cnt_num / altitude_resolution;
		cnt = 0;
		tmp = 0;
		printf("altitude: %d\r\n", altitude);
	}
	printf("altitude: %d\r\n", altitude);
	return;
}

int32_t writeByteMS(uint8_t address, uint8_t data)
{
	int32_t ercd = E_PAR;

	uint8_t data_write[2];
	data_write[1] = data;

	ms5611_i2c_ptr->iic_control(IIC_CMD_MST_SET_TAR_ADDR, CONV2VOID(address));

	//ercd = ms5611_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_RESTART));

	//ercd = ms5611_i2c_ptr->iic_write(0x1E, 1);
	//EMBARC_PRINTF("write 1 :%x \r\n", ercd);
	ercd = ms5611_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_STOP));
	//EMBARC_PRINTF("write 1 :%x \r\n", ercd);
	ercd = ms5611_i2c_ptr->iic_write(&data_write[1], 1);
	//EMBARC_PRINTF("write 2 :%x \r\n", ercd);
	if (ercd == 1)
	{
		ercd = E_OK;
	}

	return ercd;
}

int32_t readBytesMS(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *data)
{
	int32_t ercd = E_PAR;
	uint8_t data_write[1];

	data_write[0] = subAddress;

	ms5611_i2c_ptr->iic_control(IIC_CMD_MST_SET_TAR_ADDR, CONV2VOID(address));

	ercd = ms5611_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_RESTART));
	ercd = ms5611_i2c_ptr->iic_write(&data_write[0], 1);
	//EMBARC_PRINTF("reads 1 :%x \r\n", ercd);
	ercd = ms5611_i2c_ptr->iic_control(IIC_CMD_MST_SET_NEXT_COND, CONV2VOID(IIC_MODE_STOP));
	ercd = ms5611_i2c_ptr->iic_read(&data[0], count);
	//EMBARC_PRINTF("reads 2 :%x \r\n", ercd);

	if (ercd == count)
	{
		ercd = E_OK;
	}

	return ercd;
}

void MS5611_begin()
{
	int32_t ercd;

	ms5611_i2c_ptr = iic_get_dev(0);

	ercd = ms5611_i2c_ptr->iic_open(DEV_MASTER_MODE, IIC_SPEED_STANDARD);
	if ((ercd == E_OK) || (ercd == E_OPNED))
	{
		EMBARC_PRINTF("I2C0 = Start\r\n");
	}
	reset();
	board_delay_ms(100, 1);
	readCalibration();
}
/*
int32_t	getPressure(){

	//getTemperature(); 		//updates temperature _dT and _T
	uint32_t D1 = getRawPressure();
	
	int64_t OFF  = (int64_t)_C[2-1]*65536 
				 + (int64_t)_C[4-1]*_dT/128;
	
	int64_t SENS = (int64_t)_C[1-1]*32768 
				 + (int64_t)_C[3-1]*_dT/256;
	_P = (D1*SENS/2097152 - OFF)/32768;
	//printf("P=%d\r\n", _P);
	return _P;
}
*/
void getRawPressure_cmd()
{

	int32_t ercd = E_PAR;

	ercd = writeByteMS(ADD_MS5611, CMD_CONV_D1_BASE + OSR * CONV_REG_SIZE); //read sensor, prepare a data, get ready for reading the data

	return;
}

uint32_t getRawPressure_read()
{

	int32_t ercd = E_PAR;
	uint8_t data[NBYTES_CONV];
	uint32_t data_con;

	ercd = readBytesMS(ADD_MS5611, CMD_ADC_READ, NBYTES_CONV, data); //reading the data
	//printf("P: data[0]=%d, data[1]=%d, data[2]=%d\r\n", data[0], data[1], data[2]);
	data_con = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2]); // concantenate bytes

	// calculate real pressure
	uint32_t D1 = data_con;

	int64_t OFF = (int64_t)_C[2 - 1] * 65536 + (int64_t)_C[4 - 1] * _dT / 128;

	int64_t SENS = (int64_t)_C[1 - 1] * 32768 + (int64_t)_C[3 - 1] * _dT / 256;
	_P = (D1 * SENS / 2097152 - OFF) / 32768;

	return _P;
}
/*
int32_t getTemperature(){
	// Code below can be uncommented for slight speedup:
	// NOTE: Be sure what you do! Notice that Delta 1C ~= Delta 2hPa
	//**********************************
	// if(abs(millis()-_lastTime)<T_THR)
	// 	return _T;
	//_lastTime = millis();
	//**********************************

	uint32_t D2;

	D2 = getRawTemperature();
	_dT = D2-((uint32_t)_C[5-1] * 256); 		//update '_dT'
	// Below, 'dT' and '_C[6-1]'' must be casted in order to prevent overflow
	// A bitwise division can not be dobe since it is unpredictible for signed integers
	_T = 2000 + ((int64_t)_dT * _C[6-1])/8388608;
	//printf("T= %ld\r\n", _T);
	return _T;
}*/

void getRawTemperature_cmd()
{

	int32_t ercd = E_PAR;

	ercd = writeByteMS(ADD_MS5611, CMD_CONV_D2_BASE + OSR * CONV_REG_SIZE);
}

uint32_t getRawTemperature_read()
{

	int32_t ercd = E_PAR;
	uint8_t data[NBYTES_CONV];
	uint32_t data_con, D2;

	ercd = readBytesMS(ADD_MS5611, CMD_ADC_READ, NBYTES_CONV, data);
	//printf("T: data[0]=%d, data[1]=%d, data[2]=%d\r\n", data[0], data[1], data[2]);
	data_con = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2]); // concantenate bytes
	D2 = data_con;

	// calculate real temperature
	_dT = D2 - ((uint32_t)_C[5 - 1] * 256); //update '_dT'
	// Below, 'dT' and '_C[6-1]'' must be casted in order to prevent overflow
	// A bitwise division can not be dobe since it is unpredictible for signed integers
	_T = 2000 + ((int64_t)_dT * _C[6 - 1]) / 8388608;
	//printf("T= %ld\r\n", _T);
	return _T;
}

void readCalibration()
{

	int32_t ercd = E_PAR;
	uint8_t data[NBYTES_PROM];
	uint32_t data_con;

	for (uint8_t k = 0; k < 6; k++)
	{
		ercd = readBytesMS(ADD_MS5611, CMD_PROM_READ_BASE + k * 2, NBYTES_PROM, data);
		//printf("data[0]=%d, data[1]=%d\r\n", data[0], data[1]);
		data_con = ((uint32_t)data[0] << 8) | ((uint32_t)data[1]);
		_C[k] = (uint16_t)(data_con & 0xFFFF); //masking out two LSB
											   //printf("C%d=%ld\r\n", k+1, _C[k]);
	}
}

void getCalibration(uint16_t *C)
{

	for (uint8_t k = 0; k < N_PROM_PARAMS; k++)
		C[k] = _C[k];

	return;
}

void reset()
{

	int32_t ercd = E_PAR;
	ercd = writeByteMS(ADD_MS5611, CMD_RESET);

	return;
}