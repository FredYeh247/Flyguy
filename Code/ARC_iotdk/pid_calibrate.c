#include "embARC.h"
#include "embARC_debug.h"
#include "mpu9250.h"
#include "stdio.h"
#include "pwm.h"
#include "PID.h"
#include "pid_calibrate.h"
#include <math.h>

// #define calibrate 0.54
#define pi 3.141593
//#define calibrate 0.265
#define calibrate_num 0
#define pid_limit 600.0
#define pid_height_limit 150.0
#define pid_acc_limit 100.0
static float cal_error = 0;
static uint64_t pre_time = 0; // record golbal previous time
static uint64_t time_1 = 0;	  // record golbal time

void yaw_calibrate(float *error, uint64_t *time, uint64_t *pre)
{
	static uint64_t cnt = 0;
	//yaw_calibrate
	/*extern uint64_t real_timer_s(const uint64_t TIMER);
	time_1 = real_timer_s(TIMER_1);*/
	cnt++;
	if (/*pre != (*time) ||*/ cnt == 1000)
	{
		//EMBARC_PRINTF("calibrate\r\n");
		*error += calibrate_num;
		*pre = *time;
		cnt = 0;
	}

	return;
}

void calibrate_initial_parameter(calibrate_t *calib, MPU9250_DEF_PTR mpu9250_sensor)
{
	calib->pid_ret = 0; // 1: set unsuccessful; 0: set successful
	//loiter_parameter->mpu9250_data = { 0 };
	calib->kp_p = 15;
	calib->ki_p = 0;	//0.05;
	calib->kd_p = 0.22; //0.005;

	calib->kp_r = 12.5; //6.7; //3;
	calib->ki_r = 0;	//3.5;
	calib->kd_r = 0.22; //0.0035; //7.4; //2.4;

	calib->kp_y = 12;
	calib->ki_y = 0;
	calib->kd_y = 0.0035;

	calib->kp_height = 8;
	calib->ki_height = 0;
	calib->kd_height = 0.22;

	calib->kp_acc = 5;
	calib->ki_acc = 0;
	calib->kd_acc = 0.22;

	calib->integral_limit = 200;
	calib->output_limit = 600;

	calib->pitch_pid = 0;
	calib->roll_pid = 0;
	calib->yaw_pid = 0;
	calib->height_pid = 0;

	calib->speed_pitch = 0;
	calib->speed_roll = 0;
	calib->speed_yaw = 0;
	calib->speed_height = 0;

	calib->pre_error = 0;

	calib->pin_motor1 = 0;
	calib->pin_motor2 = 1;
	calib->pin_motor3 = 2;
	calib->pin_motor4 = 3;

	/*
    * 板子左右為pitch(板子原始方向)，yaw順時針轉為負，安裝時請以字母A朝前方為標準，
	* 左上為1號馬達，右上為2號，左下3號，右下4號(A朝前方為標準)
    */
	calib->speed_motor1 = 0;
	calib->speed_motor2 = 0;
	calib->speed_motor3 = 0;
	calib->speed_motor4 = 0;

	calib->calibrate_max_speed = 15; //馬達引擎動力差異上限

	calib->motor_speed = 0;

	calib->pre_p = 0;
	calib->pre_r = 0;
	calib->pre_y = 0;
	calib->pre_height = 0;

	calib->d_p = 0;
	calib->d_r = 0;
	calib->d_y = 0;
	calib->d_height = 0;
	calib->dt = 0.001; // define dt = 1 ms

	calib->stage = NOT_BALANCE;
	//calib->motor_initial_speed = initial_speed; //維持高度速度，待校正

	pid_init(&calib->pid_p, PID_MODE_DERIVATIV_CALC, calib->dt);
	pid_init(&calib->pid_r, PID_MODE_DERIVATIV_CALC, calib->dt);
	pid_init(&calib->pid_y, PID_MODE_DERIVATIV_CALC, calib->dt);
	pid_init(&calib->pid_height, PID_MODE_DERIVATIV_CALC, calib->dt);

	calib->pid_ret = pid_set_parameters(&(calib->pid_p), calib->kp_p, calib->ki_p, calib->kd_p, calib->integral_limit, calib->output_limit);

	if (calib->pid_ret)
	{
		EMBARC_PRINTF("Pid setting Failed");
	}
	else
	{
		EMBARC_PRINTF("Pid setting successful");
	}
	calib->pid_ret = pid_set_parameters(&calib->pid_r, calib->kp_r, calib->ki_r, calib->kd_r, calib->integral_limit, calib->output_limit);
	calib->pid_ret = pid_set_parameters(&calib->pid_y, calib->kp_y, calib->ki_y, calib->kd_y, calib->integral_limit, calib->output_limit);
	calib->pid_ret = pid_set_parameters(&calib->pid_height, calib->kp_height, calib->ki_height, calib->kd_height, calib->integral_limit, calib->output_limit);
	calib->pid_ret = pid_set_parameters(&calib->pid_acc, calib->kp_acc, calib->ki_acc, calib->kd_acc, calib->integral_limit, calib->output_limit);
	EMBARC_PRINTF("\r\n\r\n\r\n");
	EMBARC_PRINTF("Pitch and Roll calibrating is about to start in 3 second\n");

	for (int i = 0; i < 3; i++)
	{
		printf("%d\n", 3 - i);
		for (int k = 0; k < 10; k++)
		{
			board_delay_ms(100, 1);
		}
	}

	//mpu9250_sensor_init(mpu9250_sensor);
	/*	for (int i=0; i<6; i++)
	{
		printf("Calibrating! Don't move the drone :%d s\n",3-i);
        for (int k=0; k<10; k++)
		{
          board_delay_ms(100,1);
		}	
	}
*/
	MPU9250_DATA mpu9250_data = {0};

	EMBARC_PRINTF("\r\n\r\n\r\n");
	int jump = 0;
	for (int i = 0; jump == 0; i++)
	{
		mpu9250_sensor_read(mpu9250_sensor, &mpu9250_data);

		char datap[10];
		char datar[10];
		char datay[10];

		sprintf(datap, "%06.1f", mpu9250_data.pitch);
		sprintf(datar, "%06.1f", mpu9250_data.roll);
		sprintf(datay, "%06.1f", mpu9250_data.yaw);

		//printf("dmp:  pitch=%.3f,  roll=%.3f,  yaw=%.3f \r\n", mpu9250_data.pitch, mpu9250_data.roll, mpu9250_data.yaw);
		printf("dmp:  pitch=%s,  roll=%s,  yaw=%s \r\n", datap, datar, datay);
		board_delay_ms(10, 1);
		//EMBARC_PRINTF("\x1b[2k\x1b\x45");
		if (mpu9250_data.pitch > 10 || mpu9250_data.pitch < -10 || mpu9250_data.roll > 10 || mpu9250_data.roll < -10)
		{
			jump = 0;
		}
		else if (i > 1)
		{
			jump = 1;
		}
		EMBARC_PRINTF("\x1b[2k\x1b\x45");
	}

	calib->goal_pitch_error = mpu9250_data.pitch;
	calib->goal_roll_error = mpu9250_data.roll;

	/*
	extern void yaw_error_count_start(void);
    yaw_error_count_start();	//clear and restart the clock
	*/
	arduino_pin_init();
}
void calibrate_set_goal_height(calibrate_t *calib, float distance)
{
	calib->goal_height = distance;
}
void calibrate_set_goal_acc(calibrate_t *calib, float acc)
{
	calib->goal_acc = acc;
}
void calibrate_set_goal(calibrate_t *calib, float goal_pitch_in, float goal_roll_in, float goal_yaw_in, int initial_speed)
{
	calib->goal_pitch = goal_pitch_in + calib->goal_pitch_error;
	calib->goal_roll = goal_roll_in + calib->goal_roll_error;
	calib->goal_yaw = goal_yaw_in;

	calib->motor_speed = initial_speed;
}

static int data_idx = 0;

void calibrating(calibrate_t *calib)
{
	// printf("%06.1f\n", calib->mpu9250_data.pitch);
	// printf("%06.1f\n", calib->mpu9250_data.roll);
	// sprintf("%06.1f", mpu9250_data.yaw);
	static int i = 0;

	calib->d_p = calib->mpu9250_data.gyro_y * 2000.0 / 32768; //get derivative part from mpu9250
	calib->d_r = calib->mpu9250_data.gyro_x * 2000.0 / 32768;
	calib->d_y = calib->mpu9250_data.gyro_z * 2000.0 / 32768;
	calib->d_height = calib->distance;
	calib->d_acc = calib->acc;
	// printf("pitch = %f\n", calib->mpu9250_data.gyro_y);
	// printf("roll = %f\n", calib->mpu9250_data.gyro_x);
	// printf("yaw = %06.1f\n", calib->mpu9250_data.gyro_z);

	calib->pitch_pid = pid_calculate(&calib->pid_p, calib->goal_pitch, calib->d_p, 0, calib->dt);
	// calib->pitch_pid /= cos(calib->mpu9250_data.pitch / 180 * pi);
	if (calib->pitch_pid > pid_limit)
	{
		calib->pitch_pid = pid_limit;
	}
	calib->roll_pid = pid_calculate(&calib->pid_r, calib->goal_roll, calib->d_r, 0, calib->dt);
	// calib->roll_pid /= cos(calib->mpu9250_data.roll / 180 * pi);
	if (calib->roll_pid > pid_limit)
	{
		calib->roll_pid = pid_limit;
	}
	calib->yaw_pid = pid_calculate(&calib->pid_y, calib->goal_yaw, calib->d_y, 0, calib->dt);
	if (calib->yaw_pid > pid_limit)
	{
		calib->yaw_pid = pid_limit;
	}
	if (calib->hover_mode)
	{
		calib->height_pid = height_pid_calculate(&calib->pid_height, calib->goal_height, calib->d_height, 0, calib->dt);
		if (calib->height_pid > pid_height_limit)
		{
			calib->height_pid = pid_height_limit;
		}
		else if (calib->height_pid < -pid_height_limit)
		{
			calib->height_pid = -pid_height_limit;
		}

		calib->acc_pid = acc_pid_calculate(&calib->pid_acc, calib->goal_acc, calib->d_acc, 0, calib->dt);
		if (calib->acc_pid > pid_acc_limit)
		{
			calib->acc_pid = pid_acc_limit;
		}
		else if (calib->acc_pid < -pid_acc_limit)
		{
			calib->acc_pid = -pid_acc_limit;
		}
	}

	// only for debug
	// printf("pitch %f\r\n", calib->mpu9250_data.pitch);
	// printf("roll %f\r\n", calib->mpu9250_data.roll);
	// printf("yaw %f\r\n", calib->mpu9250_data.yaw);
	// printf("yaw pid %f\r\n", calib->yaw_pid);
	// record debug data
	if (i % 5 == 0 && data_idx < 4000)
	{
		raw_data[data_idx].pitch_dump = calib->mpu9250_data.pitch;

		raw_data[data_idx].pitch_pid = calib->pitch_pid;

		data_idx++;
	}
	// -------------------------------------------------------------------------------

	calib->speed_pitch = calib->pitch_pid; // * calib->calibrate_max_speed / calib->output_limit;
	calib->speed_roll = calib->roll_pid;   //* calib->calibrate_max_speed / calib->output_limit;
	calib->speed_yaw = calib->yaw_pid;	   //* calib->calibrate_max_speed / calib->output_limit;
	calib->speed_height = calib->height_pid;
	calib->speed_acc = calib->acc_pid;
	if (calib->hover_mode)
	{

		calib->speed_motor1 = calib->motor_speed + calib->speed_pitch + calib->speed_roll - calib->speed_yaw + calib->speed_height + calib->speed_acc;
		calib->speed_motor2 = calib->motor_speed - calib->speed_pitch - calib->speed_roll - calib->speed_yaw + calib->speed_height + calib->speed_acc;
		calib->speed_motor3 = calib->motor_speed + calib->speed_pitch - calib->speed_roll + calib->speed_yaw + calib->speed_height + calib->speed_acc;
		calib->speed_motor4 = calib->motor_speed - calib->speed_pitch + calib->speed_roll + calib->speed_yaw + calib->speed_height + calib->speed_acc;
		// // calib->speed_motor1 = calib->motor_speed + calib->speed_height;
		// // calib->speed_motor2 = calib->motor_speed + calib->speed_height;
		// // calib->speed_motor3 = calib->motor_speed + calib->speed_height;
		// // calib->speed_motor4 = calib->motor_speed + calib->speed_height;
		// calib->speed_motor1 = calib->motor_speed + calib->speed_acc;
		// calib->speed_motor2 = calib->motor_speed + calib->speed_acc;
		// calib->speed_motor3 = calib->motor_speed + calib->speed_acc;
		// calib->speed_motor4 = calib->motor_speed + calib->speed_acc;
		// printf("target_distance=%f\n",calib->goal_height);
		// printf("distance_now=%f\n",calib->distance);
		// printf("calib->speed_height = %4.1f\n", calib->speed_height);
		// printf("calib->speed_acc = %4.1f\n", calib->speed_acc);
		//	printf("%f\n",calib->mpu9250_data.gyro_y * 2000.0 / 32768);
	}
	else
	{

		calib->speed_motor1 = calib->motor_speed + calib->speed_pitch + calib->speed_roll - calib->speed_yaw;
		calib->speed_motor2 = calib->motor_speed - calib->speed_pitch - calib->speed_roll - calib->speed_yaw;
		calib->speed_motor3 = calib->motor_speed + calib->speed_pitch - calib->speed_roll + calib->speed_yaw;
		calib->speed_motor4 = calib->motor_speed - calib->speed_pitch + calib->speed_roll + calib->speed_yaw;
		// calib->speed_motor1 = calib->motor_speed + calib->speed_roll;
		// calib->speed_motor2 = calib->motor_speed - calib->speed_roll;
		// calib->speed_motor3 = calib->motor_speed - calib->speed_roll;
		// calib->speed_motor4 = calib->motor_speed + calib->speed_roll;
	}
	// printf("roll motor = %f\n ",calib->speed_roll);
	// printf("pitch motor = %f\n ", calib->speed_pitch);
	// printf("********\n");
	// // printf("motor_speed = %d\n",calib->motor_speed);
	// printf("motor_1 = %f\n", calib->speed_motor1);
	// printf("motor_2 = %f\n", calib->speed_motor2);
	// printf("motor_3 = %f\n", calib->speed_motor3);
	// printf("motor_4 = %f\n", calib->speed_motor4);
	if (calib->speed_motor1 >= 720)
		pwm_set(calib->pin_motor1, calib->speed_motor1);
	else
		pwm_set(calib->pin_motor1, 720);

	if (calib->speed_motor2 >= 720)
		pwm_set(calib->pin_motor2, calib->speed_motor2);
	else
		pwm_set(calib->pin_motor2, 720);

	if (calib->speed_motor3 >= 720)
		pwm_set(calib->pin_motor3, calib->speed_motor3);
	else
		pwm_set(calib->pin_motor3, 720);

	if (calib->speed_motor4 >= 720)
		pwm_set(calib->pin_motor4, calib->speed_motor4);
	else
		pwm_set(calib->pin_motor4, 720);

	i++;
	calib->pre_p = calib->mpu9250_data.pitch;
	calib->pre_r = calib->mpu9250_data.roll;
	calib->pre_y = calib->mpu9250_data.yaw;
	calib->pre_height = calib->distance;
}
