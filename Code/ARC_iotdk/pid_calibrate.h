#ifndef _pid_calibrate_H_
#define _pid_calibrate_H_

#include "PID.h"
#include "mpu9250.h"

typedef enum
{
	NOT_BALANCE,
	BALANCE
} calib_stage_t;

typedef struct
{

	bool hover_mode;

	float distance;
	float acc;
	int pid_ret; // 1: set unsuccessful; 0: set successful
	PID_t pid_p;
	PID_t pid_r;
	PID_t pid_y;
	PID_t pid_height;
	PID_t pid_acc;

	MPU9250_DATA mpu9250_data;
	float kp_p;
	float ki_p;
	float kd_p;
	float kp_r;
	float ki_r;
	float kd_r;
	float kp_y;
	float ki_y;
	float kd_y;
	float kp_height;
	float ki_height;
	float kd_height;
	float kp_acc;
	float ki_acc;
	float kd_acc;
	float integral_limit;
	float output_limit;

	float pitch_pid;
	float roll_pid;
	float yaw_pid;
	float height_pid;
	float acc_pid;

	float speed_pitch;
	float speed_roll;
	float speed_yaw;
	float speed_height;
	float speed_acc;

	int pin_motor1; //counterclockwise
	int pin_motor2; //clockwise
	int pin_motor3; //clockwise
	int pin_motor4; //counterclockwise

	float speed_motor1;
	float speed_motor2;
	float speed_motor3;
	float speed_motor4;

	float goal_pitch_error; //pitch starting point
	float goal_roll_error;
	float goal_height_error;
	float goal_acc_error;

	float goal_pitch;
	float goal_roll;
	float goal_yaw;
	float goal_height;
	float goal_acc;

	int calibrate_max_speed;

	int motor_initial_speed;

	int motor_speed;

	float pre_error;

	int i_1[10];
	int t;
	float pre_p; //previous PID data
	float pre_r;
	float pre_y;
	float pre_height;
	float pre_acc;

	float d_p;
	float d_r;
	float d_y;
	float d_height;
	float d_acc;

	float dt;
	calib_stage_t stage;
} calibrate_t;
void calibrate_set_goal_acc(calibrate_t *calib, float acc);
void calibrate_set_goal_height(calibrate_t *calib, float distance);
void calibrate_initial_parameter(calibrate_t *calib, MPU9250_DEF_PTR mpu9250_sensor);
void calibrate_set_goal(calibrate_t *calib, float goal_pitch_in, float goal_roll_in, float goal_yaw_in, int initial_speed);
void calibrating(calibrate_t *calib);
void yaw_calibrate(float *error, uint64_t *time, uint64_t *pre);

typedef struct pid_data
{
	float roll_dump;
	float pitch_dump;
	float roll_pid;
	float pitch_pid;
} pid_data;

pid_data raw_data[4000];

#endif
