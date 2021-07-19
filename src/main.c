#include "embARC.h"
#include "embARC_debug.h"
#include "mpu9250.h"
#include "stdio.h"
#include "LAB815_MPU9250.h"
#include "PID.h"
#include "pid_calibrate.h"
#include "pwm.h"
#include "dev_gpio.h"
#include "MS5611.h"
#include "dev_uart.h"

#define MAX_COUNT 0xffffffff
#define acc_balance 16650.0
#define loopTime_1 1000
#define loopTime 300000

#if defined(BOARD_IOTDK) || defined(BOARD_EMSDP)
#define MPU9250_IIC_ID DFSS_IIC_0_ID
#else
#define MPU9250_IIC_ID 0
#endif

typedef enum
{
	throttle_enable,
	throttle_disable
} throttle_state_t;

#define highest_altitude 14
#define balance_throttle 1900

float ground_elevation;

//-------------------for controler-----------------------
#define GPIO4B2_1_OFFSET 1
#define GPIO4B2_0_OFFSET 0
#define GPIO4B1_1_OFFSET 1
#define GPIO4B1_0_OFFSET 0

#define GPIO4B1_3_OFFSET 3
#define GPIO4B1_2_OFFSET 2
#define GPIO8B2_0_OFFSET 0
#define GPIO8B2_3_OFFSET 3
#define GPIO8B2_4_OFFSET 4
#define GPIO8B3_0_OFFSET 0
#define GPIO8B3_1_OFFSET 1

//-------------------for remote control channel------------------
//model A (解鎖請內八)
#define sbus_value_max 1750.0
#define sbus_value_mid 1050.0
#define sbus_value_min 350.0
#define sbus_range 1450.0
#define sbus_th 50.0

#define sbus_value_HU 1100.0
#define sbus_value_HL 1000.0

#define throttle_idle 900.0
#define throttle_max 1900.0
#define throttle_range (throttle_max - throttle_idle)

#define sbus_value_range (sbus_value_max - sbus_value_min)

#define roll_range 20.0
#define pitch_range 20.0

#define remote_channel_1 0	//yaw
#define remote_channel_2 1	//pitch
#define remote_channel_3 2	//throttle
#define remote_channel_4 3	//roll
#define remote_channel_5 4	//C
#define remote_channel_6 5	//left knob
#define remote_channel_7 6	//A
#define remote_channel_8 7	//B
#define remote_channel_9 8	//E
#define remote_channel_10 9 //G

//model B  (解鎖請外八)
/*
#define remote_channel_1    3  //yaw
#define remote_channel_2    1  //pitch
#define remote_channel_3    2  //throttle
#define remote_channel_4    0  //roll
#define remote_channel_5    4  //C
#define remote_channel_6    5  //left knob
#define remote_channel_7    6  //A
#define remote_channel_8    7  //B
#define remote_channel_9    8  //E
#define remote_channel_10   9  //G
*/

DEV_GPIO_PTR gpio_4b1;
DEV_GPIO_PTR gpio_4b2;
DEV_GPIO_PTR gpio_8b2;
DEV_GPIO_PTR gpio_8b3;
volatile uint16_t sbus_buf[25] = {0};
volatile uint16_t sbus_low_cnt = 0;
volatile uint16_t sbus_data_last = 0;
volatile uint16_t sbus_data_now = 0;
volatile uint8_t sbus_flag = 0;
volatile uint8_t sbus_stage = 0;
volatile uint8_t sbus_buf_cnt = 0;
volatile uint8_t sbus_data_cnt = 0;
volatile uint8_t sbus_i = 0;

volatile uint8_t gpio_stage = 0;

uint16_t channel_value[20] = {0};

volatile bool remote_control = true;

// decode the subus, definition is below the main function
int32_t SBUS_DECODE(uint16_t *channel_value);

// sbus comparsion
void sbus_compare(void)
{
	static uint16_t pre_ch[20] = {0};
	int flag = true;
	static int cnt = 0;

	for (int i = 0; i < 20 && flag; i++)
	{
		if (pre_ch[i] != channel_value[i])
		{
			flag = false;
		}
	}

	if (flag == true)
	{
		cnt++;
	}
	else
	{
		remote_control = true;
		cnt = 0;
	}

	if (cnt == 1000)
	{
		EMBARC_PRINTF("SBUS SAME\r\n");
		remote_control = false;
		cnt = 0;
	}

	for (int i = 0; i < 20; i++)
	{
		pre_ch[i] = channel_value[i];
	}

	return;
}

void gpio_init(void)
{
	gpio_4b1 = gpio_get_dev(DFSS_GPIO_4B1_ID); // get GPIO_4B1 handler
	gpio_4b2 = gpio_get_dev(DFSS_GPIO_4B2_ID); // get GPIO_4B1 handler
	gpio_8b3 = gpio_get_dev(DFSS_GPIO_8B3_ID); // get GPIO_4B1 handler

	gpio_4b1->gpio_open(1 << GPIO4B1_0_OFFSET); // open
	gpio_4b1->gpio_open(1 << GPIO4B1_1_OFFSET); // open
	gpio_4b2->gpio_open(1 << GPIO4B2_0_OFFSET); // open
	gpio_4b2->gpio_open(1 << GPIO4B2_1_OFFSET); // open
	gpio_8b3->gpio_open(1 << GPIO8B3_0_OFFSET);
	gpio_8b3->gpio_open(1 << GPIO8B3_1_OFFSET);

	gpio_4b1->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(1 << GPIO4B1_0_OFFSET)); // set dir
	gpio_4b1->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT, (void *)(1 << GPIO4B1_1_OFFSET));  // set GPIO4B1_1_OFFSET dir to input.
	gpio_4b2->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(1 << GPIO4B2_0_OFFSET)); // set dir
	gpio_4b2->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT, (void *)(1 << GPIO4B2_1_OFFSET));

	gpio_8b3->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT, (void *)(1 << GPIO8B3_0_OFFSET)); // set dir
	gpio_8b3->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT, (void *)(1 << GPIO8B3_1_OFFSET));

	gpio_4b1->gpio_write(0 << GPIO4B1_0_OFFSET, 1 << GPIO4B1_0_OFFSET); // GPIO4B1_0_OFFSET set low output
	gpio_4b2->gpio_write(0 << GPIO4B2_0_OFFSET, 1 << GPIO4B2_0_OFFSET); // GPIO4B2_0_OFFSET set low output
}
void ultrasonic_init(void)
{
	gpio_4b1 = gpio_get_dev(DFSS_GPIO_4B1_ID);											  // get GPIO_8B0 handler
	gpio_4b1->gpio_open(1 << GPIO4B1_3_OFFSET);											  // open
	gpio_4b1->gpio_open(1 << GPIO4B1_2_OFFSET);											  // open
	gpio_4b1->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(1 << GPIO4B1_2_OFFSET)); // set dir
	gpio_4b1->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT, (void *)(1 << GPIO4B1_3_OFFSET));  // set GPIO4B2_1_OFFSET dir to input.

	return;
}
void activate_button(void)
{
	uint16_t in_signal;

	do
	{
		gpio_4b2->gpio_read(&in_signal, 1 << GPIO4B2_1_OFFSET);
		EMBARC_PRINTF("%d\r\n", in_signal >> GPIO4B2_1_OFFSET);
		board_delay_ms(10, 1);
	} while ((in_signal >> GPIO4B2_1_OFFSET));
}

uint32_t check_flag = 0;
uint32_t distance = 0;
uint32_t distance_final = 0;
uint32_t value_read = 0;
uint32_t value_last = 0;
void ultrasonic(int cnt)
{

	if (cnt < 5)
		gpio_4b1->gpio_write(0 << GPIO4B1_2_OFFSET, 1 << GPIO4B1_2_OFFSET);
	else if (cnt < 40)
		gpio_4b1->gpio_write(1 << GPIO4B1_2_OFFSET, 1 << GPIO4B1_2_OFFSET);
	else
		gpio_4b1->gpio_write(0 << GPIO4B1_2_OFFSET, 1 << GPIO4B1_2_OFFSET);

	gpio_4b1->gpio_read(&value_read, 1 << GPIO4B1_3_OFFSET);
	if (value_read != value_last)
	{
		check_flag++;
	}

	if (check_flag == 1)
	{
		distance++;
	}
	else if (check_flag == 0)
	{
		distance = 0;
	}
	else if (check_flag == 2)
	{
		distance_final = distance;

		check_flag = 0;
	}
	value_last = value_read;
}
int value_read_gpio13;
int value_read_gpio12;
int value_last_gpio13;
int value_last_gpio12;
int gpio_flag;
int check_flag_gpio = 0;
int counter_gpio = 0;
void getgpio(void)
{

	gpio_8b3->gpio_read(&value_read_gpio12, 1 << GPIO8B3_0_OFFSET);
	gpio_8b3->gpio_read(&value_read_gpio13, 1 << GPIO8B3_1_OFFSET);
	if (value_last_gpio13 != value_read_gpio13)
	{
		check_flag_gpio++;
	}

	if (check_flag_gpio == 2)
		check_flag_gpio = 0;
	value_last_gpio12 = value_read_gpio12;
	value_last_gpio13 = value_read_gpio13;
}

int32_t ultra_cnt;

/** arc timer 1 interrupt routine */
static void timer1_isr_control(void *ptr)
{
	timer_int_clear(TIMER_1);
	if (ultra_cnt == 70)
		ultra_cnt = 0;
	ultra_cnt++;
	getgpio();
	ultrasonic(ultra_cnt);

	//EMBARC_PRINTF("timer 0 interrupt\r\n");
	gpio_4b1->gpio_write(gpio_stage << GPIO4B1_0_OFFSET, 1 << GPIO4B1_0_OFFSET); // GPIO4B2_0_OFFSET set low output.
	if (gpio_stage == 0)
		gpio_stage = 1;
	else
		gpio_stage = 0;

	gpio_4b1->gpio_read(&sbus_data_now, 1 << GPIO4B1_1_OFFSET);
	sbus_data_now = sbus_data_now >> GPIO4B1_1_OFFSET;

	//EMBARC_PRINTF("flag: %d\r\n", sbus_flag);

	switch (sbus_stage)
	{
	case 0:
		if ((sbus_data_now == 1) && (sbus_flag == 0))
		{
			sbus_stage = 1;
			sbus_buf_cnt = 0;
			sbus_data_cnt = 1;

			for (sbus_i = 0; sbus_i < 25; sbus_i++)
				sbus_buf[sbus_i] = 0;
			sbus_buf[sbus_buf_cnt] = sbus_buf[sbus_buf_cnt] | (sbus_data_now << 11);
		}
		else if ((sbus_data_now == 1) && (sbus_flag == 1))
		{
			sbus_stage = 2;
		}

		break;
	case 1:
		sbus_buf[sbus_buf_cnt] = sbus_buf[sbus_buf_cnt] | (sbus_data_now << (11 - sbus_data_cnt));
		sbus_data_cnt++;
		if (sbus_data_cnt >= 12)
		{
			sbus_data_cnt = 0;
			sbus_buf_cnt++;
			if (sbus_buf_cnt >= 25)
			{
				sbus_flag = 1;
				sbus_stage = 0;
			}
		}
		break;
	case 2:
		if ((sbus_data_last == 0) && (sbus_data_now == 0))
		{
			sbus_low_cnt++;
			if (sbus_low_cnt >= 100)
			{
				sbus_low_cnt = 0;
				sbus_stage = 0;
			}
		}
		else
		{
			sbus_low_cnt = 0;
		}

		break;

	default:

		break;
	}
	sbus_data_last = sbus_data_now;
}

// for the controler
void timer_1_init(void)
{
	unsigned int int_bcr;
	if (timer_present(TIMER_1))
	{
		timer_stop(TIMER_1); /* Stop it first since it might be enabled before */
		int_handler_install(INTNO_TIMER1, timer1_isr_control);
		/* to enable fiq, interrupt priority must be the highest */
		int_pri_set(INTNO_TIMER1, INT_PRI_MIN);
		int_enable(INTNO_TIMER1);
		timer_start(TIMER_1, TIMER_CTRL_IE, BOARD_CPU_CLOCK / 100150);
	}
	else
	{
		EMBARC_PRINTF("timer 1 is not present\r\n");
	}

	EMBARC_PRINTF("---control initialize finished----\r\n");
}

// ------------------------------------------------------

// define a mpu9250 object
static MPU9250_DEFINE(mpu9250_sensor, MPU9250_IIC_ID, MPU9250_IIC_ADDRESS);

// parameter for keep calibrate
calibrate_t calibrate_parameter;

void yaw_error_count_start(void)
{
	timer_stop(TIMER_1);
	timer_int_clear(TIMER_1);
	timer_start(TIMER_1, TIMER_CTRL_NH, MAX_COUNT);
}

uint64_t real_timer_s(const uint32_t TIMER)
{
	uint64_t val;
	static uint64_t pre_val;
	static uint64_t start = 0;

	timer_current(TIMER, &val);

	val /= 144000000;

	return val;
}

void pwm_init(void)
{
	uint8_t pwm_cnt = 0;

	EMBARC_PRINTF("calibrate!\r\n");
	pwm_set(0, 0);
	pwm_set(1, 0);
	pwm_set(2, 0);
	pwm_set(3, 0);

	for (int i = 0; i < 10; i++)
		board_delay_ms(100, 1);

	pwm_set(0, 720);
	pwm_set(1, 720);
	pwm_set(2, 720);
	pwm_set(3, 720);

	for (int i = 0; i < 50; i++)
		board_delay_ms(100, 1);

	EMBARC_PRINTF("done!\r\n");
}

void calibrate_init(calibrate_t *calib, float goal_pitch_in, float goal_roll_in, float goal_yaw_in, int initial_speed)
{
	calibrate_initial_parameter(calib, mpu9250_sensor);
	EMBARC_PRINTF("release!!\r\n");
	//board_delay_ms(5000, 1);
	calibrate_set_goal(calib, goal_pitch_in, goal_roll_in, goal_yaw_in, initial_speed);
	EMBARC_PRINTF("set finish!!\r\n");
}

void loiter_wait_stable(int *flag)
{
	static int cnt = 0;
	static float pre_yaw = 0;
	MPU9250_DATA m_data;

	//m_data.yaw = 0;

	mpu9250_sensor_read(mpu9250_sensor, &m_data);
	if (cnt == 0)
	{
		pre_yaw = m_data.yaw;
		cnt++;
	}
	else
	{
		if ((pre_yaw - m_data.yaw) < 5 && (pre_yaw - m_data.yaw) > -5)
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		if (cnt >= 10)
		{
			*flag = 1;
		}
		printf("yaw: %f %f\r\n", pre_yaw, m_data.yaw);
		pre_yaw = m_data.yaw;
	}

	return;
}

void total_yaw_wait_stable(int *flag, float total_yaw)
{
	static int cnt = 0;
	static float pre_total_yaw = 0;

	if (cnt == 0)
	{
		pre_total_yaw = total_yaw;
		cnt++;
	}
	else
	{
		if (pre_total_yaw == total_yaw)
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		if (cnt >= 10)
		{
			*flag = 1;
		}
		printf("total_yaw: %f %f\r\n", pre_total_yaw, total_yaw);
		pre_total_yaw = total_yaw;
	}

	return;
}

void check_mpu_and_yaw(void)
{
	float cal_error = 0;
	uint64_t pre_time = 0; // record golbal previous time
	uint64_t time_1 = 0;   // record golbal time
	static int loiter_wait_flag = 0;
	static int total_yaw_wait_flag = 0;
	timer_stop(TIMER_1);
	timer_int_clear(TIMER_1);
	timer_start(TIMER_1, TIMER_CTRL_NH, MAX_COUNT);

	do
	{
		// yaw_calibrate(&cal_error, &time_1, &pre_time);
		loiter_wait_stable(&loiter_wait_flag);
		//total_yaw_wait_stable(&total_yaw_wait_flag, cal_error);
	} while (loiter_wait_flag == 0 /*|| total_yaw_wait_flag == 0*/);

	EMBARC_PRINTF("check finish!!\r\n");

	return;
}

void mpu_initilize(void)
{
	mpu9250_sensor_init(mpu9250_sensor);

	int32_t ercd = E_OK;
	mpu_i2c_ptr = iic_get_dev(0);
	ercd = mpu_i2c_ptr->iic_open(DEV_MASTER_MODE, IIC_SPEED_STANDARD);
	if ((ercd == E_OK) || (ercd == E_OPNED))
	{
		EMBARC_PRINTF("I2C2 = Start\r\n");
	}

	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, gx_offset >> 8);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, gx_offset);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, gy_offset >> 8);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, gy_offset);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, gz_offset >> 8);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, gz_offset);

	return;
}

static int cnt = 0;

static void timer0_isr(void *ptr)
{
	timer_int_clear(TIMER_0);
	calibrating(&calibrate_parameter);
	// EMBARC_PRINTF("timer0 interrupt with the highest priority(fiq if fiq enabled)\r\n");
	cnt++;
}

void timer0_init(void)
{
	uint32_t val;
	unsigned int int_bcr;

	EMBARC_PRINTF("ARC timer and interrupt\r\n");
	int_bcr = _arc_lr_reg(AUX_BCR_IRQ);
	EMBARC_PRINTF("interrupt number:%d, ", (unsigned char)(int_bcr >> 8));
	EMBARC_PRINTF("extern interrupts:%d, ", (unsigned char)(int_bcr >> 16));
	EMBARC_PRINTF("interrupt priorities:%d, ", (unsigned char)((int_bcr >> 24) & 0xf) + 1);

	if (int_bcr & 0x10000000)
	{
		EMBARC_PRINTF("fiq enabled\r\n");
	}
	else
	{
		EMBARC_PRINTF("fiq disabled\r\n");
	}

	// check timer0 whether works
	if (timer_present(TIMER_0))
	{
		EMBARC_PRINTF("timer 0 is present\r\n");
		timer_current(TIMER_0, &val);
		EMBARC_PRINTF("cnt:%d\r\n", val);
		timer_stop(TIMER_0); /* Stop it first since it might be enabled before */
		int_handler_install(INTNO_TIMER0, timer0_isr);
		/* to enable fiq, interrupt priority must be the highest */
		int_pri_set(INTNO_TIMER0, INT_PRI_MAX);
		int_enable(INTNO_TIMER0);
	}
	else
	{
		EMBARC_PRINTF("timer 0 is not present\r\n");
	}

	// start timer0 interrupt, it interrupt period is 1 ms

	if (timer_present(TIMER_0))
	{
		timer_int_clear(TIMER_0);
		timer_start(TIMER_0, TIMER_CTRL_IE, BOARD_CPU_CLOCK / 460);
	}
	else
	{
		EMBARC_PRINTF("timer 0 is not present\r\n");
	}
}

void sbus_control(void)
{
	static int32_t sbus_error_cnt = 0;
	static int32_t sbus_total = 0;

	// controler stage maintain
	if (sbus_flag == 1)
	{
		sbus_total++;
		if (SBUS_DECODE(&channel_value[0]) != 0)
		{
			sbus_error_cnt++;
		}

		sbus_flag = 0;
	}
}

void throttle_control(void)
{
	static PID_t pid_a;
	static float altitude_pid;

	float tmp;

	float kp_a = 100;
	float ki_a = 0;
	float kd_a = 0;
	float integral_limit = 100;
	float output_limit = 300;
	int ret;
	static int state = 0;
	static float goal_altitude;
	static float pre_altitude;
	float altitude_num = 0.002; // 30*0.0008*460=11.04 cm per second
	float altitude_calib_speed; // unit:30 cm
	float altitude_speed;		// unit:30 cm

	pid_init(&pid_a, PID_MODE_DERIVATIV_CALC, 0.001);
	ret = pid_set_parameters(&pid_a, kp_a, ki_a, kd_a, integral_limit, output_limit);
	// 油門測試

	// calibrate_parameter.motor_speed = (1750 - channel_value[remote_channel_3]) / 1450.0 * 700 + 1250;
	// //limit the upper bound & lower bound
	// if (calibrate_parameter.motor_speed < 900)
	// {
	// 	calibrate_parameter.motor_speed = 900;
	// }
	// else if (calibrate_parameter.motor_speed > 2000)
	// {
	// 	calibrate_parameter.motor_speed = 2000;
	// }

	tmp = sbus_value_max - channel_value[remote_channel_3];
	// printf("tmp = %f\n", tmp);
	// if (tmp < 100.0)
	// 	tmp = 0.0;
	// else
	// {
	// 	tmp = ((tmp - 100.0) * (tmp - 100) / 1350 / 1350);
	// }
	float pwm;
	if (tmp < sbus_th)
		pwm = 900.0;
	else
	{
		pwm = -(throttle_range / (sbus_th - sbus_range) / (sbus_th - sbus_range) / (sbus_th - sbus_range) * (tmp - sbus_range) * (tmp - sbus_range) * (tmp - sbus_range)) + throttle_max;
	}
	// tmp = tmp * throttle_max + throttle_idle + 50;
	// printf("pwm = %f\n", pwm);
	if (check_flag_gpio == 1)
	{
		calibrate_parameter.motor_speed = (int)(pwm) + 500;
	}
	else
		calibrate_parameter.motor_speed = (int)(pwm);

	// printf("calibrate_parameter.motor_speed = %d\n", calibrate_parameter.motor_speed);

	return;
}

/*
#define sbus_value_max 1750
#define sbus_value_mid 1050
#define sbus_value_min 350
#define sbus_value_range (sbus_value_max - sbus_value_min)

#define sbus_value_HU 1100
#define sbus_value_HL 1000

#define roLL_range 20
#define pitch_range 20
*/

// 設定 roll 的角度，linear transformation to angle
// roll angle range: -20 ~ 20 度
void roll_control(bool *flag, bool throttle_dis)
{
	float goal_roll_by_controler, tmp;
	float roll_kp = 2; //4;//2;
	float error = 0;

	if (channel_value[remote_channel_4] < sbus_value_HU && channel_value[remote_channel_4] > sbus_value_HL)
		tmp = 0;
	else if (channel_value[remote_channel_4] > sbus_value_HU)
		tmp = (channel_value[remote_channel_4] - sbus_value_HU) / (sbus_value_max - sbus_value_HU) * roll_range;
	else if (channel_value[remote_channel_4] < sbus_value_HL)
		tmp = (sbus_value_HL - channel_value[remote_channel_4]) / (sbus_value_HL - sbus_value_min) * -roll_range;

	if (tmp < -roll_range)
		tmp = -roll_range;
	else if (tmp > roll_range)
		tmp = roll_range;

	goal_roll_by_controler = tmp;

	if (!throttle_dis && remote_control)
	{
		error = goal_roll_by_controler - calibrate_parameter.mpu9250_data.roll;
	}
	else
	{
		error = 0 - calibrate_parameter.mpu9250_data.roll;
	}
	calibrate_parameter.goal_roll = roll_kp * error;

	if (error < 5 && error > -5)
	{
		*flag = true;
	}
	else
	{
		*flag = false;
	}

	return;
}

// 設定 pitch 的角度，linear transformation to angle
// pitch angle range: -15 ~ 15 度
void pitch_control(bool *flag, bool throttle_dis)
{
	float goal_pitch_by_controler, tmp;
	float pitch_kp = 3; //4.5;//2.25;
	float error = 0;

	if (channel_value[remote_channel_2] < sbus_value_HU && channel_value[remote_channel_2] > sbus_value_HL)
		tmp = 0;
	else if (channel_value[remote_channel_2] > sbus_value_HU)
		tmp = (channel_value[remote_channel_2] - sbus_value_HU) / (sbus_value_max - sbus_value_HU) * -pitch_range;
	else if (channel_value[remote_channel_2] < sbus_value_HL)
		tmp = (sbus_value_HL - channel_value[remote_channel_2]) / (sbus_value_HL - sbus_value_min) * pitch_range;

	if (tmp < -pitch_range)
		tmp = -pitch_range;
	else if (tmp > pitch_range)
		tmp = pitch_range;

	goal_pitch_by_controler = tmp;
	if (!throttle_dis && remote_control)
	{
		error = goal_pitch_by_controler - calibrate_parameter.mpu9250_data.pitch;
	}
	else
	{
		error = 0 - calibrate_parameter.mpu9250_data.pitch;
	}
	calibrate_parameter.goal_pitch = pitch_kp * error;

	if (error < 5 && error > -5)
	{
		*flag = true;
	}
	else
	{
		*flag = false;
	}

	return;
}

// 將 300 ~ 1750 的每一間隔設計為 0.08 度
// yaw遙控器最大和最小控制範圍：-0.08 ~ 0.08 度 --> 連續旋轉
void yaw_control(bool throttle_dis)
{
	float goal_yaw_by_controler, tmp;
	float yaw_kp = 1500; //750;

	tmp = (channel_value[remote_channel_1] - 300) / 1450.0 * 0.08 - 0.04;

	if (channel_value[remote_channel_1] < 1100 && channel_value[remote_channel_1] > 1000)
	{
		goal_yaw_by_controler = 0;
	}
	else
	{
		goal_yaw_by_controler = tmp;
	}

	if (!throttle_dis && remote_control)
	{
		calibrate_parameter.goal_yaw = yaw_kp * goal_yaw_by_controler;
	}
	else
	{
		calibrate_parameter.goal_yaw = 0;
	}

	return;
}

void calib_stage_handler(bool flag_p, bool flag_r)
{
	if (flag_p && flag_r)
	{
		calibrate_parameter.stage = BALANCE;
	}
	else
	{
		calibrate_parameter.stage = NOT_BALANCE;
	}
}

void check_throttle(bool *flag)
{
	static uint32_t cnt = 0;

	const uint32_t cnt_upper = 200;

	EMBARC_PRINTF("Channel [%d] = %d\r\n", remote_channel_1, channel_value[remote_channel_1]);
	EMBARC_PRINTF("Channel [%d] = %d\r\n", remote_channel_2, channel_value[remote_channel_2]);

	if (channel_value[remote_channel_1] > 1650 && channel_value[remote_channel_2] < 400 && channel_value[remote_channel_3] > 1650 && channel_value[remote_channel_4] < 400)
	{
		cnt++;
	}
	else
	{
		cnt = 0;
	}

	if (cnt == cnt_upper)
	{
		*flag = true;
	}

	return;
}

bool throttle_state_control(throttle_state_t *state)
{
	static uint32_t cnt = 0;

	if ((channel_value[remote_channel_1] > 1650 && channel_value[remote_channel_2] < 400 && channel_value[remote_channel_3] > 1650 && channel_value[remote_channel_4] < 400) || channel_value[remote_channel_9] < 400 || channel_value[remote_channel_9] > 1650)
	{
		cnt++;
	}
	else
	{
		cnt = 0;
	}

	if (cnt == 100)
	{
		if (channel_value[remote_channel_9] < 400)
		{
			*state = throttle_disable;
		}
		else if (channel_value[remote_channel_9] > 1650)
		{
			*state = throttle_enable;
		}
		else
		{
			if (*state == throttle_enable)
			{
				*state = throttle_disable;
			}
			else
			{
				*state = throttle_enable;
				board_delay_ms(3000, 1);
			}
		}
		cnt = 0;
	}

	if ((channel_value[remote_channel_1] > 1650 && channel_value[remote_channel_2] < 400 && channel_value[remote_channel_3] > 1650 && channel_value[remote_channel_4] < 400) || channel_value[remote_channel_9] < 400)
	{
		return true;
	}
	else if (channel_value[remote_channel_9] > 1650)
	{
		return false;
	}
	else
	{
		return false;
	}
}
void alt_hold(int counter)
{
	bool hover;
	static int flag = 0;
	int last_channelvalue = 0;
	int current_channelvalue = 0;
	// static int counter = 0;

	// check which mode now(hover or not)
	if (channel_value[remote_channel_7] > 1700)
		hover = true;
	else
		hover = false;

	calibrate_parameter.hover_mode = hover;

	if (hover && flag == 0)
	{
		flag = 1;
		calibrate_set_goal_height(&calibrate_parameter, ((float)distance_final));
		calibrate_set_goal_acc(&calibrate_parameter, acc_balance);
	}
	else if (hover == false && flag == 1)
	{
		flag = 0;
	}
	// printf("%d\n", distance_final);
	// else if (hover && flag ==1)
	// {
	// 	counter++;
	// 	current_channelvalue = channel_value[remote_channel_3];
	// 	printf("abs(current_channelvalue - last_channelvalue)=%d\n",abs(current_channelvalue - last_channelvalue));

	// 	if(counter==50)
	// 	{
	// 		// counter =0;
	// 		printf("abs(current_channelvalue - last_channelvalue)=%d\n",abs(current_channelvalue - last_channelvalue));
	// 		if(abs(current_channelvalue - last_channelvalue)>100)
	// 		{
	// 			calibrate_set_goal_height(&calibrate_parameter, ((float)distance_final));
	// 		}
	// 		last_channelvalue = channel_value[remote_channel_3];
	// 	}
	// }

	calibrate_parameter.distance = ((float)distance_final); //get distance data
	calibrate_parameter.acc = ((float)calibrate_parameter.mpu9250_data.accel_z);
	// printf("%f\n", calibrate_parameter.goal_acc - ((float)calibrate_parameter.mpu9250_data.accel_z));
	return;
}
void led_init(void)
{
	gpio_8b2 = gpio_get_dev(DFSS_GPIO_8B2_ID);	// get GPIO_8B0 handler
	gpio_8b2->gpio_open(1 << GPIO8B2_0_OFFSET); // open
	gpio_8b2->gpio_open(1 << GPIO8B2_3_OFFSET);
	gpio_8b2->gpio_open(1 << GPIO8B2_4_OFFSET);											  // open
	gpio_8b2->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(1 << GPIO8B2_0_OFFSET)); // set dir
	gpio_8b2->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(1 << GPIO8B2_3_OFFSET)); // set dir
	gpio_8b2->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(1 << GPIO8B2_4_OFFSET)); // set dir

	// gpio_8b2->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT, (void *)(1 << GPIO4B1_3_OFFSET));  // set GPIO4B2_1_OFFSET dir to input.

	return;
}
int main(void)
{
	static uint32_t led_cnt = 0;

	float goal_pitch = 0;
	float goal_roll = 0;
	float goal_yaw = 0;
	int initial_speed = 1600;
	int cnt = 0;

	bool balance_pitch = false;
	bool balance_roll = false;
	bool throttle_be_ready = false;
	throttle_state_t throttle_state = throttle_enable;
	bool throttle_stop = false;

	ultrasonic_init();
	gpio_init();
	// activate_button();
	gpio_4b2->gpio_write(1 << GPIO4B2_0_OFFSET, 1 << GPIO4B2_0_OFFSET); // GPIO4B2_0_OFFSET set low output

	mpu_initilize();
	// printf("HI\n");
	arduino_pin_init();
	pwm_init();
	calibrate_init(&calibrate_parameter, goal_pitch, goal_roll, goal_yaw, initial_speed);
	// MS5611_begin();

	//notice: The function will use Timer1 to calibrate the yaw, so don't use Timer1 when using this function.
	check_mpu_and_yaw();
	timer_1_init();
	led_init();
	// timer0_init();

	throttle_be_ready = false;
	////UART
	uint8_t string_buffer[200];
	for (int i = 0; i < 200; i++)
		string_buffer[i] = 32;
	DEV_UART_PTR uart_obj = uart_get_dev(1);
	uart_obj->uart_open(38400);
	while (1)
	{

		gpio_8b2->gpio_write(1 << GPIO8B2_0_OFFSET, 1 << GPIO8B2_0_OFFSET);
		pwm_set(0, 720);
		pwm_set(1, 720);
		pwm_set(2, 720);
		pwm_set(3, 720);
		printf("***\n");
		while ((throttle_be_ready == false) || (throttle_state == throttle_disable))
		{
			sbus_control();
			throttle_state_control(&throttle_state);
			// altitude_measure(); // measure the elevation on the ground
			check_throttle(&throttle_be_ready);

			led_cnt++;

			if (led_cnt == 50)
				gpio_8b2->gpio_write(0 << GPIO8B2_0_OFFSET, 1 << GPIO8B2_0_OFFSET);
			else if (led_cnt >= 100)
			{
				led_cnt = 0;
				gpio_8b2->gpio_write(1 << GPIO8B2_0_OFFSET, 1 << GPIO8B2_0_OFFSET);
			}

			board_delay_ms(1, 1);
			EMBARC_PRINTF("throttle not ready!\r\n");
		}

		gpio_8b2->gpio_write(1 << GPIO8B2_0_OFFSET, 1 << GPIO8B2_0_OFFSET);

		ground_elevation = altitude;
		EMBARC_PRINTF("wait 2 seconds!\r\n");
		board_delay_ms(2000, 1);
		EMBARC_PRINTF("start!\r\n");

		bool hover;
		int flag = 0;
		int last_channelvalue = 0;
		int current_channelvalue = 0;
		static int counter = 0;
		int target_distance = 0;
		while ((throttle_be_ready == true) && (throttle_state == throttle_enable))
		{
			// if (value_read_gpio12 != value_last_gpio12)
			// printf("value_read = %d\n", value_read_gpio12);
			// printf("last %d now %d\n", value_last_gpio12, value_read_gpio12);
			// printf("last %d now %d\n", value_last, value_read);
			// printf("check_flag_gpio = %d \n", check_flag_gpio);
			// printf("value_read_gpio13 = %d\n", value_read_gpio13);
			// printf("value_read_gpio13 = %d value_last_gpio13 = %d\n", value_read_gpio13, value_last_gpio13);
			// printf("gpio_flag = %d\n", gpio_flag);

			alt_hold(counter);

			// calibrate_parameter.distance = ((float)distance_final); //get distance data
			mpu9250_sensor_read(mpu9250_sensor, &(calibrate_parameter.mpu9250_data));

			sbus_control();
			throttle_stop = throttle_state_control(&throttle_state);
			throttle_control();
			roll_control(&balance_pitch, throttle_stop); //check whether drone is balance at roll side
			pitch_control(&balance_roll, throttle_stop); //check whether drone is balance at pitch side
			yaw_control(throttle_stop);
			sbus_compare();

			calib_stage_handler(balance_pitch, balance_roll); //check whether the drone need to calibrate or not
			calibrating(&calibrate_parameter);
			// printf("clib_motor speed = %d\n", calibrate_parameter.motor_speed);
			if (counter < 1000)
			{
				counter++;
				board_delay_ms(1, 1);
			}
			else
			{
				counter = 0;
				sprintf(string_buffer, "\npitch %5.2f roll %5.2f yaw %5.2f\nmotor1 %6.2f\nmotor2 %6.2f\nmotor3 %6.2f\nmotor4 %6.2f\ndistance %d\n***", calibrate_parameter.mpu9250_data.pitch, calibrate_parameter.mpu9250_data.roll, calibrate_parameter.mpu9250_data.yaw, calibrate_parameter.speed_motor1, calibrate_parameter.speed_motor2, calibrate_parameter.speed_motor3, calibrate_parameter.speed_motor4, distance_final);
				uart_obj->uart_write(&string_buffer[0], sizeof(string_buffer));
			}
		}

		throttle_be_ready = false;
		throttle_state = throttle_disable;
		EMBARC_PRINTF("throttle stop!\r\n");
	}

	timer_stop(TIMER_0);
	// stop the propeller
	pwm_set(0, 0);
	pwm_set(1, 0);
	pwm_set(2, 0);
	pwm_set(3, 0);

	// print the record data
	int i = 0;
	for (int i = 0; i < 4000; i++)
	{
		printf("%6f;", raw_data[i].pitch_dump);
		printf("%6f;", raw_data[i].pitch_pid);
		printf("%6f;", raw_data[i].roll_dump);
		printf("%6f;", raw_data[i].roll_pid);
		printf("\r\n");
	}

	printf("pid:%d\r\n", cnt);

	return E_OK;
}

// control SBUS decoder
int32_t SBUS_DECODE(uint16_t *channel_value)
{
	uint8_t sbus_decode_i = 0;
	uint8_t sbus_decode_error = 0;
	uint8_t sbus_parity_cnt = 0;
	uint32_t sbus_parity_cnt2 = 0;

	uint16_t sbus_decode_cnt = 0;
	uint16_t sbus_decode_cnt2 = 0;
	uint16_t sbus_decode_index = 0;
	uint16_t sbus_decode_shift = 0;
	uint16_t sbus_decode_buf = 0;

	for (sbus_decode_i = 0; sbus_decode_i < 25; sbus_decode_i++)
	{
		if ((sbus_buf[sbus_decode_i] & 0x0800) == 0)
			sbus_decode_error++;

		if ((sbus_buf[sbus_decode_i] & 0x0003) != 0)
			sbus_decode_error++;

		sbus_parity_cnt = 0;
		for (sbus_parity_cnt2 = 0; sbus_parity_cnt2 < 12; sbus_parity_cnt2++)
		{
			if (sbus_buf[sbus_decode_i] & (0x0001 << sbus_parity_cnt2))
				sbus_parity_cnt++;
		}
		if ((sbus_parity_cnt % 2) != 0)
			sbus_decode_error++;
	}

	if (sbus_decode_error == 0)
	{
		sbus_decode_shift = 0x0400;
		sbus_decode_cnt = 0;
		sbus_decode_index = 1;

		while (sbus_decode_cnt < 16)
		{
			sbus_decode_cnt2 = 0;
			sbus_decode_buf = 0;
			while (sbus_decode_cnt2 < 11)
			{
				if (sbus_buf[sbus_decode_index] & sbus_decode_shift)
					sbus_decode_buf = sbus_decode_buf | (0x0001 << sbus_decode_cnt2);

				sbus_decode_shift >>= 1;
				if (sbus_decode_shift == 0x0004)
				{
					sbus_decode_index++;
					sbus_decode_shift = 0x0400;
				}
				sbus_decode_cnt2++;
			}
			*(channel_value + sbus_decode_cnt) = sbus_decode_buf;
			sbus_decode_cnt++;
		}
	}

	return sbus_decode_error;
}
