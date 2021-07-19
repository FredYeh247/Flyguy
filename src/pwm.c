//#include "pwm.h"
#include "embARC.h"
#include "embARC_debug.h"


ADC_DEFINE(adc_test, ADC_INT_NUM, ADC_CRTL_BASE, NULL);
int32_t user_pwm_timer_write(DEV_PWM_TIMER *pwm_timer_obj, uint32_t ch, uint32_t mode, uint32_t freq, uint32_t duty_cnt);

void arduino_pin_init(void)
{
	io_arduino_config(ARDUINO_PIN_3, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch0
	io_arduino_config(ARDUINO_PIN_5, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch1
	io_arduino_config(ARDUINO_PIN_6, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch2
	io_arduino_config(ARDUINO_PIN_9, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch3
	io_arduino_config(ARDUINO_PIN_10, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch4
	io_arduino_config(ARDUINO_PIN_11, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch5
}


void pwm_increase(int channel)
{
    int output;
    
	DEV_PWM_TIMER_PTR pwm_timer_test = pwm_timer_get_dev(DW_PWM_TIMER_0_ID);
	pwm_timer_test->pwm_timer_open();

   	for(output=0;output<100;output++){
		board_delay_ms(7, 1);
		//pwm_timer_test->pwm_timer_write(channel, DEV_PWM_TIMER_MODE_PWM, 40000, output);
		user_pwm_timer_write(pwm_timer_test, 5, DEV_PWM_TIMER_MODE_PWM, 32000,output); 
		EMBARC_PRINTF("Output = %d\n",output);
	}
}
void pwm_decrease(int channel)
{
    int output;
    
	DEV_PWM_TIMER_PTR pwm_timer_test = pwm_timer_get_dev(DW_PWM_TIMER_0_ID);
	pwm_timer_test->pwm_timer_open();

    for(output=100;output>0;output--){
	board_delay_ms(7, 1);
	//pwm_timer_test->pwm_timer_write(channel, DEV_PWM_TIMER_MODE_PWM, 40000, output);
	user_pwm_timer_write(pwm_timer_test, 5, DEV_PWM_TIMER_MODE_PWM, 32000, output);
	EMBARC_PRINTF("Output = %d\n",output);
	}
}

void pwm_set(int channel,int output)
{
	
	DEV_PWM_TIMER_PTR pwm_timer_test = pwm_timer_get_dev(DW_PWM_TIMER_0_ID);
	pwm_timer_test->pwm_timer_open();
	//pwm_timer_test->pwm_timer_write(channel, DEV_PWM_TIMER_MODE_PWM, 32000, output);
	user_pwm_timer_write(pwm_timer_test, channel, DEV_PWM_TIMER_MODE_PWM, 32000, output);
	//EMBARC_PRINTF("Channel = %d Output = %d\n",channel,output);
}



Inline uint32_t dw_pwm_timer_int_read_status(DW_PWM_TIMER_CTRL_PTR port)
{
	return port->regs->PWM_TIMERS_INT_STATUS;
}

Inline void dw_pwm_timer_timer_enable(DW_PWM_TIMER_CTRL_PTR port, uint32_t ch)
{
	port->regs->CTRL[ch].CONTROL_REG |= 0x03;
}

Inline void dw_pwm_timer_timer_disable(DW_PWM_TIMER_CTRL_PTR port, uint32_t ch)
{
	port->regs->CTRL[ch].CONTROL_REG &= (~0x03);
}

Inline void dw_pwm_timer_pwm_enable(DW_PWM_TIMER_CTRL_PTR port, uint32_t ch)
{
	port->regs->CTRL[ch].CONTROL_REG |= 0x08;
}

Inline void dw_pwm_timer_pwm_disable(DW_PWM_TIMER_CTRL_PTR port, uint32_t ch)
{
	port->regs->CTRL[ch].CONTROL_REG &= (~0x08);
}

Inline void dw_pwm_timer_count_set(DW_PWM_TIMER_CTRL_PTR port, uint8_t ch, uint16_t val)
{
	port->regs->CTRL[ch].LOAD_COUNT = val;
}

Inline uint16_t dw_pwm_timer_count_get(DW_PWM_TIMER_CTRL_PTR port, uint8_t ch)
{
	return port->regs->CTRL[ch].LOAD_COUNT;
}

Inline void dw_pwm_timer_count2_set(DW_PWM_TIMER_CTRL_PTR port, uint8_t ch, uint16_t val)
{
	port->regs->LOAD_COUNT2[ch] = val;
}

Inline uint16_t dw_pwm_timer_count2_get(DW_PWM_TIMER_CTRL_PTR port, uint8_t ch)
{
	return port->regs->LOAD_COUNT2[ch];
}

int32_t user_pwm_timer_write(DEV_PWM_TIMER *pwm_timer_obj, uint32_t ch, uint32_t mode, uint32_t freq, uint32_t duty_cnt)
{
	int32_t ercd = E_OK;
	int32_t count, count_high;
	DEV_PWM_TIMER_INFO_PTR port_info_ptr = &(pwm_timer_obj->pwm_timer_info);

	DW_PWM_TIMER_CTRL_PTR port = (DW_PWM_TIMER_CTRL_PTR)(port_info_ptr->pwm_timer_ctrl);

	if (mode == DEV_PWM_TIMER_MODE_TIMER) {
		port->mode[ch] = mode;

		count = port->clock / freq;
		dw_pwm_timer_count_set(port, ch, count/2);
		dw_pwm_timer_count2_set(port, ch, 0);

		dw_pwm_timer_timer_enable(port, ch);
		dw_pwm_timer_pwm_disable(port, ch);
		int_enable(port->intno + ch);
	} else if (mode == DEV_PWM_TIMER_MODE_PWM) {
		port->mode[ch] = mode;

		count = port->clock / freq;
		//count_high = (count * dc)/100;
		count_high = duty_cnt;
		//count_upper = 4500
		//count ++ 0~4499
		//(count_high > count)  >> output = 1
		dw_pwm_timer_count_set(port, ch, count-count_high); //cnt low
		dw_pwm_timer_count2_set(port, ch, count_high);	//cnt
		dw_pwm_timer_timer_enable(port, ch);
		dw_pwm_timer_pwm_enable(port, ch);
		int_disable(port->intno + ch);
	} else if (mode == DEV_PWM_TIMER_MODE_CLOSE) {
		port->mode[ch] = mode;
		dw_pwm_timer_count_set(port, ch, 0);
		dw_pwm_timer_count2_set(port, ch, 0);
		dw_pwm_timer_timer_disable(port, ch);
		dw_pwm_timer_pwm_disable(port, ch);
		int_disable(port->intno + ch);
	}
error_exit:
	return ercd;
}

