#ifndef _PWM_H_
#define _PWM_H_

void pwm_increase(int channel);
void pwm_decrease(int channel);
void pwm_set(int channel,int output);
void arduino_pin_init(void);

#endif
