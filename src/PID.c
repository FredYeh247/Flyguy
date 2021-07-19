#include "PID.h"
#include <math.h>

#define SIGMA 0.000001f
#define error_range 7.0
#define acc_error_range 3000.0
void pid_init(PID_t *pid, pid_mode_t mode, float dt_min)
{
	pid->mode = mode;
	pid->dt_min = dt_min;
	pid->kp = 0.0f;
	pid->ki = 0.0f;
	pid->kd = 0.0f;
	pid->integral = 0.0f;
	pid->integral_limit = 0.0f;
	pid->output_limit = 0.0f;
	pid->error_previous = 0.0f;
	pid->last_output = 0.0f;
	pid->i = 0.0f;
	pid->d = 0.0f;
}

int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
	int ret = 0;

	if (isfinite(kp))
	{
		pid->kp = kp;
	}
	else
	{
		ret = 1;
	}

	if (isfinite(ki))
	{
		pid->ki = ki;
	}
	else
	{
		ret = 1;
	}

	if (isfinite(kd))
	{
		pid->kd = kd;
	}
	else
	{
		ret = 1;
	}

	if (isfinite(integral_limit))
	{
		pid->integral_limit = integral_limit;
	}
	else
	{
		ret = 1;
	}

	if (isfinite(output_limit))
	{
		pid->output_limit = output_limit;
	}
	else
	{
		ret = 1;
	}

	return ret;
}

float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
	float error = 0;

	if (!isfinite(sp) || !isfinite(val) || !isfinite(val_dot) || !isfinite(dt))
	{
		return pid->last_output;
	}

	error = sp - val;
	// printf("error =%f\n", error);
	if (error < 5.0 && error > -5.0)
	{
		error = 0.0;
	}
	/* current error derivative */
	if (pid->mode == PID_MODE_DERIVATIV_CALC)
	{
		pid->d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = error;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_CALC_NO_SP)
	{
		pid->d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = -val;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_SET)
	{
		pid->d = -val_dot;
	}
	else
	{
		pid->d = 0.0f;
	}

	if (!isfinite(pid->d))
	{
		pid->d = 0.0f;
	}

	/* calculate PD output */
	float output = (error * pid->kp) + (pid->d * pid->kd);

	if (pid->ki > SIGMA)
	{
		// Calculate the error integral and check for saturation
		pid->i = pid->integral + (error * dt);

		/* check for saturation */
		if (isfinite(pid->i))
		{
			if ((pid->output_limit < SIGMA || (fabsf(output + (pid->i * pid->ki)) <= pid->output_limit)) &&
				fabsf(pid->i) <= pid->integral_limit)
			{
				/* not saturated, use new integral value */
				pid->integral = pid->i;
			}
		}

		/* add I component to output */
		output += pid->integral * pid->ki;
	}

	/* limit outcdput */
	if (isfinite(output))
	{
		if (pid->output_limit > SIGMA)
		{
			if (output > pid->output_limit)
			{
				output = pid->output_limit;
			}
			else if (output < -pid->output_limit)
			{
				output = -pid->output_limit;
			}
		}

		pid->last_output = output;
	}

	return pid->last_output;
}
float height_pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
	float error = 0;

	if (!isfinite(sp) || !isfinite(val) || !isfinite(val_dot) || !isfinite(dt))
	{
		return pid->last_output;
	}

	error = sp - val;
	error /= 3.0;
	if (error > 6 * error_range)
		error = 6 * error_range;
	else if (error < -6 * error_range)
		error = -6 * error_range;
	else if (error < 2 * error_range && error > -2 * error_range)
		error = 0.4 * error;
	else if (error < 3 * error_range && error > -3 * error_range)
		error = 0.6 * error;
	// else if  (error < error_range && error > -error_range)
	// 	error = 0.0;
	// printf("error = %f\n", error);
	/* current error derivative */
	if (pid->mode == PID_MODE_DERIVATIV_CALC)
	{
		pid->d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = error;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_CALC_NO_SP)
	{
		pid->d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = -val;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_SET)
	{
		pid->d = -val_dot;
	}
	else
	{
		pid->d = 0.0f;
	}

	if (!isfinite(pid->d))
	{
		pid->d = 0.0f;
	}

	/* calculate PD output */
	float output = (error * pid->kp) + (pid->d * pid->kd);

	if (pid->ki > SIGMA)
	{
		// Calculate the error integral and check for saturation
		pid->i = pid->integral + (error * dt);

		/* check for saturation */
		if (isfinite(pid->i))
		{
			if ((pid->output_limit < SIGMA || (fabsf(output + (pid->i * pid->ki)) <= pid->output_limit)) &&
				fabsf(pid->i) <= pid->integral_limit)
			{
				/* not saturated, use new integral value */
				pid->integral = pid->i;
			}
		}

		/* add I component to output */
		output += pid->integral * pid->ki;
	}

	/* limit outcdput */
	if (isfinite(output))
	{
		if (pid->output_limit > SIGMA)
		{
			if (output > pid->output_limit)
			{
				output = pid->output_limit;
			}
			else if (output < -pid->output_limit)
			{
				output = -pid->output_limit;
			}
		}

		pid->last_output = output;
	}

	return pid->last_output;
}
float acc_pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
	float error = 0;

	if (!isfinite(sp) || !isfinite(val) || !isfinite(val_dot) || !isfinite(dt))
	{
		return pid->last_output;
	}
	// printf("sp = %f val = %f\n", sp, val);
	error = sp - val;

	if (error < acc_error_range && error > -acc_error_range)
		error = 0.0;
	else
		error /= 100.0;
	// printf("error = %f\n", error);
	// printf("error = %f\n", error);
	/* current error derivative */
	if (pid->mode == PID_MODE_DERIVATIV_CALC)
	{
		pid->d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = error;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_CALC_NO_SP)
	{
		pid->d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = -val;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_SET)
	{
		pid->d = -val_dot;
	}
	else
	{
		pid->d = 0.0f;
	}

	if (!isfinite(pid->d))
	{
		pid->d = 0.0f;
	}

	/* calculate PD output */
	float output = (error * pid->kp) + (pid->d * pid->kd);

	if (pid->ki > SIGMA)
	{
		// Calculate the error integral and check for saturation
		pid->i = pid->integral + (error * dt);

		/* check for saturation */
		if (isfinite(pid->i))
		{
			if ((pid->output_limit < SIGMA || (fabsf(output + (pid->i * pid->ki)) <= pid->output_limit)) &&
				fabsf(pid->i) <= pid->integral_limit)
			{
				/* not saturated, use new integral value */
				pid->integral = pid->i;
			}
		}

		/* add I component to output */
		output += pid->integral * pid->ki;
	}

	/* limit outcdput */
	if (isfinite(output))
	{
		if (pid->output_limit > SIGMA)
		{
			if (output > pid->output_limit)
			{
				output = pid->output_limit;
			}
			else if (output < -pid->output_limit)
			{
				output = -pid->output_limit;
			}
		}

		pid->last_output = output;
	}

	return pid->last_output;
}

void pid_reset_integral(PID_t *pid)
{
	pid->integral = 0.0f;
}
/*
void pid_reset_integral(PID_t *pid)
{
	pid->integral/=2;
}
*/
float yaw_pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
	if (!isfinite(sp) || !isfinite(val) || !isfinite(val_dot) || !isfinite(dt))
	{
		return pid->last_output;
	}
	/*
	float i, d;
	*/
	/* current error value */
	float error = 0;
	error = sp - val;
	if (error > 180)
	{
		error = error - 360;
	}
	else if (error < -180)
	{
		error = error + 360;
	}
	else
	{
		error = error;
	}

	/* current error derivative */
	if (pid->mode == PID_MODE_DERIVATIV_CALC)
	{
		pid->d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = error;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_CALC_NO_SP)
	{
		pid->d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
		pid->error_previous = -val;
	}
	else if (pid->mode == PID_MODE_DERIVATIV_SET)
	{
		pid->d = -val_dot;
	}
	else
	{
		pid->d = 0.0f;
	}

	if (!isfinite(pid->d))
	{
		pid->d = 0.0f;
	}

	/* calculate PD output */
	float output = (error * pid->kp) + (pid->d * pid->kd);

	if (pid->ki > SIGMA)
	{
		// Calculate the error integral and check for saturation
		pid->i = pid->integral + (error * dt);

		/* check for saturation */
		if (isfinite(pid->i))
		{
			if ((pid->output_limit < SIGMA || (fabsf(output + (pid->i * pid->ki)) <= pid->output_limit)) &&
				fabsf(pid->i) <= pid->integral_limit)
			{
				/* not saturated, use new integral value */
				pid->integral = pid->i;
			}
		}

		/* add I component to output */
		output += pid->integral * pid->ki;
	}

	/* limit outcdput */
	if (isfinite(output))
	{
		if (pid->output_limit > SIGMA)
		{
			if (output > pid->output_limit)
			{
				output = pid->output_limit;
			}
			else if (output < -pid->output_limit)
			{
				output = -pid->output_limit;
			}
		}

		pid->last_output = output;
	}

	return pid->last_output;
}
