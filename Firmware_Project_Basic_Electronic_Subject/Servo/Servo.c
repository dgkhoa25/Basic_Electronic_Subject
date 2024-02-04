#include "Servo.h"

void servo_init(servo_typedef_t *servo_name, TIM_HandleTypeDef *tim, uint16_t channel)
{
	servo_name->tim_servo = tim;
	servo_name->tim_channel = channel;
	HAL_TIM_PWM_Start(tim, channel);
	servo_name->angle = 0;
}

static uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_set_angle(servo_typedef_t *servo_name, uint8_t angle)
{
	uint16_t ccr = (uint16_t)map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
	switch (servo_name->tim_channel)
	{
		case TIM_CHANNEL_1:
		{
			servo_name->tim_servo->Instance->CCR1 = ccr;
			break;
		}
		case TIM_CHANNEL_2:
		{
			servo_name->tim_servo->Instance->CCR2 = ccr;
			break;
		}
		case TIM_CHANNEL_3:
		{
			servo_name->tim_servo->Instance->CCR3 = ccr;
			break;
		}
		case TIM_CHANNEL_4:
		{
			servo_name->tim_servo->Instance->CCR4 = ccr;
			break;
		}
		default:
		{
			
			break;
		}
	}
}

void servo_close(servo_typedef_t *servo_name)
{
	servo_set_angle(servo_name, 0);
}

void servo_open(servo_typedef_t *servo_name)
{
	servo_set_angle(servo_name, 90);
}
