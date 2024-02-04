#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_DUTY 500
#define SERVO_MAX_DUTY 2350

typedef struct
{
	TIM_HandleTypeDef *tim_servo;
	uint16_t tim_channel;
	uint8_t angle;
} servo_typedef_t;


void servo_init(servo_typedef_t *servo_name, TIM_HandleTypeDef *tim, uint16_t channel);
void servo_set_angle(servo_typedef_t *servo_name, uint8_t angle);
void servo_close(servo_typedef_t *servo_name);
void servo_open(servo_typedef_t *servo_name);

#endif
