#include "main.h"
#include "servo.h"

void Servo_Init(servo_t *servo, TIM_HandleTypeDef *_htim, uint32_t _channel)
{
	servo->htim = _htim;
	servo->channel = _channel;

	HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	//return (x + in_max) * (out_max - out_min) / (2 * in_max) + out_min;
	//return (out_max - out_min) * x / in_max + out_min;
}

void Servo_SetAngle(servo_t *servo, uint16_t angle)
{
	if(angle < ANGLE_MIN) angle = ANGLE_MIN;
	if(angle > ANGLE_MAX) angle = ANGLE_MAX;

	  uint16_t tmp = map(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
	  __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, tmp);
}

void Servo_SetAngleFine(servo_t *servo, float angle)
{
	if(angle < 0) angle = ANGLE_MIN;
	if(angle > 180) angle = ANGLE_MAX;

	  uint16_t tmp = map(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
	  __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, tmp);
}
