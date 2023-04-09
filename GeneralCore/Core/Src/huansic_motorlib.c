/*
 * huansic_motorlib.c
 *
 *  Created on: Sep 27, 2022
 *      Author: Zonghuan Wu
 */

#include "huansic_motorlib.h"

void huansic_motor_init(Motor_HandleTypeDef *hmotor) {
	// checking some stuff
	assert(hmotor->counter);
	assert(hmotor->posTimer);	// the negative channel CAN be NULL
	assert(hmotor->dt);

	// initialize
	hmotor->lastTick = 0;
	hmotor->lastError = 0;
	hmotor->lastSpeed = 0;
	hmotor->last5Speed = 0;
	hmotor->sumError = 0;
	hmotor->goalSpeed = 0;
	hmotor->counter->Instance->CNT = 0;

	// shut down the motor for now

	if (hmotor->pos_channel == TIM_CHANNEL_1)
		hmotor->posTimer->Instance->CCR1 = 0;
	else if (hmotor->pos_channel == TIM_CHANNEL_2)
		hmotor->posTimer->Instance->CCR2 = 0;
	else if (hmotor->pos_channel == TIM_CHANNEL_3)
		hmotor->posTimer->Instance->CCR3 = 0;
	else if (hmotor->pos_channel == TIM_CHANNEL_4)
		hmotor->posTimer->Instance->CCR4 = 0;
	else
		;

	if (hmotor->negTimer) {
		if (hmotor->neg_channel == TIM_CHANNEL_1)
			hmotor->negTimer->Instance->CCR1 = 0;
		else if (hmotor->neg_channel == TIM_CHANNEL_2)
			hmotor->negTimer->Instance->CCR2 = 0;
		else if (hmotor->neg_channel == TIM_CHANNEL_3)
			hmotor->negTimer->Instance->CCR3 = 0;
		else if (hmotor->neg_channel == TIM_CHANNEL_4)
			hmotor->negTimer->Instance->CCR4 = 0;
		else
			;
	}
	// and start the counter and timer
	HAL_TIM_Encoder_Start(hmotor->counter, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(hmotor->posTimer, hmotor->pos_channel);
	if (hmotor->negTimer)
		HAL_TIM_PWM_Start(hmotor->negTimer, hmotor->neg_channel);
}

void huansic_motor_pid(Motor_HandleTypeDef *hmotor) {
	int16_t newTick = 0x0FFFF & hmotor->counter->Instance->CNT;
	if (hmotor->encoderInverted)
		newTick = -newTick;
	int16_t diffTick = newTick - hmotor->lastTick;

	hmotor->lastTick = newTick;

	hmotor->lastSpeed = (float) diffTick / hmotor->dt;
	hmotor->last5Speed = (4.0 * hmotor->last5Speed + hmotor->lastSpeed) / 5.0;

	// Derivative
	float dError = hmotor->lastError - (hmotor->goalSpeed - hmotor->lastSpeed);

	// Proportional
	hmotor->lastError = hmotor->goalSpeed - hmotor->lastSpeed;

	// Integral
	hmotor->sumError += hmotor->lastError;

	// calculate and constrain the duty cycle
	float foutput = hmotor->kp * hmotor->lastError + hmotor->ki * hmotor->sumError
			+ hmotor->kd * dError;
	foutput = foutput > 1.0 ? 1.0 : (foutput < -1.0 ? -1.0 : foutput);

	// output to the timers
	uint16_t posoutput = foutput > 0 ? roundf(fabsf(foutput) * hmotor->posTimer->Instance->ARR) : 0;
	uint16_t negoutput = foutput < 0 ? roundf(fabsf(foutput) * hmotor->negTimer->Instance->ARR) : 0;

	if (hmotor->pos_channel == TIM_CHANNEL_1)
		hmotor->posTimer->Instance->CCR1 = posoutput;
	else if (hmotor->pos_channel == TIM_CHANNEL_2)
		hmotor->posTimer->Instance->CCR2 = posoutput;
	else if (hmotor->pos_channel == TIM_CHANNEL_3)
		hmotor->posTimer->Instance->CCR3 = posoutput;
	else if (hmotor->pos_channel == TIM_CHANNEL_4)
		hmotor->posTimer->Instance->CCR4 = posoutput;
	else
		;

	if (hmotor->negTimer) {
		if (hmotor->neg_channel == TIM_CHANNEL_1)
			hmotor->negTimer->Instance->CCR1 = negoutput;
		else if (hmotor->neg_channel == TIM_CHANNEL_2)
			hmotor->negTimer->Instance->CCR2 = negoutput;
		else if (hmotor->neg_channel == TIM_CHANNEL_3)
			hmotor->negTimer->Instance->CCR3 = negoutput;
		else if (hmotor->neg_channel == TIM_CHANNEL_4)
			hmotor->negTimer->Instance->CCR4 = negoutput;
		else
			;
	}
}
