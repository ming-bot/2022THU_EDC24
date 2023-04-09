/*
 * huansic_motorlib.h
 *
 *  Created on: Sep 27, 2022
 *      Author: Zonghuan Wu
 */

#ifndef INC_HUANSIC_MOTORLIB_H_
#define INC_HUANSIC_MOTORLIB_H_

#include <math.h>
#include "huansic_types.h"

void huansic_motor_init(Motor_HandleTypeDef *hmotor);

void huansic_motor_pid(Motor_HandleTypeDef *hmotor);

#endif /* INC_HUANSIC_MOTORLIB_H_ */
