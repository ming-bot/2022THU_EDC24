/*
 * huansic_jy62lib.h
 *
 *  Created on: Sep 28, 2022
 *      Author: Zonghuan Wu
 */

#ifndef INC_HUANSIC_JY62LIB_H_
#define INC_HUANSIC_JY62LIB_H_

#include <math.h>
#include "huansic_types.h"

/*
 * 		Initializes the port of the IMU.
 * 		@param	himu	jy62 pending initialization
 * 		@retval	enum IMU_STATUS
 */
enum IMU_STATUS huansic_jy62_init(JY62_HandleTypeDef *himu);

/*
 * 		Handles the dma interrupts.
 * 		@param	himu	jy62 whose port has sent out the interrupt signal
 * 		@retval	enum IMU_STATUS
 */
enum IMU_STATUS huansic_jy62_dma_isr(JY62_HandleTypeDef *himu);

/*
 * 		Handles the interrupts.
 * 		@param	himu	jy62 whose port has sent out the interrupt signal
 * 		@retval	enum IMU_STATUS
 */
enum IMU_STATUS huansic_jy62_isr(JY62_HandleTypeDef *himu);

/*
 * 		Handles the dma errors.
 * 		@param	himu	jy62 whose port has sent out the error
 * 		@retval	enum IMU_STATUS
 */
void huansic_jy62_dma_error(JY62_HandleTypeDef *himu);

/*
 * 		Handles the errors.
 * 		@param	himu	jy62 whose port has sent out the error
 * 		@retval	enum IMU_STATUS
 */
void huansic_jy62_it_error(JY62_HandleTypeDef *himu);

#endif /* INC_HUANSIC_JY62LIB_H_ */
