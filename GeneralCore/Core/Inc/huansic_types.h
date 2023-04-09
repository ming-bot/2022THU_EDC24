/*
 * huansic_types.h
 *
 *  Created on: Oct 16, 2022
 *      Author: Zonghuan Wu
 */

#ifndef INC_HUANSIC_TYPES_H_
#define INC_HUANSIC_TYPES_H_

#include "stm32f1xx_hal.h"

enum IMU_STATUS
{
	IMU_OK = HAL_OK,
	IMU_ERROR = HAL_ERROR,
	IMU_BUSY = HAL_BUSY,
	IMU_TIMEOUT = HAL_TIMEOUT,
	IMU_SUM_ERROR,
	IMU_HEADER_ERROR,
	IMU_PID_ERROR
};

enum XB_STATUS
{
	XB_OK = HAL_OK,
	XB_ERROR = HAL_ERROR,
	XB_BUSY = HAL_BUSY,
	XB_TIMEOUT = HAL_TIMEOUT,
	XB_SUM_ERROR,
	XB_HEADER_ERROR,
	XB_ID_ERROR
};

enum IMU_STATE {
	IMU_STATE_HDR = 0,
	IMU_STATE_PID,
	IMU_STATE_XLO,
	IMU_STATE_XHI,
	IMU_STATE_YLO,
	IMU_STATE_YHI,
	IMU_STATE_ZLO,
	IMU_STATE_ZHI,
	IMU_STATE_TLO,
	IMU_STATE_THI,
	IMU_STATE_SUM
};

enum IMU_AXIS {
	IMU_AXIS_X = 0,
	IMU_AXIS_Y,
	IMU_AXIS_Z
};

typedef struct {
	int16_t x, y;		// 2 + 2
} Coordinate;			// = 4

typedef struct {
	float x, y;			// 4 + 4
} fCoordinate;			// = 4

typedef struct {
	Coordinate startCoord, destCoord;		// 4 + 4
	uint32_t timeLimit;						// 4
	uint32_t startTime;						// 4
	int32_t id;								// 4
	float reward;							// 4
} Order;									// = 24

typedef struct {
	Coordinate coord1, coord2;				// 4 + 4
} Rectangle;								// = 8

typedef struct {
	TIM_HandleTypeDef *counter;
	TIM_HandleTypeDef *posTimer, *negTimer;
	uint32_t pos_channel, neg_channel;		// (4 + 4) TIM_CHANNEL_n
	float kp, ki, kd;		// 4 + 4 + 4
	float dt;		// (4) Feedback Control Period; used to perform the calculation
	uint8_t encoderInverted;
	// uint16_t maxCount;			// divider used to normalize the error; just assume it is 16 bit
	// uint16_t timerPeriod;		// multiplier used to convert the normalized output into timer output; get this from 'timer'
	uint16_t lastTick;				// 2 (4)
	float lastError;				// 4
	float lastSpeed, last5Speed;	// 4 + 4
	float sumError;					// 4
	float goalSpeed;				// 4
} Motor_HandleTypeDef;

/*		// deprecated
 typedef struct {
 UART_HandleTypeDef *huart;

 float accel[3];		// 4 + 4 + 4
 float omega[3];		// 4 + 4 + 4
 float theta[3];		// 4 + 4 + 4
 float temperature;		// 4

 uint32_t lastUpdated;	// 4
 uint8_t buffer[10];
 uint8_t newChar;
 enum IMU_STATE state;
 } JY62_HandeleTypeDef;
 */

typedef struct {
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma;

	float accel[3];		// 4 + 4 + 4
	float omega[3];		// 4 + 4 + 4
	float theta[3];		// 4 + 4 + 4
	float temperature;		// 4

	uint32_t lastUpdated;	// 4

	uint8_t buffer[33];
	uint8_t pending_alignment;

#ifdef HUANSIC_JY62_DEBUG
	GPIO_TypeDef *port;
	uint16_t pin;
	uint8_t counter;
#endif

} JY62_HandleTypeDef;

typedef struct {
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma;

	uint32_t lastUpdated;

	// for internal use; user shall not modify this
	uint8_t nextPackageID;			// 0x00: next is header; others: check ID
	uint8_t nextPackageLength;		// length of next DMA receive

	// max is 126
	uint8_t buffer[200];	// put at the end to prevent block alignment issues

	uint8_t pending_alignment;
	uint8_t lastByte;
	uint8_t checksum;
} XB_HandleTypeDef;

typedef struct Path_t{
	Coordinate start, end;		// 4 + 4
	struct Path_t* nextPath;	// 4
	uint8_t referenceCount;		// 1 (4)
} Path;

#endif /* INC_HUANSIC_TYPES_H_ */
