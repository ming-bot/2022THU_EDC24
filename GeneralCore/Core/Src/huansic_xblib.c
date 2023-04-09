/*
 * huansic_xblib.c
 *
 *  Created on: Sep 29, 2022
 *      Author: Zonghuan Wu
 */

#include "huansic_xblib.h"

// game information 1
extern uint8_t gameStage;			// 0: pre-match(standby); 1: first half; 2: second half
extern uint8_t gameStatus;			// 0: standby; 1: running
extern uint32_t gameStageTimeLimit;		// in ms
extern uint32_t gameStageTimeSinceStart;	// in ms
extern Rectangle obstacles[5];			// area that depletes charge faster
extern Coordinate allyBeacons[3];		// ally charging station coordinate
extern Coordinate oppoBeacons[3];		// opponent charging station coordinate

// game information 2
extern Order *delivering[8];		// package picked up but not yet delivered
extern Order orders[60];			// package that has not yet been picked up

// game information 3
extern Coordinate myCoord;			// precise coordinate returned by game master
extern fCoordinate estimatedCoord;	// coordinate calculated by Kalman Filter
extern double omegaZ, accelY;		// turning speed and linear acceleration
extern float myScore;				// current score returned by Master
extern float myCharge;				// current charge returned by Master

// interchange information 1
extern uint32_t gameStageTimeLeft;		// in ms
extern uint8_t CoordinateUpdate;
extern uint8_t delivering_num;
extern uint8_t allyBeacons_num;
extern uint8_t oppoBeacons_num;
uint8_t zigbeeSend[6] = { 0x55, 0xAA, 0x00, 0x00, 0x00, 0x00 };  //小车可能发送的信息（0x00:请求游戏信息 0x02:设置充电桩）

__weak void custom_order_new_failed(uint8_t id) {

}

void huansic_xb_init(XB_HandleTypeDef *hxb) {
	hxb->pending_alignment = 0;
	hxb->nextPackageID = 0x00;
	hxb->nextPackageLength = 6;		// header length
	// flush UART buffer
	uint8_t temp = hxb->huart->Instance->SR;
	temp = hxb->huart->Instance->DR;
	temp = hxb->huart->Instance->DR;
	(void) temp;
	HAL_UART_Receive_DMA(hxb->huart, &hxb->buffer[0], hxb->nextPackageLength);
	__HAL_DMA_DISABLE_IT(hxb->hdma, DMA_IT_HT);		// disable half transfer interrupt
}

enum XB_STATUS huansic_xb_decodeHeader(XB_HandleTypeDef *hxb) {
	if (!hxb)
		return XB_ERROR;

	// record checksum
	hxb->checksum = hxb->buffer[5];

	// get and check packet ID
	if (hxb->buffer[2] != 0x01 && hxb->buffer[2] != 0x05) {
		hxb->pending_alignment = 1;
		hxb->lastByte = 0x00;
		HAL_UART_Receive_IT(hxb->huart, &hxb->buffer[0], 1);		// check next byte
		return XB_ID_ERROR;
	}
	hxb->nextPackageID = hxb->buffer[2];

	// read next package length
	hxb->nextPackageLength = hxb->buffer[3]; // the length shall not be longer than 255 (the max possible is 225)

	// set up next DMA
	// check if overrun occurred
	if (__HAL_UART_GET_FLAG(hxb->huart, UART_FLAG_ORE)) {
		// perform the clear flag sequence and read the overrun data
		volatile uint8_t i = hxb->huart->Instance->SR;
		hxb->buffer[0] = hxb->huart->Instance->DR;
		(void) i;
		while (!__HAL_UART_GET_FLAG(hxb->huart, UART_FLAG_RXNE))
			;		// wait for the data in shift register to move into data register
		hxb->buffer[1] = hxb->huart->Instance->DR;
		HAL_UART_Receive_DMA(hxb->huart, &hxb->buffer[2],
				hxb->nextPackageLength < 2 ? hxb->nextPackageLength : (hxb->nextPackageLength - 2));
	}

	// check if RX buffer is empty
	else if (__HAL_UART_GET_FLAG(hxb->huart, UART_FLAG_RXNE)) {
		hxb->buffer[0] = hxb->huart->Instance->DR;		// read data
		HAL_UART_Receive_DMA(hxb->huart, &hxb->buffer[1],
				hxb->nextPackageLength < 1 ? hxb->nextPackageLength : (hxb->nextPackageLength - 1));
	}

	// otherwise, receive normally
	else {
		HAL_UART_Receive_DMA(hxb->huart, &hxb->buffer[0], hxb->nextPackageLength);
	}

	__HAL_DMA_DISABLE_IT(hxb->hdma, DMA_IT_HT);		// disable half transfer interrupt

	return XB_OK;
}

enum XB_STATUS huansic_xb_decodeBody(XB_HandleTypeDef *hxb) {
	uint8_t listLength = 0, i, j, index = 0;
	uint32_t temp;

	if (!hxb)
		return XB_ERROR;

	// checksum
	for (i = 0, j = 0; i < hxb->nextPackageLength; i++)
		j ^= hxb->buffer[i];

	if (j != hxb->checksum) {
		hxb->pending_alignment = 1;
		hxb->lastByte = 0x00;
		HAL_UART_Receive_IT(hxb->huart, &hxb->buffer[0], 1);		// check next byte
		return XB_SUM_ERROR;
	}

	if (hxb->nextPackageID == 0x01) {		// game information
		/* game stage */
		gameStage = hxb->buffer[index++];		// 0

		/* barrier list */
		// listLength = hxb->buffer[index];		// the length is fixed to 5
		index++;
		for (i = 0; i < 5; i++) {
			obstacles[i].coord1.x = (uint16_t) hxb->buffer[index + 1] << 8;
			obstacles[i].coord1.x = hxb->buffer[index];
			index += 2;
			obstacles[i].coord1.y = (uint16_t) hxb->buffer[index + 1] << 8;
			obstacles[i].coord1.y = hxb->buffer[index];
			index += 2;
			obstacles[i].coord2.x = (uint16_t) hxb->buffer[index + 1] << 8;
			obstacles[i].coord2.x = hxb->buffer[index];
			index += 2;
			obstacles[i].coord2.y = (uint16_t) hxb->buffer[index + 1] << 8;
			obstacles[i].coord2.y = hxb->buffer[index];
			index += 2;
		}		//2 ~ 41

		/* total time of this round */
		gameStageTimeLimit = hxb->buffer[index + 3];
		gameStageTimeLimit <<= 8;
		gameStageTimeLimit |= hxb->buffer[index + 2];
		gameStageTimeLimit <<= 8;
		gameStageTimeLimit |= hxb->buffer[index + 1];
		gameStageTimeLimit <<= 8;
		gameStageTimeLimit |= hxb->buffer[index];
		index += 4;		// 42 ~ 45

		/* ally beacons */
		listLength = hxb->buffer[index++];		//46
		allyBeacons_num = listLength;
		for (i = 0; i < listLength; i++) {
			allyBeacons[i].x = (uint16_t) hxb->buffer[index + 1] << 8;
			allyBeacons[i].x |= hxb->buffer[index];
			index += 2;
			allyBeacons[i].y = (uint16_t) hxb->buffer[index + 1] << 8;
			allyBeacons[i].y |= hxb->buffer[index];
			index += 2;
		}

		/* opponent beacons */
		listLength = hxb->buffer[index++];
		oppoBeacons_num = listLength;
		for (i = 0; i < listLength; i++) {
			oppoBeacons[i].x = (uint16_t) hxb->buffer[index + 1] << 8;
			oppoBeacons[i].x |= hxb->buffer[index];
			index += 2;
			oppoBeacons[i].y = (uint16_t) hxb->buffer[index + 1] << 8;
			oppoBeacons[i].y |= hxb->buffer[index];
			index += 2;
		}
	} else if (hxb->nextPackageID == 0x05) {		// game status
		/* game status */
		gameStatus = hxb->buffer[index++];		//1

		/* time since round started */
		gameStageTimeSinceStart = hxb->buffer[index + 3];
		gameStageTimeSinceStart <<= 8;
		gameStageTimeSinceStart |= hxb->buffer[index + 2];
		gameStageTimeSinceStart <<= 8;
		gameStageTimeSinceStart |= hxb->buffer[index + 1];
		gameStageTimeSinceStart <<= 8;
		gameStageTimeSinceStart |= hxb->buffer[index];		//index = 1
		gameStageTimeLeft = gameStageTimeLimit - gameStageTimeSinceStart;
		index += 4;		//5

		/* fetch score */
		temp = hxb->buffer[index + 3];
		temp <<= 8;
		temp |= hxb->buffer[index + 2];
		temp <<= 8;
		temp |= hxb->buffer[index + 1];
		temp <<= 8;
		temp |= hxb->buffer[index];
		myScore = *(float*) &temp;			// decode float from uint32
		index += 4;			//9

		/* my position */
		myCoord.x = hxb->buffer[index + 1];
		if (myCoord.x == 255) {
			myCoord.x = hxb->buffer[index] - 256;
		}
		else {
			myCoord.x = myCoord.x << 8;
			myCoord.x |= hxb->buffer[index];
		}
		index += 2;			//11
		myCoord.y = hxb->buffer[index + 1];
		if (myCoord.y == 255) {
			myCoord.y = hxb->buffer[index] - 256;
		}
		else {
			myCoord.y = myCoord.y << 8;
			myCoord.y |= hxb->buffer[index];
		}
		index += 2;			//13
		CoordinateUpdate = 1;

		/* fetch battery */
		temp = hxb->buffer[index + 3];
		temp <<= 8;
		temp |= hxb->buffer[index + 2];
		temp <<= 8;
		temp |= hxb->buffer[index + 1];
		temp <<= 8;
		temp |= hxb->buffer[index];
		myCharge = *(float*) &temp;			// decode float from uint32
		index += 4;			//17

		/* my orders */
		int8_t updatedOrder[] = { -1, -1, -1, -1, -1 };
		uint8_t updatedOrderIndex = 0;
		Order *tempOrder;
		listLength = hxb->buffer[index++];			//after_update : 18
		delivering_num = listLength;
		for (i = 0; i < listLength; i++) {
			temp |= hxb->buffer[index + 17];
			temp <<= 8;
			temp |= hxb->buffer[index + 16];
			tempOrder = huansic_order_new(temp);
			if (!tempOrder) {
				index += 18;
				custom_order_new_failed(temp);
				continue;
			}
			// start coordinate
			tempOrder->startCoord.x = ((uint16_t) hxb->buffer[index + 1] << 8)
					| hxb->buffer[index];
			tempOrder->startCoord.y = ((uint16_t) hxb->buffer[index + 3] << 8)
					| hxb->buffer[index + 2];
			// destination
			tempOrder->destCoord.x = ((uint16_t) hxb->buffer[index + 5] << 8)
					| hxb->buffer[index + 4];
			tempOrder->destCoord.y = ((uint16_t) hxb->buffer[index + 7] << 8)
					| hxb->buffer[index + 6];
			// time limit
			temp = hxb->buffer[index + 11];
			temp <<= 8;
			temp |= hxb->buffer[index + 10];
			temp <<= 8;
			temp |= hxb->buffer[index + 9];
			temp <<= 8;
			temp |= hxb->buffer[index + 8];
			tempOrder->timeLimit = temp;
			//start time
			if (tempOrder->startTime == 0)
				tempOrder->startTime = HAL_GetTick();

			// reward
			temp = hxb->buffer[index + 15];
			temp <<= 8;
			temp |= hxb->buffer[index + 14];
			temp <<= 8;
			temp |= hxb->buffer[index + 13];
			temp <<= 8;
			temp |= hxb->buffer[index + 12];
			tempOrder->reward = *(float*) &temp;

			delivering[i] = tempOrder;
			// increment index and record id
			index += 18;			//18+listLength*18
			updatedOrder[updatedOrderIndex++] = tempOrder->id;
		}

		/* order management */
		for (i = 0; i < 8; i++) {
			if (delivering[i] != 0 && delivering[i]->id != -1) {
				for (j = 0; j < updatedOrderIndex; j++)
					if (delivering[i]->id == updatedOrder[j]) {		// pulled from remote
						j = 255;
						break;
					}
				if (j != 255 && j != 0)
					huansic_order_delete(delivering[i]);// delete the order if the order is no longer in the delivery list
			}
		}
		/* record latest order */
		temp |= hxb->buffer[index + 17];
		temp <<= 8;
		temp |= hxb->buffer[index + 16];
		tempOrder = huansic_order_new(temp);
		if (!tempOrder) {
			index += 18;
			custom_order_new_failed(temp);
		} else {
			// start coordinate
			tempOrder->startCoord.x = ((uint16_t) hxb->buffer[index + 1] << 8)
					| hxb->buffer[index];
			tempOrder->startCoord.y = ((uint16_t) hxb->buffer[index + 3] << 8)
					| hxb->buffer[index + 2];
			order_append(tempOrder);
			// end coordinate
			tempOrder->destCoord.x = ((uint16_t) hxb->buffer[index + 5] << 8)
					| hxb->buffer[index + 4];
			tempOrder->destCoord.y = ((uint16_t) hxb->buffer[index + 7] << 8)
					| hxb->buffer[index + 6];
			// time limit
			temp = hxb->buffer[index + 11];
			temp <<= 8;
			temp |= hxb->buffer[index + 10];
			temp <<= 8;
			temp |= hxb->buffer[index + 9];
			temp <<= 8;
			temp |= hxb->buffer[index + 8];
			tempOrder->timeLimit = temp;
			// reward
			temp = hxb->buffer[index + 15];
			temp <<= 8;
			temp |= hxb->buffer[index + 14];
			temp <<= 8;
			temp |= hxb->buffer[index + 13];
			temp <<= 8;
			temp |= hxb->buffer[index + 12];
			tempOrder->reward = *(float*) &temp;
		}
	} else {
		hxb->pending_alignment = 1;
		hxb->lastByte = 0x00;
		HAL_UART_Receive_IT(hxb->huart, &hxb->buffer[0], 1);		// check next byte
		return XB_ID_ERROR;
	}

	// set up next DMA
	hxb->lastUpdated = HAL_GetTick();		// update last updated time stamp
	hxb->nextPackageLength = 6;		// header length
	hxb->nextPackageID = 0x00;		// the next one is header

	// check if overrun occurred
	if (__HAL_UART_GET_FLAG(hxb->huart, UART_FLAG_ORE)) {
		// perform the clear flag sequence and read the overrun data
		i = hxb->huart->Instance->SR;
		hxb->buffer[0] = hxb->huart->Instance->DR;
		while (!__HAL_UART_GET_FLAG(hxb->huart, UART_FLAG_RXNE))
			;		// wait for the data in shift register to move into data register
		hxb->buffer[1] = hxb->huart->Instance->DR;
		HAL_UART_Receive_DMA(hxb->huart, &hxb->buffer[2],
				hxb->nextPackageLength < 2 ? hxb->nextPackageLength : (hxb->nextPackageLength - 2));
	}

	// check if RX buffer is empty
	else if (__HAL_UART_GET_FLAG(hxb->huart, UART_FLAG_RXNE)) {
		hxb->buffer[0] = hxb->huart->Instance->DR;		// read data
		HAL_UART_Receive_DMA(hxb->huart, &hxb->buffer[1],
				hxb->nextPackageLength < 1 ? hxb->nextPackageLength : (hxb->nextPackageLength - 1));
	}

	// otherwise, receive normally
	else {
		HAL_UART_Receive_DMA(hxb->huart, &hxb->buffer[0], hxb->nextPackageLength);
	}

	__HAL_DMA_DISABLE_IT(hxb->hdma, DMA_IT_HT);		// disable half transfer interrupt

	return XB_OK;
}

void huansic_xb_requestGameInfo(XB_HandleTypeDef *hxb) {
	zigbeeSend[2] = 0x00;
	HAL_UART_Transmit(hxb->huart, zigbeeSend, 6, 10);
}

void huansic_xb_setBeacon(XB_HandleTypeDef *hxb) {
	zigbeeSend[2] = 0x02;
	HAL_UART_Transmit(hxb->huart, zigbeeSend, 6, 10);
}

void huansic_xb_dma_error(XB_HandleTypeDef *hxb) {
	// nothing much to do with error
	hxb->pending_alignment = 1;
	hxb->lastByte = 0x00;
	HAL_UART_Receive_IT(hxb->huart, &hxb->buffer[0], 1);
}

void huansic_xb_it_error(XB_HandleTypeDef *hxb) {
	// nothing much to do with error
	hxb->pending_alignment = 1;
	hxb->lastByte = 0x00;
	HAL_UART_Receive_IT(hxb->huart, &hxb->buffer[0], 1);
}

enum XB_STATUS huansic_xb_isr(XB_HandleTypeDef *hxb) {
	if (!hxb)
		return XB_ERROR;

	if (hxb->buffer[0] == 0xAA && hxb->lastByte == 0x55) {		// if aligned (look for header)
		hxb->pending_alignment = 0;
		hxb->nextPackageID = 0x00;
		hxb->buffer[1] = 0x55;		// for further processing
		HAL_UART_Receive_DMA(hxb->huart, &(hxb->buffer[2]), 4);		// receive the rest of header
		__HAL_DMA_DISABLE_IT(hxb->hdma, DMA_IT_HT);		// disable half transfer interrupt
		return XB_OK;
	} else {
		hxb->pending_alignment = 1;		// enter aligning mode if not already
		hxb->lastByte = hxb->buffer[0];
		HAL_UART_Receive_IT(hxb->huart, &hxb->buffer[0], 1);		// check next byte
		return IMU_HEADER_ERROR;
	}
}

enum XB_STATUS huansic_xb_dma_isr(XB_HandleTypeDef *hxb) {
	if (!hxb)
		return XB_ERROR;

	if (hxb->nextPackageID == 0x00)
		return huansic_xb_decodeHeader(hxb);
	else
		return huansic_xb_decodeBody(hxb);
}
