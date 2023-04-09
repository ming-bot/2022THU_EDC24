/*
 * huansic_jy62_it.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Zonghuan Wu
 */

/*
 *
 #ifdef HUANSIC_JY62_DEBUG
 himu->counter++;
 HAL_GPIO_WritePin(himu->port, himu->pin, himu->counter & (1 << 5));
 #endif
 *
 */
#include <huansic_jy62lib.h>

#define	HUANSIC_JY62_HEADER		0x55
#define	HUANSIC_JY62_ACCEL		0x51
#define	HUANSIC_JY62_OMEGA		0x52
#define	HUANSIC_JY62_THETA		0x53

uint8_t JY62_RESET_Z_ANGLE[] = { 0xFF, 0xAA, 0x52 };
uint8_t JY62_BAUD_115200[] = { 0xFF, 0xAA, 0x63 };
uint8_t JY62_BAUD_9600[] = { 0xFF, 0xAA, 0x64 };

// private function prototypes
static inline void __huansic_jy62_decode_accel(JY62_HandleTypeDef *himu, uint8_t location);
static inline void __huansic_jy62_decode_omega(JY62_HandleTypeDef *himu, uint8_t location);
static inline void __huansic_jy62_decode_theta(JY62_HandleTypeDef *himu, uint8_t location);
static inline void __huansic_jy62_decode_temp(JY62_HandleTypeDef *himu, uint8_t location);

/*
 * 		Initializes the port of the IMU.
 * 		@param	himu	jy62 pending initialization
 * 		@retval	enum IMU_STATUS
 */
enum IMU_STATUS huansic_jy62_init(JY62_HandleTypeDef *himu) {
	// perform some necessary checks
	if (!himu)
		return IMU_ERROR;

	if (!himu->huart)
		return IMU_ERROR;

	if (himu->huart->Init.BaudRate != 9600) {		// if it is not 9600bps
		// make it 9600 to send the package
		HAL_UART_DeInit(himu->huart);
		himu->huart->Init.BaudRate = 9600;
		HAL_UART_Init(himu->huart);
	}
	HAL_UART_Transmit(himu->huart, JY62_BAUD_115200, 3, 10);
	HAL_UART_DeInit(himu->huart);
	himu->huart->Init.BaudRate = 115200;
	HAL_UART_Init(himu->huart);

	HAL_Delay(3);

	// reset z-axis angle
	HAL_UART_Transmit(himu->huart, JY62_RESET_Z_ANGLE, 3, 10);

	// instead, just use DMA
	himu->pending_alignment = 0;
	HAL_UART_Receive_DMA(himu->huart, &himu->buffer[0], 33);
	__HAL_DMA_DISABLE_IT(himu->hdma, DMA_IT_HT);		// disable half transfer interrupt

#ifdef HUANSIC_JY62_DEBUG
	himu->counter = 0;
#endif

	return IMU_OK;
}

/*
 * 		Handles the dma interrupts.
 * 		@param	himu	jy62 whose port has sent out the interrupt signal
 * 		@retval	enum IMU_STATUS
 */
enum IMU_STATUS huansic_jy62_dma_isr(JY62_HandleTypeDef *himu) {
	if (!himu)
		return IMU_ERROR;

	uint8_t temp8, i, i11;

	for (i = 0, i11 = 0; i < 3; i++, i11 += 11) {
		if (himu->buffer[0 + i11] != HUANSIC_JY62_HEADER) {		// header mis-aligned
			himu->pending_alignment = 1;		// enter aligning mode if not already
			HAL_UART_Receive_IT(himu->huart, &himu->buffer[0], 1);		// check next byte
			if (i) {
				__huansic_jy62_decode_temp(himu, i - 1);
				himu->lastUpdated = HAL_GetTick();		// record if there has been a valid one
			}
			return IMU_HEADER_ERROR;
		} else {
			// check sum
			temp8 = himu->buffer[0 + i11];
			temp8 += himu->buffer[1 + i11];
			temp8 += himu->buffer[2 + i11];
			temp8 += himu->buffer[3 + i11];
			temp8 += himu->buffer[4 + i11];
			temp8 += himu->buffer[5 + i11];
			temp8 += himu->buffer[6 + i11];
			temp8 += himu->buffer[7 + i11];
			temp8 += himu->buffer[8 + i11];
			temp8 += himu->buffer[9 + i11];

			if (temp8 != himu->buffer[10 + i11]) {		// check
				himu->pending_alignment = 1;		// enter aligning mode if not already
				HAL_UART_Receive_IT(himu->huart, &himu->buffer[0], 1);		// check next byte
				if (i) {
					__huansic_jy62_decode_temp(himu, i - 1);
					himu->lastUpdated = HAL_GetTick();		// record if there has been a valid one
				}
				return IMU_SUM_ERROR;
			}

			if (himu->buffer[1 + i11] == HUANSIC_JY62_ACCEL) 		// then decode
				__huansic_jy62_decode_accel(himu, i);
			else if (himu->buffer[1 + i11] == HUANSIC_JY62_OMEGA)
				__huansic_jy62_decode_omega(himu, i);
			else if (himu->buffer[1 + i11] == HUANSIC_JY62_THETA)
				__huansic_jy62_decode_theta(himu, i);
			else {
				himu->pending_alignment = 1;		// enter aligning mode if not already
				HAL_UART_Receive_IT(himu->huart, &himu->buffer[0], 1);		// check next byte
				if (i) {
					__huansic_jy62_decode_temp(himu, i - 1);
					himu->lastUpdated = HAL_GetTick();		// record if there has been a valid one
				}
				return IMU_PID_ERROR;
			}
		}

	}

	// it should only reach this point if the package is fully valid
	himu->lastUpdated = HAL_GetTick();
	__huansic_jy62_decode_temp(himu, 2);
	// start to receive the next package
	HAL_UART_Receive_DMA(himu->huart, &himu->buffer[0], 33);
	__HAL_DMA_DISABLE_IT(himu->hdma, DMA_IT_HT);		// disable half transfer interrupt

#ifdef HUANSIC_JY62_DEBUG
	himu->counter++;
	HAL_GPIO_WritePin(himu->port, himu->pin, himu->counter & (1 << 5));
#endif

	return IMU_OK;
}

/*
 * 		Handles the interrupts.
 * 		@param	himu	jy62 whose port has sent out the interrupt signal
 * 		@retval	enum IMU_STATUS
 */
enum IMU_STATUS huansic_jy62_isr(JY62_HandleTypeDef *himu) {
	if (!himu)
		return IMU_ERROR;

	if (himu->buffer[0] != HUANSIC_JY62_HEADER) {		// header mis-aligned
		himu->pending_alignment = 1;		// enter aligning mode if not already
		HAL_UART_Receive_IT(himu->huart, &himu->buffer[0], 1);		// check next byte
		return IMU_HEADER_ERROR;
	} else {
		// header just aligned
		himu->pending_alignment = 0;
		HAL_UART_Receive_DMA(himu->huart, &himu->buffer[1], 32);		// receive the rest
		__HAL_DMA_DISABLE_IT(himu->hdma, DMA_IT_HT);		// disable half transfer interrupt
		return IMU_OK;
	}
}

/*
 * 		Handles the dma errors.
 * 		@param	himu	jy62 whose port has sent out the error
 * 		@retval	enum IMU_STATUS
 */
void huansic_jy62_dma_error(JY62_HandleTypeDef *himu) {
	// nothing much to do with error
	himu->pending_alignment = 1;
//	if(himu->huart->gState != HAL_UART_STATE_READY){
//		himu->huart->gState = HAL_UART_STATE_READY;
//	}
//	if(himu->huart->RxState != HAL_UART_STATE_READY){
//		himu->huart->RxState = HAL_UART_STATE_READY;
//	}
//	if(himu->hdma->State != HAL_DMA_STATE_READY){
//		himu->hdma->State = HAL_DMA_STATE_READY;
//	}
	HAL_UART_Receive_IT(himu->huart, &himu->buffer[0], 1);
}

/*
 * 		Handles the errors.
 * 		@param	himu	jy62 whose port has sent out the error
 * 		@retval	enum IMU_STATUS
 */
void huansic_jy62_it_error(JY62_HandleTypeDef *himu) {
	// nothing much to do with error
	himu->pending_alignment = 1;
//	if(himu->huart->gState != HAL_UART_STATE_READY){
//		himu->huart->gState = HAL_UART_STATE_READY;
//	}
//	if(himu->huart->RxState != HAL_UART_STATE_READY){
//		himu->huart->RxState = HAL_UART_STATE_READY;
//	}
	HAL_UART_Receive_IT(himu->huart, &himu->buffer[0], 1);
}

/***************	functions used by the library; not visible to users		***************/

static inline void __huansic_jy62_decode_accel(JY62_HandleTypeDef *himu, uint8_t location) {
	int16_t temp;
	uint8_t i;
	for (i = 0; i < 3; i++) {
		temp = himu->buffer[3 + 2 * i + location * 11];
		temp <<= 8;
		temp |= himu->buffer[2 + 2 * i + location * 11];
		himu->accel[i] = (float) temp * 16 * 9.8 / 32768;
	}
}

static inline void __huansic_jy62_decode_omega(JY62_HandleTypeDef *himu, uint8_t location) {
	int16_t temp;
	uint8_t i;
	for (i = 0; i < 3; i++) {
		temp = himu->buffer[3 + 2 * i + location * 11];
		temp <<= 8;
		temp |= himu->buffer[2 + 2 * i + location * 11];
		himu->omega[i] = (float) temp * 2000 / 32768;
	}
}

static inline void __huansic_jy62_decode_theta(JY62_HandleTypeDef *himu, uint8_t location) {
	int16_t temp;
	uint8_t i;
	for (i = 0; i < 3; i++) {
		temp = himu->buffer[3 + 2 * i + location * 11];
		temp <<= 8;
		temp |= himu->buffer[2 + 2 * i + location * 11];
		himu->theta[i] = (float) temp * 180 / 32768;
	}
}

static inline void __huansic_jy62_decode_temp(JY62_HandleTypeDef *himu, uint8_t location) {
	int16_t temp;

	temp = himu->buffer[8 + location * 11];
	temp <<= 8;
	temp |= himu->buffer[9 + location * 11];
	himu->temperature = (float) temp / 340 + 36.53;
}
