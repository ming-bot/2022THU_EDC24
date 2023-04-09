/*
 * ming_malloc.h
 *
 *  Created on: 2022年11月21日
 *      Author: Ming
 */

#ifndef INC_MING_MALLOC_H_
#define INC_MING_MALLOC_H_

#include "stm32f1xx_hal.h"
#include "huansic_types.h"
#include "huansic_malloc.h"
#include "stdlib.h"

#define QUEUE_SIZE 2048
#define LIST_SIZE 2048
#define LANE_SIZE 32

#pragma pack (2)

typedef struct
{
	uint16_t Head;                    // 2
	uint16_t Length;                  // 2
	Coordinate buffer[LANE_SIZE];     // 4 * LANE_SIZE
}Lane;                                // 4 * LANE_SIZE + 4

typedef struct
{
	uint8_t length;
	uint8_t new;
	Coordinate buffer[100];
}Order_list;

#pragma pack ()

void Queue_init(void);
void List_init(void);
void Lane_init(void);

// Lane part
uint8_t Insert_inLane(Coordinate *head_coor, uint8_t head_index);

//order list
void order_list_init(void);
void order_append(Order *an_order);
int8_t Get_nearest_order(void);

void exitpoints_init(void);

#endif /* INC_MING_MALLOC_H_ */
