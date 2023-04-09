/*
 * ming_malloc.c
 *
 *  Created on: 2022年11月21日
 *      Author: Ming
 */
#include "ming_malloc.h"

Lane pathlane;
Order_list orders;
extern Coordinate myCoord;
extern Coordinate exitpoints[8];

void Lane_init(void){
	pathlane.Head = 0;
	pathlane.Length = 0;
}

uint8_t Insert_inLane(Coordinate *head_coor, uint8_t head_index)
{
	if(!head_coor) return 0;
	pathlane.Head = 0;
	pathlane.Length = 16 - head_index;
	for(uint8_t i = head_index; i < 16; i++)
	{
		pathlane.buffer[pathlane.Head + i - head_index] = *(head_coor + i);
	}
	return 1;
}

void order_list_init(void)
{
	orders.length = 0;
	orders.new = 0;
}

void order_append(Order *an_order)
{
	if((an_order->startCoord.x != orders.buffer[orders.new].x) || (an_order->startCoord.y != orders.buffer[orders.new].y))
	{
		orders.buffer[orders.length].x = an_order->startCoord.x;
		orders.buffer[orders.length].y = an_order->startCoord.y;
		orders.new = orders.length;
		orders.length = orders.length + 1;
	}
}

int8_t Get_nearest_order(void)
{
	uint8_t i;
	int16_t mindis = 512;
	int8_t minindex = -1;
	if(orders.length == 0)
		return minindex;
	for(i=0;i < orders.length; i++)
	{
		int16_t distance = abs(orders.buffer[i].x - myCoord.x) + abs(orders.buffer[i].y - myCoord.y);
		if(distance < mindis)
		{
			mindis = distance;
			minindex = i;
		}
	}
    return minindex;
}

void exitpoints_init(void){
	exitpoints[0].x = 127;
	exitpoints[0].y = 18;
	exitpoints[1].x = 127;
	exitpoints[1].y = 60;
	exitpoints[2].x = 236;
	exitpoints[2].y = 127;
	exitpoints[3].x = 194;
	exitpoints[3].y = 127;
	exitpoints[4].x = 127;
	exitpoints[4].y = 236;
	exitpoints[5].x = 127;
	exitpoints[5].y = 194;
	exitpoints[6].x = 18;
	exitpoints[6].y = 127;
	exitpoints[7].x = 60;
	exitpoints[7].y = 127;
}
