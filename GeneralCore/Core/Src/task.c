/*
 * task.c
 *
 *  Created on: 2023年3月16日
 *      Author: Ming
 */
#include "task.h"

extern Coordinate myCoord;
extern fCoordinate EstiCoord;
extern uint8_t CoordinateUpdate;
extern Coordinate allyBeacons[3];
extern Coordinate oppoBeacons[3];
extern Coordinate want_allyBeacons[3];
extern Rectangle obstacles[5];
extern Order *delivering[5];
extern uint8_t delivering_num;
extern XB_HandleTypeDef hxb;
extern uint8_t allyBeacons_num;
extern uint8_t oppoBeacons_num;
extern uint8_t overtime;

void set_Beacons(void)
{
	uint8_t i;
	for(i = 0;i < 3; i++){
		GotoDestination(want_allyBeacons[i], 0);
		HAL_Delay(100);
		huansic_xb_setBeacon(&hxb);
		HAL_Delay(100);
	}
}

void Cal_Battery_Coord(void)
{
	uint8_t seted = 0;
	//set a signal
	want_allyBeacons[seted].x = 127;
	want_allyBeacons[seted].y = 60;
	seted += 1;
	//set a signal
	want_allyBeacons[seted].x = 127;
	want_allyBeacons[seted].y = 120;
	seted += 1;
	//set a signal
	want_allyBeacons[seted].x = 127;
	want_allyBeacons[seted].y = 180;
	seted += 1;
}

void Get_packet(Coordinate merchant)
{
	GotoDestination(merchant,0);
}

void Send_packet(Coordinate consumer)
{
	GotoDestination(consumer,0);
}

Coordinate Get_nearest_consumer(void)
{
	int16_t mindis = 512;
	int16_t distance;
	uint8_t minindex = 0;
	if(delivering_num == 0)
		return myCoord;
	uint32_t nowtime = HAL_GetTick();
	for(uint8_t i = 0;i < delivering_num; i++)
	{
		if(delivering[i]->timeLimit + delivering[i]->startTime - nowtime < 5000){
			minindex = i;
			overtime = 1;
			break;
		}
		distance = abs(myCoord.x - delivering[i]->destCoord.x) + abs(myCoord.y - delivering[i]->destCoord.y);
		if(distance < mindis){
			mindis = distance;
			minindex = i;
		}
	}
	Coordinate nearest = delivering[minindex]->destCoord;
	return nearest;

}

Coordinate Get_nearest_Beacon(void){
	int16_t mindis = 512;
	int16_t distance;
	uint8_t minindex = 0;
//	uint8_t j = 0;
	for(uint8_t i = 0;i < allyBeacons_num;i++)
	{
//		for(j = 0;j < oppoBeacons_num; j++){
//			if((allyBeacons[i].x - oppoBeacons[j].x)*(allyBeacons[i].x - oppoBeacons[j].x) + (allyBeacons[i].y - oppoBeacons[j].y)*(allyBeacons[i].y - oppoBeacons[j].y) < 400)
//				break;
//		}
//		if(j < oppoBeacons_num)
//			continue;
		distance = abs(myCoord.x - allyBeacons[i].x) + abs(myCoord.y - allyBeacons[i].y);
		if(distance < mindis){
			mindis = distance;
			minindex = i;
		}
	}
	Coordinate nearest = allyBeacons[minindex];
	return nearest;
}
