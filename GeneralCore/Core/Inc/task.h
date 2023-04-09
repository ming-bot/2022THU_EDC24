/*
 * task.h
 *
 *  Created on: 2023年3月16日
 *      Author: Administrator
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_

#include "main.h"
#include "positionpid.h"
#include "stm32f1xx_hal.h"
#include "huansic_types.h"

//计算充电桩位置
void Cal_Battery_Coord(void);
//取外卖
void Get_packet(Coordinate merchant);
//送外卖
void Send_packet(Coordinate consumer);
//设置充电桩
void set_Beacons(void);
//获取离自己最近的顾客（送单）
Coordinate Get_nearest_consumer(void);
//获取离自己最近的充电桩
Coordinate Get_nearest_Beacon(void);

#endif /* INC_TASK_H_ */
