/*
 * positionpid.h
 *
 *  Created on: 2023年2月20日
 *      Author: ming
 */

#ifndef INC_POSITIONPID_H_
#define INC_POSITIONPID_H_

#include "huansic_types.h"
#include "ming_malloc.h"
#include "stdlib.h"
#include "math.h"
#include "collab_util.h"

//去某一点（可选是否避障）
uint8_t GotoDestination(Coordinate Destination, uint8_t mode);
//角度归一化（0~360°）
float Angle_normalization(float angle);
//计算行驶速度【min，max】
float CalSpeed(int16_t x, int16_t y);
// 获取方向和速度，并做正向运动学设置电机速度
void Position_P(fCoordinate *cur, Coordinate *goal);
//更新EstCoord
uint8_t CheckCoord(void);
//逆向运动学（x轴）
float Get_v_x(void);
//逆向运动学（y轴）
float Get_v_y(void);

#endif /* INC_POSITIONPID_H_ */
