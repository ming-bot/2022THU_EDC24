 /*
 * positionpid.c
 *
 *  Created on: 2023年2月20日
 *      Author: ming
 */
#include "positionpid.h"

extern Coordinate myCoord;
extern fCoordinate EstiCoord;
extern uint8_t CoordinateUpdate;
#define PATH_PID_TOLERANCE 5
#define MAX_SPEED 2500.0
#define MIN_SPEED 1000.0

extern float initangleZ;
extern JY62_HandleTypeDef himu;
extern Motor_HandleTypeDef cmotor_lf, cmotor_rf, cmotor_lb, cmotor_rb;
extern Lane pathlane;
extern Coordinate exitpoints[8];

uint8_t GotoDestination(Coordinate Destination, uint8_t mode)
{
	if(mode == 0)
	{
		while(1)
		{
			CheckCoord();
			Position_P(&EstiCoord, &Destination);
			CheckCoord();
			if(abs(EstiCoord.x - Destination.x) + abs(EstiCoord.y - Destination.y) <= PATH_PID_TOLERANCE)
			{
				chao_move_angle(0,0);
				return 1;
			}
		}
	}
	//只避障墙壁
	else if(mode == 2){
		uint8_t myCoordState, DesCoordState = 0;
		if(myCoord.x > 40 && myCoord.x < 214 && myCoord.y > 40 && myCoord.y < 214)
			myCoordState = 0;
		else{
			//计算起点划分
			if(myCoord.x > myCoord.y){
				if(myCoord.x <= 254 - myCoord.y)
					myCoordState = 1;
				else
					myCoordState = 2;
			}
			else{
				if(myCoord.x <= 254 - myCoord.y)
					myCoordState = 4;
				else
					myCoordState = 3;
			}
		}
		if(Destination.x > 40 && Destination.x < 214 && Destination.y > 40 && Destination.y < 214)
			DesCoordState = 0;
		else{
			// 计算终点划分
			if(Destination.x > Destination.y){
				if(Destination.x <= 254 - Destination.y)
					DesCoordState = 1;
				else
					DesCoordState = 2;
			}
			else{
				if(Destination.x <= 254 - Destination.y)
					DesCoordState = 4;
				else
					DesCoordState = 3;
			}
		}
		// 里 里，或者在外面同一区域的直接走
		if((myCoordState == 0 && DesCoordState == 0)||(myCoord.x<38&&Destination.x<38)||(myCoord.x>216&&Destination.x>216)||(myCoord.y<38&&Destination.y<38)||(myCoord.y>216&&Destination.y>216)){
			while(1){
				CheckCoord();
				Position_P(&EstiCoord, &Destination);
				CheckCoord();
				if(abs(EstiCoord.x - Destination.x) + abs(EstiCoord.y - Destination.y) <= PATH_PID_TOLERANCE){
					chao_move_angle(0,0);
					return 1;
				}
			}
		}
		//外外或者里外
		else
		{
			Coordinate middle1 , middle2;
			//里外或歪理
			if(myCoordState == 0 || DesCoordState == 0){
				uint8_t index = (myCoordState > DesCoordState)?myCoordState : DesCoordState;
				if(myCoordState == 0){
					middle1 = exitpoints[index * 2 - 1];
					middle2 = exitpoints[index * 2 - 2];
				}
				else{
					middle2 = exitpoints[index * 2 - 1];
					middle1 = exitpoints[index * 2 - 2];
				}
			}
			// 外外
			else{
				// 跨俩区域
				if(abs(DesCoordState - myCoordState) % 2 == 0)
				{
					middle1 = exitpoints[myCoordState * 2 - 2];
					middle2 = exitpoints[DesCoordState * 2 - 2];
				}
				// 只跨一个区域
				else{
					if(DesCoordState * myCoordState == 4){
						middle1.x = 18;
						middle1.y = 18;
						middle2 = middle1;
					}
					else if(DesCoordState * myCoordState == 2){
						middle1.x = 236;
						middle1.y = 18;
						middle2 = middle1;
					}
					else if(DesCoordState * myCoordState == 6){
						middle1.x = 236;
						middle1.y = 236;
						middle2 = middle1;
					}
					else if(DesCoordState * myCoordState == 12)
					{
						middle1.x = 18;
						middle1.y = 236;
						middle2 = middle1;
					}
				}
			}
			while(1){
				CheckCoord();
				Position_P(&EstiCoord, &middle1);
				CheckCoord();
				if(abs(EstiCoord.x - middle1.x) + abs(EstiCoord.y - middle1.y) <= PATH_PID_TOLERANCE){
					chao_move_angle(0,0);
					break;
				}
			}
			while(1){
				CheckCoord();
				Position_P(&EstiCoord, &middle2);
				CheckCoord();
				if(abs(EstiCoord.x - middle2.x) + abs(EstiCoord.y - middle2.y) <= PATH_PID_TOLERANCE){
					chao_move_angle(0,0);
					break;
				}
			}
			while(1){
				CheckCoord();
				Position_P(&EstiCoord, &Destination);
				CheckCoord();
				if(abs(EstiCoord.x - Destination.x) + abs(EstiCoord.y - Destination.y) <= PATH_PID_TOLERANCE){
					chao_move_angle(0,0);
					return 1;
				}
			}
		}
	}
	return 0;
}

float Angle_normalization(float angle)
{
	float raw_angle = angle;
	while(raw_angle < 0)
	{
		raw_angle += 360;
	}
	while(raw_angle > 360)
	{
		raw_angle -= 360;
	}
	return raw_angle;
}

float CalSpeed(int16_t x, int16_t y)
{
	float kp = 50.0;

	float Speed = kp * (abs(x) + abs(y));
	if(Speed > MAX_SPEED)
	{
		Speed = MAX_SPEED;
	}
	if(Speed < MIN_SPEED)
	{
		Speed = MIN_SPEED;
	}
	return Speed;
}

uint8_t CheckCoord(void)
{
	if(CoordinateUpdate == 1)
	{
		EstiCoord.x = (float)myCoord.x;
		EstiCoord.y = (float)myCoord.y;
		CoordinateUpdate = 0;
		return 1;
	}
	return 0;
}

float Get_v_x(void)
{
	float v_x = (cmotor_rf.lastSpeed - cmotor_lf.lastSpeed + cmotor_lb.lastSpeed - cmotor_rb.lastSpeed) * 50.0 / 20000;
	return v_x;
}

float Get_v_y(void)
{
	float v_y = (cmotor_rf.lastSpeed + cmotor_lf.lastSpeed + cmotor_lb.lastSpeed + cmotor_rb.lastSpeed) * 60.0 / 20000;
	return v_y;
}

void Position_P(fCoordinate* cur, Coordinate* goal)
{
	float x_error = cur->x - goal->x;
	float y_error = goal->y - cur->y;
	if (y_error == 0)
	{
		if(x_error < 0)
		{
			chao_move_angle(270, CalSpeed(x_error, 0));
		}
		else if(x_error > 0)
		{
			chao_move_angle(90, CalSpeed(x_error, 0));
		}
		else
		{
			chao_move_angle(0, 0);
		}
	}
	else
	{
		float azimuth = atan((float)(x_error)/(y_error));
		if(y_error < 0)
		{
			azimuth += M_PI;
		}
		azimuth = azimuth * 360.0 / (2 * M_PI);
		azimuth = Angle_normalization(azimuth);
		float angle = azimuth - Angle_normalization(initangleZ - himu.theta[2]);
		angle = Angle_normalization(angle);

		if(cur->x < 10 || cur->x > 244 || cur->y < 10 || cur->y >244)
			move_angle_omega(angle, CalSpeed(x_error, y_error));
		else
			chao_move_angle(angle, CalSpeed(x_error, y_error));
	}
//	CheckCoord();
	uint32_t timestart = HAL_GetTick();
	HAL_Delay(10); // delay 10 ms = 100 Hz
	if(CheckCoord() == 0)
	{
		float lf_v = cmotor_lf.lastSpeed;
		float lb_v = cmotor_lb.lastSpeed;
		float rf_v = cmotor_rf.lastSpeed;
		float rb_v = cmotor_rb.lastSpeed;
//		float v_x = -((rf_v - lf_v + lb_v - rb_v) / 500);
		float v_x = ((rf_v - lf_v + lb_v - rb_v) / 100);
		float v_y = ((rf_v + lf_v + lb_v + rb_v) / 100);
		uint32_t timeend = HAL_GetTick();
		EstiCoord.x = EstiCoord.x + (timeend - timestart) * 0.001 * v_x;
		EstiCoord.y = EstiCoord.y + (timeend - timestart) * 0.001 * v_y;
	}
}
