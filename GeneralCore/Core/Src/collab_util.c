/*
 * collab_util.c
 *
 *  Created on: Oct 19, 2022
 *      Author: Zonghuan Wu
 */

#include "collab_util.h"

/* constants and definitions */
#define PATH_DEFAULT_LINEAR_SPEED 10	// in cm/s
#define PATH_DEFAULT_ANGULAR_SPEED 1	// in rad/s
#define PATH_MAX_TOLERANCE 3 	//in cm
#define PATH_THRESH_ANGLE M_PI / 2 // in rad
#define MOTOR_TICKS_PER_REV 28		// per rotor turn
#define MOTOR_REDUCTION_RATIO 30
#define MOTOR_WHEEL_DIAMETER 33		// in mm
#define MOTOR_REV_PER_CM 82 // in rev

/*inside functions*/
#define max(a, b) (a > b ? a : b)
#define min(a, b) (a < b ? a : b)
#define HALFLENGTH 14

/* exported variables */
// game information 1
extern uint8_t gameStage; // 0: pre-match(standby); 1: first half; 2: second half
extern uint8_t gameStatus;			// 0: standby; 1: running
extern uint32_t gameStageTimeLimit;		// in ms
extern uint32_t gameStageTimeSinceStart;	// in ms
extern Rectangle obstacles[5];			// area that depletes charge faster
extern Coordinate allyBeacons[3];		// ally charging station coordinate
extern Coordinate oppoBeacons[3];		// opponent charging station coordinate

// game information 2
extern Order *delivering[8];		// package picked up but not yet delivered

// game information 3
extern Coordinate myCoord;		// precise coordinate returned by game master
extern fCoordinate estimatedCoord;	// coordinate calculated by Kalman Filter
extern double angleZ, omegaZ, accelY;	// turning speed and linear acceleration
extern float myScore;				// current score returned by Master
extern float myCharge;				// current charge returned by Master
extern Motor_HandleTypeDef cmotor_lf, cmotor_rf, cmotor_lb, cmotor_rb;

// interchange information 1
extern uint32_t gameStageTimeLeft;		// in ms
extern Order_list orders;
extern XB_HandleTypeDef hxb;
extern uint8_t delivering_num;

//0 - 360 degree, 0 degree front, clockwise
void chao_move_angle(float _angle, float speed) {
	float angle_arc = (_angle / 180) * M_PI;
	cmotor_lf.goalSpeed = speed * cos(angle_arc) + speed * sin(angle_arc);
	cmotor_rf.goalSpeed = speed * cos(angle_arc) - speed * sin(angle_arc);
	cmotor_lb.goalSpeed = speed * cos(angle_arc) - speed * sin(angle_arc);
	cmotor_rb.goalSpeed = speed * cos(angle_arc) + speed * sin(angle_arc);
}

void move_angle_omega(float _angle, float speed){
	float omega = _angle <= 180 ? _angle: (_angle - 360);
	if(abs(omega) < 15)
		omega = 0;
	else
		omega = 0.3 * omega;
	float angle_arc = (_angle / 180) * M_PI;
	if(omega * HALFLENGTH + 1.414 * speed > 3500)
		speed = 2000;
	cmotor_lf.goalSpeed = speed * cos(angle_arc) + speed * sin(angle_arc) - omega * HALFLENGTH;
	cmotor_rf.goalSpeed = speed * cos(angle_arc) - speed * sin(angle_arc) + omega * HALFLENGTH;
	cmotor_lb.goalSpeed = speed * cos(angle_arc) - speed * sin(angle_arc) - omega * HALFLENGTH;
	cmotor_rb.goalSpeed = speed * cos(angle_arc) + speed * sin(angle_arc) + omega * HALFLENGTH;
}

void move_random(void){
	uint8_t isInBarrier = 0;
	if((myCoord.x >=38 && myCoord.x<=40) && ((myCoord.y >=38 && myCoord.y<=107)||(myCoord.y >=147 && myCoord.y<=216)))
		isInBarrier = 1;//左侧俩墙
	else if((myCoord.x >=214 && myCoord.x<=216) && ((myCoord.y >=38 && myCoord.y<=107)||(myCoord.y >=147 && myCoord.y<=216)))
		isInBarrier = 3;//右侧俩墙
	else if((myCoord.y >=38 && myCoord.y<=40) && ((myCoord.x >=38 && myCoord.x<=107)||(myCoord.x >=147 && myCoord.x<=216)))
		isInBarrier = 2;//上面俩墙
	else if((myCoord.y >=214 && myCoord.y<=216) && ((myCoord.x >=38 && myCoord.x<=107)||(myCoord.x >=147 && myCoord.x<=216)))
		isInBarrier = 4;//下面俩墙
	else{
		for(uint8_t i = 0;i < 5; i++){
			if(myCoord.x >= obstacles[i].coord1.x && myCoord.y >= obstacles[i].coord1.y && myCoord.x <= obstacles[i].coord2.x && myCoord.y <= obstacles[i].coord2.y){
				isInBarrier = 5;//在场上的障碍物里
				break;
			}
		}
	}
	if(isInBarrier == 0){
		while(orders.length == 0){
			cmotor_lf.goalSpeed = 500;
			cmotor_rf.goalSpeed = -500;
			cmotor_lb.goalSpeed = 500;
			cmotor_rb.goalSpeed = -500;
		}
	}
	else{
		Coordinate safeplace;
		if(isInBarrier == 1){
			safeplace.x = myCoord.x - 10;
			safeplace.y = myCoord.y;
			GotoDestination(safeplace, 0);
		}
		else if(isInBarrier == 2){
			safeplace.x = myCoord.x;
			safeplace.y = myCoord.y - 10;
			GotoDestination(safeplace, 0);
		}
		else if(isInBarrier == 3){
			safeplace.x = myCoord.x + 10;
			safeplace.y = myCoord.y;
			GotoDestination(safeplace, 0);
		}
		else if(isInBarrier == 4){
			safeplace.x = myCoord.x;
			safeplace.y = myCoord.y + 10;
			GotoDestination(safeplace, 0);
		}
		else{
			//TODO
		}
		while((orders.length + delivering_num) == 0){
			cmotor_lf.goalSpeed = 500;
			cmotor_rf.goalSpeed = -500;
			cmotor_lb.goalSpeed = 500;
			cmotor_rb.goalSpeed = -500;
		}
	}
}
