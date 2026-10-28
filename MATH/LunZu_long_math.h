#ifndef LUNTUI_LONG_MATH_H
#define LUNTUI_LONG_MATH_H

#include "main.h"
#include <math.h>

#define Normal_long 100.0f//根据实际结构设定 mm单位

#define leg_change_speed 50.0f//腿长变化速度 mm/s

typedef struct
{
		float L1;//固定点之间的距离mm
		float L2;
		float L3;
		float L4;
		float L5;
	
		float x;
		float y;
	
		float angle1;//相对水平L1(+x轴)的绝对方向角 rad
		float angle2;
	
		float theta1;//L2相对竖直向下(-y轴)的角，向左偏为正 rad
		float theta2;//L5相对竖直向下(-y轴)的角，向右偏为正 rad
	
		float theta1_real;//DM4310反馈的关节实际角 rad
		float theta2_real;
		float theta1_err;//实际角-目标角 rad，腿摆到目标姿态时该值即为theta1_offset
		float theta2_err;//实际角-目标角 rad，腿摆到目标姿态时该值即为theta2_offset
	
		
}Leg;

void Leg_Init(Leg* leg,float L1,float L2,float L3,float L4,float L5);

void Leg_long_to_angle(Leg* leg,float long_goal,float x_goal);

extern volatile Leg leg;

#endif
