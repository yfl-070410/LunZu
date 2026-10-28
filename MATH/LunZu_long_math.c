#include "LunZu_long_math.h"
#include <math.h>

volatile Leg leg;

void Leg_Init(Leg* leg,float L1,float L2,float L3,float L4,float L5)
{
		leg->L1= L1;
		leg->L2= L2;
		leg->L3= L3;
		leg->L4= L4;
		leg->L5= L5;

		leg->x=L1/2.0f;

		/* 机械图y轴向上，足端在髋部下方，故y取负 */
		leg->y=-Normal_long;

		leg->theta1_real=0.0f;
		leg->theta2_real=0.0f;
		leg->theta1_err=0.0f;
		leg->theta2_err=0.0f;
}


void Leg_long_to_angle(Leg* leg,float long_goal,float x_goal)
{
		leg->y=-long_goal;

		/* 足端水平位置：以左腿（或右腿）左侧铰点为原点，x向右 */
		leg->x=leg->L1/2.0f + x_goal;

		float r1, r2, c1, c2;

		r1 = sqrtf(leg->x * leg->x + leg->y * leg->y);
		r2 = sqrtf((leg->x - leg->L1) * (leg->x - leg->L1) + leg->y * leg->y);

		
		if(r1 < 1e-6f) r1 = 1e-6f;
		if(r2 < 1e-6f) r2 = 1e-6f;

		c1 = (leg->L2*leg->L2 + r1*r1 - leg->L3*leg->L3) / (2.0f *leg->L2 * r1);
		c2 = (leg->L5*leg->L5 + r2*r2 - leg->L4*leg->L4) / (2.0f *leg->L5 * r2);

		if (c1 >  1.0f) c1 =  1.0f;  if (c1 < -1.0f) c1 = -1.0f;
		if (c2 >  1.0f) c2 =  1.0f;  if (c2 < -1.0f) c2 = -1.0f;

		leg->angle1 = atan2f(leg->y, leg->x)    - acosf(c1);
		leg->angle2 = atan2f(leg->y, leg->x - leg->L1) + acosf(c2);


		leg->theta1 = -(leg->angle1 + 1.5707963f);
		leg->theta2 =  (leg->angle2 + 1.5707963f);
}
