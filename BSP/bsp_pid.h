#ifndef BSP_PID_H
#define BSP_PID_H

#include "main.h"

typedef struct{
		float KP,KI,KD;
		
		float error,last_error;
	
		float integral;
	
		float out;
	
		float integral_max ,output_max;
	
		float tragl,feedback;
}PID;

void PID_Init(PID *pid,float KP,float KI,float KD,float integral_max ,float output_max);

float PID_Calc(PID *pid,float tragl,float feedback);

float PID_Cascade_Calc(PID * inpid,PID * outpid,float tragl,float infeedback,float outfeedback);
	
#endif
