#include "bsp_pid.h"


void PID_Init(PID *pid,float KP,float KI,float KD,float integral_max ,float output_max)
{
		pid->KP=KP;
		
		pid->KI=KI;
	
		pid->KD=KD;
	
		pid->integral_max=integral_max;
	
		pid->output_max=output_max;
	
		pid->error=0;
	
		pid->tragl=0;
		
		pid->feedback=0;
	
		pid->integral=0;
}

float PID_Calc(PID *pid,float tragl,float feedback)
{
		pid->tragl=tragl;
		pid->feedback=feedback;
	
		pid->error=pid->tragl-pid->feedback;
	
		pid->integral +=pid->error;
	
		if(pid->integral>pid->integral_max)
			pid->integral=pid->integral_max;
		
		if(pid->integral<-pid->integral_max)
			pid->integral=-pid->integral_max;
	
		pid->out=pid->KP *(pid->tragl-pid->feedback)+pid->KI*pid->integral+pid->KD*(pid->error-pid->last_error);
	
		
		pid->last_error=pid->error;
	
		if(pid->out>pid->output_max)
			pid->out=pid->output_max;
		if(pid->out<-pid->output_max)
			pid->out=-pid->output_max;
		
		return pid->out;
}

float PID_Cascade_Calc(PID * inpid,PID * outpid,float tragl,float infeedback,float outfeedback)
{
		inpid->tragl=tragl;
		inpid->feedback=infeedback;
	
		inpid->error=inpid->tragl-inpid->feedback;
	
		inpid->integral +=inpid->error;
	
		if(inpid->integral>inpid->integral_max)
			inpid->integral=inpid->integral_max;
		
		if(inpid->integral<-inpid->integral_max)
			inpid->integral=-inpid->integral_max;
	
		inpid->out=inpid->KP *(inpid->tragl-inpid->feedback)+inpid->KI*inpid->integral+inpid->KD*(inpid->error-inpid->last_error);
	
		
		inpid->last_error=inpid->error;
	
		if(inpid->out>inpid->output_max)
			inpid->out=inpid->output_max;
		if(inpid->out<-inpid->output_max)
			inpid->out=-inpid->output_max;
		
		
		
		outpid->tragl=inpid->out;
		outpid->feedback=outfeedback;
	
		outpid->error=outpid->tragl-outpid->feedback;
	
		outpid->integral +=outpid->error;
	
		if(outpid->integral>outpid->integral_max)
			outpid->integral=outpid->integral_max;
		
		if(outpid->integral<-outpid->integral_max)
			outpid->integral=-outpid->integral_max;
	
		outpid->out=outpid->KP *(outpid->tragl-outpid->feedback)+outpid->KI*outpid->integral+outpid->KD*(outpid->error-outpid->last_error);
	
		
		outpid->last_error=outpid->error;
	
		if(outpid->out>outpid->output_max)
			outpid->out=outpid->output_max;
		if(outpid->out<-outpid->output_max)
			outpid->out=-outpid->output_max;
		
		return outpid->out;
	
}
