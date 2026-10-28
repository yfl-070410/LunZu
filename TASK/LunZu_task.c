#include "LunZu_task.h"

float leg_long=Normal_long;

void LunZu_long_task(void *argument)
{

  while(1)
  {
			leg_long += (float)Remote_Control.ch3/660.f * 0.01f *long_chance_speed ;
			
			
  }
}
