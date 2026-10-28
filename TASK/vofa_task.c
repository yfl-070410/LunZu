#include "vofa_task.h"

Vofa_JustFloat_TypeDef vofaPrintf;
extern UART_HandleTypeDef huart7;


void vofa_task(void const * pvParameters)
{
	vofaPrintf.tail[0]=0x00;
	vofaPrintf.tail[1]=0x00;
	vofaPrintf.tail[2]=0x80;
	vofaPrintf.tail[3]=0x7f;
	while (1)
	{
		HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&vofaPrintf, sizeof(vofaPrintf));
		vTaskDelay(10);
	}
}




