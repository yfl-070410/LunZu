#include "main.h"

void imu_task(void *argument)
{
		IMU_EulerAngles_TypeDef gyro_snapshot;

		TickType_t xLastWakeTime_Imu;// 存储上一次唤醒时间的变量
    const TickType_t xDelay_Imu = pdMS_TO_TICKS(2);// 2ms 延时（转换为 tick）
    /*获取时间，用于vTaskDelayUntil*/
    xLastWakeTime_Imu = xTaskGetTickCount();
		while(1)
		{
				IMU_Update();
				gyro_snapshot = imu_euler_data;
				xQueueOverwrite(imu_to_chasis_queue,(void *)&gyro_snapshot);
			
				vTaskDelayUntil(&xLastWakeTime_Imu, xDelay_Imu);
		}

}
