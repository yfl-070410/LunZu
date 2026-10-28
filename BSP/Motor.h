#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"


#define __LIMIT__(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define DM_Max_cur 10000

typedef struct
{
		uint16_t id;
		uint16_t state;
		int p_int;
		int v_int;
		int t_int;
		int kp_int;
		int kd_int;
		float pos;
		float vel;
		float tor;
		float Kp;
		float Kd;
		float Tmos;
		float Tcoil;
}DM_MotorInfo_TypeDef;

extern volatile DM_MotorInfo_TypeDef DM_4310_Motor[4];

extern volatile uint16_t dbg_dm_rx_cnt;

extern int __FLOAT_TO_UINT__(float x_float, float x_min, float x_max, int bits);
extern float __UINT_TO_FLOAT__(int x_int, float x_min, float x_max, int bits);

extern void DM_Motor_Enable(uint8_t CAN_Instance, uint32_t TxStdId);
extern void DM_Motor_Disable(uint8_t CAN_Instance, uint32_t TxStdId);

extern void DM_Motor_CAN_TxMessage(uint8_t CAN_Instance, uint32_t TxStdId,
												           float pos, float vel,float kp, float kd, float torq);
extern void DM_Motor_Info_Update(volatile DM_MotorInfo_TypeDef *motor, uint8_t *RX_Data);

//---------------DJ---------------//


#define DJ_Max_cur 16384

/* 发送给轮电机的电流上限，8192≈10A，按整车重量调整 */
#define DJ_Wheel_Cur_Max 14000

typedef struct
{
		int16_t  angle;
    int16_t speed;
    int16_t current;
    uint8_t temperature;
    uint8_t Error;
	
}DJ_MotorInfo_TypeDef;

extern volatile DJ_MotorInfo_TypeDef DJ_3508_Motor_L ;
extern volatile DJ_MotorInfo_TypeDef DJ_3508_Motor_R ;

extern void DJ_Motor_CAN_TxMessage(uint8_t CAN_Instance, uint32_t TxStdId,
                                  float Torque_1, float Torque_2, float Torque_3, float Torque_4);
extern void DM_Motor_Send_PosVel(uint8_t id, float p_des, float v_des);
extern void DJ_Motor_Info_Update(DJ_MotorInfo_TypeDef *motor, uint8_t *RX_Data);

#endif
