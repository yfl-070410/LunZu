#ifndef CHASIS_TASK_H
#define CHASIS_TASK_H

#include "main.h"

#define chasis_max_speed 0.5f// m/s

#define wheel_R 0.05f // m 轮半径50mm，用于把3508转速折算成车体线速度 m/s

#define dt 0.002f

#define RC_LOST_MAX 50// 丢遥控判定阈值，2ms周期下约100ms

#define leg_long_min 80.0f

#define leg_long_max 150.0f

#define fallen_pitch_max 0.5f// 倒地判定俯仰角阈值 rad


/* 关节力矩输出上限，TMAX=10，先用小值试，站得住再往上加 */
#define DM_Leg_Tor_Max 2.0f

#endif
