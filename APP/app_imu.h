#ifndef APP_IMU_H
#define APP_IMU_H

#include "main.h"
#include "math.h"


/*以下是到用到的外部接口---------------------------------------------------------------------------*/	


//IMU原始数据结构体 包括角速度 加速度 温度
typedef struct
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float temperature_c;
} IMU_RawData_TypeDef;


//IMU解算后得到的欧拉角结构体
typedef struct
{
  float roll;
  float pitch;
  float yaw;
} IMU_EulerAngles_TypeDef;


// PID控制模式定义
#define PID_D_FIRST_DISABLE    0
#define PID_D_FIRST_ENABLE     1

// PID结构体定义
typedef struct {
    // 基础参数
    float target;
    float now;
    float kp, ki, kd, kf;
    float dt;
    float out;
    
    // 积分项相关
    float integral;
    float i_out_max;
    float i_separate_threshold;
    float i_variable_speed_a;
    float i_variable_speed_b;
    
    // 微分项相关
    int d_first;  // PID_D_FIRST_DISABLE 或 PID_D_FIRST_ENABLE
    
    // 限幅相关
    float out_max;
    float dead_zone;
    
    // 状态记录
    float prev_error;
    float prev_out;
    float prev_target;
    float prev_now;
} TempPID_t;


/**
 * @brief  初始化 IMU 硬件与内部状态（SPI、温控、定时器等）。
 */
void IMU_Init(void);

/**
 * @brief  读取一次IMU并在内部执行温度补偿与姿态更新
 * 等同于执行一次现有的接收处理流程
 */
void IMU_Update(void);

extern IMU_RawData_TypeDef imu_raw_data;       // IMU原始数据结构体：包含加速度、角速度、温度
extern IMU_EulerAngles_TypeDef imu_euler_data; // IMU解算后欧拉角结构体

#endif
