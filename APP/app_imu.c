#include "app_imu.h"

/**
  * @file       imu_task.c/h
  * @brief      IMU数据处理/解析任务，包含SPI初始化、加速度角速度温度读取、欧拉角解算、温度补偿
  * @history
  *  Version    Date            Author                  Modification
  *  V1.0.0     2025-11-19      FengYangjingbei         1. 完成初始版本
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */

#include "stdbool.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;

/*加速度、角速度传感器系数*/
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
static float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
static uint8_t pTxData;
static uint8_t pRxData;
static uint8_t spi1_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/*温度传感器相关系数*/
#define BMI088_TEMP_M 0x22
uint8_t temp_buf[8] = {0, 0, 0, 0, 0, 0}; // spi读取温度数据缓存

/*温度控制相关系数*/
#define IMU_HEATER_ENABLE 0                 // 温度控制总开关：0=关闭加热(仅读温度) 1=开启温度闭环
#define TARGET_TEMP 45                      // 目标温度
#define HEATER_PWM_PERIOD 50                // 加热PWM计数周期 = TIM3_ARR + 1 (PSC=95, ARR=49 → 1MHz/20kHz)
static TempPID_t temperature_pid;           // 温度控制PID结构体
static float imu_last_temperature_c = 0.0f; // 上一次采样的温度值（摄氏度，从传感器读取）

/*欧拉角解算实现相关系数*/
#define sampleFreq 500.0f                                                  // 采样频率（Hz），与 imu_task 的 2ms 周期一致
volatile float twoKp = (2.0f * 0.5f);                                      // 2*比例增益（Kp）
volatile float twoKi = (2.0f * 0.0f);                                      // 2*积分增益（Ki）
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 积分误差项（按Ki缩放）

/*IMU中间变量*/
static float gyro[3];                            // 角速度
static float accelerometer[3];                   // 加速度
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 四元数
static float INS_angle[3];                       // 欧拉角

/*IMU数据结构体*/
IMU_RawData_TypeDef imu_raw_data;       // IMU原始数据结构体：包含加速度、角速度、温度
IMU_EulerAngles_TypeDef imu_euler_data; // IMU解算后欧拉角结构体

/* 内部函数声明 */
static void IMU_Temperature_Init(void);
static void IMU_Heater_Control(const IMU_RawData_TypeDef *raw);
static void IMU_ReadAccelerometer(IMU_RawData_TypeDef *raw);
static void IMU_ReadTemperature(IMU_RawData_TypeDef *raw);
static void IMU_ReadGyroscope(IMU_RawData_TypeDef *raw);
static void IMU_UpdateOrientation(IMU_RawData_TypeDef *raw, IMU_EulerAngles_TypeDef *angles);

/*====================温度PID相关接口====================*/

/**
 * @brief          温度PID控制器参数初始化
 * @param          kp 比例系数
 * @param          ki 积分系数
 * @param          kd 微分系数
 * @param          kf 前馈系数
 * @param          dt 控制周期（秒）
 * @param          out_max 输出最大值
 * @param          i_out_max 积分输出最大值
 * @param          dead_zone 死区范围
 * @retval         None
 */
void TempPID_Init_Params(float kp, float ki, float kd, float kf, float dt,
                         float out_max, float i_out_max, float dead_zone)
{
    // 控制器参数初始化
    temperature_pid.target = 0.0f; // 目标温度
    temperature_pid.now = 0.0f;    // 当前温度
    temperature_pid.kp = kp;       // 比例系数
    temperature_pid.ki = ki;       // 积分系数
    temperature_pid.kd = kd;       // 微分系数
    temperature_pid.kf = kf;       // 前馈系数
    temperature_pid.dt = dt;       // 控制周期
    temperature_pid.out = 0.0f;    // 输出值

    // 积分环节初始化
    temperature_pid.integral = 0.0f;             // 积分值
    temperature_pid.i_out_max = i_out_max;       // 积分输出最大值
    temperature_pid.i_separate_threshold = 0.0f; // 积分分离阈值
    temperature_pid.i_variable_speed_a = 0.0f;   // 积分变速参数A
    temperature_pid.i_variable_speed_b = 0.0f;   // 积分变速参数B

    // 微分环节初始化
    temperature_pid.d_first = PID_D_FIRST_DISABLE; // 微分先行使能状态

    // 限幅环节初始化
    temperature_pid.out_max = out_max;     // 输出最大值
    temperature_pid.dead_zone = dead_zone; // 死区范围

    // 状态记录初始化
    temperature_pid.prev_error = 0.0f;
    temperature_pid.prev_out = 0.0f;
    temperature_pid.prev_target = 0.0f;
    temperature_pid.prev_now = 0.0f;
}

/**
 * @brief          设置温度PID目标值
 * @param          target: 目标温度
 * @retval         None
 */
static void TempPID_SetTarget(float target)
{
    temperature_pid.target = target;
}

/**
 * @brief          设置温度PID当前值
 * @param          now: 当前温度
 * @retval         None
 */
static void TempPID_SetNow(float now)
{
    temperature_pid.now = now;
}

/**
 * @brief          温度PID计算（包含积分变速、积分分离、微分先行、前馈控制）
 * @param          None
 * @retval         None
 */
static void TempPID_Compute(void)
{
    float error = temperature_pid.target - temperature_pid.now;
    float abs_error = fabsf(error);
    float speed_ratio;

    // 死区判断
    if (abs_error < temperature_pid.dead_zone)
    {
        temperature_pid.target = temperature_pid.now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    // P环节
    float p_out = temperature_pid.kp * error;

    // 积分变速系数计算
    if (temperature_pid.i_variable_speed_a == 0.0f && temperature_pid.i_variable_speed_b == 0.0f)
    {
        // 固定积分速度
        speed_ratio = 1.0f;
    }
    else
    {
        // 变速积分
        if (abs_error <= temperature_pid.i_variable_speed_b)
        {
            speed_ratio = 1.0f;
        }
        else if (abs_error < (temperature_pid.i_variable_speed_a + temperature_pid.i_variable_speed_b))
        {
            speed_ratio = (temperature_pid.i_variable_speed_a + temperature_pid.i_variable_speed_b - abs_error) / temperature_pid.i_variable_speed_a;
        }
        else
        {
            speed_ratio = 0.0f;
        }
    }

    // 积分限幅
    if (temperature_pid.i_out_max > 0.0f && temperature_pid.ki > 0.0f)
    {
        // 计算积分上限
        float max_integral = temperature_pid.i_out_max / temperature_pid.ki;
        if (temperature_pid.integral > max_integral)
            temperature_pid.integral = max_integral;
        else if (temperature_pid.integral < -max_integral)
            temperature_pid.integral = -max_integral;
    }

    // I环节（积分分离）
    float i_out = 0.0f;
    if (temperature_pid.i_separate_threshold == 0.0f)
    {
        // 无积分分离
        temperature_pid.integral += speed_ratio * error * temperature_pid.dt;
        i_out = temperature_pid.ki * temperature_pid.integral;
    }
    else
    {
        // 积分分离使能
        if (abs_error < temperature_pid.i_separate_threshold)
        {
            temperature_pid.integral += speed_ratio * error * temperature_pid.dt;
            i_out = temperature_pid.ki * temperature_pid.integral;
        }
        else
        {
            temperature_pid.integral = 0.0f;
            i_out = 0.0f;
        }
    }

    // D环节
    float d_out = 0.0f;
    if (temperature_pid.kd != 0.0f)
    {
        if (temperature_pid.d_first == PID_D_FIRST_DISABLE)
        {
            // 无微分先行
            d_out = temperature_pid.kd * (error - temperature_pid.prev_error) / temperature_pid.dt;
        }
        else
        {
            // 微分先行使能
            d_out = temperature_pid.kd * (temperature_pid.out - temperature_pid.prev_out) / temperature_pid.dt;
        }
    }

    // 前馈环节
    float f_out = 0.0f;
    if (temperature_pid.kf != 0.0f)
    {
        f_out = (temperature_pid.target - temperature_pid.prev_target) * temperature_pid.kf;
    }

    // 总输出计算
    temperature_pid.out = p_out + i_out + d_out + f_out;

    // 输出限幅
    if (temperature_pid.out_max > 0.0f)
    {
        if (temperature_pid.out > temperature_pid.out_max)
            temperature_pid.out = temperature_pid.out_max;
        else if (temperature_pid.out < -temperature_pid.out_max)
            temperature_pid.out = -temperature_pid.out_max;
    }

    // 状态更新
    temperature_pid.prev_now = temperature_pid.now;
    temperature_pid.prev_target = temperature_pid.target;
    temperature_pid.prev_out = temperature_pid.out;
    temperature_pid.prev_error = error;
}

/*====================IMU初始化====================*/

/**
 * @brief          初始化IMU硬件及内部状态（SPI配置、定时器、PID等）
 * @param          None
 * @retval         None
 * @note           1.cubeMX中PC0(ACC_CS)、PC3(GYRO_CS)需要配置为推挽输出
                   2.将TIM3的CH4配置为PWM模式，预分频95、自动重装值49，引脚为PB01
 */
void IMU_Init(void)
{
    // 加速度计软复位
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
    pTxData = (0x7E & 0x7F);
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = 0xB6;
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);

    // 使能加速度计
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
    pTxData = (0x7D & 0x7F);
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = 0x04;
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);

    // 陀螺仪软复位
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
    pTxData = (0x14 & 0x7F);
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = 0xB6;
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(30);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    // 温度控制PID初始化
    TempPID_Init_Params(120, 0.2, 0.15, 0, 0.002, 90, 40, 0);
    // 温度传感器初始化
    IMU_Temperature_Init();
}

/*====================加速度、角速度实现函数====================*/

/**
 * @brief          读取加速度
 * @param          raw:IMU原始数据结构体
 * @retval         None
 */
static void IMU_ReadAccelerometer(IMU_RawData_TypeDef *raw)
{
    uint8_t i = 0;
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
    pTxData = (0x12 | 0x80);
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&hspi2, &pRxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
        ;
    while (i < 6)
    {
        HAL_SPI_Receive(&hspi2, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
            ;
        spi1_buf[i] = pRxData;
        i++;
    }
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);

    accelerometer[0] = ((int16_t)((spi1_buf[1]) << 8) | spi1_buf[0]) * BMI088_ACCEL_SEN;
    accelerometer[1] = ((int16_t)((spi1_buf[3]) << 8) | spi1_buf[2]) * BMI088_ACCEL_SEN;
    accelerometer[2] = ((int16_t)((spi1_buf[5]) << 8) | spi1_buf[4]) * BMI088_ACCEL_SEN;

    if (raw != NULL)
    {
        raw->ax = accelerometer[0];
        raw->ay = accelerometer[1];
        raw->az = accelerometer[2];
    }
}

/**
 * @brief          读取角速度
 * @param          raw:IMU原始数据结构体
 * @retval         None
 */
static void IMU_ReadGyroscope(IMU_RawData_TypeDef *raw)
{
    uint8_t i = 0;

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
    pTxData = (0x00 | 0x80);
    HAL_SPI_Transmit(&hspi2, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX)
        ;
    while (i < 8)
    {
        HAL_SPI_Receive(&hspi2, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX)
            ;
        spi1_buf[i] = pRxData;
        i++;
    }
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    if (spi1_buf[0] == 0x0F)
    {
        gyro[0] = ((int16_t)((spi1_buf[3]) << 8) | spi1_buf[2]) * BMI088_GYRO_SEN;
        gyro[1] = ((int16_t)((spi1_buf[5]) << 8) | spi1_buf[4]) * BMI088_GYRO_SEN;
        gyro[2] = ((int16_t)((spi1_buf[7]) << 8) | spi1_buf[6]) * BMI088_GYRO_SEN;
    }

    if (raw != NULL)
    {
        raw->gx = gyro[0];
        raw->gy = gyro[1];
        raw->gz = gyro[2];
    }
}

/*====================温度实现函数====================*/

/**
 * @brief          SPI片选拉低
 * @param          None
 * @retval         None
 */
static void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief          SPI片选拉高
 * @param          None
 * @retval         None
 */
static void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief          SPI读写字节（发送1字节同时接收1字节）
 * @param          txdata: 要发送的字节
 * @retval         接收到的字节
 */
static uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/**
 * @brief          BMI088多字节读取
 * @param          reg: 起始寄存器地址
 * @param          buf: 数据存储缓冲区
 * @param          len: 要读取的字节数
 * @retval         None
 */
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

/**
 * @brief          BMI088加速度计多字节读取（含片选控制）
 * @param          reg: 起始寄存器地址
 * @param          data: 数据存储缓冲区
 * @param          len: 要读取的字节数
 * @retval         None
 * @note           自动处理片选拉低、发送读指令、多字节读取、片选拉高
 */
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

/**
 * @brief          读取温度
 * @param          None
 * @retval         None
 */
static void IMU_ReadTemperature(IMU_RawData_TypeDef *raw)
{
    int16_t bmi088_raw_temp;
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, temp_buf, 2);

    bmi088_raw_temp = (int16_t)((temp_buf[0] << 3) | (temp_buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    raw->temperature_c = bmi088_raw_temp * 0.125f + 23.0f;
}

/*====================欧拉角解算实现函数====================*/

/**
 * @brief          快速平方根倒数（卡马克算法）
 * @param          x: 要计算平方根倒数的数值（正数）
 * @retval         1/sqrt(x)的近似值
 * @note           该算法是一种快速近似平方根倒数的方法，适用于实时系统
 */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief          Mahony AHRS 姿态解算（仅IMU模式）
 * @param          q[4]: 输出四元数（w,x,y,z，用于存储当前姿态的四元数表示）
 * @param          gx: 陀螺仪X轴角速度 (rad/s)
 * @param          gy: 陀螺仪Y轴角速度 (rad/s)
 * @param          gz: 陀螺仪Z轴角速度 (rad/s)
 * @param          ax: 加速度计X轴加速度 (m/s2)
 * @param          ay: 加速度计Y轴加速度 (m/s2)
 * @param          az: 加速度计Z轴加速度 (m/s2)
 * @retval         None
 * @note           该算法为四元数姿态解算算法，融合陀螺仪和加速度计数据，具有计算量小、稳定性好的特点
 *                 适用于低成本IMU，对传感器噪声有一定的抑制能力
 */
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 仅当加速度计数据有效时计算反馈（避免加速度计数据为0时的NaN）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // 归一化加速度计数据
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向和磁通量垂直向量
        halfvx = q[1] * q[3] - q[0] * q[2];
        halfvy = q[0] * q[1] + q[2] * q[3];
        halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // 误差计算（估计重力方向与实测重力方向的叉乘和）
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 计算并应用积分反馈（若使能）
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // 积分误差按Ki缩放
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // 应用积分反馈
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // 防止积分饱和
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 应用比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 积分四元数变化率
    gx *= (0.5f * (1.0f / sampleFreq)); // 预乘公共因子
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

/**
 * @brief          四元数转欧拉角（Z-Y-X旋转顺序，航空系）
 * @param          q[4]: 输入四元数
 * @param          yaw: 偏航角 (rad)
 * @param          pitch: 俯仰角 (rad)
 * @param          roll: 横滚角 (rad)
 * @retval         None
 * @note           欧拉角定义：
 *                 - 偏航角(yaw)：绕Z轴旋转，范围[-π, π]
 *                 - 俯仰角(pitch)：绕Y轴旋转，范围[-π/2, π/2]
 *                 - 横滚角(roll)：绕X轴旋转，范围[-π, π]
 *                 若需角度值，需将弧度值乘以180/π转换
 */
void Get_Angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

/**
 * @brief          IMU姿态更新（核心处理函数，由外部调用）
 * @param          raw: 输入IMU原始数据（陀螺仪+加速度计）
 * @param          angles: 输出解算后欧拉角（NULL时仅更新内部变量）
 * @retval         None
 * @note           统一调用Mahony算法和角度转换，将IMU原始数据转换为欧拉角
 */
static void IMU_UpdateOrientation(IMU_RawData_TypeDef *raw, IMU_EulerAngles_TypeDef *angles)
{
    MahonyAHRSupdateIMU(quat, raw->gx, raw->gy, raw->gz, raw->ax, raw->ay, raw->az);
    Get_Angle(quat, INS_angle, INS_angle + 1, INS_angle + 2);

    if (angles != NULL)
    {
        angles->yaw = INS_angle[0];
        angles->pitch = INS_angle[1];
        angles->roll = INS_angle[2];
    }
}

/*====================温度控制实现函数====================*/
/**
 * @brief          数值限幅函数
 * @param          value: 要限幅的原始数值
 * @param          min_val: 最小值
 * @param          max_val: 最大值
 * @retval         限幅后的数值
 * @note           将数值限制在[min_val, max_val]范围内，防止数值溢出导致硬件损坏
 */
static float Constrain(float value, float min_val, float max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}
/**
 * @brief          设置加热器PWM占空比
 * @param          duty: 目标PWM占空比(%)，范围0~100
 * @retval         None
 * @note           1. 加热器通过TIM3通道4的PWM波控制，PSC=95、ARR=49，
 *                    计数频率1MHz、PWM频率20kHz，计数范围0~49
 *                 2. 占空比限幅在0~90%，避免长时间满功率加热导致IMU损坏
 *                 3. PWM值计算：duty(%) * HEATER_PWM_PERIOD / 100，最终值在[0, 49]范围
 */
static void Set_Heater_PWM(float duty)
{
    uint32_t pwm_value;

    // 限幅90%的占空比
    duty = Constrain(duty, 0, 90);

    pwm_value = (uint32_t)((duty / 100.0f) * (float)HEATER_PWM_PERIOD);
    if (pwm_value >= HEATER_PWM_PERIOD)
    {
        pwm_value = HEATER_PWM_PERIOD - 1;
    }

    TIM3->CCR4 = pwm_value;
}

/**
 * @brief          IMU温度传感器初始化
 * @param          None
 * @retval         None
 * @note           初始化TIM3为PWM模式并启动，初始PWM占空比为0
 *                 需确保TIM3已在CubeMX中配置为PWM输出模式，预分频95、重载值49，通道4为PB01引脚
 */
static void IMU_Temperature_Init(void)
{
    // 无论温控开关如何，TIM3_CH4都启动并保持0%占空比，使PB01输出确定低电平
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    Set_Heater_PWM(0); // 初始关闭加热器
}

/**
 * @brief          IMU温度闭环控制
 * @param          raw: 输入IMU原始数据指针（含当前温度，NULL表示数据无效）
 * @retval         None
 * @note           1. 采用PID算法实现温度闭环控制，使IMU温度稳定在目标温度
 *                 2. 加入温度上限保护逻辑，防止温度过高损坏IMU
 *                 3. 支持传感器数据异常时使用上一次温度值，提高系统鲁棒性
 *                 4. 由 IMU_HEATER_ENABLE 控制：为0时强制输出0%占空比，PID结果被丢弃
 */
static void IMU_Heater_Control(const IMU_RawData_TypeDef *raw)
{
    float current_temp = raw ? raw->temperature_c : imu_last_temperature_c;

    // 温度上限保护
    if (current_temp >= 48.0f)
    {
        Set_Heater_PWM(0);
        return;
    }

    TempPID_SetNow(current_temp);
    TempPID_SetTarget(TARGET_TEMP);
    TempPID_Compute();

    // IMU_HEATER_ENABLE=0 时强制输出0%占空比：PB01确定低电平，避免MOS栅极悬空；温度读取与48度上限保护仍有效
    Set_Heater_PWM(IMU_HEATER_ENABLE ? temperature_pid.out : 0.0f);
}

/*====================外部调用函数====================*/

/**
 * @brief          读取一次IMU数据并执行温度补偿和姿态解算
 *                 该函数为IMU处理任务的核心执行函数
 * @param          None
 * @retval         None
 */
void IMU_Update(void)
{
    IMU_ReadAccelerometer(&imu_raw_data);                  // 读取加速度
    IMU_ReadGyroscope(&imu_raw_data);                      // 读取角速度
    IMU_ReadTemperature(&imu_raw_data);                    // 读取温度
    IMU_Heater_Control(&imu_raw_data);                     // 加热控制（从25度到45度大约需要10~15秒）
    IMU_UpdateOrientation(&imu_raw_data, &imu_euler_data); // 数据解算并更新欧拉角
}
