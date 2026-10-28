#include "chasis_task.h"

IMU_EulerAngles_TypeDef imu;

/* 电机转向符号：电机零点 = 连杆竖直向下（与theta几何零点一致）
   theta1(L2左摆)对应电机负向；theta2(L5右摆)对应电机正向 */
#define DIR_TH1  (-1.0f)
#define DIR_TH2  ( 1.0f)

//测量后，修改初始化
/* L1,L2,L3,L4,L5,x,y,angle1,angle2,theta1,theta2 */
Leg leg_l={79.0f,72.5f,140.0f,140.0f,72.5f,39.5f,-100.0f,-2.9449f,-0.1967f, 1.3741f,1.3741f};
Leg leg_r={79.0f,72.5f,140.0f,140.0f,72.5f,39.5f,-100.0f,-2.9449f,-0.1967f, 1.3741f,1.3741f};



float trg_speed_F=0.0f;

float trg_speed_T=0.0f;

float trg_leg_long_r=Normal_long;

float trg_leg_long_l=Normal_long;

 float error_pitch;


float L_speed;
float R_speed;

PID pid_turn;
PID pid_f;
PID pid_pitch_speed;
PID pid_lun_speed_L;
PID pid_lun_speed_R;
PID pid_pitch_x;


PID pid_angle[4];
PID pid_power[4];

/* 关节下标定义 */
#define LEG_L_TH1 0   /* 左腿theta1(L2) 电机ID4 */
#define LEG_R_TH1 1   /* 右腿theta1(L2) 电机ID3 */
#define LEG_L_TH2 2   /* 左腿theta2(L5) 电机ID1 */
#define LEG_R_TH2 3   /* 右腿theta2(L5) 电机ID2 */

/* 发送用CAN ID，下标同关节下标 */
const uint8_t leg_motor_id[4] = {4, 3, 1, 2};

const uint8_t leg_fb_idx[4]   = {0, 3, 2, 1};

/* 各关节电机转向符号 */
const float   leg_motor_dir[4] = {DIR_TH1, DIR_TH1, DIR_TH2, DIR_TH2};

float trg_theta[4];
float now_theta[4];
float now_dtheta[4];
float now_tor[4];
float leg_tau_cmd[4];

/* 零位偏差：err = 电机读数 - DIR*theta，装配标定后填入 */
float leg_theta_err[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static void Data_updata()
{
		BaseType_t i=xQueueReceive(imu_to_chasis_queue,(void *)&imu,0);
		
	
		if(i)
		{
				/* 平衡环的倾角误差：以重力为绝对零点，再补 IMU 安装角偏差 */
				error_pitch=0.0f-imu.roll;
		}
	
		trg_speed_F=(Remote_Control.ch1/660.0f)*chasis_max_speed;
		trg_speed_T=(Remote_Control.ch2/660.0f)*chasis_max_speed;
			
		trg_leg_long_l -=(Remote_Control.ch3/660.0f)* 0.02f *leg_change_speed;
		trg_leg_long_r -=(Remote_Control.ch3/660.0f)* 0.02f *leg_change_speed;
				
				
		trg_leg_long_l=__LIMIT__(trg_leg_long_l,leg_long_min,leg_long_max);
		trg_leg_long_r=__LIMIT__(trg_leg_long_r,leg_long_min,leg_long_max);

	
		if(rc_lost_cnt<0xFFFF)
			rc_lost_cnt++;


	
}



void Leg_Pid_Calc(void)
{
		uint8_t i;

		trg_theta[LEG_L_TH1] = leg_l.theta1;
		trg_theta[LEG_R_TH1] = leg_r.theta1;
		trg_theta[LEG_L_TH2] = leg_l.theta2;
		trg_theta[LEG_R_TH2] = leg_r.theta2;

		for(i = 0; i < 4; i++)
		{
				
				/* 电机坐标系转几何坐标系 */
				now_theta[i]  = (DM_4310_Motor[leg_fb_idx[i]].pos - leg_theta_err[i]) / leg_motor_dir[i];
				now_dtheta[i] =  DM_4310_Motor[leg_fb_idx[i]].vel / leg_motor_dir[i];
				now_tor[i]    =  DM_4310_Motor[leg_fb_idx[i]].tor * leg_motor_dir[i];

		
				PID_Calc(&pid_angle[i], trg_theta[i], now_theta[i]);
			
				PID_Calc(&pid_power[i], pid_angle[i].out, now_tor[i]);
			

				leg_tau_cmd[i] = (pid_angle[i].out + pid_power[i].out) * leg_motor_dir[i];
		}
}

void Chasis_Calc()
{
		L_speed=(float)DJ_3508_Motor_L.speed/19.0f*6.28318f / 60.0f *wheel_R;
		
		R_speed=-(float)DJ_3508_Motor_R.speed/19.0f*6.28318f / 60.0f *wheel_R;
	
		float ZhengTi_speed =(L_speed+R_speed)/2.0f;
		
		PID_Calc(&pid_f,trg_speed_F,ZhengTi_speed);
		
		PID_Calc(&pid_pitch_speed,pid_f.out,error_pitch);
		
//		PID_Calc(&pid_turn,trg_speed_T,0);
		
		float left=pid_pitch_speed.out;
		float right=pid_pitch_speed.out;
		
		PID_Calc(&pid_lun_speed_L,left,L_speed);
		PID_Calc(&pid_lun_speed_R,right,R_speed);	
		
		/*腿往前或往后使重心都落在轮足垂线上*/
		PID_Calc(&pid_pitch_x,0.0f,error_pitch);
	
		/*异常检测*/
		if(fabsf(error_pitch)>fallen_pitch_max || rc_lost_cnt>RC_LOST_MAX)
		{
				uint8_t i;

				trg_speed_F=0.0f;
				trg_speed_T=0.0f;
				pid_f.integral=0.0f;
				pid_f.out=0.0f;
				pid_pitch_speed.out=0.0f;
				pid_turn.out=0.0f;
				pid_lun_speed_L.out=0.0f;
				pid_lun_speed_R.out=0.0f;
				pid_pitch_x.out=0.0f;

				for(i = 0; i < 4; i++)
				{
						pid_angle[i].out=0.0f;
						pid_angle[i].integral=0.0f;
						pid_power[i].out=0.0f;
						pid_power[i].integral=0.0f;
						leg_tau_cmd[i]=0.0f;
				}
		}
	
		Leg_long_to_angle(&leg_l,trg_leg_long_l,-pid_pitch_x.out);
		Leg_long_to_angle(&leg_r,trg_leg_long_r,pid_pitch_x.out);

		Leg_Pid_Calc();
}

void Send_data()
{
		uint8_t i;

		DJ_Motor_CAN_TxMessage(1,0x200,-pid_lun_speed_R.out,pid_lun_speed_L.out,0.0f,0.0f);
		
		for(i = 0; i < 4; i++)
		{
				DM_Motor_CAN_TxMessage(1, leg_motor_id[i], 0.0f, 0.0f, 0.0f, 0.0f, leg_tau_cmd[i]);
		}
}
	
void chassis_task(void *argument)
{
		uint8_t i;


		PID_Init(&pid_f,           0.55f,    0.0f, 0.0f,  200.0f,  0.5f);        
		PID_Init(&pid_pitch_speed, 10.0f,    0.0f,   0.5f,  0.0f,    3.0f);
	
		PID_Init(&pid_turn,        1.0f,    0.0f,   0.0f,  0.5f,    2.0f);        
		PID_Init(&pid_lun_speed_L, 3600.0f, 0.0f,   0.0f,  200.0f,  DJ_Wheel_Cur_Max); 
		PID_Init(&pid_lun_speed_R, 3600.0f, 0.0f,   0.0f,  200.0f,  DJ_Wheel_Cur_Max);
		
		PID_Init(&pid_pitch_x,     40.0f,    0.0f,   0.0f,  0.0f,    35.0f);


		for(i = 0; i < 4; i++)
		{
				PID_Init(&pid_angle[i], 12.0f,  0.0f,   0.0002f, 2.0f, DM_Leg_Tor_Max);
				PID_Init(&pid_power[i],  1.0f,  0.0005f, 0.0f,   1.0f, DM_Leg_Tor_Max);
		}

		
		for(i = 0; i < 4; i++)
		{
				DM_Motor_Enable(1, leg_motor_id[i]);
		}
		


		TickType_t xLastWakeTime_Chasis;// 存储上一次唤醒时间的变量
		const TickType_t xDelay_Chasis = pdMS_TO_TICKS(2);// 2ms 延时（转换为 tick）
		/*获取时间，用于vTaskDelayUntil*/
		xLastWakeTime_Chasis = xTaskGetTickCount();
	
		while(1)
		{
			
				Data_updata();
			
				Chasis_Calc();
			
				Send_data();  
			
				vTaskDelayUntil(&xLastWakeTime_Chasis, xDelay_Chasis);
		}
	
}
