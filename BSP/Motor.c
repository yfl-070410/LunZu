#include "motor.h"

volatile DJ_MotorInfo_TypeDef DJ_3508_Motor_L = {0};
volatile DJ_MotorInfo_TypeDef DJ_3508_Motor_R = {0};

int16_t __FLOAT_TO_INT__(float x_float, float x_min, float x_max)
{
    if (x_min >= x_max) { return 0; }
		x_float = __LIMIT__(x_float, x_min, x_max);
    
    float _input_range = x_max - x_min;
    float _normalized = (x_float - x_min) / _input_range;
    float _mapped = _normalized * 32768.0f - 16384.0f;
    
    int16_t _result;
    if (_mapped >= 0) {
        _result = (int16_t)(_mapped + 0.5f);
    } else {
        _result = (int16_t)(_mapped - 0.5f);
    }
    
		_result = __LIMIT__(_result, -16384, 16384);
    
    return (int16_t)_result;
}

void DJ_Motor_CAN_TxMessage(uint8_t CAN_Instance, uint32_t TxStdId,
												  float Torque_1, float Torque_2, float Torque_3, float Torque_4)
{
		int16_t _cur1,_cur2,_cur3,_cur4;
	
		_cur1 = __FLOAT_TO_INT__(Torque_1,-DJ_Max_cur,DJ_Max_cur);
		_cur2 = __FLOAT_TO_INT__(Torque_2,-DJ_Max_cur,DJ_Max_cur);
		_cur3 = __FLOAT_TO_INT__(Torque_3,-DJ_Max_cur,DJ_Max_cur);
		_cur4 = __FLOAT_TO_INT__(Torque_4,-DJ_Max_cur,DJ_Max_cur);
	
		
		extern FDCAN_HandleTypeDef hfdcan1;
		extern FDCAN_TxHeaderTypeDef CAN_TxHeader;
		CAN_TxHeader.Identifier = TxStdId;
		
		uint8_t _data[8];
		_data[0] = _cur1 >> 8; 
		_data[1] = _cur1; 
		_data[2] = _cur2 >> 8; 
		_data[3] = _cur2; 
		_data[4] = _cur3 >> 8; 
		_data[5] = _cur3; 
		_data[6] = _cur4 >> 8; 
		_data[7] = _cur4; 

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_TxHeader, _data);
		
		
}



void DJ_Motor_Info_Update(DJ_MotorInfo_TypeDef *motor, uint8_t *RX_Data)
{
	
    motor->angle = (uint16_t)((RX_Data[0] << 8) | RX_Data[1]);
    motor->speed = (uint16_t)((RX_Data[2] << 8) | RX_Data[3]);
    motor->current = (uint16_t)((RX_Data[4] << 8) | RX_Data[5]);
    motor->temperature = RX_Data[6]; 
}
	

//------------------DM-------------------//


volatile DM_MotorInfo_TypeDef DM_4310_Motor[4] = {0};

int __FLOAT_TO_UINT__(float x_float, float x_min, float x_max, int bits)
{
	float _span = x_max - x_min;
	float _offset = x_min;
	return (int) ((x_float-_offset)*((float)((1<<bits)-1))/_span);
}

float __UINT_TO_FLOAT__(int x_int, float x_min, float x_max, int bits)
{
	float _span = x_max - x_min;
	float _offset = x_min;
	return ((float)x_int)*_span/((float)((1<<bits)-1)) + _offset;
}

/* DM4310系统帧：TxStdId传电机CANID本身(1~127)，不要加0x100偏移 */
void DM_Motor_Enable(uint8_t CAN_Instance, uint32_t TxStdId)
{
		extern FDCAN_TxFrame_TypeDef DM4310_TxFrame;
	
		DM4310_TxFrame.Header.Identifier = TxStdId;
		DM4310_TxFrame.Data[0] = 0xFF;
		DM4310_TxFrame.Data[1] = 0xFF;
		DM4310_TxFrame.Data[2] = 0xFF;
		DM4310_TxFrame.Data[3] = 0xFF;
		DM4310_TxFrame.Data[4] = 0xFF;
		DM4310_TxFrame.Data[5] = 0xFF;
		DM4310_TxFrame.Data[6] = 0xFF;
		DM4310_TxFrame.Data[7] = 0xFC;

		HAL_FDCAN_AddMessageToTxFifoQ(DM4310_TxFrame.hcan,&DM4310_TxFrame.Header,DM4310_TxFrame.Data);

}

void DM_Motor_Disable(uint8_t CAN_Instance, uint32_t TxStdId)
{
		extern FDCAN_TxFrame_TypeDef DM4310_TxFrame;
	
		DM4310_TxFrame.Header.Identifier = TxStdId;
		DM4310_TxFrame.Data[0] = 0xFF;
		DM4310_TxFrame.Data[1] = 0xFF;
		DM4310_TxFrame.Data[2] = 0xFF;
		DM4310_TxFrame.Data[3] = 0xFF;
		DM4310_TxFrame.Data[4] = 0xFF;
		DM4310_TxFrame.Data[5] = 0xFF;
		DM4310_TxFrame.Data[6] = 0xFF;
		DM4310_TxFrame.Data[7] = 0xFD;

		HAL_FDCAN_AddMessageToTxFifoQ(DM4310_TxFrame.hcan,&DM4310_TxFrame.Header,DM4310_TxFrame.Data);

}

void DM_Motor_CAN_TxMessage(uint8_t CAN_Instance, uint32_t TxStdId,
												    float pos, float vel,float kp, float kd, float torq)
{
		int16_t _pos_tmp,_vel_tmp,_kp_tmp,_kd_tmp,_tor_tmp;
//		uint16_t id = _Motor->Data.id + 0x000;// MIT MODE

		_pos_tmp = __FLOAT_TO_UINT__(pos,  -3.14159f,  3.14159f,  16);
		_vel_tmp = __FLOAT_TO_UINT__(vel,  -30.0f,  30.0f,  12);
		_kp_tmp  = __FLOAT_TO_UINT__(kp,   0.0f, 500.0f, 12);
		_kd_tmp  = __FLOAT_TO_UINT__(kd,   0.0f, 5.0f, 12);
		_tor_tmp = __FLOAT_TO_UINT__(torq, -10.0f, 10.0f, 12);

		extern FDCAN_TxFrame_TypeDef DM4310_TxFrame;
	
		DM4310_TxFrame.Header.Identifier = TxStdId;
		DM4310_TxFrame.Data[0] = (_pos_tmp >> 8);
		DM4310_TxFrame.Data[1] = _pos_tmp;
		DM4310_TxFrame.Data[2] = (_vel_tmp >> 4);
		DM4310_TxFrame.Data[3] = ((_vel_tmp&0xF)<<4)|(_kp_tmp>>8);
		DM4310_TxFrame.Data[4] = _kp_tmp;
		DM4310_TxFrame.Data[5] = (_kd_tmp >> 4);
		DM4310_TxFrame.Data[6] = ((_kd_tmp&0xF)<<4)|(_tor_tmp>>8);
		DM4310_TxFrame.Data[7] = _tor_tmp;
	
		HAL_FDCAN_AddMessageToTxFifoQ(DM4310_TxFrame.hcan, &DM4310_TxFrame.Header, DM4310_TxFrame.Data);
	
}

/* DM4310位置速度模式控制帧：ID=0x100+CANID，
   D0~D3为位置float(小端)，D4~D7为梯形速度上限float(小端) */
void DM_Motor_Send_PosVel(uint8_t id, float p_des, float v_des)
{
    FDCAN_TxHeaderTypeDef tx_hdr = {0};
    uint8_t data[8];

    memcpy(data, &p_des, sizeof(float));
    memcpy(data + 4, &v_des, sizeof(float));

    tx_hdr.Identifier = 0x100u + id;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
    tx_hdr.DataLength = FDCAN_DLC_BYTES_8;
    tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
    tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_hdr, data);
}

/* DM4310 零力矩帧，Kp=Kd=0，t_ff=0，一帧一帧发 */
void BSP_DM4310_TxZeroTorque(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    extern FDCAN_TxFrame_TypeDef DM4310_TxFrame;
    uint8_t tx[8] = {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};
    DM4310_TxFrame.Header.Identifier=id;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &DM4310_TxFrame.Header,tx);

}


volatile uint16_t dbg_dm_rx_cnt = 0;

void DM_Motor_Info_Update(volatile DM_MotorInfo_TypeDef *motor, uint8_t *RX_Data)
{
			dbg_dm_rx_cnt++;
			motor->id = (RX_Data[0])&0x0F;
			motor->state = (RX_Data[0])>>4;
			motor->p_int=(RX_Data[1]<<8)|RX_Data[2];
			motor->v_int=(RX_Data[3]<<4)|(RX_Data[4]>>4);
			motor->t_int=((RX_Data[4]&0xF)<<8)|RX_Data[5];
			motor->pos = __UINT_TO_FLOAT__(motor->p_int, -3.14159f, 3.14159f, 16);
			motor->vel = __UINT_TO_FLOAT__(motor->v_int, -30.0f, 30.0f, 12);
			motor->tor = __UINT_TO_FLOAT__(motor->t_int, -10.0f, 10.0f, 12);
			motor->Tmos = (float)(RX_Data[6]);
			motor->Tcoil = (float)(RX_Data[7]);
}
