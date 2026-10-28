#include "bsp_rc.h"

uint8_t RC_RxData[RC_Lenth];

volatile Remote_Control_TypeDef Remote_Control;

/* 遥控丢信号计数：接收回调里清零，底盘任务里自增，超过阈值判定失控 */
volatile uint16_t rc_lost_cnt = 0;

void RC_Init(void)
{
	  __HAL_DMA_DISABLE_IT(&uart_rc_DMA,DMA_IT_HT);
		HAL_UARTEx_ReceiveToIdle_DMA(&uart_rc,RC_RxData,RC_Lenth);
}

/**
  * @brief          遥控器协议解析
  * @param[in]      buff: 原生数据指针
  * @param[out]     data: 遥控器数据指针
  * @retval         none
  */
void Dbus_Decode(volatile uint8_t *buff, volatile Remote_Control_TypeDef *RC)
{
    uint16_t c1, c2, c3, c4;
    uint8_t off;

    if (buff == NULL || RC == NULL) return;


    for (off = 0; off < 2; off++)
    {
        c1 = (buff[off] | (buff[off+1] << 8)) & 0x07FF;
        c2 = ((buff[off+1] >> 3) | (buff[off+2] << 5)) & 0x07FF;
        c3 = ((buff[off+2] >> 6) | (buff[off+3] << 2) | (buff[off+4] << 10)) & 0x07FF;
        c4 = ((buff[off+4] >> 1) | (buff[off+5] << 7)) & 0x07FF;

        if (((buff[off+5] & 0xC0) != 0) && ((buff[off+5] & 0x30) != 0)
            && (c1 >= 364) && (c1 <= 1684)
            && (c2 >= 364) && (c2 <= 1684)
            && (c3 >= 364) && (c3 <= 1684)
            && (c4 >= 364) && (c4 <= 1684))
        {
            break;
        }
    }

    // 摇杆通道
    RC->ch1 = c1;
    RC->ch1 -= 1024;

    RC->ch2 = c2;
    RC->ch2 -= 1024;

    RC->ch3 = c3;
    RC->ch3 -= 1024;

    RC->ch4 = c4;
    RC->ch4 -= 1024;

    // 死区处理
    if (abs(RC->ch1) <= 20) RC->ch1 = 0;
    if (abs(RC->ch2) <= 20) RC->ch2 = 0;
    if (abs(RC->ch3) <= 20) RC->ch3 = 0;
    if (abs(RC->ch4) <= 20) RC->ch4 = 0;

    // 拨杆
		RC->last_sw1 = RC->sw1;
		RC->last_sw2 = RC->sw2;
    RC->sw1 = ((buff[off+5] >> 4) & 0x000C) >> 2;
    RC->sw2 = (buff[off+5] >> 4) & 0x0003;
		if (RC->last_sw1 == 3 && RC->sw1 == 1) RC->sw1RiseEdge = 1;
		else if (RC->last_sw1 == 3 && RC->sw1 == 2) RC->sw1DescendEdge = 1;
		if (RC->last_sw2 == 3 && RC->sw2 == 1) RC->sw2RiseEdge = 1;
		else if (RC->last_sw2 == 3 && RC->sw2 == 2) RC->sw2DescendEdge = 1;
	

    // 越界保护
    if (abs(RC->ch1) > 660 || abs(RC->ch2) > 660 || abs(RC->ch3) > 660 || abs(RC->ch4) > 660)
    {
        memset((void *)RC, 0, sizeof(Remote_Control_TypeDef));  // 清空遥控结构体
        return;
    }

    // 鼠标数据
    RC->mouse_x = buff[off+6] | (buff[off+7] << 8);
    RC->mouse_y = buff[off+8] | (buff[off+9] << 8);
    RC->mouse_z = buff[off+10] | (buff[off+11] << 8);

    RC->mouse_l = buff[off+12];
    RC->mouse_r = buff[off+13];

    // 键盘按键
    RC->kb = buff[off+14] | (buff[off+15] << 8);

    // 滚轮
    RC->ch5_wheel = (buff[off+16] | (buff[off+17] << 8)) - 1024;
    
    for (uint8_t i = 0; i < 16; i++) {
        *(&RC->Kb_State.W+i) = (RC->kb >> i) & 1;
    }

}
