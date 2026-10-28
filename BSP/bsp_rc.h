#ifndef BSP_RC_H
#define BSP_RC_H

#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "uart_rc.h"


#define uart_rc huart5
#define uart_rc_DMA hdma_uart5_rx
#define RC_Lenth 36     /* 取两帧长度，防止DMA传输越界并保证走IDLE路径 */

/* volatile加在变量上而非类型上，避免队列收发时被迫cast掉volatile */
typedef struct 
{
    // 摇杆通道
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;

    // 滚轮
    int16_t ch5_wheel;

    // 拨杆
    uint8_t sw1, last_sw1, sw1RiseEdge, sw1DescendEdge;
    uint8_t sw2, last_sw2, sw2RiseEdge, sw2DescendEdge;

    // 鼠标数据
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;

    // 键盘按键
    uint16_t kb;
    
    struct {
      uint8_t W;
      uint8_t S;
      uint8_t A;
      uint8_t D;
      
      uint8_t Shift;
      uint8_t Ctrl;
      
      uint8_t Q;
      uint8_t E;
      uint8_t R;
      
      uint8_t F;
      uint8_t G;
      
      uint8_t Z;
      uint8_t X;
      uint8_t C;
      uint8_t V;    
      uint8_t B;
    }Kb_State;

} Remote_Control_TypeDef; //遥控器数据结构体



extern volatile Remote_Control_TypeDef Remote_Control;

extern volatile uint16_t rc_lost_cnt;

void RC_Init(void);
void Dbus_Decode(volatile uint8_t *buff, volatile Remote_Control_TypeDef *data);

extern uint8_t RC_RxData[RC_Lenth];

#endif
