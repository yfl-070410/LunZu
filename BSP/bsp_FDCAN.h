#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "main.h"
#include "math.h"

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t Data[8];
}FDCAN_TxFrame_TypeDef;

void BSP_FDCAN_Init(void);

extern volatile uint16_t dbg_can_rx_cnt;
void BSP_FDCAN_EnableRxInterrupt(void);
void BSP_FDCAN_RxPoll(void);
void HAL_FDCAN_Data(uint8_t *data,float D1,float D2,float D3,float D4);
void FDCAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t data[8]);
void FDCAN2_RxFifo1RxHandler(uint32_t *StdId, uint8_t data[8]);

#endif
