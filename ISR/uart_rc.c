#include "uart_rc.h"


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart==&huart5)
    {
      Dbus_Decode(RC_RxData, &Remote_Control);
      rc_lost_cnt=0;
      __HAL_UART_CLEAR_FEFLAG(&huart5);
      __HAL_UART_CLEAR_NEFLAG(&huart5);
      __HAL_UART_CLEAR_OREFLAG(&huart5);
      __HAL_UART_CLEAR_PEFLAG(&huart5);
      __HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT);
      /* 重启失败说明 DMA 仍 BUSY，强杀后重新挂载，避免接收链断掉 */
      if(HAL_UARTEx_ReceiveToIdle_DMA(&huart5,RC_RxData,RC_Lenth) != HAL_OK)
      {
        HAL_UART_AbortReceive(&huart5);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5,RC_RxData,RC_Lenth);
      }
    }


}

/**
  * @brief          串口错误回调：DMA 模式下 HAL 会把错误判为 Blocking error，
  *                 主动清 DMAR 并 abort DMA，只能在这里把接收重新挂回去
  * @param[in]      huart: 串口句柄
  * @retval         none
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart5)
    {
      HAL_UART_AbortReceive(&huart5);
      __HAL_UART_CLEAR_FEFLAG(&huart5);
      __HAL_UART_CLEAR_NEFLAG(&huart5);
      __HAL_UART_CLEAR_OREFLAG(&huart5);
      __HAL_UART_CLEAR_PEFLAG(&huart5);
      __HAL_UART_CLEAR_IDLEFLAG(&huart5);
      __HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5,RC_RxData,RC_Lenth);
    }
}
