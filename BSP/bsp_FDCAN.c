#include "bsp_FDCAN.h" 


FDCAN_RxHeaderTypeDef CAN1_RxHeader;
uint8_t CAN1_RxData[8];
FDCAN_RxHeaderTypeDef CAN2_RxHeader;
uint8_t CAN2_RxData[8];


FDCAN_TxHeaderTypeDef CAN_TxHeader =
{
    .IdType = FDCAN_STANDARD_ID, 
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = 8,
    .ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat =  FDCAN_CLASSIC_CAN,           
    .TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
    .MessageMarker = 0,
};

FDCAN_TxFrame_TypeDef M3508_TxFrame ={
    .hcan = &hfdcan1,
    .Header.IdType = FDCAN_STANDARD_ID, 
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 8,
    .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
    .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
    .Header.MessageMarker = 0,
};

FDCAN_TxFrame_TypeDef DM4310_TxFrame ={
    .hcan = &hfdcan1,
    .Header.IdType = FDCAN_STANDARD_ID, 
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 8,
    .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
    .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
    .Header.MessageMarker = 0,
};

void BSP_FDCAN_Init(void)
{

    FDCAN_FilterTypeDef FDCAN1_FilterConfig;

    FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN1_FilterConfig.FilterIndex = 0;
    FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_FilterConfig.FilterID1 = 0x00000000; 
    FDCAN1_FilterConfig.FilterID2 = 0x00000000; 

    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
    {
      Error_Handler();
    }
      
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
      Error_Handler();
    }
		 if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }
    
		/*后续还有FDCAN2等,按照这个配置即可*/
}


	
void FDCAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t data[8])
{
		
}

void FDCAN2_RxFifo1RxHandler(uint32_t *StdId, uint8_t Data[8])
{
    switch (*StdId)
    {
				
				
        default:
            break;
    }
}

void HAL_FDCAN_Data(uint8_t *data,float D1,float D2,float D3,float D4)
{
		int16_t C1=float_to_int16(D1);
		int16_t C2=float_to_int16(D2);
		int16_t C3=float_to_int16(D3);
		int16_t C4=float_to_int16(D4);
	
		data[0]=(uint8_t)((uint8_t)(C1>>8)&0XFF);
		data[1]=(uint8_t)(C1&0XFF);
		
		data[2]=(uint8_t)((uint8_t)(C2>>8)&0XFF);
		data[3]=(uint8_t)(C2&0XFF);
	
		data[4]=(uint8_t)((C3>>8)&0XFF);
		data[5]=(uint8_t)(C3&0XFF);
	
		data[6]=(uint8_t)((C4>>8)&0XFF);
		data[7]=(uint8_t)(C4&0XFF);
	
}

void HAL_FDCAN_TX(FDCAN_HandleTypeDef *hfdcan,uint8_t CAN_Instance, uint32_t TxStdId,uint8_t *data)
{
		CAN_TxHeader.Identifier = TxStdId;
		
		HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_TxHeader, data);
	
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_RxHeader, CAN1_RxData);
 
	
		FDCAN1_RxFifo0RxHandler(&CAN1_RxHeader.Identifier, CAN1_RxData);
   
}
