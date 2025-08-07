/*
 * FS-iA6B.c
 *
 *  Created on: Jun 3, 2025
 *      Author: vishal
 */

#include "FS-iA6B.h"

FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len){
	unsigned short chksum = 0xffff;

	for(int i = 0; i < len-2; i++){
		chksum = chksum - data[i];
	}

	return ((chksum&0x00ff) == data[30] && (chksum>>8)==data[31]);
}

void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus) {
	iBus->RH = (data[2] | data[3]<<8) & 0x0fff;
	iBus->RV = (data[4] | data[5]<<8) & 0x0fff;
	iBus->LV = (data[6] | data[7]<<8) & 0x0fff;;
	iBus->LH = (data[8] | data[9]<<8) & 0x0fff;
	iBus->SwA = (data[10] | data[11]<<8) & 0x0fff;
	iBus->SwB = (data[12] | data[13]<<8) & 0x0fff;
	iBus->SwC = (data[14] | data[15]<<8) & 0x0fff;
	iBus->SwD = (data[16] | data[17]<<8) & 0x0fff;
	iBus->VrA = (data[18] | data[19]<<8) & 0x0fff;
	iBus->VrB = (data[20] | data[21]<<8) & 0x0fff;

	iBus->FailSafe = (data[13] >> 4);

}

#ifdef _USE_FS_I6
	iBus->FailSafe = (data[13] >> 4);
#endif

unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus) {
	return iBus->FailSafe != 0;
}

void FSiA6B_UART4_Initialization(void){

	  LL_USART_InitTypeDef UART_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  /** Initializes the peripherals clock
	  */
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
	  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Peripheral clock enable */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

	  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
	  /**UART4 GPIO Configuration
	  PD0   ------> UART4_RX
	  PD1   ------> UART4_TX
	  */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /* UART4 interrupt Init */
	  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	  NVIC_EnableIRQ(UART4_IRQn);

	  /* USER CODE BEGIN UART4_Init 1 */

	  /* USER CODE END UART4_Init 1 */
	  UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	  UART_InitStruct.BaudRate = 115200;
	  UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	  UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	  UART_InitStruct.Parity = LL_USART_PARITY_NONE;
	  UART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
	  UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	  UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	  LL_USART_Init(UART4, &UART_InitStruct);
	  LL_USART_DisableFIFO(UART4);
	  LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
	  LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
	  LL_USART_ConfigAsyncMode(UART4);

	  /* USER CODE BEGIN WKUPType UART4 */

	  /* USER CODE END WKUPType UART4 */

	  LL_USART_Enable(UART4);
}
