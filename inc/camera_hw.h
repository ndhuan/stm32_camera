#ifndef __CAMERA_HW_H
#define __CAMERA_HW_H

#include "stdint.h"

//UART Define
#define USART_DCMI_RCC_Periph			RCC_APB1Periph_USART3
#define USART_DCMI_RCC_GPIO				RCC_AHB1Periph_GPIOD
#define USART_DCMI								USART3
#define USART_DCMI_PORT						GPIOD
#define USART_DCMI_Tx_Pin					GPIO_Pin_8
#define USART_DCMI_Rx_Pin					GPIO_Pin_9
#define USART_DCMI_Tx_Pin_Source	GPIO_PinSource8
#define USART_DCMI_Rx_Pin_Source	GPIO_PinSource9
#define USART_DCMI_AF							GPIO_AF_USART3

/* Definition for DMAx resources **********************************************/
#define USARTx_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

#define USARTx_DMA                       DMA1
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1
   
#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_STREAM             DMA1_Stream3
#define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
#define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
#define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
#define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
#define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
            
#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_RX_DMA_STREAM             DMA1_Stream1
#define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
#define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
#define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
#define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
#define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

#define USARTx_DMA_TX_IRQn               DMA1_Stream3_IRQn
#define USARTx_DMA_RX_IRQn               DMA1_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
#define USARTx_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler   

#define UARTPutc(USARTx, c)			USART_SendData(USARTx, (uint8_t)c);	\
																while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		
void Camera_HW_I2C_Init(void);																
void DMA_DCMI_Init(uint32_t mem_addr, uint32_t size);																
void Send_Capture_8b(uint8_t *pui8_Data, uint16_t Hori_size, uint16_t Vert_size);								
void UARTInit(void);
void UART_DMAInit(void);
void SendImageDMA(uint32_t *mem_addr, uint32_t image_size);																
#endif
																