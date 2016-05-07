#include "camera_hw.h"
#include "stm32f4xx.h"
#include "def.h"
#include "usbd_cdc_vcp.h"
#include "systick.h"
#include "i2c.h"
void Camera_HW_I2C_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;	
	/* I2C2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect I2C2 pins to AF4 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	/* Configure I2C2 GPIOs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C DeInit */
	I2C_DeInit(I2C2);

	/* Enable the I2C peripheral */
	I2C_Cmd(I2C2, ENABLE);

	/* Set the I2C structure parameters */
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 100000;

	/* Initialize the I2C peripheral w/ selected parameters */
	I2C_Init(I2C2, &I2C_InitStruct);	
}
void DMA_DCMI_Init(uint32_t mem_addr, uint32_t size)
{
	DMA_InitTypeDef  DMA_InitStructure;
// 	NVIC_InitTypeDef NVIC_InitStructure;

	/*Configure the DMA2_Stream7 channel1 to transfer Data from 
	DCMI DR register to the destination memory buffer */
	/* Enable DMA2 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  /* DMA2 Stream1 Configuration */
  DMA_DeInit(DMA2_Stream7);

	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
  {
  }
	
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;	
  DMA_InitStructure.DMA_Memory0BaseAddr = mem_addr;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	/* DMA2 IRQ channel Configuration */
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);

		/* Enable DMA Stream Transfer Complete interrupt */
	DMA_Cmd(DMA2_Stream7, ENABLE);
			
	while (DMA_GetCmdStatus(DMA2_Stream7) != ENABLE)
	{
	}
}
void Send_Capture_8b(uint8_t *pui8_Data, uint16_t Hori_size, uint16_t Vert_size)
{
//	uint16_t Hori_count, Vert_count;
//	for (Vert_count = 0; Vert_count < Vert_size; Vert_count++)
//	{
//		for (Hori_count = 0; Hori_count < Hori_size; Hori_count++)
//		{
//			VCP_DataTx (0, (uint32_t)(*pui8_Data++));
//		}
//	}
//	VCP_DataTx (0, (uint32_t)0x00);
//	VCP_DataTx (0, (uint32_t)0xff);
//	VCP_DataTx (0, (uint32_t)0x00);
//	VCP_DataTx (0, (uint32_t)0xff);
//	VCP_DataTx (0, (uint32_t)0x00);
//	VCP_DataTx (0, (uint32_t)0xff);
	
	
}


void UARTInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable USART GPIOs clocks */
  RCC_AHB1PeriphClockCmd(USART_DCMI_RCC_GPIO, ENABLE);
	/* Enable USART clocks */
	RCC_APB1PeriphClockCmd(USART_DCMI_RCC_Periph, ENABLE);

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(USART_DCMI_PORT, USART_DCMI_Tx_Pin_Source, USART_DCMI_AF);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(USART_DCMI_PORT, USART_DCMI_Rx_Pin_Source, USART_DCMI_AF);
	
  /* UART GPIO configuration */
  /* Tx/Rx(PD8/9) */
  GPIO_InitStructure.GPIO_Pin = USART_DCMI_Tx_Pin | USART_DCMI_Rx_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USART_DCMI_PORT, &GPIO_InitStructure);
	
	USART_DeInit(USART_DCMI);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init(USART_DCMI, &USART_InitStructure);
	USART_Cmd(USART_DCMI, ENABLE);
}
/* Private typedef -----------------------------------------------------------*/
DMA_InitTypeDef  DMA_InitStructure;
void UART_DMAInit(void)
{
	RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);
	
  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/  
  DMA_InitStructure.DMA_PeripheralBaseAddr = USARTx_DR_ADDRESS;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_DeInit(USARTx_TX_DMA_STREAM);
	DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
	
	/****************** USART will Transmit Specific Command ******************/ 
	/* Prepare the DMA to transfer the transaction command (2bytes) from the
		 memory to the USART */  
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CompressedImage.ImageData;
	DMA_InitStructure.DMA_BufferSize = (uint16_t)0;
	DMA_Init(USARTx_TX_DMA_STREAM, &DMA_InitStructure); 
}

void SendImageDMA(uint32_t *mem_addr, uint32_t image_size)
{
	DMA_DeInit(USARTx_TX_DMA_STREAM);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mem_addr;
	DMA_InitStructure.DMA_BufferSize = (uint16_t)(image_size+3);
	mem_addr[image_size] = 's';
	mem_addr[image_size] = 't';
	mem_addr[image_size] = 'p';
	DMA_Init(USARTx_TX_DMA_STREAM, &DMA_InitStructure); 
	/* Enable the USART DMA requests */
	USART_DMACmd(USART_DCMI, USART_DMAReq_Tx, ENABLE);

	/* Clear the TC bit in the SR register by writing 0 to it */
	USART_ClearFlag(USART_DCMI, USART_FLAG_TC);

	/* Enable the DMA TX Stream, USART will start sending the command code (2bytes) */
	DMA_Cmd(USARTx_TX_DMA_STREAM, ENABLE);

	/* Wait the USART DMA Tx transfer complete or time out */
	while (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_TCIF) == RESET)
	{
	}
}