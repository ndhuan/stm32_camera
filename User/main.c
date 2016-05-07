#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "systick.h"
#include "led.h"
#include "i2c.h"
#include "usart.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "camera_hw.h"
#include "MT9V034.h"
#include "MT9D111.h"
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

volatile uint8_t Cam_Capture[FULL_IMAGE_SIZE*BYTES_PER_PIXEL + 3]={0};
//volatile uint8_t Cam_Buffer[FULL_IMAGE_SIZE + 3]={0};

uint8_t DCMI_Flag = 0;
static int16_t ax, ay, az, wx, wy, wz;
static uint8_t data[14];
void XCLK_ON(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Output HSE clock on MCO2 pin(PC9) ****************************************/
	// Enable the GPIOA peripheral
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Connect MCO2 pin to AF0
	// Connect to AF0 is default after reset
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);

	// Configure MCO1 pin(PC9) in alternate function
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// HSE clock selected to output on MCO2 pin(PC9)
#if HSE_VALUE == 8000000
	RCC_MCO2Config(RCC_MCO2Source_HSE, RCC_MCO2Div_1);
#elif HSE_VALUE == 25000000
	RCC_MCO2Config(RCC_MCO2Source_HSE, RCC_MCO2Div_4);
#endif
}

bool CheckXCLK(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t EdgeCount = 0, time;

	// Output HSE clock on MCO2 pin(PC9) ****************************************/
	// Enable the GPIOA peripheral
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Configure pin(PC9) as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	time = 50000000;
	while (time--)
	{
		if (((~(GPIOC->IDR)) & GPIO_Pin_9) != 0)
		{
			EdgeCount++;
		}
	}
	
	GPIO_DeInit(GPIOC);
	if (EdgeCount > 1000)
	{
		return true;
	}
	return false;
}
void mpu6050_enable()
{
	uint8_t data = 0;	
	bool res;
	res = I2C_WriteBytes(I2C2, &data, MPU_ADDR,0x6B, 1);
	if (!res)
	{
		LEDOn(LED_ERR);		
	}	
}
int main(void)
{
	#warning change clock hse_value to 25m	
	SYSTIM_Init();
	
	LEDInit(LED_ACT);
	LEDInit(LED_COM);
	LEDInit(LED_ERR);
	
	LEDOn(LED_ACT);
	LEDOff(LED_COM);
	LEDOff(LED_ERR);
	SYSTIM_DelayTms(1000);
	LEDOff(LED_ACT);	
	
//	USBD_Init(&USB_OTG_dev,
//				USB_OTG_FS_CORE_ID,
//				&USR_desc,
//				&USBD_CDC_cb,
//				&USR_cb);

	usart_init();
	UART_DMAInit();
	if (!CheckXCLK())
	{
		XCLK_ON();	
		SYSTIM_DelayTms(100);
	}
#ifdef USE_MT9V034
	MT9V034_Init();
#endif
#ifdef USE_MT9D111	
	MT9D111_Init();	
#endif	
	#warning already enable i2c when init camera
	//mpu6050_enable();	
	memset(Cam_Capture,'1',FULL_IMAGE_SIZE);	
	Cam_Capture[FULL_IMAGE_SIZE] = 's';
	Cam_Capture[FULL_IMAGE_SIZE + 1] = 't';	
	Cam_Capture[FULL_IMAGE_SIZE + 2] = 'p';	
	DCMI_CaptureCmd(ENABLE);	
	while(1)
	{
		int i=0;
#ifdef TEST_MPU6050
		bool res;
		res = I2C_ReadBytes(I2C2,data,MPU_ADDR, MPU_DATA, 14);
		if (!res)
		{
			LEDOn(LED_ERR);		
		}
		ax = (double)(data[1] + (data[0]<<8));
		ay = (double)(data[3] + (data[2]<<8));
		az = (double)(data[5] + (data[4]<<8));
		wx = (double)(data[9] + (data[8]<<8));
		wy = (double)(data[11] + (data[10]<<8));
		wz = (double)(data[13] + (data[12]<<8));					
		SYSTIM_DelayTms(100);
#endif		

		if (DCMI_Flag)
		{
			DCMI_Flag = 0;
			LEDToggle(LED_ACT);
//			SendImageDMA((uint32_t*)&Cam_Capture[0], FULL_IMAGE_SIZE);			
//			for (i=0;i<FULL_IMAGE_SIZE;i++)
//			{
//				Cam_Buffer[i] = Cam_Capture[i];
//			}
			VCP_DataTx(Cam_Capture,FULL_IMAGE_SIZE+3);
			SYSTIM_DelayTms(5000);
 			DCMI_CaptureCmd(ENABLE); 	
		}	
	}
}
