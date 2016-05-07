///****************************************************************************
// *
// *   Copyright (C) 2013 Fortiss An-Institut TU Munchen All rights reserved.
// *   Author: Thomas Boehm <thomas.boehm@fortiss.org>
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in
// *    the documentation and/or other materials provided with the
// *    distribution.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// *
// ****************************************************************************/
/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file i2c.c
 * Definition of i2c frames.
 * @author Thomas Boehm <thomas.boehm@fortiss.org>
 * @author James Goppert <james.goppert@gmail.com>
 */
#include <stdio.h>
#include <string.h>
#include "i2c.h"
#include "led.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"


#define I2C_TIMEOUT           100000

//#include "mavlink_bridge_header.h"
//#include <mavlink.h>

/* prototypes */
bool I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l);
bool I2C_WriteBytes(I2C_TypeDef * I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l);

void i2c_init(I2C_TypeDef* I2Cx) {
	if (I2Cx == I2C2)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		I2C_InitTypeDef  I2C_InitStructure;
		
		/* GPIO configuration */
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11; 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		//note
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);    
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);  

		/* I2C configuration */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		I2C_DeInit(I2C2);
		I2C_InitStructure.I2C_ClockSpeed 			= 100000;	
		I2C_InitStructure.I2C_Mode 					= I2C_Mode_I2C;					
		I2C_InitStructure.I2C_DutyCycle 			= I2C_DutyCycle_2;							
		I2C_InitStructure.I2C_OwnAddress1 			= 0;
		I2C_InitStructure.I2C_Ack 					= I2C_Ack_Enable;						
		I2C_InitStructure.I2C_AcknowledgedAddress 	= I2C_AcknowledgedAddress_7bit;
		I2C_Init(I2C2, &I2C_InitStructure);
		I2C_Cmd(I2C2, ENABLE);
	}
}
/*******************************************************************************
 * @fn     I2C_ReadBytes     
 * @brief  Read/Receive l bytes from sensors
 * @param  c: pointer to array that will store bytes from sensors
 * @param  s: sensor's address
 * @param  a: start address of the register of the sensor
 * @param  l: length of the array
 * @retval false if fail, true if successful
 ******************************************************************************/
bool I2C_ReadBytes_NoReStart(I2C_TypeDef* I2Cx, uint8_t *c, uint8_t s, uint8_t l)
{
	uint32_t time;

    if (l == 0)   return false;

	/* While the bus is busy */
	time = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
		if((time--) == 0) return false;
	}
	
	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((time--) == 0) return false;
	}
		
	/* Send Sensor address for read */
	I2C_Send7bitAddress(I2Cx, s, I2C_Direction_Receiver);  

    /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
    __disable_irq();
    	
    /* Test on EV6 and clear it */
    time = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if ((time--) == 0) return false;
    }

    while(l)
    {
        if(l == 1)
        {
            /* This configuration should be in the last second transfer byte */
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
            
            /* Send STOP Condition */
            I2C_GenerateSTOP(I2Cx, ENABLE);
        }

        /* Test on EV7 and clear it */
        time = I2C_TIMEOUT;
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            if ((time--) == 0) return false;
        }

        /* Read the byte received from the Sensor */
        /* Point to the next location where the byte read will be saved */
        *c = I2C_ReceiveData(I2Cx);
        c++;
        
        /* Decrease the read bytes counter */
        l--;

    }

    /* Wait to make sure that STOP control bit has been cleared */
    time = I2C_TIMEOUT;
    while(I2Cx->CR1 & I2C_CR1_STOP)
    {
        if((time--) == 0) return false;
    }  
    
    /* Re-Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2Cx, ENABLE);  
    
    /* ENABLE ALL INTERRUPT */
    __enable_irq();     
    return true; 	
}
bool I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l)
{
	uint32_t time;

    if (l == 0)   return false;

	/* While the bus is busy */
	time = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
		if((time--) == 0) return false;
	}
	
	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((time--) == 0) return false;
	}
	
	/* Send Sensor address for write */
	I2C_Send7bitAddress(I2Cx, s, I2C_Direction_Transmitter);
    	
	/* Test on EV6 and clear it */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((time--) == 0) return false;
	} 
    	
	/* Send the Register address to read from: Only one byte address */
	I2C_SendData(I2Cx, a);  
	
	/* Test on EV8 and clear it */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
	{
		if((time--) == 0) return false;
	}
	
	/* Send START condition a second time to RESTART bus */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((time--) == 0) return false;
	} 
	
	/* Send Sensor address for read */
	I2C_Send7bitAddress(I2Cx, s, I2C_Direction_Receiver);  

    /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
    __disable_irq();
    	
    /* Test on EV6 and clear it */
    time = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if ((time--) == 0) return false;
    }

    while(l)
    {
        if(l == 1)
        {
            /* This configuration should be in the last second transfer byte */
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
            
            /* Send STOP Condition */
            I2C_GenerateSTOP(I2Cx, ENABLE);
        }

        /* Test on EV7 and clear it */
        time = I2C_TIMEOUT;
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            if ((time--) == 0) return false;
        }

        /* Read the byte received from the Sensor */
        /* Point to the next location where the byte read will be saved */
        *c = I2C_ReceiveData(I2Cx);
        c++;
        
        /* Decrease the read bytes counter */
        l--;

    }

    /* Wait to make sure that STOP control bit has been cleared */
    time = I2C_TIMEOUT;
    while(I2Cx->CR1 & I2C_CR1_STOP)
    {
        if((time--) == 0) return false;
    }  
    
    /* Re-Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2Cx, ENABLE);  
    
    /* ENABLE ALL INTERRUPT */
    __enable_irq();     
    return true; 
}

/*******************************************************************************
 * @fn     I2C_WriteBytes     
 * @brief  Write/Send l bytes to sensors
 * @param  c: pointer to array that will store bytes to send to sensors
 * @param  s: sensor's address
 * @param  a: start address of the register of the sensor
 * @param  l: length of the array
 * @retval false if fail, true if successful
 ******************************************************************************/
bool I2C_WriteBytes(I2C_TypeDef * I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l)
{
	uint32_t time;
    I2C_AcknowledgeConfig(I2Cx, ENABLE); 
	/* While the bus is busy */
	time = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
		if((time--) == 0) return false;
	}
	
	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((time--) == 0) return false;
	}
	
	/* Send Sensor address for write */
	I2C_Send7bitAddress(I2Cx, s, I2C_Direction_Transmitter);
    	
	/* Test on EV6 and clear it */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((time--) == 0) return false;
	} 
    
	/* Send the Register address to read from: Only one byte address */
	I2C_SendData(I2Cx, a);  

    /* Test on EV8 and clear it */
    time = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if (time-- == 0) return false;
    }
    
    /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
    __disable_irq();
    while(l)
    {
        /* Send the data & increase the pointer of write buffer */
        I2C_SendData(I2Cx, *c); 
        c++;
        l--;  
        /* Test on EV8_2 to ensure data is transmitted, can used EV_8 for faster transmission*/
        time = I2C_TIMEOUT;
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        {
            if ((time--) == 0) return false;
        }  
    }

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2Cx, ENABLE);
    /* ENABLE ALL INTERRUPT */
    __enable_irq();
    return true;
}



