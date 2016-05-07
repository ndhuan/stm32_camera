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

/**
 * @file i2c.h
 * I2C communication functions.
 * @author Thomas Boehm <thomas.boehm@fortiss.org>
 */


#ifndef I2C_H_
#define I2C_H_
#include <stdbool.h>
#include "stm32f4xx_i2c.h"

#define I2C2_OWNADDRESS_1_BASE 0x42 //7bit base address
/**
 * @brief  Configures I2C1 for communication as a slave (default behaviour for STM32F)
 */
void i2c_init(I2C_TypeDef* I2Cx);
bool I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l);
bool I2C_ReadBytes_NoReStart(I2C_TypeDef* I2Cx, uint8_t *c, uint8_t s, uint8_t l);
bool I2C_WriteBytes(I2C_TypeDef * I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l);
#endif /* I2C_H_ */

