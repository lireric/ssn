/*
 * i2c_impl.h
 *
 *  Created on: 15 окт. 2017 г.
 *      Author: eric
 */

#ifndef INC_I2C_IMPL_H_
#define INC_I2C_IMPL_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include "device.h"

void i2c_setup(sGrpDev* pGrpDev);
int i2c_WriteBufferAddress (sGrpDev* pGrpDev,  uint8_t chipAddress,  uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer );
uint32_t i2c_ReadBufferAddress (sGrpDev* pGrpDev, uint8_t chipAddress, uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer);

#endif /* INC_I2C_IMPL_H_ */
