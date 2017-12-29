/*
 * This file is part of the SSN project.
 *
 * Copyright (C) 2014-2015 Ernold Vasiliev <ericv@mail.ru>
 *

    SSN project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSN project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SSN project.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * soft_i2c.h
 *
 *  Created on: 06 окт. 2014 г.
 *      Author: eric
 */

#ifndef SOFT_I2C_H_
#define SOFT_I2C_H_

#include "device.h"
#include "bb_device.h"

void soft_i2c_init(sGrpDev* pGrpDev);
void soft_i2c_clear_wire_SDA(sGrpDev* pGrpDev);
void soft_i2c_set_wire_SDA(sGrpDev* pGrpDev);
void soft_i2c_clear_wire_SCL(sGrpDev* pGrpDev);
void soft_i2c_set_wire_SCL(sGrpDev* pGrpDev);
uint8_t soft_i2c_read_SDA(sGrpDev* pGrpDev);
uint8_t soft_i2c_start(sGrpDev* pGrpDev);
uint8_t soft_i2c_write (sGrpDev* pGrpDev, uint8_t data);
uint8_t soft_i2c_read(sGrpDev* pGrpDev, uint8_t ack );

int32_t soft_i2c_TryAddress (sGrpDev* pGrpDev, uint8_t chipAddress);

//void 	soft_i2c_start(sGrpDev* pGrpDev);
uint8_t soft_i2c_clock(sGrpDev* pGrpDev);
void 	soft_i2c_stop(sGrpDev* pGrpDev);
void soft_i2c_ack(sGrpDev* pGrpDev);
void soft_i2c_NoAck(sGrpDev* pGrpDev);
uint8_t soft_i2c_WaitAck(sGrpDev* pGrpDev);
void soft_i2c_PutByte(sGrpDev* pGrpDev, uint8_t data);
uint8_t soft_i2c_GetByte(sGrpDev* pGrpDev);
uint32_t soft_i2c_ReadBuffer (sGrpDev* pGrpDev, uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer);
uint32_t soft_i2c_ReadBufferAddress (sGrpDev* pGrpDev, uint8_t chipAddress, uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer);

uint32_t soft_i2c_ReadBufferAddress16 (sGrpDev* pGrpDev, uint8_t chipAddress, uint16_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer);
int soft_i2c_WriteBuffer (sGrpDev* pGrpDev,  uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer );
int soft_i2c_WriteBufferAddress (sGrpDev* pGrpDev,  uint8_t chipAddress,  uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer);
int soft_i2c_WriteBufferAddress16 (sGrpDev* pGrpDev,  uint8_t chipAddress,  uint16_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer);

//uint8_t soft_i2c_clock(sGrpDev* pGrpDev);
//uint8_t soft_i2c_write (sGrpDev* pGrpDev, uint8_t data);
//uint8_t soft_i2c_read(sGrpDev* pGrpDev, uint8_t ack );
void 	soft_i2c_reset(sGrpDev* pGrpDev);
//int 	soft_i2c_read_ex(sGrpDev* pGrpDev, const uint16_t saddr, uint8_t* lpdata1, uint32_t size1, uint8_t* lpdata2, uint32_t size2);
//int 	soft_i2c_send_ex(sGrpDev* pGrpDev, const uint16_t saddr, uint8_t* lpdata1, uint32_t size1, uint8_t* lpdata2, uint32_t size2);
//int 	soft_i2c_send_ex2(sGrpDev* pGrpDev, const uint16_t saddr, uint16_t lpdata1, uint8_t* lpdata2, uint32_t size2);
//int 	soft_i2c_read_ex2(sGrpDev* pGrpDev, const uint16_t saddr, uint16_t lpdata1, uint8_t* lpdata2, uint32_t size2);
//int 	soft_i2c_read_ex8(sGrpDev* pGrpDev, const uint8_t saddr, uint8_t lpdata1, uint8_t* lpdata2, uint32_t size2);

#endif /* SOFT_I2C_H_ */
