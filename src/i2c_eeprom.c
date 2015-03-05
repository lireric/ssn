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
 * i2c_eeprom.c
 *
 *  Created on: 05 окт. 2014 г.
 *      Author: eric
 */

//#include <libopencm3/stm32/i2c.h>
#include "i2c_eeprom.h"
#include "device.h"
#include "bb_device.h"
#include "soft_i2c.h"


//использвоание на примере FRAM

static const uint8_t EEPROM_SADDR = 0xA0;

/*******************************************************************************
*
*******************************************************************************/
uint8_t eeprom_read(sGrpDev* pGrpDev, uint8_t eeprom_addr, uint16_t addr, uint8_t* buf, uint32_t size)
{
//addr = SWP(addr);

uint8_t rep_cnt = 3;
uint8_t rc = 0;
while( --rep_cnt > 0 && rc == 0 )
{
//rc = soft_i2c_read_ex(pGrpDev, eeprom_addr, (uint8_t*)&addr, sizeof(addr), buf, size);
rc = soft_i2c_read_ex2(pGrpDev, eeprom_addr, addr, buf, size);
delay_nus(pGrpDev, 10);
}
return rc;
}
/******************************************************************************/



/*******************************************************************************
*
*******************************************************************************/
uint8_t eeprom_write(sGrpDev* pGrpDev, uint8_t eeprom_addr, uint16_t addr, uint8_t* buf, uint32_t size)
{
//addr = SWP(addr);
	uint8_t rc = 0;
	uint32_t iCnt = size, iBytes;



// split data on 32 byte pages
//	pages = size / 32;
//	tailbytes = size % 32;

// send tailbytes:
//	if (tailbytes > 0) {
//		rc = soft_i2c_send_ex(pGrpDev, eeprom_addr, (uint8_t*) &send_addr, sizeof(send_addr), buf, tailbytes);
//		delay_nus(pGrpDev, 5000);
//	}


// Writes are restricted to a single 32 byte page.  Therefore. if a write spans a page
// boundry we must split the write.

	while (iCnt > 0) {
		iBytes = iCnt;
	    int iCurPage = addr & ~((int)0x1f);
	    if (addr+iBytes > iCurPage+32) { // Number of bytes is too large
	      iBytes = (iCurPage+32) - addr;
	    }

//		rc = soft_i2c_send_ex(pGrpDev, eeprom_addr, (uint8_t*) &addr, sizeof(addr), buf, iBytes);
		rc = soft_i2c_send_ex2(pGrpDev, eeprom_addr, addr, buf, iBytes);
		delay_nus(pGrpDev, 10000);// delay 10ms for internal EEPROM write operation

	    iCnt -=(int)iBytes;
	    addr+=(int)iBytes;
	    buf +=(int)iBytes;
	}

return rc;
}

/******************************************************************************/

//uint8_t i2c_eeprom_read_data_single(uint32_t i2c, uint8_t device, uint8_t addr)
//{
//	uint32_t reg32 __attribute__((unused));
//	uint8_t eeprom_data;
//
//	/* Send START condition. */
//	i2c_send_start(i2c);
//
//	/* Waiting for START is send and switched to master mode. */
//	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
//		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
//
//	/* Say to what address we want to talk to. */
//	i2c_send_7bit_address(i2c, device, I2C_WRITE);
//
//	/* Waiting for address is transferred. */
//	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
//
//	/* Cleaning ADDR condition sequence. */
//	reg32 = I2C_SR2(i2c);
//
//	i2c_send_data(i2c, addr); /*  */
//	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
//
//	/*
//	 * Now we transferred that we want to get EEPROM data.
//	 * Now we send another START condition (repeated START) and then
//	 * transfer the destination but with flag READ.
//	 */
//
//	/* Send START condition. */
//	i2c_send_start(i2c);
//
//	/* Waiting for START is send and switched to master mode. */
//	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
//		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
//
//	/* Say to what address we want to talk to. */
//	i2c_send_7bit_address(i2c, device, I2C_READ);
//
//	/* 2-byte receive is a special case. See datasheet POS bit. */
//	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);
//
//	/* Waiting for address is transferred. */
//	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
//
//	/* Cleaning ADDR condition sequence. */
//	reg32 = I2C_SR2(i2c);
//
//	/* Cleaning I2C_SR1_ACK. */
//	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
//
//	/* Now the slave should begin to send us the first byte. Await BTF. */
//	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
//	eeprom_data = (uint8_t)I2C_DR(i2c);
//
//	/* Original state. */
//	I2C_CR1(i2c) &= ~I2C_CR1_POS;
//
//	return eeprom_data;
//}
