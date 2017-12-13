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
	uint32_t nRes;
//rc = soft_i2c_read_ex(pGrpDev, eeprom_addr, (uint8_t*)&addr, sizeof(addr), buf, size);
//rc = soft_i2c_read_ex2(pGrpDev, eeprom_addr, addr, buf, size);
	nRes = soft_i2c_ReadBufferAddress16(pGrpDev, eeprom_addr, addr, buf, size);
	return nRes;
}
/******************************************************************************/



/*******************************************************************************
*
*******************************************************************************/
uint8_t eeprom_write(sGrpDev* pGrpDev, uint8_t eeprom_addr, uint16_t addr, uint8_t* buf, uint32_t size)
{
//	uint8_t rc = 0;
	uint32_t nRes;
	uint32_t iCnt = size, iBytes;

// Writes are restricted to a single 32 byte page.  Therefore. if a write spans a page
// boundry we must split the write.

	while (iCnt > 0) {
		iBytes = iCnt;
	    int iCurPage = addr & ~((int)0x1f);
	    if (addr+iBytes > iCurPage+32) { // Number of bytes is too large
	      iBytes = (iCurPage+32) - addr;
	    }

//		rc = soft_i2c_send_ex(pGrpDev, eeprom_addr, (uint8_t*) &addr, sizeof(addr), buf, iBytes);
//		rc = soft_i2c_send_ex2(pGrpDev, eeprom_addr, addr, buf, iBytes);

//	    nRes = soft_i2c_WriteBufferAddress16 (pGrpDev,  eeprom_addr,  addr, buf, iBytes);
		nRes = soft_i2c_WriteBufferAddress16(pGrpDev, eeprom_addr, addr, buf, iBytes);

		delay_nus(pGrpDev, 10000);// delay 10ms for internal EEPROM write operation

	    iCnt -=(int)iBytes;
	    addr+=(int)iBytes;
	    buf +=(int)iBytes;
	}

return nRes;
}

