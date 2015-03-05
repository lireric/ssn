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
 * RTC_DS1307.c
 *
 *  Created on: 07 окт. 2014 г.
 *      Author: eric
 *      Based by chTinyRTC.c Federico Rossi
 */

#include "i2c_eeprom.h"
#include "device.h"
#include "bb_device.h"
#include "soft_i2c.h"
#include "RTC_DS1307.h"

// global variable
TinyRTC_datetime_t DS1307_time;

/*************************************************************************************************/
// static function (for functions used in this file only)

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

uint8_t RTC_DS1307_now(sGrpDev* pGrpDev) {

	uint8_t rxbuf[RTC_DS1307_RX_DEPTH];
	uint8_t txbuf[1];
	uint8_t addr = 0;
	txbuf[0]= 0x00;
	uint8_t rc = 0;
	rc = soft_i2c_send_ex(pGrpDev, DS1307_ADDRESS, (uint8_t*)&addr, sizeof(addr), txbuf, 1);
	rc = soft_i2c_read_ex(pGrpDev, DS1307_ADDRESS, (uint8_t*)&addr, sizeof(addr), rxbuf, 7);

	if (rc) {
		DS1307_time.sec = bcd2bin(rxbuf[0] & 0x7F);
		DS1307_time.min = bcd2bin(rxbuf[1]);
		DS1307_time.hour = bcd2bin(rxbuf[2]);
		DS1307_time.day  = bcd2bin(rxbuf[4]);
		DS1307_time.month  = bcd2bin(rxbuf[5]);
		DS1307_time.year  = bcd2bin(rxbuf[6]);
	}

	return rc;
}



uint8_t RTC_DS1307_adjust(sGrpDev* pGrpDev) {

	//uint8_t rxbuf;
	uint8_t txbuf[8];
	uint8_t addr = 0;

	txbuf[0]=bin2bcd(DS1307_time.sec);
	txbuf[1]=bin2bcd(DS1307_time.min);
	txbuf[2]=bin2bcd(DS1307_time.hour);
	txbuf[3]=bin2bcd(0);
	txbuf[4]=bin2bcd(DS1307_time.day);
	txbuf[5]=bin2bcd(DS1307_time.month);
	txbuf[6]=bin2bcd(DS1307_time.year);
	txbuf[7]=0;

	uint8_t rc = 0;
	rc = soft_i2c_send_ex(pGrpDev, DS1307_ADDRESS, (uint8_t*)&addr, sizeof(addr), txbuf, 8);

	return rc;
}

uint8_t RTC_DS1307_SetOutput(sGrpDev* pGrpDev, uint8_t c) {
	uint8_t out;
	uint8_t txbuf[2]; //, rxbuf;
	uint8_t rc = 0;
	uint8_t addr = 0x07;
	switch(c)
	{
	case DS1307_HIGH :
		out=DS1307_HIGH_BIT;
		break;
	case DS1307_LOW :
		out=DS1307_LOW_BIT;
		break;
	case DS1307_SQW1HZ :
		out=DS1307_SQW1HZ_BIT;
		break;
	case DS1307_SQW4KHZ :
		out=DS1307_SQW4KHZ_BIT;
		break;
	case DS1307_SQW8KHZ :
		out=DS1307_SQW8KHZ_BIT;
		break;
	case DS1307_SQW32KHZ :
		out=DS1307_SQW32KHZ_BIT;
		break;
	default:
		out=DS1307_LOW_BIT;
		break;
	}
//	txbuf[0]=0x07;
	txbuf[0]=out;
//	RTC_DS1307_status = i2cMasterTransmitTimeout(&I2CD1, DS1307_ADDRESS, txbuf, 2, rxbuf, 0, TMO);
	rc = soft_i2c_send_ex(pGrpDev, DS1307_ADDRESS, (uint8_t*)&addr, sizeof(addr), txbuf, 1);
	return rc;
}

////////////////////////////////////////////////////////////////////////////////
// RTC_Millis implementation
//
//void RTC_Millis_adjust(void) {
//
//	uint32_t chtime,dstime;
//
//	chtime = chTimeNow();
//	dstime = RTC_DS1307_unixtime();
//	RTC_Millis_offset= dstime - (chtime/1000);
//}
//
//uint32_t RTC_Millis_now(void) {
//	return (uint32_t)(RTC_Millis_offset + (chTimeNow() / 1000));
//}

////////////////////////////////////////////////////////////////////////////////

