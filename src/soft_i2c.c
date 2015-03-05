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
 * soft_i2c.c
 *
 *  Created on: 05 окт. 2014 г.
 *      Author: eric
 */

#include "device.h"
#include "bb_device.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "soft_i2c.h"

// SCL и SDA must be configured in GPIO_Mode_Out_OD

uint8_t ack = 0;

static const uint8_t I2CSWM_DIRECTION_TX = 0;
static const uint8_t I2CSWM_DIRECTION_RX = 1;

static const uint8_t I2CSWM_NACK = 1;
static const uint8_t I2CSWM_ACK = 0;

void soft_i2c_start(sGrpDev* pGrpDev)
{
// GENERATION START BIT

	bb_set_wire(pGrpDev);		// SETSDA
	bb_set_wire_SCL (pGrpDev);	// SETSCL
	bb_clear_wire (pGrpDev); 	// SDA_LOW
	delay_nus (pGrpDev, 5);
	bb_clear_wire_SCL (pGrpDev); // SCL_LOW
	delay_nus (pGrpDev, 5);
}

void soft_i2c_stop(sGrpDev* pGrpDev)
{
	bb_clear_wire (pGrpDev); 	//  SDA_LOW
	delay_nus (pGrpDev, 1);
	bb_set_wire_SCL (pGrpDev);	// SETSCL
	delay_nus (pGrpDev, 4);
	bb_set_wire(pGrpDev);		// SETSDA
}

uint8_t soft_i2c_clock(sGrpDev* pGrpDev)
{
	uint8_t l;
	bb_set_wire_SCL (pGrpDev);	// SETSCL
	delay_nus (pGrpDev, 5);

	l = bb_read_wire_data_bit(pGrpDev);
	bb_clear_wire_SCL (pGrpDev); //  SCL_LOW
	delay_nus (pGrpDev, 5);

return l;
}

uint8_t soft_i2c_write (sGrpDev* pGrpDev, uint8_t data)
{
	uint8_t db = 0x80;
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (data & db) {
			bb_set_wire(pGrpDev);	// SETSDA
		} else {
			bb_clear_wire(pGrpDev); //  SDA_LOW
		}
		soft_i2c_clock(pGrpDev);
		db >>= 1;
	}

	/* get ACK */
	bb_set_wire(pGrpDev);			// SETSDA
	return soft_i2c_clock(pGrpDev);
}

//inline static
uint8_t soft_i2c_read(sGrpDev* pGrpDev, uint8_t ack )
	{
		uint8_t rb = 0;
		uint8_t db = 0x80;
		uint8_t i;
		for (i = 0; i < 8; i++) {
			if (soft_i2c_clock(pGrpDev))
				rb |= db;
			db >>= 1;
		}

		if (ack == I2CSWM_ACK) {
			/* send ACK */
			bb_clear_wire(pGrpDev); //  SDA_LOW
			soft_i2c_clock(pGrpDev);
			bb_set_wire(pGrpDev);	// SETSDA
		} else {
			/* send NACK */
			bb_set_wire(pGrpDev);	// SETSDA
			soft_i2c_clock(pGrpDev);
		}
		return rb;
	}

void soft_i2c_reset(sGrpDev* pGrpDev)
{
		bb_set_wire(pGrpDev);			// SETSDA
		uint8_t i;
		for (i = 0; i < 15; i++) {
			bb_set_wire_SCL(pGrpDev);	// SETSCL
			delay_nus(pGrpDev, 10);
			bb_clear_wire_SCL(pGrpDev); //  SCL_LOW
			delay_nus(pGrpDev, 10);
		}
		bb_set_wire(pGrpDev);			// SETSDA
		bb_set_wire_SCL(pGrpDev);		// SETSCL
}

int soft_i2c_read_ex(sGrpDev* pGrpDev, const uint16_t saddr,
			uint8_t* lpdata1, uint32_t size1, uint8_t* lpdata2, uint32_t size2)
{
		soft_i2c_start(pGrpDev);
		if (lpdata1 != NULL) {
			if (soft_i2c_write(pGrpDev, saddr | I2CSWM_DIRECTION_TX)
					== I2CSWM_NACK) {
				soft_i2c_stop(pGrpDev);
				return 0;
			}

			int i;
			for (i = 0; i < size1; i++) {
				if (soft_i2c_write(pGrpDev, *lpdata1++) == I2CSWM_NACK) {
					soft_i2c_stop(pGrpDev);
					return 0;
				}
			}
			soft_i2c_start(pGrpDev);
		}

		if (lpdata2 != NULL) {
			if (soft_i2c_write(pGrpDev, saddr | I2CSWM_DIRECTION_RX)
					== I2CSWM_NACK) {
				soft_i2c_stop(pGrpDev);
				return 0;
			}
			bb_set_wire_SCL(pGrpDev);	// SETSCL
			int i;
			for (i = 1; i < size2; i++) {
				*lpdata2 = soft_i2c_read(pGrpDev, I2CSWM_ACK);
				lpdata2++;
			}
			*lpdata2 = soft_i2c_read(pGrpDev, I2CSWM_NACK);
			lpdata2++;
		}
		soft_i2c_stop(pGrpDev);
		return 1;
}

int soft_i2c_send_ex(sGrpDev* pGrpDev, const uint16_t saddr, uint8_t* lpdata1, uint32_t size1, uint8_t* lpdata2, uint32_t size2)
{
		soft_i2c_start(pGrpDev);
		if (lpdata1 != NULL) {
			if (soft_i2c_write(pGrpDev, saddr | I2CSWM_DIRECTION_TX)
					== I2CSWM_NACK) {
				soft_i2c_stop(pGrpDev);
				return 0;
			}

			int i;
			for (i = 0; i < size1; i++) {
				if (soft_i2c_write(pGrpDev, *lpdata1++) == I2CSWM_NACK) {
					soft_i2c_stop(pGrpDev);
					return 0;
				}
			}
		}

		if (lpdata2 != NULL) {

			int i;
			for (i = 0; i < size2; i++) {
				if (soft_i2c_write(pGrpDev, *lpdata2++) == I2CSWM_NACK) {
					soft_i2c_stop(pGrpDev);
					return 0;
				}
			}
		}
		soft_i2c_stop(pGrpDev);
		return 1;
}

int soft_i2c_send_ex2(sGrpDev* pGrpDev, const uint16_t saddr, uint16_t lpdata1, uint8_t* lpdata2, uint32_t size2)
{
		soft_i2c_start(pGrpDev);
			if (soft_i2c_write(pGrpDev, saddr | I2CSWM_DIRECTION_TX)
					== I2CSWM_NACK) {
				soft_i2c_stop(pGrpDev);
				return 0;
			}

			if (soft_i2c_write(pGrpDev, (lpdata1 >> 8) & 0xff) == I2CSWM_NACK) {
					soft_i2c_stop(pGrpDev);
					return 0;
				} else {
					if (soft_i2c_write(pGrpDev, (lpdata1  & 0xff)) == I2CSWM_NACK) {
						soft_i2c_stop(pGrpDev);
						return 0;
					}
				}

		if (lpdata2 != NULL) {

			int i;
			for (i = 0; i < size2; i++) {
				if (soft_i2c_write(pGrpDev, *lpdata2++) == I2CSWM_NACK) {
					soft_i2c_stop(pGrpDev);
					return 0;
				}
			}
		}
		soft_i2c_stop(pGrpDev);
		return 1;
}

int soft_i2c_read_ex2(sGrpDev* pGrpDev, const uint16_t saddr, uint16_t lpdata1, uint8_t* lpdata2, uint32_t size2)
{
		soft_i2c_start(pGrpDev);
			if (soft_i2c_write(pGrpDev, saddr | I2CSWM_DIRECTION_TX)
					== I2CSWM_NACK) {
				soft_i2c_stop(pGrpDev);
				return 0;
			}

			if (soft_i2c_write(pGrpDev, (lpdata1 >> 8) & 0xff) == I2CSWM_NACK) {
					soft_i2c_stop(pGrpDev);
					return 0;
				} else {
					if (soft_i2c_write(pGrpDev, (lpdata1  & 0xff)) == I2CSWM_NACK) {
						soft_i2c_stop(pGrpDev);
						return 0;
					}
				}


			soft_i2c_start(pGrpDev);

		if (lpdata2 != NULL) {
			if (soft_i2c_write(pGrpDev, saddr | I2CSWM_DIRECTION_RX)
					== I2CSWM_NACK) {
				soft_i2c_stop(pGrpDev);
				return 0;
			}
			bb_set_wire_SCL(pGrpDev);	// SETSCL
			int i;
			for (i = 1; i < size2; i++) {
				*lpdata2 = soft_i2c_read(pGrpDev, I2CSWM_ACK);
				lpdata2++;
			}
			*lpdata2 = soft_i2c_read(pGrpDev, I2CSWM_NACK);
			lpdata2++;
		}
		soft_i2c_stop(pGrpDev);
		return 1;
}

