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

#ifndef _OWI_DEVICE_H_
#define _OWI_DEVICE_H_

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#define mainMAX_OWI_DEVICES				(30) // default max number of 1-wire devices

/****************************************************************************
 ROM commands (common)
****************************************************************************/
#define     OWI_ROM_READ    0x33    //!< READ ROM command code.
#define     OWI_ROM_SKIP    0xcc    //!< SKIP ROM command code.
#define     OWI_ROM_MATCH   0x55    //!< MATCH ROM command code.
#define     OWI_ROM_SEARCH  0xf0    //!< SEARCH ROM command code.
#define 	OWI_READ_POWER_SUPPLY 0xB4

/****************************************************************************
 Return codes
****************************************************************************/
#define     OWI_ROM_SEARCH_FINISHED     0x00    //!< Search finished return code.
#define     OWI_ROM_SEARCH_FAILED       0xff    //!< Search failed return code.
#define     OWI_CRC_OK      0x00    //!< CRC check succeded
#define     OWI_CRC_ERROR   0x01    //!< CRC check failed

#define SEARCH_SUCCESSFUL     0x00
#define SEARCH_CRC_ERROR      0x01
#define SEARCH_ERROR          0xff
#define AT_FIRST              0xff

// Common structure for all OWI devices
typedef struct
{
     unsigned char ucROM[8];
} OWI_device;

//  -- common functions for all bb-control devices
void 	delay_timer_init();
void 	delay_nus2(uint32_t nDelay);
void 	delay_nus(sGrpDev* pGrpDev, uint32_t nCount);	// nCount 16 bit!!!
void 	delay_ms(sGrpDev* pGrpDev, uint32_t nCount);	// nCount 16 bit!!!

uint8_t bb_read_wire_data_bit(sGrpDev* pGrpDev);
void 	bb_wire_in (sGrpDev* pGrpDev);
void 	bb_wire_out(sGrpDev* pGrpDev);
void	bb_clear_wire(sGrpDev* pGrpDev);
void 	bb_set_wire(sGrpDev* pGrpDev);
void 	bb_clear_wire2(sGrpDev* pGrpDev);
void 	bb_set_wire2(sGrpDev* pGrpDev);

// -- for owi devices:
OWI_device* owi_device_init(sGrpDev* pGrpDev);
void 	owi_device_delete (OWI_device* pOWIDev);

uint8_t 	owi_reset_pulse(sGrpDev* pGrpDev);
void 		owi_write_bit(volatile uint8_t bit,sGrpDev* pGrpDev);
uint16_t 	owi_read_bit(sGrpDev* pGrpDev);
void 		owi_write_byte(volatile uint8_t byte, sGrpDev* pGrpDev);
uint8_t 	owi_read_byte(sGrpDev* pGrpDev);
unsigned char owi_search_rom(unsigned char * bitPattern, unsigned char lastDeviation, sGrpDev* pGrpDev);
unsigned char owi_search_devices(OWI_device * devices, uint8_t numDevices, sGrpInfo* pGroup, uint8_t *num);
unsigned char owi_ComputeCRC8(unsigned char inData, unsigned char seed);
unsigned char owi_CheckRomCRC(unsigned char * romValue);
void	owi_SkipRom(sGrpDev* pGrpDev);
void	owi_ReadRom(sGrpDev* pGrpDev, OWI_device* pDev);
void 	owi_MatchRom(sGrpDev* pGrpDev, OWI_device* pDev);
void 	owi_device_put_rom(OWI_device* pDev, char * romValue);
uint8_t charToInt(char letter);

#endif


