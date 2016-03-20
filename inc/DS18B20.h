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


#ifndef _DS18B20_H_
#define _DS18B20_H_

#include "device.h"
#include "bb_device.h"

#ifndef _DS18B20_LOCAL
#define DS18B20_EXTERN    extern
#else
#define DS18B20_EXTERN
#endif

// ---------------------------------------------
#define DS18B20_FAMILY_NUM      0x28

/****************************************************************************
 ROM commands (ds18b20)
****************************************************************************/
#define 	OWI_ALARM_SEARCH    0xEC
#define 	OWI_CONVERT         0x44
#define 	OWI_WRITE_SCRATCHPAD 0x4E
#define 	OWI_READ_SCRATCHPAD 0xBE
#define 	OWI_COPY_SCRATCHPAD 0x48
#define 	OWI_RECALL          0xB8


typedef struct
{
	OWI_device* owiDevice;		// common OWI data of this device
    uint32_t 	uiLastUpdate;	// last update device value
    signed int 	iDevValue;		// device value
    signed int 	nDevPrevValue;		// device value
} ds18b20_device;


#define DS18B20_9BIT            0x1F
#define DS18B20_10BIT           0x3F
#define DS18B20_11BIT           0x5F
#define DS18B20_12BIT           0x7F

// ---------------------------------------------

extern uint8_t ds_buff[9];

ds18b20_device* ds18b20_init (sGrpInfo* GrpInfo, char* pcROMid);
void 			ds18b20_delete(ds18b20_device* pDS18b20);

uint8_t ds_start_convert_single(sGrpDev* pGrpDev, OWI_device* pDev);
uint8_t ds_start_convert_all(sGrpInfo* pGrpInfo, sDevice* devArray[], uint16_t devIndex);
uint8_t ds_read_data_single(uint8_t *ds_buff, sGrpDev* pGrpDev);
uint8_t ds_read_temperature(sGrpDev* pGrpDev, ds18b20_device* ds18b20Dev);
uint8_t ds_read_data_ROM(uint8_t *buff, sGrpDev* pGrpDev, OWI_device* pDev);

#endif
