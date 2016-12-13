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

#define _DS18B20_LOCAL

#include "DS18B20.h"
#include <libopencm3/stm32/rtc.h>
#include "FreeRTOS.h"

// ============================================================================================
ds18b20_device* ds18b20_init (sGrpInfo* pGrpInfo, char* pcROMid)
{
	ds18b20_device* pDS18b20;

	pDS18b20 = (ds18b20_device*) pvPortMalloc(sizeof(ds18b20_device));
	if (!pDS18b20) return pdFAIL;

	pDS18b20->owiDevice = owi_device_init(&pGrpInfo->GrpDev);

	owi_device_put_rom(pDS18b20->owiDevice, pcROMid);
	pDS18b20->uiLastUpdate = 0;
	if (!pDS18b20) return pdFAIL;

	return pDS18b20;

}

void ds18b20_delete(ds18b20_device* pDS18b20)
{
	owi_device_delete(pDS18b20->owiDevice);
	vPortFree((void*)pDS18b20);
}
//*********************************************************************************************
// function  read temperature
// argument   group & sensor structures
// return    0 - Ok, 1 - error
// measurment result in ds18b20_device structure
//*********************************************************************************************
uint8_t ds_read_temperature(sGrpDev* pGrpDev, ds18b20_device* ds18b20Dev)
{
  signed char integer = 0;
  signed char frac;
  signed int  result;
  uint8_t     buff[8];
  uint8_t cnt = 3;	// tries counter
  OWI_device* pDev = ds18b20Dev->owiDevice;

// read data from sensor
  do {
	  result = ds_read_data_ROM(buff, pGrpDev, pDev);
  } while ((result !=0) && (cnt-- > 0));
  if(result !=0 ) return 1;

  frac    = buff[0] & 0x0f;                            //получить дробную часть
  integer = (buff[0]>>4) | ((buff[1] & 0x0f)<<4);      //получить целую часть

// if negative temperature
  if(integer<0)
  {
    integer = 0 - integer - 1;
    result  = integer *10;                            //учитываем целую часть
    frac = frac | 0xf0;
    frac = 0 - frac ;
  }
 // if positive temperature
  else     result  = integer *10;                    //учитываем целую часть

  result = result + ((frac*10)/16);                  //учитываем дробную часть

  ds18b20Dev->nDevPrevValue = ds18b20Dev->iDevValue;
  ds18b20Dev->iDevValue = result;
  ds18b20Dev->uiLastUpdate = rtc_get_counter_val();

  return 0;
}

//**************************************************************************************************
//function  start temperature measuring on whole line (line - sensors on single pin)
//arguments: device group, pointer to device array, first device in this group index in array
//return    0 - Ok, 1 - sensor not find, 2 - short circuit
//**************************************************************************************************
uint8_t ds_start_convert_all(sGrpInfo* pGrpInfo, sDevice* devArray[], uint16_t devIndex)
{
  uint8_t i, result = 0, result2 = 0;
  sGrpDev* pGrpDev = &pGrpInfo->GrpDev;
//taskENTER_CRITICAL(); {
  result = owi_reset_pulse(pGrpDev);       // send reset pulse
  if (result) return result;               // if error return error code
  owi_write_byte(OWI_ROM_SKIP,pGrpDev);    // all sensors
  owi_write_byte(OWI_CONVERT,pGrpDev);     // conversation
//} taskEXIT_CRITICAL();

//  delay_nus(pGrpDev, 100000);				// wait end of the conversation
  delay_ms(pGrpDev, 100);				// wait end of the conversation

// record temperature values into the devices array:
  for (i = devIndex; i < (pGrpInfo->iDevQty+devIndex); i++) {
	  if (devArray[i]->pDevStruct) {
//		  taskENTER_CRITICAL(); {
	  	  result2 = ds_read_temperature(pGrpDev, (ds18b20_device*) devArray[i]->pDevStruct);
	  	  if (result2) { result++; }
//		  } taskEXIT_CRITICAL();
	  	  delay_nus(pGrpDev, 1000);				// wait
	  }
	}
  return result;	// more errors -> more return value
}

//*********************************************************************************************
//function  start temperature measuring for sensor by it ROM
//argument  device info data
//return    0 - Ok, 1 - sensor not find, 2 - short circuit
//*********************************************************************************************
uint8_t ds_start_convert_single(sGrpDev* pGrpDev, OWI_device* pDev)
{
  uint8_t result;
  result = owi_reset_pulse(pGrpDev);
  if(result) return result;
  owi_MatchRom(pGrpDev, pDev);            // select sensor
  owi_write_byte(OWI_CONVERT,pGrpDev);    // start conversation
  return 0;
}

//*********************************************************************************************
//function  read sensor memory (for single sensor on line)
//argument  pointer to the buffer, device info data
//return    0 - Ok, 1 - sensor not find, 2 - short circuit
//*********************************************************************************************
uint8_t ds_read_data_single(uint8_t *buff, sGrpDev* pGrpDev)
{
  uint8_t crc = 0;
  uint8_t data;
  uint8_t i; //,j;
  uint8_t tmp;

  tmp = owi_reset_pulse(pGrpDev);

  if(tmp) return tmp;

  owi_write_byte(OWI_ROM_SKIP,pGrpDev);            // select any sensor

  owi_write_byte(OWI_READ_SCRATCHPAD,pGrpDev);     // request for 9 bytes of memory

  // read 8 bytes and calculate CRC:
  for( i=0; i<8; i++)
  {
    data = owi_read_byte(pGrpDev);
    buff[i] = data;
    crc = owi_ComputeCRC8(data, crc);
  }

  data = owi_read_byte(pGrpDev);          // read last byte with CRC
  if (crc==data) return 0;
  return 3;                               // CRC error
}

//*********************************************************************************************
//function  read sensor memory for sensor by it ROM
//argument  pointer to the buffer, device info data
//return    0 - Ok, 1 - sensor not find, 2 - short circuit, 3 - CRC error
//*********************************************************************************************
uint8_t ds_read_data_ROM(uint8_t *buff, sGrpDev* pGrpDev, OWI_device* pDev)
{
  uint8_t crc = 0;
  uint8_t data;
  uint8_t i; //,j;
  uint8_t tmp;

  tmp = owi_reset_pulse(pGrpDev);

  if (tmp) return tmp;

  owi_MatchRom(pGrpDev, pDev);
  owi_write_byte(OWI_READ_SCRATCHPAD,pGrpDev);   // request for 9 bytes of memory

  for( i=0; i<8; i++)
  {
    data = owi_read_byte(pGrpDev);
    buff[i] = data;
    crc = owi_ComputeCRC8(data, crc);
  }
  data = owi_read_byte(pGrpDev);          // read last byte with CRC

  if (crc==data) return 0;

  return 3;                               // CRC error
}
