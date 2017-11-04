/*
 * This file is part of the SSN project.
 *
 * Copyright (C) 2014-2015 Ernold Vasiliev <ericv@mail.ru>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * RTC_DS1307.h
 *
 *  Created on: 07 окт. 2014 г.
 *      Author: eric
 */

#ifndef RTC_DS1307_H_
#define RTC_DS1307_H_

#include "device.h"
#include "bb_device.h"
#include "i2c_impl.h"
//#include "soft_i2c.h"
#include <libopencm3/stm32/i2c.h>

#define DS1307_ADDRESS        0xD0	// rtc ds1307 address
#define RTC_DS1307_RX_DEPTH       8
#define DS1307_SQW1HZ             7
#define DS1307_SQW4KHZ            8
#define DS1307_SQW8KHZ            9
#define DS1307_SQW32KHZ          10
#define DS1307_LOW                0
#define DS1307_HIGH               1
// Prescaler
#define DS1307_LOW_BIT		       0b00000000
#define DS1307_HIGH_BIT		       0b10000000
#define DS1307_SQW1HZ_BIT	       0b00010000
#define DS1307_SQW4KHZ_BIT	       0b00010001
#define DS1307_SQW8KHZ_BIT	       0b00010010
#define DS1307_SQW32KHZ_BIT       0b00010011

#define EPOCH_LINUX
#define DS1307_EPOCH_YEAR         2000
#define DS1307_EPOCH_YEAR_linux   1970

typedef struct{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
} TinyRTC_datetime_t;

//uint16_t RTC_DS1307_date2days(uint16_t year, uint8_t month, uint8_t day);
//uint32_t RTC_DS1307_date2seconds(uint16_t year, uint8_t month, uint8_t day,uint8_t hour, uint8_t minute, uint8_t second);
//long RTC_DS1307_time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s);
//void RTC_DS1307_DateTime32 (uint32_t t);
//void RTC_DS1307_DateTime8 (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);
//void RTC_DS1307_DateTimestr (const char* date, const char* time);
//uint8_t RTC_DS1307_dayOfWeek(void);
//uint32_t RTC_DS1307_unixtime(void);
//uint8_t RTC_DS1307_isrunning(void);
int32_t RTC_DS1307_adjust(sGrpDev* pGrpDev);
int32_t RTC_DS1307_now(sGrpDev* pGrpDev);
int32_t RTC_DS1307_SetOutput(sGrpDev* pGrpDev, uint8_t c);

//void RTC_Millis_adjust(void);
//uint32_t RTC_Millis_now(void);

extern TinyRTC_datetime_t DS1307_time;


#endif /* RTC_DS1307_H_ */
