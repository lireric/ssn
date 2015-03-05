/*
 * This file is part of the SSN project.
 *
 * Copyright (C) 2014-2015 Ernold Vasiliev <ericv@mail.ru>
 * modifications for using libopencm3
 */
/*--------------------------------------------------------------------------  */
/*  RTC controls for STM32                                                    */
/*  Copyright (c) 2009, Martin Thomas 4/2009, BSD-license                     */
/*  partly based on code from STMircoelectronics, Peter Dannegger, "LaLaDumm" */
/*--------------------------------------------------------------------------  */
/*
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


#ifndef RTC_H_
#define RTC_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/nvic.h>

typedef struct {
	uint16_t year;	/* 1..4095 */
	uint8_t  month;	/* 1..12 */
	uint8_t  mday;	/* 1.. 31 */
	uint8_t  wday;	/* 0..6, Sunday = 0*/
	uint8_t  hour;	/* 0..23 */
	uint8_t  min;	/* 0..59 */
	uint8_t  sec;	/* 0..59 */
	uint8_t  dst;	/* 0 Winter, !=0 Summer */
} RTC_t;

void 	counter_to_struct( uint32_t sec, RTC_t *t );
int 	initialise_rtc(void);
void 	rtc_gettime (RTC_t*);					/* Get time */
void 	rtc_settime (const RTC_t*);				/* Set time */


#endif
