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

/* Common preferences for SSN project */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

// include common definitions:
#include "ssn_defs.h"

// include custom application specific preferences:
#include "ssn_prefs.h"


/* Standard includes. */
#include <assert.h>
#include <string.h>
#include "xprintf.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* Library includes. */
#include "libopencm3/stm32/rcc.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/timer.h>


#include "device.h"
#include "soft_i2c.h"
#include "i2c_eeprom.h"
#include "RTC_DS1307.h"
#include "rtc_func.h"
#include "crc16.h"

#include "cJSON.h"

/* Private define 				========================================== */
#define APP_NAME "*SmartSensorNet"

/* Driver includes. */
#ifdef  M_LCD
#include "lcd.h"
#endif
#ifdef  M_USART
	#include "STM32_USART.h"
#endif
#ifdef  M_DS18B20
	#include "DS18B20.h"
#endif
#ifdef  M_DHT
	#include "DHT22.h"
#endif
#ifdef  M_GSM
	#include "gsm.h"
#endif


#define BUFFER1_SIZE             ( 80 ) // (countof(Tx1Buffer)-1)

#define INI_USE_STACK 0

#define countof(a) (sizeof(a) / sizeof(*(a)))

uint8_t Rx1Buffer[BUFFER1_SIZE];


#ifndef  mainINPUT_QUEUE_SIZE
#define mainINPUT_QUEUE_SIZE	15
#endif
#ifndef  mainSENSORS_QUEUE_SIZE
#define mainSENSORS_QUEUE_SIZE	15
#endif
#ifndef  mainDEBUG_QUEUE_SIZE
#define mainDEBUG_QUEUE_SIZE	5
#endif
#ifndef  mainBASEOUT_QUEUE_SIZE
#define mainBASEOUT_QUEUE_SIZE	50
#endif


//void print_debug (const char *str);
void 		print_debug_FROM_ISR (const char *str);
sGrpInfo* 	getGrpInfo(unsigned char ucGrpNum);
int32_t 	apply_preferences(cJSON *json_data);
uint32_t vMainStartGSMTask(void* pParam);
xTimerHandle mainTimerCreate(char* pcTimerName, uint32_t nPeriod, uint32_t isPeriodic, sEvtElm* pEvtElm);
xTimerHandle mainTimerCreateOneShot(char* pcTimerName, uint32_t nPeriod, sEvtElm* pEvtElm);
uint16_t 	getMC_Object(void);
void 		setMC_Object(uint16_t uiObj);

/*-------------------------- QUEUES ---------------------------------*/
/* The queue used to send messages from different input sources. */
xQueueHandle xInputQueue;
/* The queue used to send data from different sensors. */
xQueueHandle xSensorsQueue;
/* The queue used to send common messages. */
xQueueHandle xBaseOutQueue;
// Queue for logging messages.
xQueueHandle xLogOutQueue;


#endif /* __MAIN_H */

