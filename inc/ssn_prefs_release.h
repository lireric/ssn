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
 * ssn_prefs_release.h
 * preferences for public publications in GIT
 */

#ifndef INC_SSN_PREFS_RELEASE_H_
#define INC_SSN_PREFS_RELEASE_H_

/* Used modules */

//#define M_RTC_DS1307
//#define M_LCD
#define M_USART
#define M_DS18B20
#define M_DHT
#define M_GSM

#define WATCHDOG

#define WATCHDOG_PERIOD 10000	// period of watchdog timer (ms)

//#define DEBUG_S	// debug task statistics

#define MC_OBJECT	1

// hardware specific FREERTOS settings
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )
#define configTICK_RATE_HZ			( ( portTickType ) 300 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 56 * 1024 ) )

/*-----------------------------------------------------------*/
/* Hardware application settings */
#define mainMAX_DEV_GROUPS				(30) 	// max number sensor groups
#define mainMAX_GRP_DEVICES				(7) 	// default max number devices on one group
#define mainMAX_ALL_DEVICES				(50) 	// default max number of all devices
#define mainMAX_DHT_DEVICES				(3) 	// default max number of dht devices
#define mainMAX_ROUTES					(2)		// max number routing interfaces


/* Persistence settings *
 * (select one) */
//#define PERSIST_EEPROM_I2C
#define PERSIST_STM32FLASH


/* EEPROM I2C ------------------------------------------------*/
#define I2CBITRATE 				( 100000 )
//#define I2C 	I2C1
#define EEPROM_ADDRESS        	0xA0	// at24c32
//#define DS1307_ADDRESS        0xD0	// rtc ds1307 address
#define EEPROM_PAGE_SIZE      	32


/* soft i2c hardware settings */
/*  soft i2c group must be = 0 */
#define SOFTI2C_1_PIN_SDA        7	//	PB7
#define SOFTI2C_1_PIN_SCL        6	//	PB6
#define SOFTI2C_1_PORT      	 GPIOB
#define SOFTI2C_TIMER_1 		TIM3    // timer for module delays
#define SOFTI2C_1_GRP			( 0 )	// group soft i2c
#define SOFTI2C_1_MAXDEVS		( 2 )

/* COM port and baud rate used by the base out task. */
#define mainBAUD_RATE						( 57600 )
#define mainBASECOM							( mainCOM0 )


/* Maximum elements number in parsed action formula string */
#define MAX_ACTION_ARRAY_SIZE			(50)
#define mainMAX_ACTIONS					(50) 	// max number of actions

#define mainLOG_ACTIONS_SIZE			(10) 	// size of log actions array

//#define mainDEFAULT_LOG_INTERFACE		main_IF_GSM 	// interface to routing log info

#define configUSE_TRACE_FACILITY 0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

extern void log_event (void* poldt, void* pnewt, uint32_t xTickCount);
extern void log_event2 (char c, void* pt);


/* xTaskThatWasRunning is defined in main.c. */
extern void* xTaskThatWasRunning;
extern void* xBaseOutTaskHnd;


#endif /* INC_SSN_PREFS_RELEASE_H_ */
