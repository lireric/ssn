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
 * ssn_prefs_prod1.h
 *
 *      Preferences for Lobnja flat ***********************
 */

#ifndef INC_SSN_PREFS_PROD1_H_
#define INC_SSN_PREFS_PROD1_H_

/* Used modules */
#define M_RTC_DS1307
//#define M_LCD
#define M_USART
#define M_DS18B20
#define M_DHT
//#define M_GSM

/* Persistence settings *
 * (select one) */
#define PERSIST_EEPROM_I2C
//#define PERSIST_STM32FLASH

//#define WATCHDOG
#define WATCHDOG_PERIOD 10000	// period of watchdog timer (ms)

//#define DEBUG_S	// debug task statistics

#define MC_OBJECT	10

// hardware specific FREERTOS settings
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )
#define configTICK_RATE_HZ			( ( portTickType ) 100 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 45 * 1024 ) )

/*-----------------------------------------------------------*/
/* Hardware application settings */
#define mainMAX_DEV_GROUPS				(20) 	// max number sensor groups
#define mainMAX_GRP_DEVICES				(7) 	// default max number devices on one group
#define mainMAX_ALL_DEVICES				(30) 	// default max number of all devices
#define mainMAX_DHT_DEVICES				(3) 	// default max number of dht devices
#define mainMAX_ROUTES					(3)		// max number routing interfaces

// memory type device saving period (sec)
#define mainMEMORY_DEV_SAVE_PERIOD		36000
// maximum quantity of memory devices. Usually enough 1 element
#define mainMEMORY_DEV_MAX_QTY			2


// skip preferences loading button:
#define mainSKIP_PREF_PIN		12		//	PD12
#define mainSKIP_PREF_PORT		GPIOD
#define mainSKIP_PREF_VALUE		pdFALSE	// pin value for skip prefs actuate


///#define I2C 	I2C1
#define EEPROM_ADDRESS        	0xA0	// at24c32
#define DS1307_ADDRESS        	0xD0	// rtc ds1307 address

#define EEPROM_MAX_SIZE      	4096	// at24c32
#define EEPROM_PAGE_SIZE      	32
/* soft i2c hardware settings */
/* Группа soft i2c номер 0 */
#define SOFTI2C_1_PIN_SDA       1	//	PC0
#define SOFTI2C_1_PIN_SCL       0	//	PC1
#define SOFTI2C_1_PORT       	GPIOC
#define SOFTI2C_TIMER_1 		TIM3    //задаем таймер, используемый для формирования задержек
#define SOFTI2C_1_GRP			( 0 )	// группа датчиков soft i2c
#define SOFTI2C_1_MAXDEVS		( 2 )	// максимальное количество датчиков на линии

#define mainBAUD_RATE						( 57600 )
#define mainBASECOM							( mainCOM2 )
//#define mainBASECOM3_RTSBANK				GPIOB
//#define mainBASECOM3_RTSPIN					GPIO10

/* Maximum elements number in parsed action formula string */
#define MAX_ACTION_ARRAY_SIZE			(50)
#define mainMAX_ACTIONS					(50) 	// max number of actions

#define mainLOG_ACTIONS_SIZE			(10) 	// size of log actions array

#endif /* INC_SSN_PREFS_PROD1_H_ */
