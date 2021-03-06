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
 * ssh_prefs.h
 *
 *  Created on: 26 февр. 2015 г.
 *      Author: eric
 */

/* SSN board specific application preferences */

#ifndef INC_SSN_PREFS_H_
#define INC_SSN_PREFS_H_

#define CONF_TEST
//#define CONF_RELEASE
//#define CONF_PROD1 	// Kokorino
//#define CONF_PROD2 		// Lobnja


#ifdef CONF_TEST
	#include "ssn_prefs_test.h"
#endif

#ifdef CONF_RELEASE
	#include "ssn_prefs_release.h"
#endif

#ifdef CONF_TEST2
	#include "ssn_prefs_test_rbt6.h"
#endif

#ifdef CONF_PROD2
//	#include "ssn_prefs_prod1.h"
	#include "ssn_prefs_v1_p2.h"
#endif

#ifdef CONF_PROD1
//	#include "ssn_prefs_prod2.h"
	#include "ssn_prefs_v1_p1.h"
#endif

/* periodic task rates */
#define mainSensorRateLR			( ( portTickType ) 40000 / portTICK_RATE_MS )
#define mainSensorRateMR			( ( portTickType ) 6000 / portTICK_RATE_MS )
#define mainSensorRateHR			( ( portTickType ) 100 / portTICK_RATE_MS )
#define mainCronRate				( ( portTickType ) 100 / portTICK_RATE_MS )
#define mainDEBUG_STAT_RATE			( ( portTickType ) 10000 / portTICK_RATE_MS )
#define mainEthernetRateTCP			( ( portTickType ) 100 / portTICK_RATE_MS )
#define mainEthernetRateUDP			( ( portTickType ) 100 / portTICK_RATE_MS )

/* Task priorities. */
#define mainDEBUG_OUT_TASK_PRIORITY			( tskIDLE_PRIORITY)
#define mainINPUT_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define gsmGSM_TASK_START_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainCHECK_SENSOR_MR_TASK_PRIORITY	( tskIDLE_PRIORITY + 1 )
#define mainCHECK_SENSOR_HR_TASK_PRIORITY	( tskIDLE_PRIORITY + 2 )
#define mainPROC_SENSOR_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainECHO_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBASEOUT_TASK_PRIORITY			( tskIDLE_PRIORITY)

#define  devMAX_MSG_LEN				(127)

/* COM ports */
#define mainCOM0							( 0 ) // USART1
#define mainCOM1							( 1 ) // USART2
#define mainCOM2							( 2 ) // USART3
#define mainCOM3							( 3 ) // UART4
#define mainCOM4							( 4 ) // UART5

#define mainBASECOM_Priority				(200)

#define mainMINMEMORYALLOCATE				( 8 ) // minimum memory size for allocate
#define mainACTIONSARRAYTIMEOUT				(10)	// timeout submitting actions log (sec)

/* FLASH prefs: */
/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#if defined (STM32F10X_HD) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
#define STM32FLASH_PREF_SIZE	8192	// 8 Kb
#define STM32FLASH_BEGIN_ADDR	(((uint32_t)0x08000000) + ((uint32_t)0x7E000))	// last 8Kb flash memory
#define STM32FLASH_PAGE_SIZE	((uint16_t)0x800)
#else
#define STM32FLASH_PREF_SIZE	4096	// 4 Kb
#define STM32FLASH_BEGIN_ADDR	(((uint32_t)0x08000000) + ((uint32_t)0x1F000)) // last 4Kb flash memory
#define STM32FLASH_PAGE_SIZE	((uint16_t)0x400)
#endif

#ifndef SOFTI2C_TIMER_1
#define SOFTI2C_TIMER_1 		TIM6    // timer for module delays
#endif

#ifndef MAX_ACTION_ARRAY_SIZE
#define MAX_ACTION_ARRAY_SIZE	50
#endif

#ifndef mainMAX_ACTIONS
#define mainMAX_ACTIONS					(30) 	// max number of actions
#endif

#ifndef mainHEARTBEAT_PERIOD
#define mainHEARTBEAT_PERIOD	30
#endif

#ifndef mainINPUT_TASK_STACK
#define mainINPUT_TASK_STACK	250
#endif
#ifndef mainPROCSENSORS_TASK_STACK
#define mainPROCSENSORS_TASK_STACK	200
#endif

// Local MAC address
#ifndef ETHERNET_LOCAL_MAC
#define ETHERNET_LOCAL_MAC {0x00,0x08,0xdc,0x11,0x11,0x12} // to do
#endif
// Subnet mask
#ifndef ETHERNET_SSN_SN
#define ETHERNET_SSN_SN { 255, 255, 255, 0} // to do
#endif
// Gateway ip address
#ifndef ETHERNET_SSN_GW
#define ETHERNET_SSN_GW {192,168,1,5} // to do
#endif
// DNS server ip address
#ifndef ETHERNET_DHCP_DNS
#define ETHERNET_DHCP_DNS {192,168,1,5} // to do
#endif
// DNS server ip address
#ifndef ETHERNET_DNS_SERVER
#define ETHERNET_DNS_SERVER "192.168.1.5" // to do
#endif
// HOST NAME
#ifndef ETHERNET_DHCP_NAME
#define ETHERNET_DHCP_NAME "ssnmcu"
#endif
// Local ip address default
#ifndef ETHERNET_SSN_SIP
#define ETHERNET_SSN_SIP {192,168,1,111}
#endif
// Local ip address default
#ifndef ETHERNET_SSN_SERVER_PORT
#define ETHERNET_SSN_SERVER_PORT 6001
#endif


// USE TCP
#ifndef ETHERNET_TCP
#define ETHERNET_TCP
#endif
// USE UDP
#ifndef ETHERNET_UDP
#define ETHERNET_UDP
#endif

#endif /* INC_SSN_PREFS_H_ */
