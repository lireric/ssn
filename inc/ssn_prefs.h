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

//#define CONF_TEST
//#define CONF_RELEASE
//#define CONF_PROD2
#define CONF_PROD1


#ifdef CONF_TEST
	#include "ssn_prefs_test.h"
#endif

#ifdef CONF_RELEASE
	#include "ssn_prefs_release.h"
#endif

#ifdef CONF_TEST2
	#include "ssn_prefs_test_rbt6.h"
#endif

#ifdef CONF_PROD1
	#include "ssn_prefs_prod1.h"
#endif

#ifdef CONF_PROD2
	#include "ssn_prefs_prod2.h"
#endif

/* periodic task rates */
#define mainSensorRateLR			( ( portTickType ) 40000 / portTICK_RATE_MS )
#define mainSensorRateMR			( ( portTickType ) 6000 / portTICK_RATE_MS )
#define mainSensorRateHR			( ( portTickType ) 500 / portTICK_RATE_MS )
#define mainCronRate				( ( portTickType ) 100 / portTICK_RATE_MS )
#define mainDEBUG_STAT_RATE			( ( portTickType ) 10000 / portTICK_RATE_MS )

/* Task priorities. */
#define mainDEBUG_OUT_TASK_PRIORITY			( tskIDLE_PRIORITY)
#define mainINPUT_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define gsmGSM_TASK_START_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainCHECK_SENSOR_MR_TASK_PRIORITY	( tskIDLE_PRIORITY + 1 )
#define mainCHECK_SENSOR_HR_TASK_PRIORITY	( tskIDLE_PRIORITY + 3 )
#define mainPROC_SENSOR_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainECHO_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainBASEOUT_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )

#define  devMAX_MSG_LEN				(127)

/* COM ports */
#define mainCOM0							( 0 )
#define mainCOM1							( 1 )
#define mainCOM2							( 2 )
#define mainCOM3							( 3 )
#define mainCOM4							( 4 )

#define mainBASECOM_Priority				(200)

#define mainMINMEMORYALLOCATE				( 8 ) // minimum memory size for allocate
#define mainACTIONSARRAYTIMEOUT				(30)	// timeout submitting actions log (sec)

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

#ifndef MAX_ACTION_ARRAY_SIZE
#define MAX_ACTION_ARRAY_SIZE	50
#endif

#ifndef mainMAX_ACTIONS
#define mainMAX_ACTIONS					(30) 	// max number of actions
#endif

#ifndef mainINPUT_TASK_STACK
#define mainINPUT_TASK_STACK	800
#endif
#ifndef mainPROCSENSORS_TASK_STACK
#define mainPROCSENSORS_TASK_STACK	400
#endif


#endif /* INC_SSN_PREFS_H_ */
