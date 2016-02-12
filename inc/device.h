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
 * Functions for work with device and actions arrays
 * Abstract layer for all devices
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICE_H
#define __DEVICE_H

//#include "DS18B20.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "rtc_func.h"


#ifdef __cplusplus
 extern "C" {
#endif

//#define device_RATE_NONE			( 0 )
//#define device_RATE_HI				( 1 )
//#define device_RATE_MID				( 2 )
//#define device_RATE_LOW				( 3 )

#define  devMAX_MSG_LEN				(127)
#define  devMAX_ACTIONS_CASH		(100)
#define  devMAX_RPN_STACK_SIZE		(50)

 /* Device types
 *
*/
#define device_TYPE_VIRTUAL				( 0 )	// abstract virtual device (e.g. web application, monitoring computer etc)
#define device_TYPE_DS18B20				( 1 )	// DS18B20 temperature sensor
#define device_TYPE_DHT22				( 2 )	// DHT22 humidity and temperature sensor
#define device_TYPE_DS1307				( 3 )	// DS1307 RTC device
#define device_TYPE_BB1BIT_IO_PP		( 4 )	// 1 pin port IO Push-Pull device (digital out)
#define device_TYPE_BB1BIT_IO_OD		( 5 )	// 1 pin port IO Open Drain device
#define device_TYPE_BB1BIT_IO_INPUT		( 6 )	// 1 pin port IO Input device
#define device_TYPE_BB1BIT_IO_AI		( 7 )	// 1 pin port IO Analog Input device
#define device_TYPE_BB1BIT_IO_AO		( 8 )	// 1 pin port IO Analog Output device
#define device_TYPE_EEPROM				( 9 )	// EEPROM i2c device (e.g. at24c32)
#define device_TYPE_GSM					( 10 )	// GSM Modem
#define device_TYPE_RF					( 11 )	// wireless transceiver /* to do */
#define device_TYPE_ETH					( 12 )	// ethernet	/* to do */
#define device_TYPE_MEMORY				( 13 )	// pseudo device - memory store variable	/* to do */

// actions flags
#define devACTION_FLAG_NOLOG		0x01	//	No logging option for this action
#define devACTION_FLAG_SINGLE_TIMER	0x02	//	single pulse timer

// actions execution flags
 #define devEXEC_ACTION_FLAG_OUT_ARG		0x01	//	action has Out arguments

#ifndef NULL
#define NULL ((void*)0)
#endif

// the structure physical describing of the device group
typedef struct
{
	uint32_t 	pPort;		// the port of the device group
	uint8_t 	ucPin;		// the pin of the device group or SDA for soft i2c
	uint8_t 	ucPin2;		// SCL for soft i2c
    uint32_t 	pTimer;		// the timer of the device group
} sGrpDev;

// the structure logical describing of the device group
typedef struct
{
    uint16_t	uiObj;			// device's object
	uint8_t 	uiGroup;		// device group number
//	uint8_t 	ucDevRate;		// group scan rate
	uint8_t 	iDevQty;		// device quantity in group
    sGrpDev 	GrpDev;			//  physical describing of the device group
} sGrpInfo;

typedef struct
{
	uint16_t 	nId;			// unique ID of device
	uint16_t	nDevObj;		// object with this device
	uint8_t 	ucType;			// device type
	uint8_t		nLastPinValue;	// previous device value (only for 1 pin i/o devices)
	uint32_t	nEventTime;		// number of sec/10 from event (setDevEvent)
    sGrpInfo * 	pGroup;			// pointer to the device group
    void * 		pDevStruct;		// pointer to the structure with device description
    void * 		pActionsCash;	// pointer to the actions array linked with this device
	uint8_t 	nActionsCashSize;	// number of actions in array
} sDevice;

// structure for ADC device:
typedef struct{
	int16_t 	nADCValueArray[16]; // array of measured values
	uint8_t 	nChannelArray[16];	// array of used channels
    uint32_t 	uiLastUpdate;		// last update device value
} sADC_data_t;

// action events operands elements
typedef struct
{
	uint32_t	nElmData1;	// basic element data (device ID, numeric value, date, etc...) or pointer (if nElmType == eElmAction)
	void* 		pNextElm;	// pointer to next element or NULL if end stack
	void* 		pPrevElm;	// pointer to previous element or NULL if begin stack
	uint16_t	nElmData2;	// additional element data (device value index, etc...)
	uint8_t		nElmType;	// element type (ref. eElmTypes)
} sEvtElm;


// logic actions
typedef struct
{
	uint16_t 	nActId;			// Action ID
	uint16_t	nFlags;			// different flags according gsmActionFlags___ constants
	uint32_t	nActRepeat;		// interval within it not executing repeat action process
	uint32_t	nLastResult;	// last result of formula calculation
	uint32_t	nLastChange;	// last time of result changes
	sEvtElm*	pStartElmEvents;	// first element in events formula
	sEvtElm*	pStartElmActions;	// first element in actions list
	xTimerHandle* xActTimer;		// action timer (for periodic actions)
	xTimerHandle* xActTimerOneShot;	// action one shot timer
} sAction;

/* Element types */
typedef enum
{
	eElmInteger = 0,	/* integer */
	eElmString,			/* string */
	eElmGetDevValue		// get device value
} eElmDataTypes;

// action elements
typedef struct
{
	uint32_t	nElmDataIn;		// action set device value (if string type - pointer to string)
	uint32_t	nElmDataOut;	// action reset device value
	sAction*	pAction;		// referring Action pointer
	uint8_t		nElmDataType;	// data type (In and Out Data, ref. eElmDataTypes)
	uint8_t		nActCmdIn;		// action set device command
	uint8_t		nActCmdOut;		// action reset device command
	uint8_t		ngetDevCmdIn;	// command for get device value data type
	uint16_t 	nDevId;			// device ID
	uint16_t 	nActFlag;		// action flag: 0x0001 - action has Out argument
} sActElm;


/* Element types */
typedef enum
{
	eElmOperation = 0,	/* operation */
	eElmAction,			/* action (detailed info stored in sActElm structure) */
	eElmConst,			/* numeric constant */
	eElmDevValue,		/* device value */
	eElmTimeValue,		/* current time value */
	eElmDateYear,		/* current year */
	eElmDateMonth,		/* current month */
	eElmDateDay,		/* current day of month */
	eElmDateDayWeek,	/* current day of week */
	eElmInterval		/* interval event */
} eElmTypes;

/* Operations */
typedef enum
{
	eOpResume = 0,	/* = */
	eOpEq,		/* == 	*/
	eOpL,		/* < 	*/
	eOpLE,		/* <= 	*/
	eOpG,		/* > 	*/
	eOpGE,		/* >= 	*/
	eOpAND,		/* &&	*/
	eOpOR,		/* ||	*/
	eOpNOT,		/* ~	*/
	eOpPlus,	/* +	*/
	eOpMinus,	/* -	*/
	eOpMul,		/* *	*/
	eOpDiv,		/* /	*/
	eOpSO,		/* (	*/
	eOpSC		/* )	*/
} eOper;

extern sDevice* 		devArray[];
extern uint16_t 		all_devs_counter;
extern sAction* 		actArray[];
extern uint16_t 		act_counter;
extern sGrpInfo* 		grpArray[];
extern uint8_t 		grp_counter;

extern void 		vMainStartTimer(sAction* pAct);

sGrpInfo* 			getGroupByID (uint8_t nGrpID);
sDevice*			getDeviceByID (uint16_t nDevID);
uint8_t 			getNumDevValCodes(uint8_t ucType);
int32_t 			getDevValueByID(uint8_t nValCode, uint16_t nDevID);
int32_t 			getDevValue(uint8_t nValCode, sDevice* dev);
uint32_t				getDevLastUpdate(uint8_t nValCode, sDevice* dev);
int32_t				getDevData(sDevice* dev, uint8_t nValCode, int32_t* nDevValue, uint32_t* nLastUpdate);

void 				setDevValueByID(int32_t nValue, uint8_t nDevCmd, uint16_t nDevID, uint8_t nDataType);
void 				setDevValue(int32_t nValue, uint8_t nDevCmd, sDevice* dev, uint8_t nDataType);
sAction* 			getActionByID (uint16_t nActId);
int16_t 			getDeviceObjByID (uint16_t nDevID);
int32_t				setAction (uint16_t nActId, char* pAStr, uint32_t arep, uint16_t nFlags);
int32_t				refreshActions2DeviceCash ();
void				clearActionsDeviceCash (sDevice* pDev);
int32_t				calcAndDoAction (sAction* pAct);
int32_t 			adc_setup(sDevice* pDev, char* psChannels);

#ifdef __cplusplus
}
#endif

#endif /*__DEVICE_H */
