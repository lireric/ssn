/*
 * processor.h
 *
 *  Created on: 4 июл. 2016 г.
 *      Author: eric
 */

#ifndef INC_PROCESSOR_H_
#define INC_PROCESSOR_H_

#include "FreeRTOS.h"
#include "ssn_prefs.h"

// actions flags
#define devACTION_FLAG_NOLOG		0x01	//	No logging option for this action
#define devACTION_FLAG_SINGLE_TIMER	0x02	//	single pulse timer

// actions execution flags
#define devEXEC_ACTION_FLAG_OUT_ARG	0x01	//	action has Out arguments



// action events operands elements
typedef struct
{
	uint32_t	nElmData1;	// basic element data (device ID, numeric value, date, etc...) or pointer (if nElmType == eElmAction)
	void* 		pNextElm;	// pointer to next element or NULL if end stack
	void* 		pPrevElm;	// pointer to previous element or NULL if begin stack
	uint16_t	nElmData2;	// additional element data (device value index, etc...)
	uint8_t		nElmType;	// element type (ref. eElmTypes)
	uint8_t 	nReserve;		//
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

extern sAction* 		actArray[];
extern uint16_t 		act_counter;
extern void	logAction(uint16_t nActId, uint16_t nDevId, uint8_t nDevCmd, uint32_t nValue);

sAction* 			getActionByID (uint16_t nActId);
int32_t				setAction (uint16_t nActId, char* pAStr, uint32_t arep, uint16_t nFlags);
int32_t				calcAndDoAction (sAction* pAct);


#endif /* INC_PROCESSOR_H_ */
