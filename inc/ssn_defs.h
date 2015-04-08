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

/* Common definitions for SSN project */

#ifndef INC_SSN_DEFS_H_
#define INC_SSN_DEFS_H_

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

#ifndef  mainMAX_MSG_LEN
#define  mainMAX_MSG_LEN			127
#endif

#ifndef  mainMINMEMORYALLOCATE
#define  mainMINMEMORYALLOCATE				( 8 ) // minimum memory size for allocate
#endif

/*---------------------------------------------------------------------------*/
/* -- SSN serial protocol description -----------------------------------------
 *
 * Format: "===ssn1DDDDSSSSTTLLLL...CCCC"
 *
 * ===1 - start packet (protocol version 1)
 * DDDD - destination object (2 byte: hex chars - 0-9, A-F)
 * SSSS - source object (2 byte: hex chars - 0-9, A-F)
 * TT - message type (1 byte: hex chars - 0-9, A-F)
 * LLLL - packet length (2 byte: hex chars - 0-9, A-F)
 * ... data
 * CCCC - CRC16 (2 byte: hex chars - 0-9, A-F)
 *
 * data sending in ascii format
 * timeout = 1 sec
 *
 * */
#define SSN_TIMEOUT 	( 2000 )	// timeout 2000 ms

/* -- SSN serial protocol states -----------------------------------------*/
#define SSN_STATE_INIT		0	// header ready for loading
#define SSN_STATE_LOADING   1	// header is loading now
#define SSN_STATE_DATA      2	// header loaded in buffer, loading data
#define SSN_STATE_READY     3	// data loaded in buffer, ready for parsing
#define SSN_STATE_ERROR     4	// Error in packet loading (CRC error, timeout error)

/* SSN protocol data unit */
typedef struct
{
	uint8_t 	state;			// state of SSN buffer (SSN_STATE_XXX constants)
	uint8_t		obj_dest;		// destination object
	uint8_t		obj_src;		// source object
	uint8_t		message_type;	// message type (mainXXX_MESSAGE constants)
	uint16_t	crc16;			// CRC16 (get from message)
	uint16_t 	counter;		// pointer to current position in buffer
	uint16_t	nDataSize;		// data length
	char 		cSSNBuffer[15];	// buffer for loading SSN protocol header
	char		*buffer;		// pointer to data buffer
	uint32_t	uiLastTick;		// last get data
} sSSNPDU;

/* -- JSON settings -----------------------------------------*/
#define JSON_BUFFER_INCREMENT   500
#define JSON_STATE_INIT			0	// Buffer ready for loading
#define JSON_STATE_LOADING      1	// Buffer in loading now
#define JSON_STATE_READY        2	// JSON loaded in buffer, ready for parsing
#define JSON_STATE_ERROR        3	// Error in buffer loading

/* JSON buffer */
typedef struct
{
	uint8_t 	state;			// state of JSON buffer
	uint16_t 	counter;		// pointer to current position in buffer
	char 		*buffer;
	uint16_t	nBufSize;
} xJSON;



/*-----------------------------------------------------------*/
/* The types of message (xMessageType) that can be sent to the input queue. */
/* COMMAND: the message consist information about some directives to execute */
#define mainCOMMAND_MESSAGE			( 0 )
/* INFO_MESSAGE: the message consist some information  */
#define mainINFO_MESSAGE			( 1 )
/* JSON_MESSAGE: the message consist JSON structure  */
#define mainJSON_MESSAGE			( 2 )
#define mainTELEMETRY_MESSAGE		( 3 )
#define mainGSM_MESSAGE_OUT			( 4 )	// message to web service
#define mainGSM_MESSAGE_IN			( 5 )	// message from web service
#define mainLOG_MESSAGE				( 6 )

/*-----------------------------------------------------------*/
/* The types of interfaces (to/from sources/destinations - xSourceType/xDestType) that can be routed by input queue. */
#define main_IF_PROGRAM				( 0 )	// destination point for automatic routing
#define main_IF_UART1				( 1 )
#define main_IF_UART2				( 2 )
#define main_IF_UART3				( 3 )
#define main_IF_UART4				( 4 )
#define main_IF_UART5				( 5 )
#define main_IF_GSM					( 6 )	// GSM modem
#define main_IF_SCR1				( 7 )	// screen device (LCD, LED etc)
#define main_IF_SCR2				( 8 )	// screen device (LCD, LED etc)
#define main_IF_ETH					( 9 )	// ethernet
#define main_IF_RF					( 10 )	// wireless

/* Type of the message sent to the input queue. */
typedef struct
{
	uint8_t 	version;
	uint8_t 	xMessageType;
	uint16_t	uiDestObject; 	// destination object number, 0 for automatic routing
	uint16_t 	xSourceDevice;
	uint16_t 	xDestDevice;	// xDestDevice = 0 for automatic routing
	uint16_t 	nMsgSize;	 	// size of message
	uint16_t 	nCommand;	 	// for command type message
	void* 		pcMessage;	 	// pointer to message
} xInputMessage;

// routing information
typedef struct
{
	uint16_t	nDestObj;		// destination object
	uint8_t 	xDestType;		// interface type to route
} sRoute;

#define mainCOMMAND_NONE			0
#define mainCOMMAND_REBOOT			1
#define mainCOMMAND_GETDEVVALS		2
#define mainCOMMAND_GETEXTCMD		3	// get command from external systems
#define mainCOMMAND_DISMODEMCHARGE	4	// disable gsm modem charging
#define mainCOMMAND_ENBMODEMCHARGE	5	// enable gsm modem charging
#define mainCOMMAND_GETPREFERENCES	6	// get JSON preferences message
#define mainCOMMAND_UPDATEACTION	7	// update action settings or create new action
#define mainCOMMAND_COMMITED		8


typedef struct
{
	uint32_t	nCmdID;		// command ID (from ext system)
	uint16_t	nCmd;		// command code
	uint16_t	nCmdsLeft;	// count commands in external queue for loading
	char*		pcData;		// command data (if exist)
	uint16_t	uiDevDest;	// destination device
} sSSNCommand;

typedef struct
{
	uint16_t nActId;
	uint16_t nDevId;
	uint32_t nValue;
	uint32_t nTimestamp;
} slogAction;


#endif /* INC_SSN_DEFS_H_ */
