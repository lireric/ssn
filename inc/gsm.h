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
 * gsm.h
 *
 *  Created on: 14 окт. 2014 г.
 *      Author: eric
 */

#ifndef GSM_H_
#define GSM_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "STM32_USART.h"
#include "device.h"
#include "cJSON.h"

#define LOWBAUDRATE				57600
#define BAUDRATE				115200
#define HIBAUDRATE				(115200 * 4)
#define gsmMAX_GSM_LEN			400
#define gsmMAX_GSM_RESP_LINES	4

#define gsmMODEM_STATE_INIT		0
#define gsmMODEM_STATE_READY	1
#define gsmMODEM_STATE_REGISTER	2


#define gsmBASE64						// Use BASE64 in POST requests if defined
#define AES128							// use AES128  if defined
#define IV								"ssn2014"	// iV for AES
#define gsmRESEND_TRY					0			// number resent tries
#define gsmINTERNET_INIT_TRY			10			// number of initialization Internet tries
#define gsmINTERNET_INIT_MAX_FAILURES	20		// number of failures in Internet initialization

#define mainGSM_SEND_QUEUE_SIZE			10
#define mainGSM_RECV_QUEUE_SIZE			2
#define mainGSM_UNSOLICITED_QUEUE_SIZE	2
#define gsm_SEND_HTTP_REQUEST_SIZE		12

#define gsmCmdPower			1	// Power on/off and initialize GSM module
#define gsmCmdSendSMS		2	// Send SMS
#define gsmCmdSendLog		3	// Start/Stop sending log message to Log web service
//#define gsmCmdPoolCommand	4	// Pool Control web service for a control command
#define gsmCmdSendTelemetry	5	// Send telemetry data to Telemetry web service
#define gsmCmdSendWSCmd		6	// Send command to web service

// Commands to web service
#define gsmWSCmdGetCommands		1	// get next control command
#define gsmWSCmdCommitCommand	2	// commit executing command

#define gsmGSM_TASK_PRIORITY	( tskIDLE_PRIORITY + 1 )
#define gsmGSMCOM_Priority		(230)

#define ASCII_SIB	0x1a
#define ASCII_CTRLZ	(ASCII_SIB)
#define ASCII_ESC	0x1b

typedef enum
{
	gsmDATA_INIT = 0,	// structure with gsm data not contain any valuable data
	gsmDATA_SENT,		// command sent to modem
	gsmDATA_RECV,		// response is received from modem
	gsmDATA_RECBLCWAIT,	// wait http data block (collect block length data <- from http header +IPD___)
	gsmDATA_HEADER_RECBLC,		// receiving block data from modem
	gsmDATA_HEADER_Ok,	// http header data block received
	gsmDATA_HTTP_BLCWAIT,	// wait http data block (collect block length data <- from http header +IPD___)
	gsmDATA_HTTP_RECBLC,		// receiving block data from modem
	gsmDATA_Ok,			// http data block received
	gsmDATA_ERROR		// set if http errors
} eGsmDataStates;

#define gsmWAIT_DATA_TIMEOUT	( ( portTickType ) 2000 / portTICK_RATE_MS )	// timeout waiting command processing

/* GSM Internet connection statuses */
typedef enum
{
	eGSMInternetInitial = 0,	/* ip initial */
	eGSMInternetIPStart,		/* ip start   */
	eGSMInternetIPConfig,
	eGSMInternetIPGPRSAct,
	eGSMInternetIPStatus,		/* IP STATUS */
	eGSMInternetConnecting,
	eGSMInternetConnectOk,
	eGSMInternetClosing,
	eGSMInternetClosed,
	eGSMInternetPDPDeact
} eGSMInternetStatus;

typedef struct
{
	uint8_t		uiUSART;		// USART number (1,2,3)
	uint32_t	uiPortRTS;		// RTS port
	uint8_t		uiPinRTS;		// RTS pin
	uint32_t	uiPortChgCtrl;	// Chg Ctrl port
	uint8_t		uiPinChgCtrl;	// Chg Ctrl pin
	uint32_t	uiPortPwrKey;	// Pwr Key port
	uint8_t		uiPinPwrKey;	// Pwr Key pin
	uint32_t	uiPortDTR;		// DTR port
	uint8_t		uiPinDTR;		// DTR pin
	uint32_t	uiPortRI;		// RI port
	uint8_t		uiPinRI;		// RI pin
	char 		cAPN[20];			// access point name (internet address)
	char		cSrvAddr[30];		// server address (internet)
	char		cPriDNS[20];		// primary DNS Server address (internet)
	char		cSecDNS[20];		// secondary DNS Server address (internet)
	char		cGPRSUserID[15];	// User ID (internet)
	char		cGPRSUserPassw[10];	// User password (internet)
	char		cAESKey[16];	// AES key for encryption/decryption
	uint16_t	uiSrvPort;		// TCP/IP server port
	char		cSMSNumber[12];	// phone number for sending SMS
	char		chip[10];		// chipset version or manufacturer
	uint16_t		uiSSNAcc;		// Account  number
	uint16_t		uiSSNObject;	// Object  number
	xQueueHandle	pvQueueSend;	// queue for send messages to modem
	xQueueHandle	pvQueueRecv;	// queue for receive messages from modem
	xQueueHandle	pvQueueUnsolicited;	// queue for unsolicited messages from modem
	xQueueHandle	pvQueueDebug;	// Queue for sending debug information
} sGSMDevice;

typedef struct
{
	char 	pcCommand[gsmMAX_GSM_LEN];		// command to gsm modem
	char 	pcResponse[gsmMAX_GSM_RESP_LINES][gsmMAX_GSM_LEN];	// response from gsm modem
	uint8_t uiNumLines;					// number filled lines in response
	uint8_t	uiState;					// state of data (init, sent, resp, etc..)
	uint8_t uiCurrentReadLine;			// current line for reading
	uint16_t uiHttpRespCounter;			// http response block counter
	uint16_t uiHttpRespBufSize;			// size of http buffer
	void*	pHttpRespBuffer;			// http response block buffer
} sGSMData;


#define gsmHTTP_GET			1
#define gsmHTTP_POST		2
#define gsmHTTP_PUT			3
#define gsmHTTP_DELETE		4

typedef struct
{
	uint8_t 	uiNumParts;				// total count parts in request
	uint8_t 	uiCurrentPart;			// total count parts in request
	uint8_t		uiMethod;				// gsmHTTP_xxx defines
	uint8_t		uiTryCnt;				// resend counter
	uint16_t	uiReqSize;				// Size of request data
	uint16_t	uiCommand;				// SSN Command
	uint32_t	uiData1;		// universal 32bit data attribute
	uint16_t	uiData2;		// universal 16bit data attribute
	char 		*pcRequest;				// part of request
} sGSMRequest;

// token for sending crypted data in GET request
typedef struct
{
	uint32_t	uiTimestamp;
	uint16_t	uiSSNAcc;		// Account  number
	uint16_t	uiSSNObject;	// Object  number
	uint16_t	uiCommand;
	uint16_t	uiData2;		// universal 16bit data attribute
	uint32_t	uiData1;		// universal 32bit data attribute
} sToken;

typedef union
{
	uint8_t  	buf[16];
	sToken		token;
} uToken;

//static void print_gsm_debug (const char *str);
void 		deviceProcAttributes_gsm(sDevice* pDev, char* sName, char* sValue);
void		gsm_hw_init_pwrkey();
int32_t 	gsm_hw_init(int boud);
sGSMDevice* gsm_preinit (cJSON *devitem, xQueueHandle xDebugQueue);
int32_t 	gsm_init(sDevice* pDevice);
int32_t 	gsm_send_command(const char *line);
void 		setGSMDevValue(int32_t nValue, uint8_t nDevCmd, sDevice* dev, uint8_t nDataType);
int32_t 	gsm_send_request(const char *line, uint16_t nSize);
char* 		make_token (uint16_t nCmd, uint32_t nData1, uint16_t nData2);
int32_t 	gsm_send_ws_get_cmd (uint16_t nCmd, uint32_t nData1, uint16_t nData2);
void 		gsm_free_recv_buf ();
int32_t 	gsm_send_sms(const char *pStr);
void gsm_preinit_ini (sGSMDevice* pGSMDev, xQueueHandle xDebugQueue);

#endif /* GSM_H_ */
