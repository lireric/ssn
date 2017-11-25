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
 * gsm.c
 *
 *  Created on: 14 окт. 2014 г.
 *      Author: eric
 */

#include "gsm.h"
#include <string.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include "device.h"
#include "bb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "xprintf.h"
#include "utils.h"
#include <math.h>
#include "commands.h"

//#ifdef gsmBASE64
#include "base64.h"
//#endif

#define CBC 1
#define ECB 1

static int32_t gsm_is_sane;
static uint32_t uiTick = 0;
static xTimerHandle xAutoReloadTimer;

//#ifdef AES128
#include "aes.h"
static uint8_t cIV[16] = IV;
//#endif

/*-------------------------------------------------------------------------*/

static char line[gsmMAX_GSM_LEN];
static uint16_t current_char_i;
static sGSMData GSMData;
static sDevice* pDev = 0;
static sGSMDevice* pGSMDev = 0;
static uint8_t nInternetTries = 0;
static void* 	vpDataRecv;
static uint32_t uiDataRecvTimeout;
static uint32_t uiGSMModemState;
static uint32_t uiGSMInternateState;

xQueueHandle xGSMSendQueue;
xQueueHandle xGSMRecvQueue;
xQueueHandle xGSMUnsolicitedQueue;
xQueueHandle xGSMHTTPQueue;

static uint32_t getTimerTick();
static void 	prvGSMSendTask( void *pvParameters );
static void 	prvGSMRespTask( void *pvParameters );
static void 	prvGSMUnsolicitedTask( void *pvParameters );
static void 	prvUSARTGSMRespTask( void *pvParameters );
static void 	prvGSMSendHttpTask( void *pvParameters );
static void 	gsm_wait_init(uint16_t nTimeOut);
static char * 	gsm_read_response(uint16_t nTimeOut);
static char * 	gsm_read_response2(uint16_t nTimeOut, char* pc);
static void 	gsm_clear_data();
static int32_t 	gsm_initiate_connection(void);
static int 		str_starts_with(const char *str, const char *start);
static int32_t 	gsm_internet_init();

static int 		str_starts_with(const char *str, const char *start)
{
	return !strncmp(str, start, strlen(start));
}

static uint32_t	getTimerTick()
{
	return uiTick;
}

void gsm_free_recv_buf ()
{
	if (vpDataRecv) {
		vPortFree(vpDataRecv);
	}
	vpDataRecv = 0;
}

void gsm_delay_ms(uint16_t uiDelay)
{
	// 1 tick = 100 ms
	uint32_t uiCntEnd = getTimerTick() + uiDelay/100;
	for (; getTimerTick() < uiCntEnd;) {}
}

void print_gsm_debug (char *str)
{
	xprintfMsg(str);
}

char* make_token (uint16_t nCmd, uint32_t nData1, uint16_t nData2)
{
	char* pTocken = 0;
	uint8_t	buffer[16];
	uToken tokenData;

	tokenData.token.uiTimestamp = rtc_get_counter_val();
	tokenData.token.uiSSNAcc = pGSMDev->uiSSNAcc;
	tokenData.token.uiSSNObject = getMC_Object();
	tokenData.token.uiCommand = nCmd;
	tokenData.token.uiData1 = nData1;
	tokenData.token.uiData2 = nData2;
	AES128_ECB_encrypt(tokenData.buf, (uint8_t*)pGSMDev->cAESKey, buffer);
	pTocken = pvPortMalloc(16*1.5); // allocate more memory for base64
	base64_encode(buffer, 16, pTocken);

	return pTocken;
}

static void vAutoReloadTimerFunction(xTimerHandle xTimer) {
	uiTick++;
}

static void gsm_clear_data()
{
	GSMData.uiState = gsmDATA_INIT;
}

static void gsm_wait_init(uint16_t nTimeOut)
{
	uint32_t uiTimeoutEnd = getTimerTick() + nTimeOut/100;
	while ((GSMData.uiState != gsmDATA_INIT) && (GSMData.uiState != gsmDATA_ERROR) && (getTimerTick() < uiTimeoutEnd));
}

static void gsm_wait_register(uint16_t nTimeOut)
{
	uint32_t uiTimeoutEnd = getTimerTick() + nTimeOut/100;
	while ((uiGSMModemState != gsmMODEM_STATE_REGISTER) && (getTimerTick() < uiTimeoutEnd));
}

/* wait response string */
static char * gsm_read_response(uint16_t nTimeOut)
{
	uint32_t uiTimeoutEnd = getTimerTick() + nTimeOut/100;
	char *p = 0;

	while (((GSMData.uiState != gsmDATA_RECV) || (GSMData.uiCurrentReadLine == GSMData.uiNumLines) ) && (getTimerTick() < uiTimeoutEnd)) {}

	if ((GSMData.uiState == gsmDATA_RECV) && (GSMData.uiCurrentReadLine < GSMData.uiNumLines))
		{
			p = GSMData.pcResponse[GSMData.uiCurrentReadLine++];
		}
	return p;
}

// Search response with status. Return pointer for response string if success or NULL if fail.
static char * gsm_read_response2(uint16_t nTimeOut, char* pc)
{
	char* xReturn = NULL;
	uint32_t uiTimeoutEnd = getTimerTick() + nTimeOut/100;
	uiDataRecvTimeout = uiTimeoutEnd;
	char *p = 0;

	while (!xReturn && (getTimerTick() < uiTimeoutEnd))
	{
		p = gsm_read_response(nTimeOut);
		if(p && strstr(p,pc)) {
			xReturn = p;
			break;
		}
	}
	return xReturn;
}

void gsm_hw_init_pwrkey()
{
	// turn on device
	/* PWRKEY must be open-drain, and must be asserted for 1.5 seconds. */
	/* CHG_CTRL should be disabled (0 volts) when pwrkey is active */
	gpio_set_mode(pGSMDev->uiPortChgCtrl, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGSMDev->uiPinChgCtrl);
	gpio_set_mode(pGSMDev->uiPortPwrKey, GPIO_MODE_OUTPUT_2_MHZ,
				GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGSMDev->uiPinPwrKey);

	uiGSMModemState = gsmMODEM_STATE_INIT;
	uiGSMInternateState = eGSMInternetInitial;

	if (strcmp(pGSMDev->chip,"SIM300") == 0) {
		// initialisation SIM300 modem
		// disable charging of battery
		print_gsm_debug("\r\nInit SIM300 modem. \r\ndisable charging of battery...");
		gpio_clear(pGSMDev->uiPortChgCtrl, 1<<pGSMDev->uiPortChgCtrl);
		gsm_delay_ms(1000);

		// press and hold the PWRKEY
		print_gsm_debug("\r\npress and hold the PWRKEY...");
		gpio_clear(pGSMDev->uiPortPwrKey, 1<<pGSMDev->uiPinPwrKey);
		gsm_delay_ms(2500);
		gpio_set(pGSMDev->uiPortPwrKey, 1<<pGSMDev->uiPinPwrKey);
		gsm_delay_ms(1000);

		// restore charging of battery
		print_gsm_debug("\r\nrestore charging of battery...");
		gpio_set(pGSMDev->uiPortChgCtrl, 1<<pGSMDev->uiPortChgCtrl);

		gsm_delay_ms(6500);
	} else {
		print_gsm_debug("\r\nInit SIM900 or same modem...");
		gpio_clear(pGSMDev->uiPortPwrKey, 1<<pGSMDev->uiPinPwrKey);	// pulse reset pin of modem module
		gsm_delay_ms(200);
		gpio_set(pGSMDev->uiPortPwrKey, 1<<pGSMDev->uiPinPwrKey);
		gpio_set(pGSMDev->uiPortChgCtrl, 1<<pGSMDev->uiPortChgCtrl);
		gsm_delay_ms(6000);
	}
	print_gsm_debug("\r\nModem turned on.");


}


int32_t gsm_hw_init(int boud)
{
	int32_t result = pdFAIL;
	char msg[60];

// DTR must be set low
	gpio_set_mode(pGSMDev->uiPortDTR, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, 1 << pGSMDev->uiPinDTR);
	gpio_clear(pGSMDev->uiPortDTR, 1<<pGSMDev->uiPinDTR);

// Initialize RTS, RI, DTR
	if (pGSMDev->uiPortRTS) {
		gpio_set_mode(pGSMDev->uiPortRTS, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGSMDev->uiPinRTS);
		gpio_clear(pGSMDev->uiPortRTS, 1<<pGSMDev->uiPinRTS);
	}
	if (pGSMDev->uiPortRI) {
		gpio_set_mode(pGSMDev->uiPortRI, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGSMDev->uiPinRI);
		gpio_clear(pGSMDev->uiPortRI, 1<<pGSMDev->uiPinRI);
	}
	if (pGSMDev->uiPortDTR) {
		gpio_set_mode(pGSMDev->uiPortDTR, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGSMDev->uiPinDTR);
		gpio_clear(pGSMDev->uiPortDTR, 1<<pGSMDev->uiPinDTR);
	}

	result = lCOMPortInit( (unsigned long) pGSMDev->uiUSART - 1, (unsigned long) boud, gsmGSMCOM_Priority);
	xsprintf(msg, "\n\rBoud:%d, init modem port=%d", boud, result);
	print_gsm_debug(msg);

	return result;
}

void gsm_preinit_ini (sGSMDevice* pGSMDev, xQueueHandle xDebugQueue)
{
	xGSMSendQueue = xQueueCreate( mainGSM_SEND_QUEUE_SIZE, gsmMAX_GSM_LEN);
	xGSMRecvQueue = xQueueCreate( mainGSM_RECV_QUEUE_SIZE, gsmMAX_GSM_LEN);
	xGSMUnsolicitedQueue = xQueueCreate( mainGSM_UNSOLICITED_QUEUE_SIZE, gsmMAX_GSM_LEN);
	xGSMHTTPQueue = xQueueCreate( gsm_SEND_HTTP_REQUEST_SIZE, sizeof(sGSMRequest));

	pGSMDev->pvQueueSend = xGSMSendQueue;
	pGSMDev->pvQueueRecv = xGSMRecvQueue;
	pGSMDev->pvQueueUnsolicited = xGSMUnsolicitedQueue;
	pGSMDev->pvQueueDebug = xDebugQueue;

}

sGSMDevice* gsm_preinit (cJSON *devitem, xQueueHandle xDebugQueue)
{
	char* pc;

	sGSMDevice* pGSMDev = pvPortMalloc(sizeof(sGSMDevice));
	pGSMDev->uiPortDTR = get_port_by_name((char*) cJSON_GetObjectItem(devitem, "PortDTR")->valuestring);
	pGSMDev->uiPinDTR = (uint8_t) cJSON_GetObjectItem(devitem, "PinDTR")->valueint;
	pGSMDev->uiPortPwrKey = get_port_by_name((char*) cJSON_GetObjectItem(devitem, "PortPwrKey")->valuestring);
	pGSMDev->uiPinPwrKey = (uint8_t) cJSON_GetObjectItem(devitem, "PinPwrKey")->valueint;
	pGSMDev->uiPortChgCtrl = get_port_by_name((char*) cJSON_GetObjectItem(devitem, "PortChgCtrl")->valuestring);
	pGSMDev->uiPinChgCtrl = (uint8_t) cJSON_GetObjectItem(devitem, "PinChgCtrl")->valueint;
	pGSMDev->uiPortRTS = get_port_by_name((char*) cJSON_GetObjectItem(devitem, "PortRTS")->valuestring);
	pGSMDev->uiPinRTS = (uint8_t) cJSON_GetObjectItem(devitem, "PinRTS")->valueint;
	pGSMDev->uiUSART = (uint8_t) cJSON_GetObjectItem(devitem, "USART")->valueint;
	pGSMDev->uiSSNAcc = (uint16_t) cJSON_GetObjectItem(devitem, "acc")->valueint;

//	pGSMDev->uiSSNObject = (uint16_t) cJSON_GetObjectItem(devitem, "obj")->valueint;
	pc = cJSON_GetObjectItem(devitem, "v")->valuestring;	// modem chipset version or manufacturer
	memcpy(&pGSMDev->chip,pc,strlen(pc));
	pc = cJSON_GetObjectItem(devitem, "APN")->valuestring;
	memcpy(&pGSMDev->cAPN,pc,strlen(pc));
	pc = cJSON_GetObjectItem(devitem, "SrvAddr")->valuestring;
	memcpy(&pGSMDev->cSrvAddr,pc,strlen(pc));
	pGSMDev->uiSrvPort = (uint16_t) cJSON_GetObjectItem(devitem, "SrvPort")->valueint;
	pc = cJSON_GetObjectItem(devitem, "SMSNumber")->valuestring;
	memcpy(&pGSMDev->cSMSNumber,pc,strlen(pc));
	pc = cJSON_GetObjectItem(devitem, "PriDNS")->valuestring;
	memcpy(&pGSMDev->cPriDNS,pc,strlen(pc));
	pc = cJSON_GetObjectItem(devitem, "SecDNS")->valuestring;
	memcpy(&pGSMDev->cSecDNS,pc,strlen(pc));
	pc = cJSON_GetObjectItem(devitem, "GUser")->valuestring;
	memcpy(&pGSMDev->cGPRSUserID,pc,strlen(pc));
	pc = cJSON_GetObjectItem(devitem, "GUserPswd")->valuestring;
	memcpy(&pGSMDev->cGPRSUserPassw,pc,strlen(pc));
	pc = cJSON_GetObjectItem(devitem, "AESKey")->valuestring;
	memcpy(&pGSMDev->cAESKey,pc,strlen(pc));

	xGSMSendQueue = xQueueCreate( mainGSM_SEND_QUEUE_SIZE, gsmMAX_GSM_LEN);
	xGSMRecvQueue = xQueueCreate( mainGSM_RECV_QUEUE_SIZE, gsmMAX_GSM_LEN);
	xGSMUnsolicitedQueue = xQueueCreate( mainGSM_UNSOLICITED_QUEUE_SIZE, gsmMAX_GSM_LEN);
	xGSMHTTPQueue = xQueueCreate( gsm_SEND_HTTP_REQUEST_SIZE, sizeof(sGSMRequest));

	pGSMDev->pvQueueSend = xGSMSendQueue;
	pGSMDev->pvQueueRecv = xGSMRecvQueue;
	pGSMDev->pvQueueUnsolicited = xGSMUnsolicitedQueue;
	pGSMDev->pvQueueDebug = xDebugQueue;

	return pGSMDev;
}

int32_t gsm_init(sDevice* pDevice)
{
	char *resp;
	int32_t result = pdFAIL;
	xTaskHandle pTmpTask;
	uiGSMInternateState = eGSMInternetInitial;

	pDev = pDevice;
	pGSMDev = (sGSMDevice*) pDev->pDevStruct;
	xAutoReloadTimer = xTimerCreate("AutoReloadTimer", 100 / portTICK_RATE_MS, pdTRUE, 0, vAutoReloadTimerFunction);
	xTimerStart(xAutoReloadTimer, 0);


	print_gsm_debug("\r\nTurning on modem...");

	// init hardware
	if (strcmp(pGSMDev->chip,"SIM300") == 0) {
		result = gsm_hw_init(BAUDRATE);
	} else {
		result = gsm_hw_init(LOWBAUDRATE);
	}

	if (!result) return pdFALSE;

	result = xTaskCreate( prvUSARTGSMRespTask, ( char * ) "USARTGSMRespTask", 250, pDev, gsmGSM_TASK_PRIORITY, &pTmpTask );
	if (result != pdPASS)
	{
		print_gsm_debug("\r\ntUSARTGSMRespTask - create failed!");
		return result;
	}
	result = xTaskCreate( prvGSMSendTask, ( char * ) "GSMSendTask", 500, pDev, gsmGSM_TASK_PRIORITY+1, &pTmpTask );

	if (result != pdPASS)
	{
		print_gsm_debug("\r\nGSMSendTask - create failed!");
		return result;
	}
	result = xTaskCreate( prvGSMRespTask, ( char * ) "GSMRespTask", 400, pDev, gsmGSM_TASK_PRIORITY+1, &pTmpTask );

	if (result != pdPASS)
	{
		print_gsm_debug("\r\nGSMRespTask - create failed!");
		return result;
	}
	result = xTaskCreate( prvGSMUnsolicitedTask, ( char * ) "GSMUnsolicitedTask", 400, pDev, gsmGSM_TASK_PRIORITY, &pTmpTask );

	if (result != pdPASS)
	{
		print_gsm_debug("\r\nGSMUnsolicitedTask - create failed!");
		return result;
	}
	result = xTaskCreate( prvGSMSendHttpTask, ( char * ) "GSMSendHttpTask", 850, pDev, gsmGSM_TASK_PRIORITY, &pTmpTask );

	if (result != pdPASS)
	{
		print_gsm_debug("\r\nGSMSendHttpTask - create failed!");
		return result;
	}

	gsm_hw_init_pwrkey();       // turn on device

	gsm_is_sane = gsm_initiate_connection();

	if (!gsm_is_sane) {
		// try once more

		if (strcmp(pGSMDev->chip,"SIM300") == 0) {
			result = gsm_hw_init(BAUDRATE);
		} else {
			result = gsm_hw_init(BAUDRATE);
		}
		if (!result) return pdFALSE;

		if (uiGSMModemState != gsmMODEM_STATE_READY) {
			gsm_is_sane = gsm_initiate_connection();
		}
	}
	// try to communicate at higher baudrate
	if(!gsm_is_sane && (strcmp(pGSMDev->chip,"SIM300") == 0)) {
		print_gsm_debug("\r\ntry to communicate at higher baudrate");
		gsm_hw_init_pwrkey();       // turn on device
		gsm_hw_init(115200 * 4);
		gsm_is_sane = gsm_initiate_connection();

		// if working at higher baud then reduce it!
		if(gsm_is_sane) {
			gsm_send_command("AT+IPR=115200\r\n");
			resp = gsm_read_response(500);
			print_gsm_debug(resp);
			if( resp && !strcmp(resp, "OK")) {
				// switch to 115200 was sucessful, change UART to new baud as well
				gsm_hw_init(115200);
				gsm_delay_ms(1000);
				gsm_is_sane = gsm_initiate_connection();

				// now save configuration to non volatile memory
				if(gsm_is_sane) {
					print_gsm_debug("save configuration to non volatile memory");
					gsm_send_command("AT&W");
					resp = gsm_read_response(1500);
					if( resp && !strcmp(resp, "OK") ) {
						// stored sucessfully
						print_gsm_debug("stored sucessfully. BAUD=115200bps");
						gsm_delay_ms(1500);
						return pdTRUE;
					}
				}
			}

			print_gsm_debug("ERROR CHANGING BAUDRATE");
			gsm_is_sane = 0;
			gsm_delay_ms(1500);
		}
	}

	// prepare first internet connection
	result = gsm_internet_init();

	return result;
}

static int32_t gsm_initiate_connection(void)
{
	char *p;
	int8_t ntry;
	int32_t stat = pdFALSE;

	/* just in case hardware is in text input mode */
	//gsm_hw_send_byte(ASCII_ESC);

	print_gsm_debug("\r\ntry to init modem");

	if (uiGSMModemState == gsmMODEM_STATE_READY)
		stat = pdTRUE;
// if modem send URC "RDY", then skip check and config
	if (!stat) {
		for(ntry=3; ntry>=0; ntry--) {
			gsm_clear_data();
			gsm_send_command("AT\r\n");
			p = gsm_read_response(500);
			if(!p)
				continue;
			if(!strncmp(p,"AT",2)) {		/* check for echo */
				p = gsm_read_response(100);	// read next response line
				/* disable echo */
				gsm_send_command("ATE0\r\n");
				p = gsm_read_response(500);
				gsm_clear_data();
				if(!p)
					continue;
			}
			if(!strncmp(p,"OK",2)) {
				uiGSMModemState = gsmMODEM_STATE_READY;
				gsm_clear_data();
				break;
			} else {
				gsm_clear_data();
			}
		}
		if(ntry < 0) {
			stat = pdFAIL;
		} else {
			stat = pdTRUE;
		}

		// to do - process responses...
		if (stat) {
			gsm_clear_data();
			gsm_send_command("AT+IPR=57600\r");
			p = gsm_read_response(200);

			gsm_clear_data();
			gsm_send_command("AT+CLIP=1\r");
			p = gsm_read_response(500);
			gsm_clear_data();
			gsm_send_command("AT+CSCS=\"GSM\"\r");
			p = gsm_read_response(500);
			gsm_clear_data();
			gsm_send_command("AT+CMGF=1\r");
			p = gsm_read_response(500);
			gsm_send_command("AT&W\r");
			p = gsm_read_response(1500);

		}
	}

	gsm_send_command("AT+CREG=1\r");
	gsm_wait_register(5000);

	return stat;
}

/*-------------------------------------------------------------------------*/
/* function return pdTRUE if internet connection ready for data transmitting
 *
 */
static int32_t gsm_internet_init()
{
	int32_t ret = pdFALSE;
	char buf[gsmMAX_GSM_LEN];
	char *p;
	char *p2;
	uint8_t	counter = 0;
	(void)p;

	// try to make Internet connection. we'll use some tries..
	for (counter=0; ((counter < gsmINTERNET_INIT_TRY) && !ret); counter++)
	{
	// Get Connection Status, needs to be 'IP STATUS' before we can connect to a server
		gsm_delay_ms(200);
		gsm_clear_data();
		gsm_send_command("AT+CIPSTATUS\r");
		p = gsm_read_response2(3000,"STATE:");

		// if modem not response or big number failed tries, then restart modem
		if (!p)
		{
			if (nInternetTries > gsmINTERNET_INIT_MAX_FAILURES)
			{
				nInternetTries = 0;
				gsm_hw_init_pwrkey();
			} else
				nInternetTries++;
		}

		if(p && (strncmp(p,"STATE: IP STATUS",16)==0)) {
			uiGSMInternateState = eGSMInternetIPStatus;
			nInternetTries = 0;		// reset global counter of connection attempts
			ret = pdTRUE;			// ready to make connection, return
			break;
		} else
		{
			if(p && (strcmp(p,"STATE: IP INITIAL")==0))
			{
				uiGSMInternateState = eGSMInternetInitial;
				nInternetTries = 0;
				//	Attach to GPRS Service
				//  Check current connection:
				gsm_clear_data();
				gsm_send_command("AT+CGATT?\r");
				p2 = gsm_read_response(2000);

				// if response like "+CGATT: 1,1" than we already have gprs/edge connection, else - make new:
				if(!(p2 && ((strncmp(p2,"+CGATT:",7)==0) && strrchr(p2,'1'))))
				{
					nInternetTries++;
					gsm_delay_ms(1000);
					gsm_clear_data();
					gsm_send_command("AT+CGATT=1\r");
					if (!gsm_read_response2(4000,"OK"))
							continue;
				}

				//	Define PDP Context (cid, PDP type, APN)
				gsm_clear_data();
				xsprintf(buf,"AT+CGDCONT=1,\"IP\",\"%s\"\r", pGSMDev->cAPN);
				gsm_send_command(buf);
				if (!gsm_read_response2(2500,"OK"))
						continue;

				//	Configure Domain Name Server (primary DNS, secondary DNS)
				gsm_clear_data();
				xsprintf(buf,"AT+CDNSCFG=\"%s\",\"%s\"\r", pGSMDev->cPriDNS, pGSMDev->cSecDNS);
				gsm_send_command(buf);
				if (!gsm_read_response2(2500,"OK"))
						continue;
				gsm_delay_ms(1000);

				// use manual get data from network
//				gsm_clear_data();
//				gsm_send_command("AT+CIPRXGET=1\r");
//				if (!gsm_read_response2(500,"OK"))
//						continue;

				gsm_clear_data();
				gsm_send_command("AT+CIPSRIP=1\r");
				if (!gsm_read_response2(500,"OK"))
						continue;
				//	Start Task & set APN, User ID, and password
//inetInitStartTask:
				gsm_clear_data();
				xsprintf(buf,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r", pGSMDev->cAPN, pGSMDev->cGPRSUserID, pGSMDev->cGPRSUserPassw);
				gsm_send_command(buf);
				if (!gsm_read_response2(2500,"OK"))
						continue;
				// here we are in "IP START" status
				uiGSMInternateState = eGSMInternetIPStart;
				goto inetInitSTATE_IPStart;	// skip get status for time reasons
			}

			if(p && (strcmp(p,"STATE: IP GPRSACT")==0) )
			{
				uiGSMInternateState = eGSMInternetIPGPRSAct;
				goto inetInitSTATE_IPStart;
			}

			if(p && (strcmp(p,"STATE: IP START")==0))
			{
inetInitSTATE_IPStart:
				nInternetTries = 0;
				//	Bring up wireless connection with GPRS - THIS MAY TAKE A WHILE
				gsm_clear_data();
				gsm_send_command("AT+CIICR\r");
				if (!gsm_read_response2(6000,"OK"))
						continue;
				gsm_delay_ms(1000);
				// here we are in "IP CONFIG" status
				uiGSMInternateState = eGSMInternetIPConfig;
				//	Get Local IP address
				gsm_clear_data();
				gsm_send_command("AT+CIFSR\r\n");
				if (!gsm_read_response2(500,"OK"))
						continue;
				// here we are in "IP STATUS" status
				uiGSMInternateState = eGSMInternetIPStatus;
				ret =  pdTRUE;
				break;
			}
			if(p && (strcmp(p,"STATE: CONNECT OK")==0) )
			{
				// Internet connection ready for data exchange
				uiGSMInternateState = eGSMInternetConnectOk;
				ret =  pdTRUE;
				break;
			}

			if(p && (strcmp(p,"STATE: TCP CONNECTING")==0) )
			{
				// if status connecting now, then break it and try again
				uiGSMInternateState = eGSMInternetConnecting;
				nInternetTries = 0;
				gsm_clear_data();
				gsm_send_command("AT+CIPCLOSE\r\n");
				if (gsm_read_response2(3000,"CLOSE OK"))
				{
					// process connection closing begin, little wait and try make new
					uiGSMInternateState = eGSMInternetClosing;
					gsm_delay_ms(1000);
					uiGSMInternateState = eGSMInternetClosed;
						continue;
				} else {
					nInternetTries++;
					// to do: process error connection closing
				}
			}

			if(p && (strcmp(p,"STATE: PDP DEACT")==0) )
			{
				uiGSMInternateState = eGSMInternetPDPDeact;
				goto inetInitShutdown;
			}

			if(p && (strcmp(p,"STATE: TCP CLOSED")==0) )
			{
				uiGSMInternateState = eGSMInternetClosed;
inetInitShutdown:
				nInternetTries++;	// increment global conter
				gsm_clear_data();
				gsm_send_command("AT+CIPSHUT\r\n");
				if (!gsm_read_response2(2000,"OK"))
						continue;
			}
		}
	} // for

	return ret;
}

int32_t gsm_send_sms(const char *pStr)
{
	int32_t xReturn = pdFALSE;
	char buf[gsmMAX_GSM_LEN];

	if (uiGSMModemState == gsmMODEM_STATE_REGISTER){
		gsm_clear_data();
		xsprintf(buf, "AT+CMGS=\"%s\"\r%s%c", pGSMDev->cSMSNumber, pStr, ASCII_CTRLZ);
		gsm_send_command(buf);

		if(!gsm_read_response2(10000,"OK")) {
			xReturn = pdTRUE;
		}
	}

	return xReturn;
}

int32_t gsm_send_command(const char *line)
{
	int32_t xReturn = xQueueSend( pGSMDev->pvQueueSend, line, 0 );

	return xReturn;
}

int32_t gsm_send_ws_get_cmd (uint16_t nCmd, uint32_t nData1, uint16_t nData2)
{
	int32_t xReturn;
	char msg[20];

	sGSMRequest	xGSMRequest;
	xGSMRequest.uiNumParts = 0;	// 0 is 1 part of request
	xGSMRequest.uiCurrentPart = 0;
	xGSMRequest.uiMethod = gsmHTTP_GET;
	xGSMRequest.uiReqSize = 0;
	xGSMRequest.pcRequest = (void*) 0;
	xGSMRequest.uiCommand = nCmd;
	xGSMRequest.uiData1 = nData1;
	xGSMRequest.uiData2 = nData2;
	xGSMRequest.uiTryCnt = 0;

	xReturn = xQueueSend(xGSMHTTPQueue, &xGSMRequest, 0 );

	xsprintf(msg, "\n\rGSMHTTPQueue:%d", uxQueueMessagesWaiting(xGSMHTTPQueue));
	print_gsm_debug(msg);

	return xReturn;
}


/*	Send message to queue for prepare tcp/ip request
//  if needed, message split into small structures
*/
int32_t gsm_send_request(const char *line, uint16_t nSize)
{
	sGSMRequest	xGSMRequest;
	uint8_t	nCnt;
	uint16_t nLineSize = nSize;

	uint16_t uiBufLen;
	uint8_t	nParts = ceil(nLineSize/gsmMAX_GSM_LEN);
	int32_t xReturn = 0;
	char msg[35];
	// check free space in queue for all parts
	if ((gsm_SEND_HTTP_REQUEST_SIZE - uxQueueMessagesWaiting(xGSMHTTPQueue)) >= nParts)
	{
	// all parts of message must have sequential order -> disable interrupts for message preparing:
	taskENTER_CRITICAL();
	for (nCnt = 0; nCnt <= nParts; nCnt++) {
		xGSMRequest.pcRequest = (void*) pvPortMalloc(gsmMAX_GSM_LEN);
		if (xGSMRequest.pcRequest) {
			xGSMRequest.uiNumParts = nParts;
			xGSMRequest.uiCurrentPart = nCnt;
			xGSMRequest.uiMethod = gsmHTTP_POST;
			xGSMRequest.uiReqSize = nLineSize;
			xGSMRequest.uiTryCnt = 0;
			uiBufLen = nLineSize - nCnt * (gsmMAX_GSM_LEN - 1);
			if (uiBufLen > gsmMAX_GSM_LEN - 1) {
				uiBufLen = gsmMAX_GSM_LEN - 1;
			}
			memcpy(xGSMRequest.pcRequest, &line[nCnt * (gsmMAX_GSM_LEN - 1)],
					uiBufLen);
			xGSMRequest.pcRequest[uiBufLen] = 0;
			xReturn = xQueueSend(xGSMHTTPQueue, &xGSMRequest, 0);
			xsprintf(msg, "\n\rGSMHTTPQueue:%d, queue send = %d", uxQueueMessagesWaiting(xGSMHTTPQueue), xReturn);
			print_gsm_debug(msg);
			if (!xReturn) {
				// if sent to queue failed free memory now
				vPortFree(xGSMRequest.pcRequest);
			}

		} else {
			xReturn = pdFALSE;
		}
	}
	taskEXIT_CRITICAL();
	}

	return xReturn;
}


/*-----------------------------------------------------------*/

static void prvGSMSendHttpTask( void *pvParameters )
{
	sGSMRequest	xGSMRequest;
	sDevice* pDev = (sDevice*) pvParameters;
	sGSMDevice* pGSMDev = (sGSMDevice*) pDev->pDevStruct;
	uint32_t xReturn;
//	char *p;
	char msg[100];
//	uint8_t nRetriesCount=0;
	uint8_t nAES128 = 0;
	#ifdef AES128
	nAES128 = 1;
	#endif

	char buf[gsmMAX_GSM_LEN];

	while (1) {
		/* Wait for a message from GSM_HTTP send queue */
		while( xQueueReceive(xGSMHTTPQueue, &xGSMRequest, portMAX_DELAY ) != pdPASS );
//		while( xQueuePeek(xGSMHTTPQueue, &xGSMRequest, portMAX_DELAY ) != pdPASS );
		xReturn = pdTRUE;
		// check internet connection and make it if necessary. It's making only for first packet
		if (xGSMRequest.uiCurrentPart == 0) {
			xReturn = gsm_internet_init();
			// if internet connection can't be established skip all packet
			if (xReturn) {
				// if state already Connect, then go to send data:
				if (uiGSMInternateState == eGSMInternetConnectOk)
					goto httpTaskReadyToSend;

				//	Tells module to add an 'IP Header' to receive data
						gsm_clear_data();
						gsm_send_command("AT+CIPHEAD=1\r\n");

						if (!gsm_read_response2(2000,"OK")) {xReturn = pdFALSE; }
						else {
							gsm_clear_data();
							if (strcmp(pGSMDev->chip,"SIM300") == 0) {
								// Indicates whether connection request will be IP address (0), or domain name (1)
								gsm_send_command("AT+CDNSORIP=1\r\n");
							} else {
								// get IP address of server
								xsprintf(buf,"AT+CDNSGIP=\"%s\"\r\n",pGSMDev->cSrvAddr);
								gsm_send_command(buf);
							}

							if (!gsm_read_response2(5000,"OK")) {
								xReturn = pdFALSE;
							}
							else {
				// Start up TCP connection (mode, IP address/name, port)
								gsm_clear_data();
								xsprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r", pGSMDev->cSrvAddr, pGSMDev->uiSrvPort);
								gsm_send_command(buf);
									if(!gsm_read_response2(20000,"CONNECT OK")) {
										xReturn = pdFALSE;
									} else
									{
				// << CONNECT OK      - Indicates we've connected to the server - IT MAKE TAKE A WHILE FOR THIS TO BE RETURNED
				// Issue Send Command
				// wait for module to return '>' to indicate it's ready to receive data
httpTaskReadyToSend:
									gsm_clear_data();
									gsm_send_command("AT+CIPSEND\r");

									if(!gsm_read_response2(40000,">"))
									{
										print_gsm_debug("\r\nERROR in CIPSEND");
										// to do: process error
									} else {

									uint8_t nBase64 = 0;
									#ifdef gsmBASE64
										nBase64 = 1;
									#endif
											gsm_clear_data();
											if (xGSMRequest.uiMethod == gsmHTTP_POST) {
												xsprintf(buf, "POST /ssn/index.php HTTP/1.1\r\nHost: %s\r\nUser-Agent: ssn\r\nContent-Type: application/octet-stream\r\nConnection: keep-alive\r\nKeep-Alive: 180\r\nssn-acc: %d\r\nssn-obj: %d\r\nssn-base64: %d\r\nssn-aes128: %d\r\nContent-Length: %d\r\n\r\n%c", pGSMDev->cSrvAddr, pGSMDev->uiSSNAcc, getMC_Object(), nBase64, nAES128, xGSMRequest.uiReqSize, ASCII_CTRLZ);
											} else
											if (xGSMRequest.uiMethod == gsmHTTP_GET)
											{
												char* pt = make_token(xGSMRequest.uiCommand,xGSMRequest.uiData1,xGSMRequest.uiData2);
												xsprintf(buf, "GET /ssn/index.php?t=%s%c",pt, ASCII_CTRLZ);
												vPortFree(pt);
											}
											gsm_send_command(buf);
											if(gsm_read_response2(40000,"SEND OK"))
											{
												xReturn = pdTRUE;
											} else
											{
												xReturn = pdFALSE;
											}
									} // CIPSEND OK
								} // CONNECT OK
							}
						} // CIPHEAD OK
			} // internet connection OK
		} // first packet
		else {
			// for others packets in request status must be CONNECT OK
			// Get Connection Status, needs to be 'IP STATUS' before we can connect to a server
						gsm_clear_data();
						gsm_send_command("AT+CIPSTATUS\r");
						if (gsm_read_response2(500,"STATE: CONNECT OK"))
						{
							xReturn = pdTRUE;
						} else {
						// delete this message from queue - not needed to resend - previous part of message is lost
						//	xReturn = xQueueReceive(xGSMHTTPQueue, &xGSMRequest, 0);
							xReturn = pdFALSE;
						}
		}

// for next packets simply try to send
		if (xReturn)
		{
// Issue Send Command with request data
// wait for module to return '>' to indicate it's ready to receive data
			gsm_clear_data();
			if (xGSMRequest.pcRequest) {
				xsprintf(buf, "AT+CIPSEND=%d\r", strlen(xGSMRequest.pcRequest));
				gsm_send_command(buf);
				if(!gsm_read_response2(40000,">"))
				{
					print_gsm_debug("\r\nERROR in CIPSEND");
					// to do: process error
				}
				gsm_send_command(xGSMRequest.pcRequest);
				char* cResp;
//				if ((xGSMRequest.uiCurrentPart == xGSMRequest.uiNumParts) && (xGSMRequest.uiMethod == gsmHTTP_POST)) {
//					cResp = "CLOSED";
//				} else {
					cResp = "SEND OK";
//				}

				if (gsm_read_response2(40000,cResp))
					xReturn = pdTRUE;
				else
					xReturn = pdFALSE;
			} else {
				xReturn = pdTRUE;
			}

			// if this is last part of request, then also sent finish part of GET request
			if (xGSMRequest.uiCurrentPart == xGSMRequest.uiNumParts) {
				if (xGSMRequest.uiMethod == gsmHTTP_GET)
				{
					gsm_clear_data();
					xsprintf(buf, "AT+CIPSEND\r");
					gsm_send_command(buf);
					if(!gsm_read_response2(40000,">"))
					{
						print_gsm_debug("\r\nERROR in CIPSEND");
						// to do: process error
					}
//					gsm_clear_data();
					xsprintf(buf, " HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\nKeep-Alive: 180\r\nAccept: */*\r\nAccept-Language: en-us\r\nssn-acc: %d\r\n\r\n%c", pGSMDev->cSrvAddr, pGSMDev->uiSSNAcc, ASCII_CTRLZ);
					gsm_send_command(buf);

//					if(!(gsm_read_response2(20000,"CLOSE")))
					if(!(gsm_read_response2(30000,"SEND OK")))
					{
						xReturn = pdFALSE;
					}
				}

// waiting for http response completed:
//				gsm_clear_data();
//				gsm_send_command("AT+CIPRXGET=2,1024\r");
				gsm_wait_init(40000);

				if (GSMData.uiState == gsmDATA_INIT)
				{
					xReturn = pdTRUE;
				} else {
					print_gsm_debug("\r\nERROR in HTTP response");
					xReturn = pdFALSE;
				}
//						gsm_clear_data();
//						gsm_send_command("AT+CIPCLOSE\r\n");
//						if(gsm_read_response2(2000,"CLOSE OK")) { xReturn = pdPASS; } else {xReturn = pdFALSE; }
			}
	}

		if (xReturn || (xGSMRequest.uiTryCnt > gsmRESEND_TRY)) {
			if (xGSMRequest.pcRequest) {
				vPortFree(xGSMRequest.pcRequest);
			}
			// xReturn = xQueueReceive(xGSMHTTPQueue, &xGSMRequest, 0);
		} else {
			// try to resent
			xsprintf(msg, "\r\n\r\nGSMHTTPTask. Retry:%d, Mtd:%d, Cmd:%d", xGSMRequest.uiTryCnt, xGSMRequest.uiMethod, xGSMRequest.uiCommand);
			print_gsm_debug(msg);
			xGSMRequest.uiTryCnt++;
			xReturn = xQueueSendToFront(xGSMHTTPQueue, &xGSMRequest, 0 );
			if (!xReturn) {
				vPortFree(xGSMRequest.pcRequest);
			}
		}
		taskYIELD();
 } // while
}

/*-----------------------------------------------------------*/

static void prvGSMSendTask( void *pvParameters )
{
	sDevice* pDev = (sDevice*) pvParameters;
	sGSMDevice* pGSMDev = (sGSMDevice*) pDev->pDevStruct;
	uint32_t	lPort = (uint32_t)(pGSMDev->uiUSART - 1);
	uint32_t xReturn;

	char msg[gsmMAX_GSM_LEN+20];
	char buf[gsmMAX_GSM_LEN];

	while (1) {
		/* Wait for a message from GSM send queue */
		while( xQueueReceive( pGSMDev->pvQueueSend, &buf, portMAX_DELAY ) != pdPASS );
		// wait while current GSMData will be processed or timeout
			if (GSMData.uiState != gsmDATA_INIT) {
				vTaskDelay(gsmWAIT_DATA_TIMEOUT );
			}
			GSMData.uiState = gsmDATA_SENT;
			memcpy(&GSMData.pcCommand,&buf,sizeof(buf));

			xsprintf(msg, "\n\r>modem:=%s", buf);
			print_gsm_debug(msg);

			xReturn = lSerialPutString ( lPort, buf, strlen(buf) );
			(void)xReturn; // to do - process errors
			taskYIELD();
	}
}
/*-----------------------------------------------------------*/

static void prvGSMRespTask( void *pvParameters )
{
	sDevice* pDev = (sDevice*) pvParameters;
	sGSMDevice* pGSMDev = (sGSMDevice*) pDev->pDevStruct;
	uint32_t xReturn;
	uint8_t nCurRespLine = 0;
	(void)xReturn; // to do - process errors
	char msg[gsmMAX_GSM_LEN+20];

	char buf[gsmMAX_GSM_LEN];
	uint16_t uiBufLen;

	while (1) {
		/* Wait for a message from GSM resp queue */
		while( xQueueReceive( pGSMDev->pvQueueRecv, &buf, portMAX_DELAY ) != pdPASS );

		if (strlen(buf) < (gsmMAX_GSM_LEN-1)) {
			uiBufLen = strlen(buf);
		} else {
			uiBufLen = gsmMAX_GSM_LEN-1;
			buf[uiBufLen]=0;
		}

		xsprintf(msg, "\r\n<modem:=%s", buf);
		print_gsm_debug(msg);

		if (strncmp(buf, "OVER-VOLTAGE WARNNING",21)==0) {
			// disable charging of battery
			vSendInputMessage (1, 0, mainCOMMAND_MESSAGE, 0, pDev->nId, main_IF_PROGRAM, (void*)pGSMDev, 0, mainCOMMAND_DISMODEMCHARGE);
		}
		if (strncmp(buf, "UNDER-VOLTAGE WARNNING",22)==0) {
			// disable charging of battery
			vSendInputMessage (1, 0, mainCOMMAND_MESSAGE, 0, pDev->nId, main_IF_PROGRAM, (void*)pGSMDev, 0, mainCOMMAND_ENBMODEMCHARGE);
		}
//		if (strncmp(buf, "CLOSED",6)==0) {
//			// begin receiving http body
//			GSMData.uiState = gsmDATA_RECBLCWAIT;
//		}
//		if (strncmp(buf, "+IPD",4)==0)
//		{
//			// begin receiving http body
////			GSMData.uiState = gsmDATA_RECBLCWAIT;
//			GSMData.uiState = gsmDATA_RECBLC;
//			nNumLen = strspn (&buf[4], "1234567890");
//			memcpy (sNum, &buf[4], nNumLen);
//			sNum[nNumLen]=0;
//			nDataRecvRemain = conv2d (sNum);
//			if (vpDataRecv) {
//				vPortFree(vpDataRecv);
//			}
//			if (nDataRecvRemain > 0) {
//				nDataBlockSize = nDataRecvRemain;
//				vpDataRecv = pvPortMalloc(nDataRecvRemain);
//			}
//			strcpy (vpDataRecv, &buf[5+nNumLen]);
//			nDataRecvRemain -= (strlen(buf)-5-nNumLen);
//		}

		// state must be gsmDATA_SENT therefore we are waiting response for a command:
		if (GSMData.uiState == gsmDATA_SENT)
		{
			nCurRespLine = 0;
			GSMData.uiState = gsmDATA_RECV;
			memcpy(&GSMData.pcResponse[nCurRespLine], &buf, uiBufLen + 1);
//			GSMData.pcResponse[nCurRespLine][uiBufLen]=0;
			GSMData.uiCurrentReadLine = 0;
			nCurRespLine++;
			GSMData.uiNumLines = nCurRespLine;
		} else {
			// if state already gsmDATA_RECV - may be message next line of data response or http data:

			// check for next lines modem response
			if ((GSMData.uiState == gsmDATA_RECV) || (strncmp(buf, "CLOSED",6)==0)) {
				// may be it unsolicited messages?
				if (!strcmp(buf, "RING")) {
//					xReturn = xQueueSend(pGSMDev->pvQueueUnsolicited, buf, 0);
					// to do: process ring
				} else if (str_starts_with(buf, "+CLIP:")) {
					// to do: process
//					xReturn = xQueueSend(pGSMDev->pvQueueUnsolicited, buf, 0);
				}
				// to do - process other needed unsolicited messages...

// if nCurRespLine > gsmMAX_GSM_RESP_LINES then simply skip:
				if (nCurRespLine < (gsmMAX_GSM_RESP_LINES - 1)) {
					memcpy(&GSMData.pcResponse[nCurRespLine], &buf,	uiBufLen + 1);
					GSMData.uiNumLines = ++nCurRespLine;
				}
			} else {
 					if (GSMData.uiState == gsmDATA_INIT) {
						// if state not gsmDATA_RECV or gsmDATA_SENT then it unsolicited messages:
						xReturn = xQueueSend(pGSMDev->pvQueueUnsolicited, buf, 0);
					}
				}
			}
		taskYIELD();
	} // while
}

/*-----------------------------------------------------------*/

static void prvGSMUnsolicitedTask( void *pvParameters )
{
		sDevice* pDev = (sDevice*) pvParameters;
		sGSMDevice* pGSMDev = (sGSMDevice*) pDev->pDevStruct;
		uint32_t xReturn;
		char msg[gsmMAX_GSM_LEN+20];
		(void)xReturn; // to do - process errors

		char buf[gsmMAX_GSM_LEN];

		while (1)
		{
			/* Wait for a message from GSM unsolicited queue */
			while( xQueueReceive( pGSMDev->pvQueueUnsolicited, &buf, portMAX_DELAY ) != pdPASS );
 			xsprintf(msg, "\n\rmodem unsolcd:=%s", buf);
			print_gsm_debug(msg);
			// to do
			if (strstr(buf, "+CREG: 1,1")) {
				uiGSMModemState = gsmMODEM_STATE_REGISTER;
				print_gsm_debug("\n\r**Modem STATE_REGISTER");
			}
			if (strstr(buf, "RDY")) {
				goto UnslTskModemRDY;
			}
			if (strcmp(buf, "+CPIN: READY")==0) {
UnslTskModemRDY:
				// set modem status READY
				uiGSMModemState = gsmMODEM_STATE_READY;
				print_gsm_debug("\n\r**Modem STATE_READY");
			}

			taskYIELD();
		}
}

/*-----------------------------------------------------------*/
static void prvUSARTGSMRespTask( void *pvParameters )
{
signed char cChar;
const char*	cRECV_HTTP_DATA_BEGIN = "+IPD,";
uint8_t 	nScanCnt = 0;
char 		sNum[5];
uint8_t 	nNumLen;


sDevice* 	pDev = (sDevice*) pvParameters;
sGSMDevice* pGSMDev = (sGSMDevice*) pDev->pDevStruct;
uint32_t	lPort = (uint32_t)(pGSMDev->uiUSART - 1);

while (1) {
		xSerialGetChar( lPort, &cChar, portMAX_DELAY);

xSerialPutChar(0, cChar, 5 );

		// check for begin http data response
		if (cChar == cRECV_HTTP_DATA_BEGIN[nScanCnt]) {
			nScanCnt++;
		} else {
			nScanCnt = 0;
		}

		// begin receiving http block
		if (nScanCnt == (strlen(cRECV_HTTP_DATA_BEGIN))) {
			nNumLen = 0;
			if (GSMData.uiState == gsmDATA_RECV) {
				GSMData.uiState = gsmDATA_RECBLCWAIT;
			}
			if (GSMData.uiState == gsmDATA_HEADER_Ok) {
				GSMData.uiState = gsmDATA_HTTP_BLCWAIT;
			}
			continue;
		}

		if ( (GSMData.uiState == gsmDATA_RECBLCWAIT) || (GSMData.uiState == gsmDATA_HTTP_BLCWAIT) )
		{
			if (cChar >= '0' && cChar <= '9') {
				sNum[nNumLen++] = cChar;
				if (nNumLen == sizeof(sNum)) {
					// error, change to gsmDATA_ERROR mode
					// to do: process error
					// GSMData.uiState = gsmDATA_ERROR;
					GSMData.uiState = gsmDATA_RECV;
				}
			} else if (nNumLen > 0) {
				sNum[nNumLen]=0;
				GSMData.uiHttpRespBufSize = conv2d (sNum);
				// free buffer if it was allocated for header data (http data buffer free process - data receiver)
				if ((gsmDATA_RECBLCWAIT == gsmDATA_RECBLCWAIT) && (GSMData.pHttpRespBuffer)) {
					vPortFree(GSMData.pHttpRespBuffer);
				}
				if (GSMData.uiHttpRespBufSize > 0) {
					GSMData.pHttpRespBuffer = pvPortMalloc(GSMData.uiHttpRespBufSize + 1);

					// start receiving http block
					GSMData.uiHttpRespCounter = 0;

					if (GSMData.uiState == gsmDATA_RECBLCWAIT) {
						GSMData.uiState = gsmDATA_HEADER_RECBLC;
					} else {
						GSMData.uiState = gsmDATA_HTTP_RECBLC;
					}
					// skip ':' char:
					continue;
				}
			} else {
				// to do: process error
				GSMData.uiState = gsmDATA_RECV;
			}
		}

		if (GSMData.uiState == gsmDATA_HEADER_RECBLC)
		{
			((char*)GSMData.pHttpRespBuffer)[GSMData.uiHttpRespCounter++] = cChar;
			if (GSMData.uiHttpRespCounter == GSMData.uiHttpRespBufSize) {
				// finish http data block receiving
				((char*)GSMData.pHttpRespBuffer)[GSMData.uiHttpRespCounter] = 0;
				GSMData.uiState = gsmDATA_HEADER_Ok;
			}

		}
		if (GSMData.uiState == gsmDATA_HEADER_Ok)
		{
			print_gsm_debug("\n\r##HTTP header:\r\n");
			print_gsm_debug((char*)GSMData.pHttpRespBuffer);

			if (strcmp((char*)GSMData.pHttpRespBuffer, "Content-Length: 0") == 0) {
			// if response return Content-Length: 0, then break receiving data without sending to the Input queue
					// to do: process eror
					// GSMData.uiState = gsmDATA_ERROR;
					GSMData.uiState = gsmDATA_RECV;
			} else {
				GSMData.uiState = gsmDATA_HTTP_RECBLC;
			}
		}

		// receive next parts of http response
		if (GSMData.uiState == gsmDATA_HTTP_RECBLC)
		{
			((char*)GSMData.pHttpRespBuffer)[GSMData.uiHttpRespCounter++] = cChar;
			if (GSMData.uiHttpRespCounter == GSMData.uiHttpRespBufSize) {
				// finish http data block receiving
				((char*)GSMData.pHttpRespBuffer)[GSMData.uiHttpRespCounter] = 0;
				GSMData.uiState = gsmDATA_Ok;
			}
		}

		if (GSMData.uiState == gsmDATA_Ok)
		{
			print_gsm_debug("\n\r##HTTP response:\r\n");
			print_gsm_debug((char*)GSMData.pHttpRespBuffer);

//			if ((nDataRecvRemain <= 0) || (getTimerTick() >= uiDataRecvTimeout) || (strncmp(buf, "==end==",7)==0))
			vSendInputMessage (1, 0, mainGSM_MESSAGE_IN, 0, pDev->nId, main_IF_PROGRAM, GSMData.pHttpRespBuffer, GSMData.uiHttpRespCounter,0);
			GSMData.uiState = gsmDATA_INIT;
		}

		// check response AT+CIPSEND:
//		if ((GSMData.uiState == gsmDATA_SENT) && (cChar == '>'))
		if ((current_char_i == 0) && (cChar == '>'))
		{
			line[current_char_i++] = cChar;
			line[current_char_i] = '\0';
			xQueueSend( pGSMDev->pvQueueRecv, line, 0 );
			current_char_i = 0;
		}
		// process other modem responses:
		if ((GSMData.uiState == gsmDATA_RECV) || (GSMData.uiState == gsmDATA_INIT) || (GSMData.uiState == gsmDATA_SENT) || (GSMData.uiState == gsmDATA_ERROR))
		{
				if (cChar == '\n') {
						line[current_char_i] = '\0';
						if (strlen(line)) {
//							if (strcspn(line, " ") > 0) {
								xQueueSend( pGSMDev->pvQueueRecv, line, 0 );
//							}
						}
							current_char_i = 0;
				} else if ((current_char_i < (gsmMAX_GSM_LEN - 1)) && (cChar != '\r')) {
						line[current_char_i++] = cChar;
						if (current_char_i == gsmMAX_GSM_LEN)
						{
							// received string more then buffer limit, send to queue and reset buffer
							xQueueSend( pGSMDev->pvQueueRecv, line, 0 );
							current_char_i = 0;
						}
				}
		}

		taskYIELD();
	}
}

void setGSMDevValue(int32_t nValue, uint8_t nDevCmd, sDevice* dev, uint8_t nDataType)
{
	char * pcTeleData;
	char* buffer;
	char msg[127];
	switch (nDevCmd) {
	case gsmCmdPower:
		// to do: check nDataType, now let's always int
		if (nValue == 0) {
			gsm_clear_data();
			// switch off modem
			gsm_send_command("AT+CPOWD=1\r\n");
			gsm_read_response(1000);
		} else {
			// switch on modem
			gsm_init(dev);
		}
		break;
	case gsmCmdSendSMS:
		if (nDataType == eElmString) {
			xsprintf(msg, "%s", (char*)nValue);
			vPortFree((void*)nValue);
		} else {
			xsprintf(msg, "Alarm SENSOR: %d", nValue);
		}
		gsm_send_sms(msg);
		break;
	case gsmCmdSendLog:
		// to do:
		break;
	case gsmCmdSendWSCmd:
		vSendInputMessage (1, 0, mainGSM_MESSAGE_OUT, 0, main_IF_PROGRAM, dev->nId, (void*) 0, 0, (int16_t) nValue);
		break;
	case gsmCmdSendTelemetry:
		pcTeleData = process_getdevvals(devArray, all_devs_counter, 0);
		uint16_t nSizeTeledata = strlen(pcTeleData);
		uint16_t nSize = nSizeTeledata;
		buffer = pcTeleData;

#ifdef AES128
		// align buffer to 16 Byte block
		nSizeTeledata = nSizeTeledata + 16 - (nSizeTeledata % 16);
		buffer = pvPortMalloc(nSizeTeledata);
		nSize = nSizeTeledata;
		AES128_CBC_encrypt_buffer((uint8_t*)buffer, (uint8_t*)pcTeleData, nSizeTeledata, (uint8_t*)pGSMDev->cAESKey, (uint8_t*)cIV);
		vPortFree(pcTeleData);
#endif
#ifdef gsmBASE64
		nSize = nSizeTeledata * 1.5;
		char* buffer2 = pvPortMalloc(nSize); // allocate more memory for base64
		base64_encode((uint8_t*)buffer, nSizeTeledata, buffer2);
		nSize = strlen(buffer2);
		vPortFree(buffer);
		buffer = buffer2;
#endif

		if (buffer) {
			vSendInputMessage (1, 0, mainTELEMETRY_MESSAGE, 0, main_IF_PROGRAM, dev->nId, (void*) buffer, nSize,0);
		}
		break;
	}
	xsprintf(msg, "\n\rFreeHeapSize:=%d ==========", xPortGetFreeHeapSize());
	print_gsm_debug(msg);

}

