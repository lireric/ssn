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
 * commands.c
 *
 *  Created on: 16 окт. 2014 г.
 *      Author: eric
 */

#include "commands.h"
#include "xprintf.h"
#include <stdarg.h>
#include "dev_memory.h"

#ifdef  M_ETHERNET
#include "ethernet.h"
#include "dhcp.h"
#include "w5500/w5500.h"
#include "w5500/socket.h"
#endif

#include "utils.h"
#ifdef PERSIST_STM32FLASH
#include <libopencm3/stm32/flash.h>
#endif

char * cPassMessage[ mainMAX_MSG_LEN ];
//#ifdef  M_GSM
extern	void prvStartGSMTask( void *pvParameters );
//#endif
#ifdef  M_ETHERNET
#include "ethernet.h"
#include "dhcp.h"
#include "w5500/w5500.h"
#include "w5500/socket.h"
#include "sockutil.h"
extern	uint8 mac[6];
extern	uint8 lip[4];
extern	uint8 sub[4];
extern	uint8 gw[4];
extern	uint16 port;
#endif

//extern int16_t* 	pADCValueArray;
//extern uint32_t* 	pADCLastUpdate;

uint16_t getCommandsByName(const char* sCmd)
{
	uint16_t nCmd = mainCOMMAND_NONE;
	if (strcmp(sCmd, "reboot") == 0) {
		nCmd = mainCOMMAND_REBOOT;
	} else if (strcmp(sCmd, "getdevvals") == 0) {
		nCmd = mainCOMMAND_GETDEVVALS;
	} else if (strcmp(sCmd, "getextcmd") == 0) 	{
		nCmd = mainCOMMAND_GETEXTCMD;
	} else if (strcmp(sCmd, "loadprefs") == 0) 	{
		nCmd = mainCOMMAND_GETPREFERENCES;
	} else if (strcmp(sCmd, "updateaction") == 0) {
		nCmd = mainCOMMAND_UPDATEACTION;
	} else if (strcmp(sCmd, "gsmcmd") == 0) 	{
		nCmd = mainCOMMAND_GSMCMD;
	} else if (strcmp(sCmd, "getowilist") == 0) {
		nCmd = mainCOMMAND_GETOWILIST;
	} else if (strcmp(sCmd, "sdv") == 0) 		{
		nCmd = mainCOMMAND_SETDEVVALUE;
	} else if (strcmp(sCmd, "setdatetime") == 0) {
		nCmd = mainCOMMAND_SETDATETIME;
	}
	return nCmd;
}


uint16_t 	getLog_Object(void)
{
	return uiLog_Object;
}

void 		setLog_Object(uint16_t uiObj)
{
	uiLog_Object = uiObj;
}

void 		setIp(char* sBuffer)
{
	inet_addr_((unsigned char*)sBuffer, lip);
}

void 		setNet(char* sBuffer)
{
	inet_addr_((unsigned char*)sBuffer, sub);
}
void 		setPort(uint16_t nPort)
{
	port = nPort;
}
void 		setGw(char* sBuffer)
{
	inet_addr_((unsigned char*)sBuffer, gw);
}

/* get route (NODE interface) to object
 * if object not found return default route (0) */
uint8_t getObjRoute(uint16_t	nDestObj)
{
	uint8_t i;
	uint8_t nRoute = 0;
	for (i=0; i<=route_counter; i++) {
		if (routeArray[i].nDestObj == nDestObj) {
			nRoute = routeArray[i].xDestType;
			break;
		}
	}
	return nRoute;
}

uint32_t setObjRoute(uint16_t	nDestObj, uint8_t nRoute)
{
	uint8_t i;
	for (i=0; i<=route_counter; i++) {
		if (routeArray[i].nDestObj == nDestObj) {
			routeArray[i].xDestType  = nRoute;
			return pdTRUE;
		}
	}
	if (route_counter < mainMAX_ROUTES) {
		routeArray[++route_counter].nDestObj = nDestObj;
		routeArray[route_counter].xDestType = nRoute;
		return pdTRUE;
	} else {
		return pdFALSE;
	}
}

uint32_t delObjRoute(uint16_t	nDestObj)
{
	uint8_t i;
	for (i=0; i<=route_counter; i++) {
		if (routeArray[i].nDestObj == nDestObj) {
			routeArray[i].nDestObj = routeArray[route_counter].nDestObj;
			routeArray[i].xDestType = routeArray[route_counter].xDestType;
			route_counter--;
			return pdTRUE;
		}
	}
	return pdFALSE;
}

void process_setdatetime(cJSON *json_data)
{
	RTC_t rtc;
	//												counter_to_struct( dtsec, &rtc );
	rtc.year = (uint16_t) cJSON_GetObjectItem(json_data, "y")->valueint;
	rtc.month = (uint8_t) cJSON_GetObjectItem(json_data, "m")->valueint;
	rtc.mday = (uint8_t) cJSON_GetObjectItem(json_data, "d")->valueint;
	rtc.hour = (uint8_t) cJSON_GetObjectItem(json_data, "h")->valueint;
	rtc.min = (uint8_t) cJSON_GetObjectItem(json_data, "min")->valueint;
	rtc.sec = (uint8_t) cJSON_GetObjectItem(json_data, "sec")->valueint;
	rtc_settime(&rtc);
#if defined M_RTC_DS1307
	sGrpInfo* pGrpInfo = getGrpInfo(SOFTI2C_1_GRP);
	sGrpDev* pGrpDev2 = &pGrpInfo->GrpDev;
	DS1307_time.year = rtc.year - 2000;
	DS1307_time.month = rtc.month;
	DS1307_time.day = rtc.mday;
	DS1307_time.hour = rtc.hour;
	DS1307_time.min = rtc.min;
	DS1307_time.sec = rtc.sec;
	RTC_DS1307_adjust(pGrpDev2);
#endif
	xprintfMsg("\r\nSet local date/time from JSON: %d-%d-%d %d:%d:%d", rtc.year, rtc.month, rtc.mday, rtc.hour, rtc.min, rtc.sec);

}

void process_getdatetime(cJSON *json_data)
{
	hw_loadtime();
}

void hw_loadtime()
{
#if defined (M_RTC_DS1307)
	sGrpInfo* pGrpInfo = getGrpInfo(SOFTI2C_1_GRP);
	sGrpDev* pGrpDev2 = &pGrpInfo->GrpDev;
		// load RTC from RTC_DS1307:
		if ( RTC_DS1307_now(pGrpDev2) )
			{
				RTC_t rtc;
				rtc.year = DS1307_time.year + 2000;
				rtc.month = DS1307_time.month;
				rtc.mday = DS1307_time.day;
				rtc.hour = DS1307_time.hour;
				rtc.min = DS1307_time.min;
				rtc.sec = DS1307_time.sec;
				rtc_settime(&rtc);
				xprintfMsg("\r\nSet date/time from RTC DS1307: %d-%d-%d %d:%d:%d", rtc.year, rtc.month, rtc.mday, rtc.hour, rtc.min, rtc.sec);

			}
			else {
				xprintfMsg("\r\nRTC DS1307 not responding");
			};
#endif

}

// Get list devices, connected to pin
// input: g - group number
void process_getowilist(cJSON *json_data)
{
	uint8_t res;

	OWI_device* owi_tmp = (OWI_device*)pvPortMalloc(sizeof(OWI_device) * mainMAX_GRP_DEVICES);
	sGrpInfo* pGrpInfo = getGrpInfo((uint8_t) cJSON_GetObjectItem(json_data,"g")->valueint);

// TO DO: replace to debugMsg...

	taskENTER_CRITICAL();
	{
	res = owi_search_devices(owi_tmp, mainMAX_GRP_DEVICES, pGrpInfo, &pGrpInfo->iDevQty);
	}
	if (!res) {
		xprintfMsg("{\"ssn\":{\"v\":1,\"ret\":\"getowilist\", \"data\":{ \"num\":%d, \"owiid\":[", pGrpInfo->iDevQty);

	char comma = ',';
	uint8_t i;
		for (i = 0; i<pGrpInfo->iDevQty; i++) {
			if (i==(pGrpInfo->iDevQty-1)) {comma = ' ';};
			xprintfMsg("\"%02x%02x%02x%02x%02x%02x%02x%02x\"%c", owi_tmp[i].ucROM[0],owi_tmp[i].ucROM[1],owi_tmp[i].ucROM[2],owi_tmp[i].ucROM[3],owi_tmp[i].ucROM[4],owi_tmp[i].ucROM[5],owi_tmp[i].ucROM[6],owi_tmp[i].ucROM[7], comma);
		}
		xprintfMsg("]}}}");
	}
	taskEXIT_CRITICAL();
	vPortFree(owi_tmp);
}

/* Load preferences from INI message to EEPROM or FLASH */
// ... to do

// Handler for processing INI preferences format
uint32_t process_loadprefs_ini_handler(char* sSection, char* sName, char* sValue, sIniHandlerData* pIniHandlerData, int* pnSectionNo)
{
	int32_t nRes = pdPASS; //pdFAIL;
	sGrpInfo *pGrpInfo = NULL;
	sDevice * pDev;

#ifdef  M_GSM
	sGSMDevice* ptGSMDev = NULL;
#endif
	#define MATCH(s, n) strcmp(sSection, s) == 0 && strcmp(sName, n) == 0

	if (MATCH("app", "logobj")) {
		setLog_Object(conv2d(sValue));		// store object-logger
	} else if (MATCH("app", "port")) {
		setPort(conv2d(sValue));		// store TCP server port
	} else if (MATCH("app", "ip")) {
		setIp(sValue);		// store TCP server port
	} else if (MATCH("app", "net")) {
		setNet(sValue);		// store TCP server port
	} else if (MATCH("app", "gw")) {
		setGw(sValue);		// store TCP server port
	}
	else if (strcmp(sSection, "grp") == 0) {
		// section "grp" --------------------------------------------
		// check for new group
		if (strcmp(pIniHandlerData->sLastSection, sSection) != 0) {
			pGrpInfo = newGrpInfo();
			if (!pGrpInfo) {
				return pdFAIL;
			}

			pGrpInfo->uiObj = getMC_Object();
			pGrpInfo->iDevQty = 0;
			pGrpInfo->GrpDev.pTimer = 0;

		} else {
			pGrpInfo = getCurrentGrp();
		}
		if (strcmp(sName, "grpnum") == 0) {
			pGrpInfo->uiGroup = conv2d(sValue);

		} else if (strcmp(sName, "grpport") == 0) {
			pGrpInfo->GrpDev.pPort = get_port_by_name(sValue);

		} else if (strcmp(sName, "grptimer") == 0) {
			pGrpInfo->GrpDev.pTimer = get_port_by_name(sValue);

		} else if (strcmp(sName, "grppin1") == 0) {
			pGrpInfo->GrpDev.ucPin = conv2d(sValue);

		} else if (strcmp(sName, "grppin2") == 0) {
			pGrpInfo->GrpDev.ucPin2 = conv2d(sValue);
		}
	} else if (strcmp(sSection, "dev") == 0) {
		// section "dev" --------------------------------------------
		// check for new device
		if (strcmp(pIniHandlerData->sLastSection, sSection) != 0) {

			if (all_devs_counter > mainMAX_ALL_DEVICES) {
				return pdFAIL;
			}
			pDev = newDev();

			if (!pDev) {
				return pdFAIL;
			}
			pDev->nDevObj = getMC_Object();
		}
		pDev = getCurrentDev();
		if (strcmp(sName, "grp") == 0) {
			pDev->pGroup = getGrpInfo(conv2d(sValue));

			if (!pDev->pGroup) {
				return pdFAIL;
			}
			pDev->pGroup->iDevQty++;

		} else

			return devicePreInit(pDev, sName, sValue); // perform special device initialization

	} else if (strcmp(sSection, "route") == 0) {
		// section "route" --------------------------------------------
		if (pIniHandlerData->pnPrevSectionNo != *pnSectionNo) {
			route_counter++;
		}
		if (strcmp(sName, "obj") == 0) {
			routeArray[route_counter].nDestObj = conv2d(sValue);
		} else if (strcmp(sName, "if") == 0) {
			routeArray[route_counter].xDestType = conv2d(sValue);
		}

	} else if (strcmp(sSection, "logic") == 0) {
		// section "logic" --------------------------------------------
		if ((pIniHandlerData->pnPrevSectionNo != *pnSectionNo) && pIniHandlerData->xTempAction.aid
				&& (strlen(pIniHandlerData->xTempAction.astr) > 0))
		{
			nRes = setAction (pIniHandlerData->xTempAction.aid, pIniHandlerData->xTempAction.astr, pIniHandlerData->xTempAction.arep, pIniHandlerData->xTempAction.nFlags);
			// reset tmp structure:
			pIniHandlerData->xTempAction.aid = 0;
			pIniHandlerData->xTempAction.astr[0] = 0;
			pIniHandlerData->xTempAction.arep = 0;
			pIniHandlerData->xTempAction.nFlags = 0;
		}
		if (strcmp(sName, "aid") == 0) {
			pIniHandlerData->xTempAction.aid = conv2d(sValue);
		} else if (strcmp(sName, "arep") == 0) {
			pIniHandlerData->xTempAction.arep = conv2d(sValue);
		} else if (strcmp(sName, "astr") == 0) {
			strncpy0(pIniHandlerData->xTempAction.astr, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "nolog") == 0) {
			if (conv2d(sValue) == 1) {
				pIniHandlerData->xTempAction.nFlags |= devACTION_FLAG_NOLOG;
			} else {
				pIniHandlerData->xTempAction.nFlags ^= devACTION_FLAG_NOLOG;
			}
		} else if (strcmp(sName, "s") == 0) {
			// single timer flag
			if (conv2d(sValue) == 1) {
				pIniHandlerData->xTempAction.nFlags |= devACTION_FLAG_SINGLE_TIMER;
			} else {
				pIniHandlerData->xTempAction.nFlags ^= devACTION_FLAG_SINGLE_TIMER;
			}
		}

	}
	return nRes;
}


// read preferences string from EEPROM or Flash
int32_t restorePreferences(char** sBuf, uint16_t *nBufSize)
{
	int32_t nRes = pdFAIL;
//	uint8_t Rx1Buffer[4];
	sPrefsMetaData sPMeta;
//	sPrefsMetaData **pPMeta = &sPMeta;

// first 2 byte in EEPROM consist size of JSON structure with SSN hardware and logic preferences, started from 5 byte
	nRes =  restorePersistentData(&sPMeta, (uint16_t)4, 0); // read first bytes with preferences string size
	if (!nRes)
		goto restorePreferencesEnd;

//	*nBufSize = Rx1Buffer[0] + (Rx1Buffer[1] << 8);
	*nBufSize = sPMeta.nPrefsSize;

	if (*nBufSize < xPortGetFreeHeapSize())
	{
		*sBuf = (char*)pvPortMalloc(*nBufSize+1);
		if (*sBuf) {
			nRes =  restorePersistentData(*sBuf, *nBufSize, 5); // read all preferences string
			if (nRes) {
				*(sBuf+*nBufSize)=0;
			}
		} else {
			nRes = pdFAIL;
		}
	} else {
		xprintfMsg("\r\nError preferences size info (EEPROM or FLASH): %d", *nBufSize);
	}

	restorePreferencesEnd:
	return nRes;
}

// write preferences string into EEPROM or Flash
int32_t storePreferences(char* sBuf, uint16_t nBufSize)
{
	int32_t nRes = pdFAIL;
	void *pBufTmp = pvPortMalloc(nBufSize+4);
	if (!pBufTmp)
		goto storePreferencesEnd;

	// first 2 byte in EEPROM consist size of JSON structure with SSN hardware and logic preferences, started from 5 byte
	memcpy((uint8_t*)(pBufTmp+5), sBuf, nBufSize);
	*(uint16_t*)(pBufTmp) = nBufSize;

	nRes = storePersistentData((void*)pBufTmp, nBufSize+4, 0);
	if (nRes) {
		xprintfMsg("\r\nNew preferences saved into EEPROM. Reboot");
//			    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
//			    while (1);
	} else {
		xprintfMsg("\r\nError saving preferences into EEPROM");
	}

	vPortFree(pBufTmp);

//	sGrpInfo* pGrpInfo = getGrpInfo (0);
//	delay_nus(&pGrpInfo->GrpDev, 10000);	// 10ms
	delay_nus2(10000);	// 10ms

	storePreferencesEnd:
	return nRes;
}


char* process_getdevvals(sDevice** devArray, uint16_t all_devs_counter, uint16_t nDevId)
{
	uint16_t j;
	uint8_t nNumValTypes;
//	const uint16_t nBufSize = 512;
//	char* tele_data = pvPortMalloc(nBufSize);
//	char* tele_data2;
//	char *buf = pvPortMalloc(mainMAX_MSG_LEN);

	void *pBufDev;
	void *pBufAll = pvPortMalloc(mainMAX_MSG_LEN); // = NULL;
	void *pBufTmp;
	uint16_t	uiBufLen;
	const char* pcTeleFooter = "{}]}}}";


//	uint16_t	nStrCounter = 0;
//	uint8_t nExtBufCounter = 1;
	int32_t nDevValue;
	uint32_t nDevLastUpdate;
	sDevice * pDev;

//	taskENTER_CRITICAL();
	{
		xprintfMsgStr(pBufAll, "{\"ssn\":{\"v\":1,\"obj\":%d,\"ret\":\"getdevvals\", \"data\":{\"devs\":[",  getMC_Object() );

		for (j = 0; j <= all_devs_counter; j++) {
			pDev = getDevByNo(j);
			if (pDev) {
				if ((nDevId > 0) && (pDev->nId != nDevId)) {
					continue;
				}
//				*buf = 0;
				nNumValTypes = getNumberDevValues(pDev);
				for (uint8_t k = 0; k < nNumValTypes; k++) {
					getDevData(pDev, k, &nDevValue, &nDevLastUpdate);
					pBufDev = pvPortMalloc(mainMAX_MSG_LEN);
					if (pBufDev) {
						xprintfMsgStr(pBufDev,
								"{\"dev\":%d, \"n\":%d, \"i\":%d, \"val\":%d, \"updtime\":%ld},",
								pDev->nId, nNumValTypes, k, nDevValue,
								nDevLastUpdate);

						uiBufLen = strlen(pBufAll) + strlen(pBufDev) + strlen(pcTeleFooter);

						pBufTmp = pvPortMalloc(uiBufLen);
						if (pBufTmp) {

							memcpy(pBufTmp, pBufAll, strlen(pBufAll));
							memcpy(pBufTmp + strlen(pBufAll), pBufDev, strlen(pBufDev));
							*(uint8_t*)(pBufTmp + strlen(pBufAll) + strlen(pBufDev)) = 0;

							vPortFree(pBufAll);
							vPortFree(pBufDev);
							pBufAll = pBufTmp;
						} else {
							if (pBufAll)
								vPortFree(pBufAll);
							if (pBufDev)
								vPortFree(pBufDev);
							goto getdevvalsEnd;
						}
					} else {
						if (pBufAll)
							vPortFree(pBufAll);
						goto getdevvalsEnd;
					}
				}
			}
		}

		uiBufLen = strlen(pBufAll);
		memcpy(pBufAll + strlen(pBufAll), pcTeleFooter, strlen(pcTeleFooter));
		*(uint8_t*)(pBufAll + uiBufLen + strlen(pcTeleFooter)) = 0;

//	taskEXIT_CRITICAL();
	}
	getdevvalsEnd:
	return pBufAll;
}

void vSendInputMessage (uint8_t version, uint16_t	uiDestObject, uint8_t xMessageType, uint16_t uiSrcObject, uint16_t xSourceDev, uint16_t xDestDev, void* pcMessage, uint16_t nSize, uint16_t nCommand)
{
	xInputMessage sInputMessage;

	/* Setup the message we are going to send to the Input queue. */
	sInputMessage.version = version;
	sInputMessage.uiDestObject = uiDestObject;
	sInputMessage.xSourceDevice = xSourceDev;
	sInputMessage.uiSrcObject = uiSrcObject;
	sInputMessage.xDestDevice = xDestDev;
	sInputMessage.pcMessage = pcMessage;
	sInputMessage.xMessageType = xMessageType;
	sInputMessage.nMsgSize = nSize;
	sInputMessage.nCommand = nCommand;
	xQueueSend(xInputQueue, &sInputMessage, portMAX_DELAY);

}

void vSendSSNPacket (uint16_t nObjDst, uint16_t nObjSrc, uint8_t nMessType, char* cData)
{
	uint16_t nDataLength = strlen(cData);
//	char* tmpBuf = pvPortMalloc(nDataLength+30);
	xprintfMsg("\n===ssn1%04x%04x%02x%04x%s%04x", nObjDst, nObjSrc, nMessType, nDataLength, cData, crc16((uint8_t*) cData, nDataLength));
//	vPortFree(tmpBuf);
}

void xprintfMsg (			/* Put a formatted string to the memory */
		const char*	fmt,	/* Pointer to the format string */
		...					/* Optional arguments */
	)
	{
		char* buff = pvPortMalloc(1024);			/* Pointer to the output buffer */
		// to do: calculate memory more accurately
		va_list arp;

		va_start(arp, fmt);
		xvprintf2(buff, fmt, arp);
		va_end(arp);
		sendBaseOut (buff);
		vPortFree(buff);
	}

void xprintfMsgStr (
		char* buff,         /* Put a formatted string to the memory */
		const char*	fmt,	/* Pointer to the format string */
		...					/* Optional arguments */
	)
	{
		va_list arp;

		va_start(arp, fmt);
		xvprintf2(buff, fmt, arp);
		va_end(arp);
	}


void debugMsg (char *str)
{
//	xQueueSend( xLogOutQueue, str, 0);

	uint16_t uiCnt;
	uint16_t uiBufLen;
	uint32_t xReturn;
//	char msg[mainMAX_MSG_LEN];
	char *msg = pvPortMalloc(mainMAX_MSG_LEN);

		for (uiCnt=0; uiCnt<=strlen(str)/mainMAX_MSG_LEN; uiCnt++)
		{
			uiBufLen = strlen(str)-uiCnt*mainMAX_MSG_LEN;
			if (uiBufLen > (mainMAX_MSG_LEN)) {
				uiBufLen = mainMAX_MSG_LEN;
			} else {
				memset (msg,0,mainMAX_MSG_LEN); // clear buffer
			}

			memcpy(msg, &str[uiCnt*mainMAX_MSG_LEN], uiBufLen);
			if (xLogOutQueue) {
				xReturn = xQueueSend( xLogOutQueue, msg, 0 );
			}
		}

		vPortFree(msg);
		( void ) xReturn;

}

void sendBaseOut (char *str)
{
	uint16_t uiCnt;
	uint16_t uiBufLen;
	uint32_t xReturn;
//	char msg[mainMAX_MSG_LEN];
	char *msg = pvPortMalloc(mainMAX_MSG_LEN);
	if (getEthernetState() == SOCK_ESTABLISHED) {
		send(0,(uint8*)str,strlen(str));
	}
		for (uiCnt=0; uiCnt<=strlen(str)/mainMAX_MSG_LEN; uiCnt++)
		{
			uiBufLen = strlen(str)-uiCnt*mainMAX_MSG_LEN;
			if (uiBufLen > (mainMAX_MSG_LEN)) {
				uiBufLen = mainMAX_MSG_LEN;
			} else {
				memset (msg,0,mainMAX_MSG_LEN); // clear buffer
			}

			memcpy(msg, &str[uiCnt*mainMAX_MSG_LEN], uiBufLen);
			if (xBaseOutQueue) {
				xReturn = xQueueSend( xBaseOutQueue, msg, 0 );
			}
		}
	vPortFree(msg);
	( void ) xReturn;
}

void vCommandSelector(sSSNCommand* xSSNCommand)
{
//	char msg[30];
	if (xSSNCommand) {
		switch (xSSNCommand->nCmd) {
			case mainCOMMAND_COMMITED: {
				xprintfMsg("\r\nCOMMITED: %d ", xSSNCommand->nCmdID);

				// do nothing
				break;
			}
			case mainCOMMAND_UPDATEACTION: {
				UpdateAction(xSSNCommand->pcData);
				break;
			}
			case mainCOMMAND_REBOOT: {
				main_reboot();
				break;
			}
			case mainCOMMAND_MEMSAVE: {
				storeAllMemDevs();
				break;
			}
			case mainCOMMAND_DISMODEMCHARGE: {
				// disable charging of battery
	#ifdef  M_GSM
				//sendBaseOut("\r\nOVER-VOLTAGE, disable charging");
				xprintfMsg("\r\nOVER-VOLTAGE, disable charging");
				sGSMDevice* pGSMDev = (sGSMDevice*) xSSNCommand->pcData;
				if (pGSMDev) {
					gpio_clear(pGSMDev->uiPortChgCtrl, 1 << pGSMDev->uiPortChgCtrl);
				}
	#endif
				break;
			}
			case mainCOMMAND_ENBMODEMCHARGE: {
				// disable charging of battery
	#ifdef  M_GSM
				//sendBaseOut("\r\nLOW-VOLTAGE, enable charging");
				xprintfMsg("\r\nLOW-VOLTAGE, enable charging");
				sGSMDevice* pGSMDev = (sGSMDevice*) xSSNCommand->pcData;
				if (pGSMDev) {
					gpio_set(pGSMDev->uiPortChgCtrl, 1 << pGSMDev->uiPortChgCtrl);
				}
	#endif
				break;
			}
			case mainCOMMAND_GETDEVVALS: {
				char * pcTeleData;
				pcTeleData = process_getdevvals(devArray, all_devs_counter, 0);
				if (pcTeleData) {
					vSendInputMessage(1, xSSNCommand->uiObjSrc, mainTELEMETRY_MESSAGE, getMC_Object(), 0,
							xSSNCommand->uiDevDest, (void*) pcTeleData, strlen(pcTeleData), 0);
				}
				break;
			}
		}
		xprintfMsg("\r\nFreeHeapSize:=%d **********", xPortGetFreeHeapSize());

	}
}

void main_reboot()
{
	storeAllMemDevs();

    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    while (1);
}


/* Update action or add new. Action based on formula in "astr" attribute. Examples:
 *
 */
//{"aid":1,"astr":"(d(2002,1)<900)&&(d(3001,2)>50)=a(4004,0,1,0,0)","nolog":"1"},
//{"aid":2,"astr":"d(2002,1)+10<=20=a(10001,2,'Alarm: t<2')","arep":"3600"},
//{"aid":3,"astr":"d(2002,1)<250=a(4004,0,1),a(4005,0,0),a(10001,2,'Alarm!')"},
//{"aid":4,"astr":"d(2002,1)>260=a(4004,0,0)"},
//{"aid":5,"astr":"(tt>'7:00:00')&&(tt<'21:00:00')=a(4004,0,0,0,1)"},
//{"aid":6,"astr":"(ty>2015)&&(tm==3)&&(td==8)=a(10001,2,'Greetings!')"},
//{"aid":7,"astr":"(tw>=1)&&(tw<=5)=a(4005,0,1,0,0)"},
//{"aid":8,"astr":"i(3000,100)=a(4004,0,1,0,0)","nolog":"1"},
//{"aid":9,"astr":"i(18000,0)=a(10001,5,1)"},
//{"aid":10,"astr":"i(30000,0)=a(10001,6,1)"},

void	UpdateActionJSON(cJSON *devactitem)
{
	char*	pActStr;
	cJSON*  tmpJSON;
	uint16_t arep = 0;
	uint16_t nFlags = 0;
	uint32_t ret;
//	char msg [100];

	if (cJSON_GetObjectItem(devactitem, "astr")) {

		uint16_t nActID = (uint16_t) cJSON_GetObjectItem(devactitem, "aid")->valueint;
		pActStr = cJSON_GetObjectItem(devactitem, "astr")->valuestring;

		tmpJSON = cJSON_GetObjectItem(devactitem, "arep");

		if (tmpJSON) {
			arep = (uint16_t) tmpJSON->valueint;
		}

		tmpJSON = cJSON_GetObjectItem(devactitem, "nolog");
		if (tmpJSON) {
				if ((uint8_t)tmpJSON->valuestring[0] == '1')
				{
					nFlags |= devACTION_FLAG_NOLOG;
				}
		}
		// single timer flag
		if (cJSON_GetObjectItem(devactitem, "s")) {
			if ((uint8_t)cJSON_GetObjectItem(devactitem, "s")->valuestring[0] == '1')
			{
				nFlags |= devACTION_FLAG_SINGLE_TIMER;
			}
		}

		ret = setAction (nActID, pActStr, arep, nFlags);

		if (!ret) {
			xprintfMsg("\r\nError processing action rules! AID=%d", nActID);
		}
	}
}

int32_t	UpdateAction(char* pcDevAction)
{
	int32_t result;
	cJSON *pjsActItem;

	if (!pcDevAction) return pdFAIL;

	pjsActItem = cJSON_Parse(pcDevAction);
	if (pjsActItem) {
		UpdateActionJSON(pjsActItem);
		cJSON_Delete(pjsActItem);
	}

	result = refreshActions2DeviceCash();
	return result;
}

void log_event (void* poldt, void* pnewt, uint32_t xTickCount)
{
	xprintfMsg("\r\n***%s switched out, %s switched in, tick count = %u",
			pcTaskGetTaskName((TaskHandle_t) poldt),
			pcTaskGetTaskName((TaskHandle_t) pnewt),
			xTickCount );
}

void log_event2 (char c, void* pt)
{
	xprintfMsg("\r\n***%c: task %s delay",c, pcTaskGetTaskName((TaskHandle_t) pt));
}

/*
 * Generate and send heartbeat message
 */
void heartBeatSend(uint32_t nCounter, uint32_t nTimestamp)
{
	char* tmpBuffer = pvPortMalloc(50);

	xprintfMsgStr(tmpBuffer, "{\"hb\":[{\"t\":%d, \"c\":%d}]}", nTimestamp, nCounter);
	vSendInputMessage(1, 0, mainJSON_MESSAGE, getMC_Object(), 0, 0, (void*) tmpBuffer, strlen(tmpBuffer), 0);

}

/* log action event into temporary buffer and send it if buffer full */
void	logAction(uint16_t nActId, uint16_t nDevId, uint8_t nDevCmd, uint32_t nValue)
{
	uint16_t i;
	uint16_t j;
	char* pBuffer;
//	char tmpBuffer[100];
	char *tmpBuffer = pvPortMalloc(100);
	uint32_t nCurTimestamp = rtc_get_counter_val();

	if (nDevId) {
		logActionsArray[logActCounter].nActId = nActId;
		logActionsArray[logActCounter].nDevId = nDevId;
		logActionsArray[logActCounter].nDevCmd = nDevCmd;
		logActionsArray[logActCounter].nValue = nValue;
		logActionsArray[logActCounter].nTimestamp = nCurTimestamp;
		logActCounter++;
	}
	if (logActCounter && ((logActCounter >= mainLOG_ACTIONS_SIZE) || (nCurTimestamp >= (nlogActLastUpdate + mainACTIONSARRAYTIMEOUT))))
		{
			// send log to logger object
			pBuffer = pvPortMalloc(70*logActCounter+10);	// allocate memory for current log elements
			if (!pBuffer) {
				xprintfMsg("\r\nError allocate memory for log data!");
				return;
			}

			xprintfMsgStr(tmpBuffer, "{\"log\":[");
			memcpy(&pBuffer[0], tmpBuffer, strlen(tmpBuffer));
			j = strlen(tmpBuffer);

			for (i = 0; i < logActCounter; i++)
			{
				xprintfMsgStr(tmpBuffer, "{\"a\":%d,\"d\":%d,\"c\":%d,\"v\":%d,\"t\":%d},", logActionsArray[i].nActId,
						logActionsArray[i].nDevId, logActionsArray[i].nDevCmd, logActionsArray[i].nValue,
						logActionsArray[i].nTimestamp);
				memcpy(&pBuffer[j], tmpBuffer, strlen(tmpBuffer));
				j += strlen(tmpBuffer);
			}
			xprintfMsgStr(tmpBuffer, "{}]}");
			memcpy(&pBuffer[j], tmpBuffer, strlen(tmpBuffer));
			pBuffer[j+strlen(tmpBuffer)] = 0;

			// send log data to input buffer for routing to destination:
			vSendInputMessage(1, 0, mainLOG_MESSAGE, getMC_Object(), 0,	0, (void*) pBuffer, strlen(pBuffer), 0);
			nlogActLastUpdate = nCurTimestamp;
			logActCounter = 0;
	}
	vPortFree(tmpBuffer);

}
