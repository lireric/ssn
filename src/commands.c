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
#include "utils.h"
#ifdef PERSIST_STM32FLASH
#include <libopencm3/stm32/flash.h>
#endif

char * cPassMessage[ mainMAX_MSG_LEN ];

uint16_t 	getLog_Object(void)
{
	return uiLog_Object;
}

void 		setLog_Object(uint16_t uiObj)
{
	uiLog_Object = uiObj;
}

/* get route (NODE interface) to object
 * if object not found return default route (0) */
uint8_t getObjRoute(uint16_t	nDestObj)
{
	uint8_t i;
	uint8_t nRoute = 0;
	for (i=0; i<route_counter; i++) {
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
	for (i=0; i<route_counter; i++) {
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
	for (i=0; i<route_counter; i++) {
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
	sGrpInfo* pGrpInfo = getGrpInfo(SOFTI2C_1_GRP);
	sGrpDev* pGrpDev2 = &pGrpInfo->GrpDev;

	//												counter_to_struct( dtsec, &rtc );
	rtc.year = (uint16_t) cJSON_GetObjectItem(json_data, "y")->valueint;
	rtc.month = (uint8_t) cJSON_GetObjectItem(json_data, "m")->valueint;
	rtc.mday = (uint8_t) cJSON_GetObjectItem(json_data, "d")->valueint;
	rtc.hour = (uint8_t) cJSON_GetObjectItem(json_data, "h")->valueint;
	rtc.min = (uint8_t) cJSON_GetObjectItem(json_data, "min")->valueint;
	rtc.sec = (uint8_t) cJSON_GetObjectItem(json_data, "sec")->valueint;
	rtc_settime(&rtc);
	DS1307_time.year = rtc.year - 2000;
	DS1307_time.month = rtc.month;
	DS1307_time.day = rtc.mday;
	DS1307_time.hour = rtc.hour;
	DS1307_time.min = rtc.min;
	DS1307_time.sec = rtc.sec;
	RTC_DS1307_adjust(pGrpDev2);
	sendBaseOut("\n\rSet local & RTC DS1307 date/time from JSON");

}

// Get list devices, connected to pin
// input: g - group number
void process_getowilist(cJSON *json_data)
{
	uint8_t res;

	OWI_device* owi_tmp = (OWI_device*)pvPortMalloc(sizeof(OWI_device) * mainMAX_GRP_DEVICES);
	sGrpInfo* pGrpInfo = getGrpInfo((uint8_t) cJSON_GetObjectItem(json_data,"g")->valueint);

	taskENTER_CRITICAL();
	{
	res = owi_search_devices(owi_tmp, mainMAX_GRP_DEVICES, pGrpInfo, &pGrpInfo->iDevQty);
	}
	taskEXIT_CRITICAL();
	if (!res) {
	xsprintf((char *) cPassMessage, "{\"ssn\":{\"v\":1,\"ret\":\"getowilist\", \"data\":{ \"num\":%d, \"owiid\":[", pGrpInfo->iDevQty);
	sendBaseOut((char *)cPassMessage);
	char comma = ',';
	uint8_t i;
		for (i = 0; i<pGrpInfo->iDevQty; i++) {
			if (i==(pGrpInfo->iDevQty-1)) {comma = ' ';};
			xsprintf((char *) cPassMessage, "\"%02x%02x%02x%02x%02x%02x%02x%02x\"%c", owi_tmp[i].ucROM[0],owi_tmp[i].ucROM[1],owi_tmp[i].ucROM[2],owi_tmp[i].ucROM[3],owi_tmp[i].ucROM[4],owi_tmp[i].ucROM[5],owi_tmp[i].ucROM[6],owi_tmp[i].ucROM[7], comma);
			sendBaseOut((char *)cPassMessage);
		}
		sendBaseOut("]}}}");
	}
	vPortFree(owi_tmp);
}

/* Load preferences from JSON message to EEPROM */
void process_loadprefs(cJSON *json_data, char * jsonMsg, sGrpInfo*  grpArray[])
{
	uint8_t res;
//  char msg[mainMAX_MSG_LEN];
// first try to apply new preferences - apply_preferences() from main for check it correctness
// if success, then save it into EEPROM and reboot
// if not, then read and apply from EEPROM last correct settings

	vTaskSuspendAll();

	res = apply_preferences(json_data);		// res == pdPASS -> good!
	if (res) {
		uint16_t jsonSize = strlen(jsonMsg);
#ifdef PERSIST_EEPROM_I2C
// save into EEPROM
		res = eeprom_write(&grpArray[0]->GrpDev, EEPROM_ADDRESS, 4, (uint8_t*)jsonMsg, jsonSize);	// res == 1 -> good!
		if (res) {
			uint8_t tmpBuf[2];
			tmpBuf[0]=jsonSize & 0x000FF;
			tmpBuf[1]=(jsonSize & 0x0FF00) >> 8;
			delay_nus(&grpArray[0]->GrpDev, 1000);	// 10ms
			res = eeprom_write(&grpArray[0]->GrpDev, EEPROM_ADDRESS, 0, (uint8_t*)tmpBuf, 2);
			if (res) {
				sendBaseOut("\n\rNew preferences saved into EEPROM. Reboot");
				delay_nus(&grpArray[0]->GrpDev, 10000);	// 10ms
			    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
			    while (1);
			}

		} else {
			sendBaseOut("\n\rError saving preferences into EEPROM");
		}
#endif
#ifdef PERSIST_STM32FLASH
// save into STM32 flash
		uint32_t i;
		uint16_t j;
		uint32_t data;
		if (jsonSize > STM32FLASH_PREF_SIZE)
			res = pdFALSE;
		else {
			flash_unlock();
			// erase necessary flash pages
			for (i = STM32FLASH_BEGIN_ADDR; i < (STM32FLASH_BEGIN_ADDR + STM32FLASH_PREF_SIZE); i+=STM32FLASH_PAGE_SIZE )
			{
				flash_erase_page(i);
			}
			// write data into flash

			i = STM32FLASH_BEGIN_ADDR + 4;
			for (j = 0; j < jsonSize; j+=4)
			{
//				data = (((uint8_t**)jsonMsg)[i]) + (((uint8_t**)jsonMsg)[i+1] >> 8) + (((uint8_t**)jsonMsg)[i+2] >> 16) + (((uint8_t**)jsonMsg)[i+3] >> 24);
				data = (uint32_t)jsonMsg[j+3]   << 24 |
				       (uint32_t)jsonMsg[j+2] << 16 |
				       (uint32_t)jsonMsg[j+1] << 8  |
				       (uint32_t)jsonMsg[j];

				flash_program_word(i, data);
				i += 4;
			}

			data = (uint32_t)jsonSize;
			flash_program_word(STM32FLASH_BEGIN_ADDR, data);

			res = pdTRUE;
			flash_lock();

			sendBaseOut("\n\rNew preferences saved into EEPROM. Reboot");
			delay_nus(&grpArray[0]->GrpDev, 10000);	// 10ms
		    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
		    while (1);

		}
#endif

	} else {
		sendBaseOut("\n\rError loading preferences");
	}
	xTaskResumeAll();
}

char* process_getdevvals(sDevice* devArray[], uint16_t all_devs_counter)
{
	uint16_t j;
	uint8_t ut;
//	uint8_t i;
	uint8_t nNumValTypes;
	const uint16_t nBufSize = 512;
	char* tele_data = pvPortMalloc(nBufSize);
	char* tele_data2;
	uint16_t	nStrCounter = 0;
	uint16_t	uiBufLen;
	uint8_t nExtBufCounter = 1;
	char buf [mainMAX_MSG_LEN];
	int32_t nDevValue;
	uint32_t nDevLastUpdate;
//	char comma = ',';

	taskENTER_CRITICAL();
	{
		const char* pcTeleHeader = "{\"ssn\":{\"v\":1,\"ret\":\"getdevvals\", \"data\":{\"devs\":[";
		uiBufLen = strlen(pcTeleHeader);
		memcpy(tele_data, pcTeleHeader, uiBufLen);
		nStrCounter+=uiBufLen;

		for (j = 0; j < all_devs_counter; j++) {
// select hi rate group:
				ut = devArray[j]->ucType;
				buf[0]=0;
				nNumValTypes = getNumDevValCodes(ut);
				if (nNumValTypes == 1)
				{
					getDevData(devArray[j], 0, &nDevValue, &nDevLastUpdate);
					xsprintf(buf, "{\"dev\":%d, \"n\":1, \"val\":%d, \"updtime\":%ld},",  devArray[j]->nId, nDevValue, nDevLastUpdate );
				}
				if (nNumValTypes == 2)
				{
					int32_t nDevValue2;
					getDevData(devArray[j], 0, &nDevValue, &nDevLastUpdate);
					getDevData(devArray[j], 1, &nDevValue2, &nDevLastUpdate);
					xsprintf(buf, "{\"dev\":%d, \"n\":2, \"i\":0, \"val\":%d, \"updtime\":%ld},{\"dev\":%d, \"n\":2, \"i\":1, \"val\":%d, \"updtime\":%ld},", devArray[j]->nId, nDevValue, nDevLastUpdate,  devArray[j]->nId, nDevValue2, nDevLastUpdate);
				}

				uiBufLen = strlen(buf);
			if (uiBufLen > 0) {
				if ((nStrCounter + uiBufLen) > nBufSize) {
					nExtBufCounter++;
					tele_data2 = pvPortMalloc(nExtBufCounter * nBufSize);
					memcpy(tele_data2, tele_data, strlen(tele_data));
					vPortFree(tele_data);
					tele_data = tele_data2;
				}
					memcpy(&tele_data[nStrCounter], buf, uiBufLen);
					nStrCounter += uiBufLen;
					tele_data[nStrCounter] = 0;
			}
		}
		const char* pcTeleFooter = "{\"dev\":0, \"n\":0, \"val1\":0}]}}}";
		uiBufLen = strlen(pcTeleFooter);
		if ((nStrCounter+uiBufLen) > nBufSize) {
			nExtBufCounter++;
			tele_data2 = pvPortMalloc(nExtBufCounter*nBufSize);
			memcpy(tele_data2,tele_data,strlen(tele_data));
			vPortFree(tele_data);
			tele_data = tele_data2;
		}
			memcpy(&tele_data[nStrCounter], pcTeleFooter, uiBufLen);
			nStrCounter+=uiBufLen;
			tele_data[nStrCounter]=0;
	}
	taskEXIT_CRITICAL();
	return tele_data;
}

void vSendInputMessage (uint8_t version, uint16_t	uiDestObject, uint8_t xMessageType, uint16_t xSourceDev, uint16_t xDestDev, void* pcMessage, uint16_t nSize, uint16_t nCommand)
{
	xInputMessage sInputMessage;

	/* Setup the message we are going to send to the Input queue. */
	sInputMessage.version = version;
	sInputMessage.uiDestObject = uiDestObject;
	sInputMessage.xSourceDevice = xSourceDev;
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
	char* tmpBuf = pvPortMalloc(nDataLength+26);
	xsprintf(tmpBuf, "===ssn1%04x%04x%02x%04x%s%04x", nObjDst, nObjSrc, nMessType, nDataLength, cData, crc16((uint8_t*) cData, nDataLength));
	sendBaseOut(tmpBuf);
	vPortFree(tmpBuf);
}

void sendBaseOut (char *str)
{
	uint8_t uiCnt;
	uint16_t uiBufLen;
	uint32_t xReturn;
	char msg[mainMAX_MSG_LEN];

		for (uiCnt=0; uiCnt<=strlen(str)/mainMAX_MSG_LEN; uiCnt++)
		{
			uiBufLen = strlen(str)-uiCnt*mainMAX_MSG_LEN;
			if (uiBufLen > (mainMAX_MSG_LEN)) {
				uiBufLen = mainMAX_MSG_LEN;
			} else {
				memset (&msg,0,mainMAX_MSG_LEN); // clear buffer
			}

			memcpy(&msg, &str[uiCnt*mainMAX_MSG_LEN], uiBufLen);
			if (xBaseOutQueue) {
				xReturn = xQueueSend( xBaseOutQueue, msg, 0 );
			}
		}

	( void ) xReturn;
}

void vCommandSelector(sSSNCommand* xSSNCommand)
{
	char msg[30];
	if (xSSNCommand) {
		switch (xSSNCommand->nCmd) {
		case mainCOMMAND_COMMITED: {
			xsprintf(msg, "\r\nCOMMITED: %d ", xSSNCommand->nCmdID);
			sendBaseOut(msg);

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
		case mainCOMMAND_DISMODEMCHARGE: {
			// disable charging of battery
#ifdef  M_GSM
			sendBaseOut("\r\nOVER-VOLTAGE, disable charging");
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
			sendBaseOut("\r\nLOW-VOLTAGE, enable charging");
			sGSMDevice* pGSMDev = (sGSMDevice*) xSSNCommand->pcData;
			if (pGSMDev) {
				gpio_set(pGSMDev->uiPortChgCtrl, 1 << pGSMDev->uiPortChgCtrl);
			}
#endif
			break;
		}
		case mainCOMMAND_GETDEVVALS: {
			char * pcTeleData;
			pcTeleData = process_getdevvals(devArray, all_devs_counter);
			if (pcTeleData) {
				vSendInputMessage(1, 0, mainTELEMETRY_MESSAGE, 0,
						xSSNCommand->uiDevDest, (void*) pcTeleData, strlen(pcTeleData), 0);
			}
			break;
		}
		}
		xsprintf(msg, "\r\nFreeHeapSize:=%d **********", xPortGetFreeHeapSize());
		sendBaseOut(msg);

	}
}

void main_reboot()
{
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
	char msg [100];

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
			xsprintf(msg, "\r\nError processing action rules! AID=%d", nActID);
			sendBaseOut(msg);
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
	char cBuffer [200];
	xsprintf( cBuffer, "\r\n***%s switched out, %s switched in, tick count = %u",
			pcTaskGetTaskName((TaskHandle_t) poldt),
			pcTaskGetTaskName((TaskHandle_t) pnewt),
			xTickCount );
	sendBaseOut(cBuffer);
}

void log_event2 (char c, void* pt)
{
	char cBuffer [200];
	xsprintf( cBuffer, "\r\n***%c: task %s delay",c, pcTaskGetTaskName((TaskHandle_t) pt));
	sendBaseOut(cBuffer);
}

/* log action event into temporary buffer and send it if buffer full */
void	logAction(uint16_t nActId, uint16_t nDevId, uint32_t nValue)
{
	uint16_t i;
	uint16_t j;
	char* pBuffer;
	char tmpBuffer[20];

	if (logActCounter < mainLOG_ACTIONS_SIZE)
	{
		logActCounter++;

	} else {
		// send log to logger object
		pBuffer = pvPortMalloc(60*mainLOG_ACTIONS_SIZE);	// allocate memory for all data JSON elements
		if (!pBuffer) {
			sendBaseOut("\r\nError allocate memory for log data!");
			return;
		}

		xsprintf( tmpBuffer, "{\"log\":[");
		memcpy(&pBuffer[0], &tmpBuffer, strlen(tmpBuffer));
		j = strlen(tmpBuffer);

		for (i = 0; i < mainLOG_ACTIONS_SIZE; i++)
		{
			xsprintf( tmpBuffer, "{\"a\":%d,\"d\":%d,\"v\":%d,\"t\":%d},", logActionsArray[logActCounter].nActId, logActionsArray[logActCounter].nDevId, logActionsArray[logActCounter].nValue, logActionsArray[logActCounter].nTimestamp);
			memcpy(&pBuffer[j], &tmpBuffer, strlen(tmpBuffer));
			j += strlen(tmpBuffer);
		}
		xsprintf( tmpBuffer, "{}]}");
		memcpy(&pBuffer[j], &tmpBuffer, strlen(tmpBuffer));
		pBuffer[j+strlen(tmpBuffer)] = 0;

		// send log data to input buffer for routing to destination:
		vSendInputMessage(1, 0, mainLOG_MESSAGE, 0,	0, (void*) pBuffer, strlen(pBuffer), 0);
		logActCounter = 0;
	}

	logActionsArray[logActCounter].nActId = nActId;
	logActionsArray[logActCounter].nDevId = nDevId;
	logActionsArray[logActCounter].nValue = nValue;
	logActionsArray[logActCounter].nTimestamp = rtc_get_counter_val();

}
