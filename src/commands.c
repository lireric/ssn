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
//#ifdef  M_GSM
extern	void prvStartGSMTask( void *pvParameters );
//#endif

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
	//sendBaseOut("\n\rSet local & RTC DS1307 date/time from JSON");
	debugMsg("\n\rSet local & RTC DS1307 date/time from JSON");

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

/* Load preferences from INI message to EEPROM or FLASH */
// ... to do

// Handler for processing INI preferences format
uint32_t process_loadprefs_ini_handler(char* sSection, char* sName, char* sValue, sIniHandlerData* pIniHandlerData, int* pnSectionNo)
{
	int32_t nRes = pdPASS; //pdFAIL;
#ifdef  M_GSM
	sGSMDevice* ptGSMDev = NULL;
#endif
	#define MATCH(s, n) strcmp(sSection, s) == 0 && strcmp(sName, n) == 0

	if (MATCH("app", "logobj")) {
		setLog_Object(conv2d(sValue));		// store object-logger
	} else if (strcmp(sSection, "grp") == 0) {
		// check for new group
		if (strcmp(pIniHandlerData->sLastSection, sSection) != 0) {
			if (grp_counter++ > mainMAX_DEV_GROUPS) {
							return pdFAIL;
						}
			grpArray[grp_counter] = (sGrpInfo*) pvPortMalloc(sizeof(sGrpInfo));
			if (!grpArray[grp_counter]) {
				return pdFAIL;
			}
			grpArray[grp_counter]->uiObj = getMC_Object();
			grpArray[grp_counter]->iDevQty = 0;
		}
		if (strcmp(sName, "grpnum") == 0) {
			grpArray[grp_counter]->uiGroup = conv2d(sValue);

		} else if (strcmp(sName, "grpport") == 0) {
			grpArray[grp_counter]->GrpDev.pPort = get_port_by_name(sValue);

		} else if (strcmp(sName, "grptimer") == 0) {
			grpArray[grp_counter]->GrpDev.pTimer = get_port_by_name(sValue);

		} else if (strcmp(sName, "grppin1") == 0) {
			grpArray[grp_counter]->GrpDev.ucPin = conv2d(sValue);

		} else if (strcmp(sName, "grppin2") == 0) {
			grpArray[grp_counter]->GrpDev.ucPin2 = conv2d(sValue);
		}
	} else if (strcmp(sSection, "dev") == 0) {
		// check for new device
		if (strcmp(pIniHandlerData->sLastSection, sSection) != 0) {

			if (all_devs_counter++ > mainMAX_ALL_DEVICES) {
				return pdFAIL;
			}
			devArray[all_devs_counter] = (sDevice*) pvPortMalloc(sizeof(sDevice));

			if (!devArray[all_devs_counter]) {
				return pdFAIL;
			}
			devArray[all_devs_counter]->nDevObj = getMC_Object();
		}
		if (strcmp(sName, "grp") == 0) {
			devArray[all_devs_counter]->pGroup = getGroupByID(conv2d(sValue));

			if (!devArray[all_devs_counter]->pGroup) {
				return pdFAIL;
			}
			devArray[all_devs_counter]->pGroup->iDevQty++;

		} else if (strcmp(sName, "devid") == 0) {
			devArray[all_devs_counter]->nId = conv2d(sValue);

		} else if (strcmp(sName, "devtype") == 0) {
			devArray[all_devs_counter]->ucType = conv2d(sValue);

			if (devArray[all_devs_counter]->ucType == device_TYPE_DS18B20) {
				devArray[all_devs_counter]->pDevStruct = NULL;

			} else if (devArray[all_devs_counter]->ucType == device_TYPE_DHT22) {
#ifdef  M_DHT
				devArray[all_devs_counter]->pDevStruct = (void*) dht_device_init(&devArray[all_devs_counter]->pGroup->GrpDev);
#endif
			} else if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_OD) {
				gpio_set_mode(devArray[all_devs_counter]->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_OPENDRAIN, 1 << devArray[all_devs_counter]->pGroup->GrpDev.ucPin);

			} else if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_PP) {
				gpio_set_mode(devArray[all_devs_counter]->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_PUSHPULL, 1 << devArray[all_devs_counter]->pGroup->GrpDev.ucPin);

			} else if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_INPUT) {
				gpio_set_mode(devArray[all_devs_counter]->pGroup->GrpDev.pPort, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, 1 << devArray[all_devs_counter]->pGroup->GrpDev.ucPin);

			} else if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_AO) {
				// to do:

			} else if (devArray[all_devs_counter]->ucType == device_TYPE_PWM) {

				devArray[all_devs_counter]->pDevStruct = pvPortMalloc(sizeof(sPWM_data_t));

			} else if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_AI) {

				devArray[all_devs_counter]->pDevStruct = pvPortMalloc(sizeof(sADC_data_t));
					if (!devArray[all_devs_counter]->pDevStruct) {
						return pdFAIL;
					}

			} else if (devArray[all_devs_counter]->ucType == device_TYPE_GSM) {
#ifdef  M_GSM
				devArray[all_devs_counter]->pDevStruct = pvPortMalloc(sizeof(sGSMDevice));
				if (!devArray[all_devs_counter]->pDevStruct) {
					return pdFAIL;
				}
				//gsm_preinit_ini ((sGSMDevice*) devArray[all_devs_counter]->pDevStruct, xBaseOutQueue);
				nRes = vMainStartGSMTask((void*)devArray[all_devs_counter]);
				if (!nRes)
					return pdFAIL;
#endif

			}
		} else if (devArray[all_devs_counter]->ucType == device_TYPE_MEMORY) {
			// memory element pseudo device
			if (strcmp(sName, "e") == 0) {
				devArray[all_devs_counter]->pGroup->iDevQty = conv2d(sValue);
				// allocate memory for memory elements array:
				devArray[all_devs_counter]->pDevStruct = (void*)pvPortMalloc(sizeof(uint32_t)*devArray[all_devs_counter]->pGroup->iDevQty);
				if (!devArray[all_devs_counter]->pDevStruct) {
					return pdFAIL;
				}
			}
		} else if (devArray[all_devs_counter]->ucType == device_TYPE_PWM) {
			if (strcmp(sName, "ch") == 0) {
				if (!pwm_setup(devArray[all_devs_counter], sValue)) {
					return pdFAIL;
				}
			} else if (strcmp(sName, "freq") == 0) {
				((sPWM_data_t*)devArray[all_devs_counter]->pDevStruct)->nFreq = conv2d(sValue);
			}

		} else if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_AI) {
			if (strcmp(sName, "ch") == 0) {
				if (!adc_setup(devArray[all_devs_counter], sValue)) {
					return pdFAIL;
				}
			}
		} else if (strcmp(sName, "romid") == 0) {
#ifdef  M_DS18B20
			devArray[all_devs_counter]->pDevStruct = (void*) ds18b20_init(devArray[all_devs_counter]->pGroup, sValue);
#endif
		}
#ifdef  M_GSM
		else if (strcmp(sName, "PortDTR") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPortDTR = get_port_by_name(sValue);
		} else if (strcmp(sName, "PinDTR") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPinDTR = conv2d(sValue);
		} else if (strcmp(sName, "PortPwrKey") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPortPwrKey = get_port_by_name(sValue);
		} else if (strcmp(sName, "PinPwrKey") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPortPwrKey = conv2d(sValue);
		} else if (strcmp(sName, "PortChgCtrl") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPortChgCtrl = get_port_by_name(sValue);
		} else if (strcmp(sName, "PinChgCtrl") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPinChgCtrl = conv2d(sValue);
		} else if (strcmp(sName, "PortRTS") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPortRTS = get_port_by_name(sValue);
		} else if (strcmp(sName, "PinRTS") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiPinRTS = conv2d(sValue);
		} else if (strcmp(sName, "USART") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiUSART = conv2d(sValue);
		} else if (strcmp(sName, "acc") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiSSNAcc = conv2d(sValue);
		} else if (strcmp(sName, "v") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->chip, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "APN") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cAPN, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "SrvAddr") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cSrvAddr, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "SrvPort") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			ptGSMDev->uiSrvPort = conv2d(sValue);
		} else if (strcmp(sName, "SMSNumber") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cSMSNumber, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "PriDNS") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cPriDNS, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "SecDNS") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cSecDNS, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "GUser") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cGPRSUserID, sValue, strlen(sValue));
		} else if (strcmp(sName, "GUserPswd") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cGPRSUserPassw, sValue, strlen(sValue)+1);
		} else if (strcmp(sName, "AESKey") == 0) {
			ptGSMDev = (sGSMDevice*)devArray[all_devs_counter]->pDevStruct;
			strncpy0(ptGSMDev->cAESKey, sValue, strlen(sValue)+1);
		}
#endif

	} else if (strcmp(sSection, "route") == 0) {
		if (pIniHandlerData->pnPrevSectionNo != *pnSectionNo) {
			route_counter++;
		}
		if (strcmp(sName, "obj") == 0) {
			routeArray[route_counter].nDestObj = conv2d(sValue);
		} else if (strcmp(sName, "if") == 0) {
			routeArray[route_counter].xDestType = conv2d(sValue);
		}

	} else if (strcmp(sSection, "logic") == 0) {
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



// write preferences string into EEPROM or Flash
uint32_t storePreferences(char* sBuf, uint16_t nBufSize)
{
	int32_t nRes = pdPASS; //pdFAIL;

#ifdef PERSIST_EEPROM_I2C
// save into EEPROM
	nRes = eeprom_write(&grpArray[0]->GrpDev, EEPROM_ADDRESS, 4, (uint8_t*)sBuf, nBufSize);	// res == 1 -> good!
		if (nRes) {
			uint8_t tmpBuf[2];
			tmpBuf[0]=nBufSize & 0x000FF;
			tmpBuf[1]=(nBufSize & 0x0FF00) >> 8;
			delay_nus(&grpArray[0]->GrpDev, 1000);	// 10ms
			nRes = eeprom_write(&grpArray[0]->GrpDev, EEPROM_ADDRESS, 0, (uint8_t*)tmpBuf, 2);
			if (nRes) {
				//sendBaseOut("\n\rNew preferences saved into EEPROM. Reboot");
				debugMsg("\n\rNew preferences saved into EEPROM. Reboot");
				delay_nus(&grpArray[0]->GrpDev, 10000);	// 10ms
//			    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
//			    while (1);
			}

		} else {
			//sendBaseOut("\n\rError saving preferences into EEPROM");
			debugMsg("\n\rError saving preferences into EEPROM");
		}
#endif
#ifdef PERSIST_STM32FLASH
// save into STM32 flash
		uint32_t i;
		uint16_t j;
		uint32_t data;
		if (nBufSize > STM32FLASH_PREF_SIZE)
			nRes = pdFALSE;
		else {
			flash_unlock();
			// erase necessary flash pages
			for (i = STM32FLASH_BEGIN_ADDR; i < (STM32FLASH_BEGIN_ADDR + STM32FLASH_PREF_SIZE); i+=STM32FLASH_PAGE_SIZE )
			{
				flash_erase_page(i);
			}
			// write data into flash

			i = STM32FLASH_BEGIN_ADDR + 4;
			for (j = 0; j < nBufSize; j+=4)
			{
				data = (uint32_t)sBuf[j+3]   << 24 |
				       (uint32_t)sBuf[j+2] << 16 |
				       (uint32_t)sBuf[j+1] << 8  |
				       (uint32_t)sBuf[j];

				flash_program_word(i, data);
				i += 4;
			}

			data = (uint32_t)nBufSize;
			flash_program_word(STM32FLASH_BEGIN_ADDR, data);

			nRes = pdTRUE;
			flash_lock();

//			sendBaseOut("\n\rNew preferences saved into EEPROM. Reboot");
			delay_nus(&grpArray[0]->GrpDev, 10000);	// 10ms
//		    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
//		    while (1);

		}
#endif
	return nRes;
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
		res = storePreferences(jsonMsg, jsonSize);
	    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ; // reboot


	} else {
		//sendBaseOut("\r\nError loading preferences");
		debugMsg("\r\nError loading preferences");
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
//		const char* pcTeleHeader = "{\"ssn\":{\"v\":1,\"ret\":\"getdevvals\", \"data\":{\"devs\":[";
		xsprintf(buf, "{\"ssn\":{\"v\":1,\"obj\":%d,\"ret\":\"getdevvals\", \"data\":{\"devs\":[",  getMC_Object() );
		uiBufLen = strlen(buf);
		memcpy(tele_data, buf, uiBufLen);
		nStrCounter+=uiBufLen;

		for (j = 0; j <= all_devs_counter; j++) {
// select hi rate group:
				ut = devArray[j]->ucType;
				buf[0]=0;
				if ((ut == device_TYPE_MEMORY) || (ut == device_TYPE_BB1BIT_IO_AI)) {
					nNumValTypes = devArray[j]->pGroup->iDevQty;
				} else {
					nNumValTypes = getNumDevValCodes(ut);
				}
				for (uint8_t k=0; k<nNumValTypes; k++) {
					getDevData(devArray[j], k, &nDevValue, &nDevLastUpdate);
					xsprintf(buf, "{\"dev\":%d, \"n\":%d, \"i\":%d, \"val\":%d, \"updtime\":%ld},",  devArray[j]->nId, nNumValTypes, k, nDevValue, nDevLastUpdate );
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

		}
		const char* pcTeleFooter = "{}]}}}";
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
	char* tmpBuf = pvPortMalloc(nDataLength+30);
	xsprintf(tmpBuf, "\n===ssn1%04x%04x%02x%04x%s%04x", nObjDst, nObjSrc, nMessType, nDataLength, cData, crc16((uint8_t*) cData, nDataLength));
	sendBaseOut(tmpBuf);
	vPortFree(tmpBuf);
}

void debugMsg (char *str)
{
//	uint32_t xReturn;
	xQueueSend( xLogOutQueue, str, 0);
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
				//sendBaseOut(msg);
				debugMsg(msg);

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
				//sendBaseOut("\r\nOVER-VOLTAGE, disable charging");
				debugMsg("\r\nOVER-VOLTAGE, disable charging");
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
				debugMsg("\r\nLOW-VOLTAGE, enable charging");
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
					vSendInputMessage(1, xSSNCommand->uiObjSrc, mainTELEMETRY_MESSAGE, getMC_Object(), 0,
							xSSNCommand->uiDevDest, (void*) pcTeleData, strlen(pcTeleData), 0);
				}
				break;
			}
		}
		xsprintf(msg, "\r\nFreeHeapSize:=%d **********", xPortGetFreeHeapSize());
		debugMsg(msg);
		//sendBaseOut(msg);

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
			debugMsg(msg);
			//sendBaseOut(msg);
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
	//sendBaseOut(cBuffer);
	debugMsg(cBuffer);
}

void log_event2 (char c, void* pt)
{
	char cBuffer [200];
	xsprintf( cBuffer, "\r\n***%c: task %s delay",c, pcTaskGetTaskName((TaskHandle_t) pt));
	//sendBaseOut(cBuffer);
	debugMsg(cBuffer);
}

/* log action event into temporary buffer and send it if buffer full */
void	logAction(uint16_t nActId, uint16_t nDevId, uint8_t nDevCmd, uint32_t nValue)
{
	uint16_t i;
	uint16_t j;
	char* pBuffer;
	char tmpBuffer[100];
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
				//sendBaseOut("\r\nError allocate memory for log data!");
				debugMsg("\r\nError allocate memory for log data!");
				return;
			}

			xsprintf( tmpBuffer, "{\"log\":[");
			memcpy(&pBuffer[0], &tmpBuffer, strlen(tmpBuffer));
			j = strlen(tmpBuffer);

			for (i = 0; i < logActCounter; i++)
			{
				xsprintf( tmpBuffer, "{\"a\":%d,\"d\":%d,\"c\":%d,\"v\":%d,\"t\":%d},", logActionsArray[i].nActId,
						logActionsArray[i].nDevId, logActionsArray[i].nDevCmd, logActionsArray[i].nValue,
						logActionsArray[i].nTimestamp);
				memcpy(&pBuffer[j], &tmpBuffer, strlen(tmpBuffer));
				j += strlen(tmpBuffer);
			}
			xsprintf( tmpBuffer, "{}]}");
			memcpy(&pBuffer[j], &tmpBuffer, strlen(tmpBuffer));
			pBuffer[j+strlen(tmpBuffer)] = 0;

			// send log data to input buffer for routing to destination:
			vSendInputMessage(1, 0, mainLOG_MESSAGE, getMC_Object(), 0,	0, (void*) pBuffer, strlen(pBuffer), 0);
			nlogActLastUpdate = nCurTimestamp;
			logActCounter = 0;
	}

}
