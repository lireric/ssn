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
 * commands.h
 *
 *  Created on: 16 окт. 2014 г.
 *      Author: eric
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "device.h"
#include "cJSON.h"
#include "ssn.h"

extern sRoute 		routeArray[];
extern uint8_t 		route_counter;
extern slogAction	logActionsArray[];
extern uint16_t 	logActCounter;
extern uint16_t		uiLog_Object;
extern sDevice* 	devArray[];
extern uint16_t 	all_devs_counter;
extern uint32_t		nlogActLastUpdate;
extern sDevice* 	uiMemoryDevsArray[];
extern uint8_t		mem_devs_counter;

//extern xTaskHandle pCheckSensorHRTaskHnd;
//extern xTaskHandle pCheckSensorMRTaskHnd;
//#ifdef  M_GSM
//extern	 static void prvStartGSMTask( void *pvParameters );
//#endif

uint32_t process_loadprefs_ini_handler(char* sSection, char* sName, char* sValue, sIniHandlerData* pIniHandlerData, int* pnSectionNo);
void 	process_setdatetime(cJSON *json_data);
void 	process_getowilist(cJSON *json_data);
void 	process_loadprefs(cJSON *json_data, char * jsonMsg, sGrpInfo* grpArray[]);
char* 	process_getdevvals(sDevice* devArray[], uint16_t all_devs_counter);
void 	vSendInputMessage (uint8_t version, uint16_t	uiDestObject, uint8_t xMessageType, uint16_t uiSrcObject, uint16_t xSourceDev, uint16_t xDestDev, void* pcMessage, uint16_t nSize, uint16_t nCommand);
void 	vSendSSNPacket (uint16_t nObjDst, uint16_t nObjSrc, uint8_t nMessType, char* cData);
void 	sendBaseOut (char *str);
void 	vCommandSelector(sSSNCommand* xSSNCommand);
void 	main_reboot();
int32_t	UpdateAction(char* pcDevAction);
void	UpdateActionJSON(cJSON *devactitem);
uint8_t getObjRoute(uint16_t	nDestObj);
uint32_t setObjRoute(uint16_t	nDestObj, uint8_t nRoute);
uint32_t delObjRoute(uint16_t	nDestObj);
uint16_t 	getLog_Object(void);
void 		setLog_Object(uint16_t uiObj);

void	logAction(uint16_t nActId, uint16_t nDevId, uint8_t nDevCmd, uint32_t nValue);

//void log_event (TaskHandle_t xTH);
void 	log_event (void* poldt, void* pnewt, uint32_t xTickCount);
void 	log_event2 (char c, void* pt);
uint16_t getCommandsByName(const char* sCmd);
uint32_t storePreferences(char* sBuf, uint16_t nBufSize);
int32_t storeMemDevs();
int32_t restoreMemDevs();

#endif /* COMMANDS_H_ */
