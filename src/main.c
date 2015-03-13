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

#include "utils.h"
#include "task.h"
#include "commands.h"
#include "../inc/ssn.h"

#ifdef  M_LCD
#include "lcd.h"
#endif

#ifdef  WATCHDOG
	#include <libopencm3/stm32/iwdg.h>
#endif

#define NVIC_CCR ((volatile unsigned long *)(0xE000ED14))

/* Global variables 			========================================== */

unsigned long ulIdleCycleCount;

/* Variables used by the trace hook macros. */
void* xTaskThatWasRunning = NULL;
void* xBaseOutTaskHnd;

#ifdef  M_GSM
	#include "gsm.h"
#endif


sGrpInfo *grpArray[mainMAX_DEV_GROUPS];
uint8_t 	grp_counter = 0;

sDevice *devArray[mainMAX_ALL_DEVICES];
uint16_t 	all_devs_counter = 0;

sAction *actArray[mainMAX_ACTIONS];
uint16_t 	act_counter = 0;

slogAction	logActionsArray[mainLOG_ACTIONS_SIZE];
uint16_t 	logActCounter = 0;

long lComState = pdFAIL;

uint16_t	uiMC_SSNObject;	// local controller's object number
uint16_t	uiLog_Object;	// object - receiver log messages

sRoute routeArray[mainMAX_ROUTES];
uint8_t 	route_counter = 0;

xJSON xJSONBuffer;

const char* cSSNSTART = "===ssn1";
sSSNPDU xSSNPDU;

int uiDevCounter=0;
static uint32_t uiMainTick = 0;

xTaskHandle pCheckSensorHRTaskHnd;
xTaskHandle pCheckSensorMRTaskHnd;

/* Private function prototypes ========================================== */
void RCC_Configuration(void);
void NVIC_Configuration(void);
void nmi_handler(void);
void hardfault_handler(void);
int  main(void);
void myDelay(unsigned long delay );
void Clk_Init (void);
sGrpInfo* getGrpInfo(unsigned char ucGrpNum);

static void vMainTimerFunctionInterval1(void* pParam);
static void vMainTimerFunctionInterval2(void* pParam);

#ifdef DEBUG_S
static void prvDebugStatTask(void* pParam);
char cDebugBuf[2000];
#endif

void vMainStartTimer(sAction* pAct);

/* The main input processing task */
static void prvInputTask( void *pvParameters );

/* The poll sensors tasks. */
static void prvCheckSensorHRTask( void *pvParameters );
static void prvCheckSensorMRTask( void *pvParameters );

/* The process sensors data task. */
static void prvProcSensorTask( void *pvParameters );

static void prvUSARTEchoTask( void *pvParameters );

static uint32_t	getMainTimerTick();
static void prvCronFunc( void *pvParameters );

/* The basic output message processing task. */
static void prvBaseOutTask( void *pvParameters );

void vApplicationIdleHook( void );

// Define the vector table
unsigned int * myvectors[4]
   __attribute__ ((section("vectors")))= {
   	(unsigned int *)	0x20010000,			// stack pointer
   	(unsigned int *) 	main,				// code entry point
   	(unsigned int *)	nmi_handler,		// NMI handler (not really)
   	(unsigned int *)	hardfault_handler	// hard fault handler (let's hope not)
};


/*-----------------------------------------------------------*/

uint16_t getMC_Object(void)
{
	return uiMC_SSNObject;
}

void setMC_Object(uint16_t uiObj)
{
	uiMC_SSNObject = uiObj;
}

void print_debug_FROM_ISR (const char *str)
{
	portBASE_TYPE xReturn;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	  xHigherPriorityTaskWoken = pdFALSE;

	if( lComState == pdPASS )
		xReturn = xQueueSendFromISR( xBaseOutQueue, str, &xHigherPriorityTaskWoken);
	  if( xHigherPriorityTaskWoken == pdTRUE )
	  {
		  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE );
	  }
	( void ) xReturn;
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOs clock */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_GPIOG);


	/* Enable clocks for special ports  */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SDIO);
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_FSMC);

	rcc_periph_clock_enable(RCC_ADC3); // to do: make it pref from TS_ADC...

}



/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{

	*NVIC_CCR = *NVIC_CCR | 0x200;

	scb_set_priority_grouping(0);
}

sGrpInfo* getGrpInfo(unsigned char ucGrpNum)
{
	unsigned char i;
	for(i=0; i < mainMAX_DEV_GROUPS; i++)
	{
		if (grpArray[i]->uiGroup == ucGrpNum) return grpArray[i];
	}
	return (sGrpInfo*)0;
}

//////////////////////////////////////////////////////////////////////////
/*************************************************************************
 * Function Name: main
 * Parameters: none
 * Return: Int32U
 *
 * Description: The main subroutine
 *
 *************************************************************************/
//////////////////////////////////////////////////////////////////////////
int main(void)
{
#ifdef DEBUG
//  debug();
#endif

  ulIdleCycleCount = 0UL;
  portBASE_TYPE xReturn;
  uint8_t res;
  char msg[mainMAX_MSG_LEN];
  setMC_Object(MC_OBJECT);

#ifdef  WATCHDOG
  iwdg_start();
  iwdg_set_period_ms(WATCHDOG_PERIOD);	// set watchdog period
#endif

  xTaskHandle pTmpTask;

  xBaseOutQueue = xQueueCreate( mainDEBUG_QUEUE_SIZE, mainMAX_MSG_LEN);

/* System Clocks Configuration **********************************************/
	RCC_Configuration();
/* NVIC Configuration *******************************************************/
	NVIC_Configuration();

#ifdef  M_LCD
	xReturn = lcd_init();
#endif

  initialise_rtc();

/* Initialize base COM */
#ifdef  M_USART
	  	lComState = lCOMPortInit( mainBASECOM, mainBAUD_RATE, mainBASECOM_Priority );
#endif
/*
 * to do: make init other UARTs if needed...
 */
		sendBaseOut("\r\n\r\nStart SSN");

// ------------------------------------------------------------------------------
	  	sGrpInfo * pGrp;
		sDevice* pVirtualDev;	// abstract device for timer actions cashing

		grpArray[grp_counter] = (sGrpInfo*) pvPortMalloc(sizeof(sGrpInfo));
		pGrp = grpArray[grp_counter];

		pVirtualDev = (sDevice*) pvPortMalloc(sizeof(sDevice));
		pVirtualDev->nId = 0;
		pVirtualDev->nDevObj = 0;
		pVirtualDev->pGroup = pGrp;
		pVirtualDev->ucType = device_TYPE_VIRTUAL;
		devArray[all_devs_counter++] = pVirtualDev;

#if defined (M_RTC_DS1307) || defined (PERSIST_EEPROM_I2C)
		sGrpDev * pGrpDev;
		pGrpDev = &pGrp->GrpDev;
		// software i2c group index = 0 (grp_counter = 0)
		// This group combine RTC_DS1307 and EEPROM
		pGrp->uiGroup = SOFTI2C_1_GRP;
//		pGrp->ucDevRate = device_RATE_NONE;
		pGrpDev->pPort = SOFTI2C_1_PORT;
		pGrpDev->ucPin = SOFTI2C_1_PIN_SDA;
		pGrpDev->ucPin2 = SOFTI2C_1_PIN_SCL;
		pGrpDev->pTimer = SOFTI2C_TIMER_1;
		grp_counter++;
		owi_device_init_i2c(pGrpDev);
#endif

#ifdef M_RTC_DS1307
		// load RTC from RTC_DS1307:
		if ( RTC_DS1307_now(pGrpDev) )
			{
				RTC_t rtc;
				rtc.year = DS1307_time.year + 2000;
				rtc.month = DS1307_time.month;
				rtc.mday = DS1307_time.day;
				rtc.hour = DS1307_time.hour;
				rtc.min = DS1307_time.min;
				rtc.sec = DS1307_time.sec;
				rtc_settime(&rtc);
				sendBaseOut("\n\rSet date/time from RTC DS1307");
			}
			else {
				sendBaseOut("\n\rRTC DS1307 not responding");
			};
#endif


// first 2 byte in EEPROM consist size of JSON structure with SSN hardware and logic preferences, started from 5 byte

// Read size of recorded JSON:
#ifdef PERSIST_EEPROM_I2C
	res = eeprom_read(pGrpDev, EEPROM_ADDRESS, 0, (uint8_t*)&Rx1Buffer, 2);
#endif
#ifdef PERSIST_STM32FLASH
	memcpy(&Rx1Buffer, (void*)STM32FLASH_BEGIN_ADDR, 2);
	res = pdTRUE;
#endif

	if (res) {
		uint16_t jsonSize = Rx1Buffer[0] + (Rx1Buffer[1] << 8);
		xJSONBuffer.nBufSize = jsonSize;
		if (jsonSize < xPortGetFreeHeapSize())
			xJSONBuffer.buffer = pvPortMalloc(jsonSize);

		if (xJSONBuffer.buffer) {
#ifdef PERSIST_EEPROM_I2C
			res = eeprom_read(pGrpDev, EEPROM_ADDRESS, 4, (uint8_t*) xJSONBuffer.buffer, jsonSize);
#endif
#ifdef PERSIST_STM32FLASH
			// simply copy into buffer data from flash memory where stored preferences
			memcpy((void*)xJSONBuffer.buffer, (void*)(STM32FLASH_BEGIN_ADDR+4), jsonSize);
			res = pdTRUE;
#endif
			if (res) {
				xJSONBuffer.state = JSON_STATE_READY;
				xJSONBuffer.counter = jsonSize;
#ifdef PERSIST_STM32FLASH
				sendBaseOut("\n\rConfiguration loaded from FLASH");
#else
				sendBaseOut("\n\rConfiguration loaded from EEPROM");
#endif

				cJSON *json_root, *json_ssn;
				json_root = cJSON_Parse(xJSONBuffer.buffer);
				if (!json_root) {
					res = 1;
				} else {
					json_ssn = cJSON_GetObjectItem(json_root, "ssn");
					if (!json_ssn) {
						xsprintf(( portCHAR *) msg, "\n\rJSON error before: [%s]\n\r", cJSON_GetErrorPtr());
						sendBaseOut((char *) &msg);
					} else {
						int version =
								cJSON_GetObjectItem(json_ssn, "v")->valueint;
						if (version == 1) {
							cJSON *json_data = cJSON_GetObjectItem(json_ssn, "data");
							if (!json_data) {
								sendBaseOut("\n\rError JSON data");
							} else {
								res = apply_preferences(json_data);	// res == 0 -> good!
							}
						}
					}
				}

				if (!res) {
					sendBaseOut("\n\rError of apply the configuration");
				}
			}

		vPortFree(xJSONBuffer.buffer);
		} else {
			sendBaseOut("\n\rError JSON size info (EEPROM)");
		}
	} else {
		//error
		sendBaseOut("\n\rError reading configuration from EEPROM");
	}


// ------------------------------------------------------------------------------
	xReturn = xTaskCreate( prvBaseOutTask, ( char * ) "BaseOutTask", 200, NULL, mainBASEOUT_TASK_PRIORITY, &pTmpTask );
	xBaseOutTaskHnd = (void*) pTmpTask;

	xInputQueue = xQueueCreate( mainINPUT_QUEUE_SIZE, sizeof( xInputMessage ) );
	xSensorsQueue = xQueueCreate( mainSENSORS_QUEUE_SIZE, sizeof(void*) );

	xReturn = xTaskCreate( prvUSARTEchoTask, ( char * ) "Echo", 200, NULL, mainECHO_TASK_PRIORITY, &pTmpTask );

	xTimerHandle xTimer;
(void) xTimer;
	xTimer = xTimerCreate("Cron", mainCronRate, pdTRUE, NULL, prvCronFunc);

#ifdef DEBUG_S
	xReturn = xTaskCreate( prvDebugStatTask, ( char * ) "Debug_S", 210, NULL, tskIDLE_PRIORITY + 2, &pTmpTask );
#endif

	xReturn = xTaskCreate( prvInputTask, ( char * ) "InputTask", 300, NULL, mainINPUT_TASK_PRIORITY, &pTmpTask );

	xReturn = xTaskCreate( prvCheckSensorMRTask, ( char * ) "CheckSensorMRTask", 200, devArray, mainCHECK_SENSOR_MR_TASK_PRIORITY, &pTmpTask );
	pCheckSensorMRTaskHnd = pTmpTask;


	xReturn = xTaskCreate( prvCheckSensorHRTask, ( char * ) "CheckSensorHRTask", configMINIMAL_STACK_SIZE, devArray, mainCHECK_SENSOR_HR_TASK_PRIORITY, &pTmpTask );
	pCheckSensorHRTaskHnd = pTmpTask;



	xReturn = xTaskCreate( prvProcSensorTask, ( char * ) "ProcSensorTask", 400, devArray, mainPROC_SENSOR_TASK_PRIORITY, &pTmpTask );


	( void ) xReturn;

/* Start the scheduler. */

  vTaskStartScheduler();
  xReturn = uxTaskGetStackHighWaterMark(NULL);

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );

}


/* The main user input processing task */
static void prvInputTask( void *pvParameters )
{
	/* Just to avoid compiler warnings. */
	(void) pvParameters;
	xInputMessage xInputMessage;
	char cPassMessage[mainMAX_MSG_LEN];
	char * jsonMsg;
	int32_t xReturn;
	sSSNCommand xSSNCommand;
	int8_t uiDestInterface;

	while (1) {
		/* Wait for a message from Input queue */
		while ( xQueueReceive(xInputQueue, &xInputMessage, portMAX_DELAY) != pdPASS);

		/* Check the version of message */
		if (xInputMessage.version == 1) {
			uiDestInterface = getObjRoute(xInputMessage.uiDestObject);

			/* route message if needed */
			if (xInputMessage.uiDestObject != getMC_Object()) {
				switch (uiDestInterface) {
					case main_IF_PROGRAM:
					{
						// automatic routing:
						if (xInputMessage.xMessageType == mainTELEMETRY_MESSAGE) {
							// for mainTELEMETRY_MESSAGE default interface main_IF_GSM
							uiDestInterface = main_IF_GSM;
						} else
						if (xInputMessage.xMessageType == mainLOG_MESSAGE) {

							uiDestInterface = getObjRoute(getLog_Object());
							xInputMessage.xDestDevice = getLog_Object();
						} else
							if (getDeviceObjByID (xInputMessage.uiDestObject) == getMC_Object())
								goto processLocalMessages;
						// else goto UART..
						// to do: process other message types

					}
					case main_IF_UART1:
					case main_IF_UART2:
					case main_IF_UART3:
					case main_IF_UART4:
					case main_IF_UART5:
					{
						vSendSSNPacket (xInputMessage.xDestDevice, getMC_Object(), xInputMessage.xMessageType, xInputMessage.pcMessage);
						break;
					}
#ifdef  M_GSM
					case main_IF_GSM:
					{
						/* to do: make function for route message via gsm interface */
						break;
					}
#endif
				/* to do: add other node types...  */
				}
				vPortFree(xInputMessage.pcMessage);
			} else {
processLocalMessages:
			// process message for our object (node)
			/* Check the message type */
			switch (xInputMessage.xMessageType) {
				case mainLOG_MESSAGE:
				{
					switch (uiDestInterface) {
#ifdef  M_GSM
						case main_IF_GSM:
						{
							if (xInputMessage.pcMessage) {
								xReturn = gsm_send_request(xInputMessage.pcMessage, xInputMessage.nMsgSize);
								if (!xReturn) {
									sendBaseOut("\r\nGPRS LOG request failed");
								}
							}
							break;
						}
#endif
						case main_IF_PROGRAM:
						case main_IF_UART1:
						case main_IF_UART2:
						case main_IF_UART3:
						case main_IF_UART4:
						case main_IF_UART5:
						{
							vSendSSNPacket (xInputMessage.xDestDevice, getMC_Object(), xInputMessage.xMessageType, xInputMessage.pcMessage);
							break;
						}
						/* to do: add other node types...  */
					}
					vPortFree(xInputMessage.pcMessage);
					break;
				}
				case mainTELEMETRY_MESSAGE: {
					switch (uiDestInterface) {
#ifdef  M_GSM
						case main_IF_GSM:
						{
							if (xInputMessage.pcMessage) {
								xReturn = gsm_send_request(xInputMessage.pcMessage, xInputMessage.nMsgSize);
								if (!xReturn) {
									sendBaseOut("\r\nGPRS TELEMETRY request failed");
								}
							}
							break;
						}
#endif
						case main_IF_PROGRAM:
						case main_IF_UART1:
						case main_IF_UART2:
						case main_IF_UART3:
						case main_IF_UART4:
						case main_IF_UART5:
						{
								sendBaseOut((char *) xInputMessage.pcMessage);
								break;
						}
						/* to do: add other node types...  */
					}
					vPortFree(xInputMessage.pcMessage);
					break;
				}
				case mainINFO_MESSAGE: {
					switch (uiDestInterface) {
						case main_IF_PROGRAM:
						case main_IF_UART1:
						case main_IF_UART2:
						case main_IF_UART3:
						case main_IF_UART4:
						case main_IF_UART5:
						{
							sendBaseOut("\r\nInfo msg: ");
							sendBaseOut((char *) xInputMessage.pcMessage);
							break;
						}
#ifdef  M_GSM
						case main_IF_GSM:
						{
							/* to do: make function for route message via gsm interface */
							break;
						}
#endif
					/* to do: add other node types...  */
					}
					if (xInputMessage.pcMessage) {
						vPortFree(xInputMessage.pcMessage);
					}
					break;
				}
				case mainGSM_MESSAGE_IN: {
					sendBaseOut("\r\nGSM_IN msg: ");
					sendBaseOut((char *) xInputMessage.pcMessage);
					xReturn = fillCommandStruct((char *) xInputMessage.pcMessage, &xSSNCommand);
					// process commands if they exists
					if (xReturn) {
						xSSNCommand.uiDevDest = xInputMessage.xSourceDevice;
						// EXTERNAL COMMANDS =======================================================
						vCommandSelector(&xSSNCommand);
#ifdef  M_GSM

						if ((xSSNCommand.nCmd != mainCOMMAND_COMMITED) && (xSSNCommand.nCmd != mainCOMMAND_NONE) ) {
							// try to notify web service about command executing
							gsm_send_ws_get_cmd (gsmWSCmdCommitCommand, xSSNCommand.nCmdID, 0);
						}
						if (xSSNCommand.nCmdsLeft > 0) {
							// get next commands
							gsm_send_ws_get_cmd (gsmWSCmdGetCommands, 0, 0);
						}
#endif

					}
					vPortFree(xInputMessage.pcMessage);
					break;
				}
#ifdef  M_GSM
				case mainGSM_MESSAGE_OUT: {
					gsm_send_ws_get_cmd (xInputMessage.nCommand, 0, 0);
					break;
				}
#endif
				case mainCOMMAND_MESSAGE: {
					// EXTERNAL COMMANDS =======================================================
					xSSNCommand.nCmd = xInputMessage.nCommand;
					xSSNCommand.nCmdID = 0;
					xSSNCommand.nCmdsLeft = 0;
					xSSNCommand.pcData = xInputMessage.pcMessage;
					xSSNCommand.uiDevDest = xInputMessage.xDestDevice;
					vCommandSelector(&xSSNCommand);
					vPortFree(xInputMessage.pcMessage);
					break;
				}
				case mainJSON_MESSAGE: {
// if JSON message ...
/* processing JSON message
 common part JSON structure:
 {	"ssn": {"v":"version_number", "cmd":"command", "data": { ... } } }
*/
					xsprintf(cPassMessage, "\r\nFreeHeap=%d, StHWM=%d =PRCJSON===", xPortGetFreeHeapSize(), uxTaskGetStackHighWaterMark(0));
					sendBaseOut(cPassMessage);

								jsonMsg = (char*) xInputMessage.pcMessage;
								cJSON *json_root;
								cJSON *json_ssn;
								json_root=cJSON_Parse(jsonMsg);
								if (!json_root) {
									xsprintf(cPassMessage, "\n\rError before: [%s]\n\r",cJSON_GetErrorPtr());
									sendBaseOut(cPassMessage);
								}
								else
								{
									json_ssn=cJSON_GetObjectItem(json_root,"ssn");
									if (!json_ssn) {
										xsprintf(cPassMessage, "\n\rJSON error before: [%s]\n\r",cJSON_GetErrorPtr());
										sendBaseOut(cPassMessage);
									} else {
// now processing command:
										int version = cJSON_GetObjectItem(json_ssn,"v")->valueint;
										if (version==1) {
											char* cmd = (char*)cJSON_GetObjectItem(json_ssn,"cmd")->valuestring;
											uint16_t nObjDest = cJSON_GetObjectItem(json_ssn,"obj")->valueint;

											xsprintf(cPassMessage, "\n\rProcess JSON command: %s", cmd );
											sendBaseOut(cPassMessage);

											cJSON *json_data = cJSON_GetObjectItem(json_ssn,"data");

											if (strcmp(cmd, "setdatetime") == 0) {
												process_setdatetime(json_data);
											}
											if (strcmp(cmd, "getowilist") == 0) {
												process_getowilist(json_data);
											}
											if (strcmp(cmd, "loadprefs") == 0) {
												process_loadprefs(json_data, jsonMsg, grpArray);
											}
											if (strcmp(cmd, "updateaction") == 0) {
												UpdateActionJSON(json_data);
											}
											if (strcmp(cmd, "sdv") == 0) {
												if (json_data) {
													uint32_t dev_val = (uint32_t) cJSON_GetObjectItem(json_data,"aval")->valueint;
													uint16_t dev_id = (uint16_t) cJSON_GetObjectItem(json_data,"adev")->valueint;
													uint8_t  dev_cmd = (uint8_t) cJSON_GetObjectItem(json_data,"acmd")->valueint;
													xsprintf(cPassMessage, "\n\rSet dev[%d]=(%d)%d", dev_id, dev_cmd, dev_val);
													sendBaseOut(cPassMessage);
													setDevValueByID(dev_val, dev_cmd, dev_id, eElmInteger);
													// to do: process string data type
												}
											}
#ifdef  M_GSM
											if (strcmp(cmd, "gsmcmd") == 0) {
//												gsm_send_command((char*)cJSON_GetObjectItem(json_ssn,"data")->valuestring);
												if (json_data) {
													gsm_send_command((char*)json_data->valuestring);
												}
											}
#endif
// process command without parameters. Send they into Input queue for processing
											uint32_t nCmd;
											void* pData = 0;
											if (strcmp(cmd, "reboot") == 0) {
												nCmd = mainCOMMAND_REBOOT;
												vSendInputMessage (1, nObjDest, mainCOMMAND_MESSAGE, xInputMessage.xSourceDevice, main_IF_PROGRAM, pData, 0, nCmd);
											}
											if (strcmp(cmd, "getdevvals") == 0) {
												nCmd = mainCOMMAND_GETDEVVALS;
												vSendInputMessage (1, nObjDest, mainCOMMAND_MESSAGE, 0, xInputMessage.xSourceDevice, pData, 0, nCmd);
											}

										}	else 	{
											sendBaseOut("Wrong SSN message version");
										}
									cJSON_Delete(json_root);
								} // ssn
								} // root
								vPortFree(xInputMessage.pcMessage);
								break;
							}
			} // switch
			} // if - routing
	} // if - message version
		char msg[30];
		xsprintf(msg, "\r\nFreeHeap:=%d =INPUTTSK===", xPortGetFreeHeapSize());
		sendBaseOut(msg);
		taskYIELD();
	} // while
}


/* The poll Middle Rate sensors task. */
static void prvCheckSensorMRTask( void *pvParameters )
{
/* Just to avoid compiler warnings. */
	(void) pvParameters;
	portTickType xLastWakeTime;
	uint16_t j;
	uint8_t ut;
	uint32_t res;
	(void) res;

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
#ifdef  M_DS18B20
		uint8_t flag18b20_conv = 0;
#endif
		RTC_t rtc;
		rtc_gettime(&rtc);

					for (j = 0; j < all_devs_counter; j++) {
// select middle rate group:
//							if (devArray[j]->pGroup->ucDevRate == device_RATE_MID) {
							ut = devArray[j]->ucType;
							switch (ut) {

#ifdef  M_DS18B20
							case device_TYPE_DS18B20:
// check ds18b20 conversation status on this line (it's make only for one (first) device - if set for this group, than skip):
								if (flag18b20_conv != devArray[j]->pGroup->uiGroup) {
								    res = ds_start_convert_all(devArray[j]->pGroup, devArray, j);     // start temperature measuring
								    flag18b20_conv = devArray[j]->pGroup->uiGroup;
								}
// send message into Sensors queue for next processing:
								res = xQueueSend(xSensorsQueue, (void*)&devArray[j], 0);
								if (!res) {
									sendBaseOut("\r\nError sending into Sensors Queue!");
								}
								break;
#endif

#ifdef  M_DHT
							case device_TYPE_DHT22:
								taskENTER_CRITICAL(); {
								res = dht_get_data (&devArray[j]->pGroup->GrpDev, (DHT_data_t*) devArray[j]->pDevStruct);
								} taskEXIT_CRITICAL();
								res = xQueueSend(xSensorsQueue, (void*)&devArray[j], 0);
								break;
#endif
//							case device_TYPE_BB1BIT_IO_INPUT:
//								break;
//							case device_TYPE_BB1BIT_IO_PP:
//							case device_TYPE_BB1BIT_IO_OD:
//								res = (int8_t) bb_read_wire_data_bit(&devArray[j]->pGroup->GrpDev);
//								if (res != devArray[j]->nLastPinValue) {
//									devArray[j]->nLastPinValue = res;
//									xQueueSend(xSensorsQueue, (void*)&devArray[j], portMAX_DELAY);
//								}
//								break;
							}
//					}
		}
		taskYIELD();
		vTaskDelayUntil(&xLastWakeTime, mainSensorRateMR);
	}
}

/* The process High Rate sensors data task. */
static void prvCheckSensorHRTask( void *pvParameters )
{
	( void ) pvParameters;
	uint8_t res, ut;
	uint16_t j;
	while (1) {
					for (j = 0; j < all_devs_counter; j++) {
// select hi rate group:
//							if (devArray[j]->pGroup->ucDevRate == device_RATE_HI) {
							ut = devArray[j]->ucType;
							switch (ut) {
							case device_TYPE_BB1BIT_IO_INPUT:
								res = (int8_t) bb_read_wire_data_bit(&devArray[j]->pGroup->GrpDev);
								if (res != devArray[j]->nLastPinValue) {
									devArray[j]->nLastPinValue = res;
									xQueueSend(xSensorsQueue, &devArray[j], 0);
								}
								break;
							case device_TYPE_BB1BIT_IO_PP:
							case device_TYPE_BB1BIT_IO_OD:
								break;
							}
//					}
					}
		taskYIELD();
		vTaskDelay(mainSensorRateHR );
	}
}

static uint32_t	getMainTimerTick()
{
	return uiMainTick;
}

/* The cron function (invoke by timer) */
static void prvCronFunc( void *pvParameters )
{
	( void ) pvParameters;
	uint16_t i;
	RTC_t rtc;
	uint32_t res;
	sAction* pAct;

		uiMainTick++;
		rtc_gettime(&rtc);

		// check SSN protocol USART timeout
		if ((uiMainTick > (xSSNPDU.uiLastTick + SSN_TIMEOUT/10)) && ((xSSNPDU.state == SSN_STATE_LOADING) || (xSSNPDU.state == SSN_STATE_DATA))) {
			xSSNPDU.state = SSN_STATE_ERROR;
			vPortFree(xSSNPDU.buffer);
			sendBaseOut("\r\nSSN receive data timeout error!");
		}

#ifdef  WATCHDOG
		iwdg_reset();	// reset watchdog timer
#endif

		sDevice* pDev = devArray[0]; // get virtual time device
		// scan thru actions with time events:
		for (i=0; i<pDev->nActionsCashSize; i++) {

			pAct = &((sAction*)pDev->pActionsCash)[i];
			if (pAct) {
				res = calcAndDoAction (pAct);
				if (!res) {
					// to do: process error
				}
			}
		}
}

/* The process sensors data task. */
static void prvProcSensorTask( void *pvParameters )
{
	sDevice* dev;
	char msg[mainMAX_MSG_LEN];
	uint8_t i;
	uint32_t res;
	( void ) pvParameters;

	while (1)
	{
		/* Wait for a message from Sensors queue */
		while ( xQueueReceive(xSensorsQueue, &dev, portMAX_DELAY)	!= pdPASS) ;
		if (dev) {
			xsprintf(msg, "\n\rDevice [%d]: ", dev->nId );
			sendBaseOut(msg);
				switch (dev->ucType) {
#ifdef  M_DS18B20
				case device_TYPE_DS18B20:
					xsprintf(msg, "temperature = %d lastupdate = %ld", ((ds18b20_device*) dev->pDevStruct)->iDevValue, ((ds18b20_device*) dev->pDevStruct)->uiLastUpdate );
					break;
#endif
#ifdef  M_DHT
				case device_TYPE_DHT22:
					xsprintf(msg, "temperature = %d humidity = %d lastupdate = %ld", ((DHT_data_t*) dev->pDevStruct)->temperature, ((DHT_data_t*) dev->pDevStruct)->humidity, ((DHT_data_t*) dev->pDevStruct)->uiLastUpdate );
					break;
#endif
				}
				sendBaseOut(msg);

				// scan thru actions linked with this device:
				for (i=0; i<dev->nActionsCashSize; i++) {
					sAction* pAct = ((sAction**)dev->pActionsCash)[i];
					if (pAct) {
						res = calcAndDoAction (pAct);
						if (!res) {
							// to do: process error
						}
					}
				}
		}
			taskYIELD();
	}
}


// ----------------------------------------------

static void prvUSARTEchoTask( void *pvParameters )
{
signed char cChar = 0;
char cTmpBuf[mainMAX_MSG_LEN];
uint8_t nScanCnt;
uint16_t calcCRC;

	/* Just to avoid compiler warnings. */
	(void) pvParameters;
	nScanCnt = 0;
	xSSNPDU.buffer = 0;
	xSSNPDU.state = SSN_STATE_INIT;

	while (1) {
		/* Block to wait for a character to be received on COM0. */
#ifdef  M_USART
		xSerialGetChar( mainBASECOM, &cChar, portMAX_DELAY);
#endif
#ifndef  M_USART
		vTaskDelay(portMAX_DELAY);	// wait forever
#endif
		xSSNPDU.uiLastTick = getMainTimerTick();	// check timeouts in Cron task!

		switch (xSSNPDU.state) {
		case SSN_STATE_INIT:
		case SSN_STATE_ERROR:
		case SSN_STATE_READY:
// check for SSN packet start
			if (cChar == cSSNSTART[nScanCnt]) {
				nScanCnt++;
			} else {
				nScanCnt = 0;
			}
				if (nScanCnt == (strlen(cSSNSTART))) {
					xSSNPDU.state = SSN_STATE_LOADING;
					xSSNPDU.counter = 0;
				}
			break;
		case SSN_STATE_LOADING:
			nScanCnt = 0;
			xSSNPDU.cSSNBuffer[xSSNPDU.counter++] = cChar;
			if (xSSNPDU.counter == 4) {
				// get object destination
				cTmpBuf[3] = xSSNPDU.cSSNBuffer[3];
				cTmpBuf[2] = xSSNPDU.cSSNBuffer[2];
				cTmpBuf[1] = xSSNPDU.cSSNBuffer[1];
				cTmpBuf[0] = xSSNPDU.cSSNBuffer[0];
				cTmpBuf[4] = 0;
				xSSNPDU.obj_dest = convHex2d(cTmpBuf);
				if (xSSNPDU.obj_dest != getMC_Object()) {
					// if other destination object cancel loading and begin waiting next packet
					xSSNPDU.state = SSN_STATE_INIT;
					nScanCnt = 0;
				}
			} else {
				if (xSSNPDU.counter == 8) {
					// get source object
					cTmpBuf[3] = xSSNPDU.cSSNBuffer[7];
					cTmpBuf[2] = xSSNPDU.cSSNBuffer[6];
					cTmpBuf[1] = xSSNPDU.cSSNBuffer[5];
					cTmpBuf[0] = xSSNPDU.cSSNBuffer[4];
					cTmpBuf[4] = 0;
					xSSNPDU.obj_src = convHex2d(cTmpBuf);
				} else {
					if (xSSNPDU.counter == 10) {
						// get message type
						cTmpBuf[1] = xSSNPDU.cSSNBuffer[9];
						cTmpBuf[0] = xSSNPDU.cSSNBuffer[8];
						cTmpBuf[2] = 0;
						xSSNPDU.message_type = convHex2d(cTmpBuf);
					} else {
						if (xSSNPDU.counter == 14) {
							// get message length
							cTmpBuf[3] = xSSNPDU.cSSNBuffer[13];
							cTmpBuf[2] = xSSNPDU.cSSNBuffer[12];
							cTmpBuf[1] = xSSNPDU.cSSNBuffer[11];
							cTmpBuf[0] = xSSNPDU.cSSNBuffer[10];
							cTmpBuf[4] = 0;
							xSSNPDU.nDataSize = convHex2d(cTmpBuf);
							if (xSSNPDU.nDataSize < mainMINMEMORYALLOCATE) {
								xSSNPDU.buffer = pvPortMalloc(mainMINMEMORYALLOCATE);
							} else {
								xSSNPDU.buffer = pvPortMalloc(xSSNPDU.nDataSize);
							}
							if (!xSSNPDU.buffer) {
								xSSNPDU.state = SSN_STATE_ERROR;
								sendBaseOut("\r\nSSN buffer allocation error!");
								nScanCnt = 0;
								break;
							} else {
								xSSNPDU.state = SSN_STATE_DATA;
								xSSNPDU.counter = 0;
							}
						}
			}}}
			break;
		case SSN_STATE_DATA:
			if (xSSNPDU.counter < xSSNPDU.nDataSize) {
				xSSNPDU.buffer[xSSNPDU.counter++] = cChar;
			} else {
				if ((xSSNPDU.counter >= xSSNPDU.nDataSize) && (xSSNPDU.counter < xSSNPDU.nDataSize + 4)) {
					// get CRC
					cTmpBuf[xSSNPDU.counter++ - xSSNPDU.nDataSize] = cChar;
					if (xSSNPDU.counter >= xSSNPDU.nDataSize + 4) {
							// all data loaded, check CRC
							cTmpBuf[4] = 0;
							xSSNPDU.crc16 = convHex2d(cTmpBuf);
							calcCRC = crc16((uint8_t*) xSSNPDU.buffer, xSSNPDU.nDataSize);
							if (xSSNPDU.crc16 == calcCRC) {
								xSSNPDU.state = SSN_STATE_READY;
								nScanCnt = 0;
								/* Write the received message to common Input queue. */
								// to do: xSSNPDU.obj_src
								vSendInputMessage (1, xSSNPDU.obj_dest, xSSNPDU.message_type, 0, 0, (void*) xSSNPDU.buffer, xSSNPDU.nDataSize,0);
							} else {
								xSSNPDU.state = SSN_STATE_ERROR;
								xsprintf(cTmpBuf, "\r\nSSN data CRC error! (calc=%04x, msg=%04x)", calcCRC, xSSNPDU.crc16);
								sendBaseOut(cTmpBuf);
	//							sendBaseOut(xSSNPDU.buffer);
								vPortFree(xSSNPDU.buffer);
								nScanCnt = 0;
							}
					}
			}
			}
			break;
		default:
			/* Write the received character back to COM0. */
//			xSerialPutChar( mainBASECOM, cChar, 0);
			break;
		}
		taskYIELD();
	}
}


/*-----------------------------------------------------------*/

static void prvBaseOutTask( void *pvParameters )
{
	( void ) pvParameters;
	  portBASE_TYPE xReturn;
	char buf[mainMAX_MSG_LEN+1];
	memset (&buf,0,mainMAX_MSG_LEN+1);
	while (1) {
		/* Wait for a message from Debug queue */
		while( (xQueueReceive( xBaseOutQueue, &buf, portMAX_DELAY ) != pdPASS) && (xSSNPDU.state != SSN_STATE_LOADING) && (xSSNPDU.state != SSN_STATE_DATA));
		if (lComState == pdPASS) {
#ifdef  M_USART
			xReturn = lSerialPutString ( mainBASECOM, buf, strlen(buf) );
#endif
// to do: make alternatives output interfaces
		}
		taskYIELD();
	}
	( void ) xReturn;
}

#ifdef  M_GSM

static void prvStartGSMTask( void *pvParameters )
{
	  int32_t xReturn;
	  xReturn = gsm_init((sDevice*)pvParameters); // if Ok return pdPASS
	  (void) xReturn;
	  vTaskDelete( NULL );

}
#endif

// ************************************************************************************************

void nmi_handler(void)
{
	return ;
}

void hardfault_handler(void)
{
	return ;
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

void vApplicationIdleHook( void )
{
	  ulIdleCycleCount++;
}

int32_t apply_preferences(cJSON *json_data) {
	uint8_t i,j;
	cJSON *json_app = cJSON_GetObjectItem(json_data, "app");	// get application preferences
	cJSON *json_groups = cJSON_GetObjectItem(json_data, "grp");	// get array of groups
	cJSON *grp_devs;
	cJSON *json_routes = cJSON_GetObjectItem(json_data, "routes");	// get array of routes
	cJSON *json_actions = cJSON_GetObjectItem(json_data, "logic");	// get array of actions
	int32_t res = pdFAIL;
	(void) res;

	if (json_app)
	{
//		cJSON *json_tmp;
		uint16_t	nLogObj = (uint16_t) cJSON_GetObjectItem(json_app, "logobj")->valueint;
		setLog_Object(nLogObj);		// store object-logger
	}

	if (json_groups)
	{
		for (i=0;i<cJSON_GetArraySize(json_groups);i++)
		{
		cJSON *grpitem=cJSON_GetArrayItem(json_groups,i);
		if (grpitem) {
			grpArray[grp_counter] = (sGrpInfo*) pvPortMalloc(sizeof(sGrpInfo));
			if (!grpArray[grp_counter]) break;

			grpArray[grp_counter]->uiGroup = (uint8_t) cJSON_GetObjectItem(grpitem, "grpnum")->valueint;
//			grpArray[grp_counter]->ucDevRate = (uint8_t) cJSON_GetObjectItem(grpitem, "devrate")->valueint;
			grpArray[grp_counter]->iDevQty = (uint8_t) cJSON_GetObjectItem(grpitem, "devsqty")->valueint;
			grpArray[grp_counter]->GrpDev.pPort = get_port_by_name((char*) cJSON_GetObjectItem(grpitem, "grpport")->valuestring);
			grpArray[grp_counter]->GrpDev.pTimer = get_port_by_name((char*) cJSON_GetObjectItem(grpitem, "grptimer")->valuestring);
			grpArray[grp_counter]->GrpDev.ucPin = (uint8_t) cJSON_GetObjectItem(grpitem, "grppin1")->valueint;
			grpArray[grp_counter]->GrpDev.ucPin2 = (uint8_t) cJSON_GetObjectItem(grpitem, "grppin2")->valueint;
			grp_devs = cJSON_GetObjectItem(grpitem, "devs");	// get array of devices

			if (grp_devs) {
			for (j=0;j<cJSON_GetArraySize(grp_devs);j++)
			{
				cJSON *devitem=cJSON_GetArrayItem(grp_devs,j);
				if (devitem) {
					devArray[all_devs_counter] = pvPortMalloc(sizeof(sDevice));
					devArray[all_devs_counter]->nId = (uint16_t) cJSON_GetObjectItem(devitem, "devid")->valueint;
					devArray[all_devs_counter]->ucType = (uint8_t) cJSON_GetObjectItem(devitem, "devtype")->valueint;
					devArray[all_devs_counter]->pGroup = grpArray[grp_counter];

					if (devArray[all_devs_counter]->ucType == device_TYPE_DS18B20) {
#ifdef  M_DS18B20
						devArray[all_devs_counter]->pDevStruct = (void*) ds18b20_init(grpArray[grp_counter], (char *) cJSON_GetObjectItem(devitem, "romid")->valuestring);
#endif
					}
					if (devArray[all_devs_counter]->ucType == device_TYPE_DHT22) {
#ifdef  M_DHT
						devArray[all_devs_counter]->pDevStruct = (void*) dht_device_init(&grpArray[grp_counter]->GrpDev);
#endif
					}
					if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_OD) {
						gpio_set_mode(devArray[all_devs_counter]->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_50_MHZ,
								GPIO_CNF_OUTPUT_OPENDRAIN, 1 << devArray[all_devs_counter]->pGroup->GrpDev.ucPin);
					}
					if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_PP) {
						gpio_set_mode(devArray[all_devs_counter]->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_50_MHZ,
								GPIO_CNF_OUTPUT_PUSHPULL, 1 << devArray[all_devs_counter]->pGroup->GrpDev.ucPin);
					}
					if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_INPUT) {
						gpio_set_mode(devArray[all_devs_counter]->pGroup->GrpDev.pPort, GPIO_MODE_INPUT,
								GPIO_CNF_INPUT_FLOAT, 1 << devArray[all_devs_counter]->pGroup->GrpDev.ucPin);
					}
					if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_AI) {
						gpio_set_mode(devArray[all_devs_counter]->pGroup->GrpDev.pPort, GPIO_MODE_INPUT,
								GPIO_CNF_INPUT_ANALOG, 1 << devArray[all_devs_counter]->pGroup->GrpDev.ucPin);
						// to do:
					}
					if (devArray[all_devs_counter]->ucType == device_TYPE_BB1BIT_IO_AO) {
						// to do:
					}
#ifdef  M_GSM
					if (devArray[all_devs_counter]->ucType == device_TYPE_GSM) {
						devArray[all_devs_counter]->pDevStruct = (void*) gsm_preinit (devitem, xBaseOutQueue);
						xTaskHandle pTmpTask;
						res = xTaskCreate( prvStartGSMTask, ( char * ) "StartGSMTask", configMINIMAL_STACK_SIZE, devArray[all_devs_counter], gsmGSM_TASK_START_PRIORITY, &pTmpTask );

						if (!res) return pdFAIL;
					}
#endif
				}
				all_devs_counter++;
			}
			}
			grp_counter++;
		}
	}
	cJSON_Delete(json_groups);
	}

// ************************************************************ json_routes
	uint16_t	nDestObj;		// destination object
	uint8_t 	xDestType;		// interface type to route

	if (json_routes) {
		for (i=0; i<cJSON_GetArraySize(json_routes); i++)
		{
			cJSON *routeitem=cJSON_GetArrayItem(json_routes,i);
			if (routeitem) {
				nDestObj  = (uint16_t) cJSON_GetObjectItem(routeitem, "obj")->valueint;
				xDestType = (uint16_t) cJSON_GetObjectItem(routeitem, "if")->valueint;
				res = setObjRoute(nDestObj, xDestType);
				// to do: process error
			}
		}
		cJSON_Delete(json_routes);
	}

// ************************************************************ json_actions
	if (json_actions) {
		uint16_t unActCount = cJSON_GetArraySize(json_actions);

	for (i=0;i<unActCount;i++)
	{
		cJSON *devactitem=cJSON_GetArrayItem(json_actions,i);
		if (devactitem) {
			UpdateActionJSON(devactitem);
		}
	}
	cJSON_Delete(json_actions);
	res = refreshActions2DeviceCash();
	}

	return pdPASS;
}

static void vMainTimerFunctionInterval1(void* pParam)
{
	sEvtElm* pEvtElm = (sEvtElm*) pvTimerGetTimerID( pParam );
	sActElm* pActElm;
	sAction* pAction;
	char cPassMessage[ mainMAX_MSG_LEN ];
	// search all Action elements:
	while (pEvtElm)
	{
			if (pEvtElm->nElmType == eElmAction) {
				pActElm = (sActElm*)pEvtElm->nElmData1;
				if (pActElm) {
					pAction = pActElm->pAction;
					if (!(pAction->nFlags & devACTION_FLAG_NOLOG)) {
						if (pActElm->nElmDataType == eElmString)
							xsprintf( cPassMessage, "\n\rTimer set dev[%d] = (%d, %s) ", pActElm->nDevId, pActElm->nActCmdIn, (char*)pActElm->nElmDataIn);
						else
							xsprintf( cPassMessage, "\n\rTimer set dev[%d] = (%d, %d) ", pActElm->nDevId, pActElm->nActCmdIn, pActElm->nElmDataIn);
						sendBaseOut(cPassMessage);
					}
					setDevValueByID(pActElm->nElmDataIn, pActElm->nActCmdIn, pActElm->nDevId, pActElm->nElmDataType);

					if (pAction->xActTimerOneShot) {
						xTimerStart(pAction->xActTimerOneShot, 0);
					}

				}
			}
			pEvtElm = (sEvtElm*)pEvtElm->pPrevElm;
	}
}

static void vMainTimerFunctionInterval2(void* pParam)
{
	sEvtElm* pEvtElm = (sEvtElm*) pvTimerGetTimerID( pParam );
	sActElm* pActElm;
	sAction* pAction;
	char cPassMessage[ mainMAX_MSG_LEN ];
	// search all Action elements:
	while (pEvtElm)
	{
			if (pEvtElm->nElmType == eElmAction) {
				pActElm = (sActElm*)pEvtElm->nElmData1;
				if (pActElm) {
					pAction = pActElm->pAction;

					if (!(pAction->nFlags && devACTION_FLAG_NOLOG)) {

						if (pActElm->nElmDataType == eElmString)
							xsprintf(cPassMessage, "\n\rOne shot timer set dev[%d] = (%d, %s) ", pActElm->nDevId, pActElm->nActCmdOut, (char*)pActElm->nElmDataOut);
						else
							xsprintf(cPassMessage, "\n\rOne shot timer set dev[%d] = (%d, %d) ", pActElm->nDevId, pActElm->nActCmdOut, pActElm->nElmDataOut);

						sendBaseOut(cPassMessage);
					}
					setDevValueByID(pActElm->nElmDataOut, pActElm->nActCmdOut, pActElm->nDevId, pActElm->nElmDataType);
				}
			}
			pEvtElm = (sEvtElm*)pEvtElm->pPrevElm;
	}
}

void vMainStartTimer(sAction* pAct)
{
	if (pAct) {
		if (pAct->xActTimer)
			xTimerStart(pAct->xActTimer, 0);
	}

}

xTimerHandle mainTimerCreate(char* pcTimerName, uint32_t nPeriod, uint32_t isPeriodic, sEvtElm* pEvtElm)
{
	return xTimerCreate(pcTimerName, nPeriod / portTICK_RATE_MS, isPeriodic, pEvtElm, vMainTimerFunctionInterval1);
}

xTimerHandle mainTimerCreateOneShot(char* pcTimerName, uint32_t nPeriod, sEvtElm* pEvtElm)
{
	return xTimerCreate(pcTimerName, nPeriod / portTICK_RATE_MS, pdFALSE, pEvtElm, vMainTimerFunctionInterval2);
}

#ifdef DEBUG_S
static void prvDebugStatTask(void* pParam)
{
	(void) pParam;
	while (1) {

			sendBaseOut("\r\n\r\nTaskStat info: \r\nName	State	Priority	Stack	Num\r\n");
			vTaskList((char*)&cDebugBuf);
			sendBaseOut(cDebugBuf);
			taskYIELD();

			vTaskDelay(mainDEBUG_STAT_RATE);
		}
}
#endif
