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
#include "ssnpdu.h"
#include "commands.h"
#include "../inc/ssn.h"
#include "../inc/ini.h"
#include "processor.h"
#include <stdlib.h>
#include "dev_memory.h"

const int  __attribute__((used)) uxTopUsedPriority = configMAX_PRIORITIES;

#ifdef  M_LCD
#include "lcd.h"
#endif

#ifdef  WATCHDOG
	#include <libopencm3/stm32/iwdg.h>
#endif

#define NVIC_CCR ((volatile unsigned long *)(0xE000ED14))

#define SSN_VERSION "2019-04-10.1"

/* Global variables 			========================================== */

unsigned long ulIdleCycleCount;

/* Variables used by the trace hook macros. */
void* xTaskThatWasRunning = NULL;
void* xBaseOutTaskHnd;

#ifdef  M_GSM
	#include "gsm.h"
#endif

#ifdef  M_STEPMOTOR

#endif

#ifdef  M_MHZ19
#include "../inc/mh-z19b.h"
#endif

#ifdef  M_ETHERNET
#include "ethernet.h"
#include "dhcp.h"
#include "w5500/w5500.h"
#include "w5500/socket.h"
uint8 nCurrentInternetState = 0;

uint8 mac[6]=ETHERNET_LOCAL_MAC;
uint8 lip[4]=ETHERNET_SSN_SIP;
uint8 sub[4]=ETHERNET_SSN_SN;
uint8 gw[4]=ETHERNET_SSN_GW;
uint16 port=ETHERNET_SSN_SERVER_PORT;

#endif

//sGrpInfo *grpArray[mainMAX_DEV_GROUPS];
sGrpInfo	**grpArray;
uint8_t		grp_counter = -1;

//sDevice *devArray[mainMAX_ALL_DEVICES];
sDevice 	**devArray;
uint16_t 	all_devs_counter = -1;

//sAction *actArray[mainMAX_ACTIONS];
sAction 	**actArray;
uint16_t 	act_counter = 0;

slogAction	logActionsArray[mainLOG_ACTIONS_SIZE];
uint16_t 	logActCounter = 0;
uint32_t 	nlogActLastUpdate = 0;

long lComState = pdFAIL;

uint16_t	uiMC_SSNObject;	// local controller's object number
uint16_t	uiLog_Object;	// object - receiver log messages

sRoute 		routeArray[mainMAX_ROUTES];
uint8_t 	route_counter = 0;

//sDevice 	*uiMemoryDevsArray[mainMEMORY_DEV_MAX_QTY];
//uint8_t 	mem_devs_counter = 0;

uint32_t 	uiLastSaveMemoryTick = 0; // last memory device update timestamp (for regular saving to persistent store)

uint32_t 	uiLastHeartBeatTick = 0;
uint32_t 	uiLastHeartBeatTS = 0;

sPrefsBuffer xPrefsBuffer;

sSSNPDU xSSNPDU;

int uiDevCounter=0;
static uint32_t uiMainTick = 0;

xTaskHandle pCheckSensorHRTaskHnd;
xTaskHandle pCheckSensorMRTaskHnd;

xTaskHandle pDevInitTask;

/* Private function prototypes ========================================== */
void RCC_Configuration(void);
void NVIC_Configuration(void);
//void nmi_handler(void);
//void hardfault_handler(void);
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

static void prvEthernetTCPTask( void *pvParameters );
static void prvEthernetUDPTask( void *pvParameters );

static void prvUSARTEchoTask( void *pvParameters );

static uint32_t	getMainTimerTick();
static void prvCronFunc( void *pvParameters );

/* The basic output message processing task. */
static void prvBaseOutTask( void *pvParameters );
static void prvLogOutTask( void *pvParameters );

#ifdef  M_GSM
//static void prvStartGSMTask( void *pvParameters );
#endif

#ifdef  M_STEPMOTOR
//static void 	StepMotorTimerFunction(void* pParam);
#endif

/*
void vApplicationIdleHook( void );

// Define the vector table
unsigned int * myvectors[4]
   __attribute__ ((section("vectors")))= {
   	(unsigned int *)	0x20010000,			// stack pointer
   	(unsigned int *) 	main,				// code entry point
   	(unsigned int *)	nmi_handler,		// NMI handler (not really)
   	(unsigned int *)	hardfault_handler	// hard fault handler (let's hope not)
};

*/



static int handler(void* user, const char* section, const char* name,
                   const char* value, int* pnSectionNo)
{
    sIniHandlerData* pIniHandlerData = (sIniHandlerData*)user;
//	char msg[mainMAX_MSG_LEN];
	uint32_t nRes = pdPASS;

    #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
	switch (pIniHandlerData->iniSSNCommand->nCmd) {
		case 0:
		{
			if (MATCH("ssn", "v") && conv2d(value) != 1) {
//				xsprintf(msg, "\r\nWrong SSN protocol version: %s ", value);
//				debugMsg(msg);
				xprintfMsg("\r\nWrong SSN protocol version: %s ", value);
				nRes =  pdFAIL;
			} else if (MATCH("ssn", "obj")) {
				pIniHandlerData->iniSSNCommand->uiDevDest = conv2d(value);
			} else if (MATCH("ssn", "cmd")) {
				pIniHandlerData->iniSSNCommand->nCmd = getCommandsByName(value);
				if (pIniHandlerData->iniSSNCommand->nCmd == mainCOMMAND_GETPREFERENCES) {
//					vTaskSuspendAll();	// stop scheduler if load preferences command

//					uint16_t i;
//					for (i = 1; i < all_devs_counter; i++) {
//						vPortFree(devArray[i]);
//					}
//					for (i = 1; i < grp_counter; i++) {
//						vPortFree(grpArray[i]);
//					}
//					for (i = 0; i < act_counter; i++) {
//						vPortFree(actArray[i]);
//					}


//					grp_counter = -1;
//					all_devs_counter = -1;
//					act_counter = 0;
//					route_counter = 0;

				}
			} else {
			//	nRes = pdFAIL;  /* unknown section/name, error */
			}
			break;
		}
		case mainCOMMAND_GETPREFERENCES:
		{
			nRes = process_loadprefs_ini_handler((char*)section, (char*)name, (char*)value, pIniHandlerData, pnSectionNo);
			break;
		}
	}
	strncpy0(pIniHandlerData->sLastSection, section, strlen(section)+1);
	strncpy0(pIniHandlerData->sLastName, name, strlen(name)+1);
	pIniHandlerData->pnPrevSectionNo = *pnSectionNo;
    return nRes;
}

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
		xReturn = xQueueSendFromISR( xLogOutQueue, str, &xHigherPriorityTaskWoken);
	  if( xHigherPriorityTaskWoken == pdTRUE )
	  {
//		  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE );
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

	// TO DO: move to device init...
	/* Enable GPIOs clock */

// periph. clocks enabling in device init functions

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
//	rcc_periph_clock_enable(RCC_GPIOF);
//	rcc_periph_clock_enable(RCC_GPIOG);
//
//
//	/* Enable clocks for special ports  */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SDIO);
//	rcc_periph_clock_enable(RCC_DMA1);
//	rcc_periph_clock_enable(RCC_FSMC);
//
//	rcc_periph_clock_enable(RCC_ADC3); // to do: make it pref from TS_ADC...

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

	/* NVIC Configuration *******************************************************/
		NVIC_Configuration();
	/* System Clocks Configuration **********************************************/
		RCC_Configuration();

		//  initialise_rtc();
//		  rtc_awake_from_off(RCC_LSE);
//		  rtc_awake_from_off(RCC_LSI);
		  rtc_auto_awake(RCC_LSE, 0x7FFF); // prescale set 1s
//		  rtc_auto_awake(RCC_LSI, 0x7FFF); // prescale set 1s

  grpArray = (sGrpInfo **) pvPortMalloc(mainMAX_DEV_GROUPS * sizeof(void*));
  devArray = (sDevice **) pvPortMalloc(mainMAX_ALL_DEVICES * sizeof(void*));
  actArray = (sAction **) pvPortMalloc(mainMAX_ACTIONS * sizeof(void*));

  delay_timer_init(); // init common timer (SOFTI2C_TIMER_1)

  ulIdleCycleCount = 0UL;
  portBASE_TYPE xReturn;
  uint8_t res;
//  char msg[mainMAX_MSG_LEN];
  setMC_Object(MC_OBJECT);

#ifdef  WATCHDOG
  iwdg_start();
  iwdg_set_period_ms(WATCHDOG_PERIOD);	// set watchdog period
#endif

  xTaskHandle pTmpTask;

  xBaseOutQueue = xQueueCreate( mainBASEOUT_QUEUE_SIZE, mainMAX_MSG_LEN);
  xLogOutQueue = xQueueCreate( mainDEBUG_QUEUE_SIZE, mainMAX_MSG_LEN);
  xDevInitQueue = xQueueCreate( mainDEVINIT_QUEUE_SIZE, sizeof(uint16_t));


#ifdef  M_LCD
	xReturn = lcd_init();
#endif



/* Initialize base COM */
#ifdef  M_USART
	  	lComState = lCOMPortInit( mainBASECOM, mainBAUD_RATE, mainBASECOM_Priority );
#endif

	  	xprintfMsg("\r\n\r\nStart SSN. Version: %s ", SSN_VERSION);

	  	EthernetHardwareInit();
	  	ethernetInit();

/*
 *
 * to do: make init other UARTs if needed...
 */
		//sendBaseOut("\r\n\r\nStart SSN");
//	  	xsprintf(( portCHAR *) msg, "\r\n\r\nStart SSN. Version: %s\n", SSN_VERSION);
//		debugMsg(msg);

// ------------------------------------------------------------------------------

	  	sGrpInfo *pGrp;
		sDevice *pVirtualDev;	// abstract group and device for timer actions cashing
//		sGrpDev *pGrpDev;

		pGrp = newGrpInfo();
		pGrp->uiGroup = SOFTI2C_1_GRP;
		pGrp->iDevQty = 0; // not applicable to virtual group
//		pGrpDev = &pGrp->GrpDev;
		pGrp->GrpDev.pTimer = SOFTI2C_TIMER_1; // obsolete - use common timer
//		pGrpDev->pTimer = SOFTI2C_TIMER_1;

		pVirtualDev = newDev();
		pVirtualDev->nId = 0;
		pVirtualDev->nDevObj = 0;
		pVirtualDev->pGroup = pGrp;
		pVirtualDev->ucType = device_TYPE_VIRTUAL;
//		devArray[all_devs_counter++] = pVirtualDev;

#if defined (M_RTC_DS1307) || defined (PERSIST_EEPROM_I2C)
		// software i2c group index = 0 (grp_counter = 0)
		// This group combine RTC_DS1307 and EEPROM
		pGrp->GrpDev.pPort = SOFTI2C_1_PORT;
		pGrp->GrpDev.ucPin = SOFTI2C_1_PIN_SDA;
		pGrp->GrpDev.ucPin2 = SOFTI2C_1_PIN_SCL;
		pGrp->GrpDev.pTimer = SOFTI2C_TIMER_1;
		soft_i2c_init(&pGrp->GrpDev);
//		i2c_setup(pGrpDev);

#endif

		hw_loadtime();


// check skip preferences button:

				gpio_set_mode(mainSKIP_PREF_PORT, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, 1 << mainSKIP_PREF_PIN);
				uint32_t button = GPIO_IDR(mainSKIP_PREF_PORT) & (1 << mainSKIP_PREF_PIN);
				if ((button && pdTRUE ) == mainSKIP_PREF_VALUE) {
					xprintfMsg("\n\rSkip reading configuration from EEPROM or FLASH");
					goto skipLoadPrefs;
				}

// read preferences:
res = restorePreferences(&xPrefsBuffer.buffer, &xPrefsBuffer.nBufSize);

if (res && xPrefsBuffer.buffer) {
		xPrefsBuffer.state = JSON_STATE_READY;
		xPrefsBuffer.counter = xPrefsBuffer.nBufSize;
// begin preferences processing
// define format preferences: if first char = "{" than JSON, else INI
				if (xPrefsBuffer.buffer[0] != '{') {
// INI format
				    sIniHandlerData xIniHandlerData;
				    sSSNCommand xSSNCommand;
				    xSSNCommand.nCmd = 0;
				    xIniHandlerData.iniSSNCommand = &xSSNCommand;
				    xIniHandlerData.sLastName[0] = 0;
				    xIniHandlerData.sLastSection[0] = 0;
				    buffer_ctx ctx;
				    ctx.ptr = (char*) xPrefsBuffer.buffer;
				    ctx.bytes_left = xPrefsBuffer.nBufSize;
				    res = ini_parse_stream((ini_reader)ini_buffer_reader, &ctx, handler, &xIniHandlerData);
				    if (res != 0) {
				    	xprintfMsg("\r\nErrors was in INI parsing, last error line: %d\r\n", res);
				    } else {
				    	xprintfMsg("\r\nParsed INI format\r\n");
				    	//res = pdPASS;
				    }
			    	res = restoreAllMemDevs();
			    	uiLastSaveMemoryTick = rtc_get_counter_val();
			    	xprintfMsg("\r\nLoaded devices: %d\r\n", all_devs_counter);
			    	xprintfMsg("\r\nLoaded actions: %d\r\n", act_counter);
				} else {
// JSON format
// skip JSON format!!!
					res = pdFAIL;
				}
		res = refreshActions2DeviceCash();

		vPortFree(xPrefsBuffer.buffer);
	} else {
		//error
		xprintfMsg("\r\nError reading configuration from EEPROM or FLASH");
	}


skipLoadPrefs:

//init_dhcp_client();
//check_DHCP_state(0);

// ------------------------------------------------------------------------------
	xReturn = xTaskCreate( prvBaseOutTask, ( char * ) "BaseOutTask", 210, NULL, mainBASEOUT_TASK_PRIORITY, &pTmpTask );
	xBaseOutTaskHnd = (void*) pTmpTask;
	xReturn = xTaskCreate( prvLogOutTask, ( char * ) "LogOutTask", 150, NULL, mainDEBUG_OUT_TASK_PRIORITY, NULL );

	xInputQueue = xQueueCreate( mainINPUT_QUEUE_SIZE, sizeof( xInputMessage ) );
//	xSensorsQueue = xQueueCreate( mainSENSORS_QUEUE_SIZE, sizeof(void*) );
	xSensorsQueue = xQueueCreate( mainSENSORS_QUEUE_SIZE, sizeof(xSensorMessage) );

	xReturn = xTaskCreate( prvUSARTEchoTask, ( char * ) "Echo", 250, NULL, mainECHO_TASK_PRIORITY, &pTmpTask );

	xTimerHandle xTimer;
	//(void) xTimer;
	prefix_unused(xTimer);
	xTimer = xTimerCreate("Cron", mainCronRate, pdTRUE, NULL, prvCronFunc);
	xTimerStart(xTimer, 0);

#ifdef DEBUG_S
	xReturn = xTaskCreate( prvDebugStatTask, ( char * ) "Debug_S", 210, NULL, tskIDLE_PRIORITY + 2, &pTmpTask );
#endif

	xReturn = xTaskCreate( prvInputTask, ( char * ) "InputTask", mainINPUT_TASK_STACK, NULL, mainINPUT_TASK_PRIORITY, &pTmpTask );

	completeAllInit(); // complete delayed device init procedures


	xReturn = xTaskCreate( prvCheckSensorMRTask, ( char * ) "CheckSensorMRTask", 200, devArray, mainCHECK_SENSOR_MR_TASK_PRIORITY, &pTmpTask );
	pCheckSensorMRTaskHnd = pTmpTask;


	xReturn = xTaskCreate( prvCheckSensorHRTask, ( char * ) "CheckSensorHRTask", configMINIMAL_STACK_SIZE, devArray, mainCHECK_SENSOR_HR_TASK_PRIORITY, &pTmpTask );
	pCheckSensorHRTaskHnd = pTmpTask;


	xReturn = xTaskCreate( prvProcSensorTask, ( char * ) "ProcSensorTask", mainPROCSENSORS_TASK_STACK, devArray, mainPROC_SENSOR_TASK_PRIORITY, &pTmpTask );

#ifdef M_ETHERNET
#ifdef ETHERNET_TCP
	xReturn = xTaskCreate( prvEthernetTCPTask, ( char * ) "EtherTskTCP", 210, NULL, tskIDLE_PRIORITY + 2, &pTmpTask );
#endif
#ifdef ETHERNET_UDP
	xReturn = xTaskCreate( prvEthernetUDPTask, ( char * ) "EtherTskUDP", 210, NULL, tskIDLE_PRIORITY + 2, &pTmpTask );
#endif
#endif

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
//	char cPassMessage[mainMAX_MSG_LEN*3];
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

			xprintfMsg("\r\nInputMessage: dest=%d, src=%d, msgtype=%d, cmd=%d",
					xInputMessage.uiDestObject, xInputMessage.uiSrcObject,
					xInputMessage.xMessageType, xInputMessage.nCommand);

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
						vSendSSNPacket (xInputMessage.uiDestObject, getMC_Object(), xInputMessage.xMessageType, xInputMessage.pcMessage);
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
									//sendBaseOut("\r\nGPRS LOG request failed");
									xprintfMsg("\r\nGPRS LOG request failed");
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
							vSendSSNPacket (xInputMessage.uiDestObject, getMC_Object(), xInputMessage.xMessageType, xInputMessage.pcMessage);
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
									//sendBaseOut("\r\nGPRS TELEMETRY request failed");
									xprintfMsg("\r\nGPRS TELEMETRY request failed");
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
							vSendSSNPacket (xInputMessage.uiDestObject, getMC_Object(), mainJSON_MESSAGE, xInputMessage.pcMessage);
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
							xprintfMsg("\r\nInfo msg: %s", xInputMessage.pcMessage);
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
					xprintfMsg("\r\nGSM_IN msg:  %s", (char *) xInputMessage.pcMessage);
					xReturn = fillCommandStruct((char *) xInputMessage.pcMessage, &xSSNCommand);
					// process commands if they exists
					if (xReturn) {
						xSSNCommand.uiDevDest = xInputMessage.xSourceDevice;
						//xSSNCommand.
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
					xSSNCommand.uiObjSrc = xInputMessage.xSourceDevice;
					vCommandSelector(&xSSNCommand);
					vPortFree(xInputMessage.pcMessage);
					break;
				}
// ============= INI format processing:
// common part INI structure:
//
				case mainINI_MESSAGE: {
					xprintfMsg("\r\nFreeHeap=%d, StHWM=%d =PRCINI", xPortGetFreeHeapSize(), uxTaskGetStackHighWaterMark(0));
					xReturn = storePreferences(xInputMessage.pcMessage, strlen(xInputMessage.pcMessage));
					xprintfMsg("\r\nPreferences saved: %d", xReturn);
					vTaskDelay( 1000 / portTICK_PERIOD_MS );
					main_reboot(); 	// reboot
				    while (1) {};

					break;
				}
// ============= JSON format processing:
				case mainJSON_MESSAGE: {
/* processing JSON message
 common part JSON structure:
 {	"ssn": {"v":"version_number", "cmd":"command", "data": { ... } } }
*/
					xprintfMsg("\r\nFreeHeap=%d, StHWM=%d =PRCJSON", xPortGetFreeHeapSize(), uxTaskGetStackHighWaterMark(0));

								jsonMsg = (char*) xInputMessage.pcMessage;
								cJSON *json_root;
								cJSON *json_ssn;
								json_root=cJSON_Parse(jsonMsg);
								if (!json_root) {
									xprintfMsg("\r\nError before: [%s]\r\n",cJSON_GetErrorPtr());
								}
								else
								{
									json_ssn=cJSON_GetObjectItem(json_root,"ssn");
									if (!json_ssn) {
										xprintfMsg("\r\nJSON error before: [%s]\r\n",cJSON_GetErrorPtr());
									} else {
// now processing command:
										int version = cJSON_GetObjectItem(json_ssn,"v")->valueint;
										if (version==1) {
											char* cmd = (char*)cJSON_GetObjectItem(json_ssn,"cmd")->valuestring;
											uint16_t nObjDest = (uint16_t) cJSON_GetObjectItem(json_ssn,"obj")->valueint;
//											uint16_t nObjSrc  = (uint16_t) cJSON_GetObjectItem(json_ssn,"obj_src")->valueint;

											xprintfMsg("\r\nProcess JSON command: %s", cmd );

											cJSON *json_data = cJSON_GetObjectItem(json_ssn,"data");

											if (strcmp(cmd, "setdatetime") == 0) {
												process_setdatetime(json_data);
											}
											if (strcmp(cmd, "getdatetime") == 0) {
												process_getdatetime(json_data);
											}
											if (strcmp(cmd, "getowilist") == 0) {
												process_getowilist(json_data);
											}
											if (strcmp(cmd, "updateaction") == 0) {
												UpdateActionJSON(json_data);
											}
								if (strcmp(cmd, "scani2c") == 0) {
									if (json_data) {
										uint16_t dev_id =
												(uint16_t) cJSON_GetObjectItem(json_data, "adev")->valueint;
										sDevice *pTmpDev = getDeviceByID(dev_id);
										if (pTmpDev) {
											uint8_t nCnt = 0;
											uint8_t addr;

											for (addr=0; addr<=254; addr++) {
												int32_t nRes = soft_i2c_TryAddress (&pTmpDev->pGroup->GrpDev, addr);
												if (nRes) {
													nCnt++;
													xprintfMsg("\r\nscani2c: Dev[%d] - i2c device found at address [%x]", dev_id, addr);
												}
											}
											xprintfMsg("\r\nscani2c: Scan completed found devices [%d]", nCnt);
										} else {
											xprintfMsg("\r\nscani2c: Error - Dev not found [%d]", dev_id);
										}
									}
								}

											if (strcmp(cmd, "sdv") == 0) {
												if (json_data) {
													char* strVal 	= (char*) cJSON_GetObjectItem(json_data,"aval")->valuestring;
													uint32_t dev_val = (uint32_t) cJSON_GetObjectItem(json_data,"aval")->valueint;
													uint16_t dev_id = (uint16_t) cJSON_GetObjectItem(json_data,"adev")->valueint;
													uint8_t  dev_cmd = (uint8_t) cJSON_GetObjectItem(json_data,"acmd")->valueint;
													if (strVal) {
														// process string data type:
														setDevValueByID((int32_t)strVal, dev_cmd, dev_id, eElmString);
														logAction(0, dev_id, dev_cmd, dev_val);
														xprintfMsg("\r\nSet dev[%d](%d)=%s", dev_id, dev_cmd, strVal);
													} else {
														// process numeric data type:
														setDevValueByID(dev_val, dev_cmd, dev_id, eElmInteger);
														logAction(0, dev_id, dev_cmd, dev_val);
														xprintfMsg("\r\nSet dev[%d]=(%d)%d", dev_id, dev_cmd, dev_val);
													}
													// send notification about command executing:
//													xsprintf(cPassMessage, "{\"status\":\"0\", \"comment\":\"set dev[%d](%d)=%d\"}", dev_id, dev_cmd, dev_val);
//													char* pResp = pvPortMalloc(strlen(cPassMessage));
//													memcpy(pResp,cPassMessage,strlen(cPassMessage));
//													pResp[strlen(cPassMessage)]=0;

//													vSendInputMessage (1, nObjSrc, mainJSON_MESSAGE, xInputMessage.uiSrcObject, dev_id, main_IF_PROGRAM, pResp, strlen(cPassMessage), 0);
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
												vSendInputMessage (1, nObjDest, mainCOMMAND_MESSAGE, xInputMessage.uiSrcObject, xInputMessage.xSourceDevice, main_IF_PROGRAM, pData, 0, nCmd);
											} else
											if (strcmp(cmd, "getdevvals") == 0) {
												//nCmd = mainCOMMAND_GETDEVVALS;
												if (json_data) {
													uint16_t dev_id = (uint16_t) cJSON_GetObjectItem(json_data,"d")->valueint;
													char * pcTeleData;
													pcTeleData = process_getdevvals(devArray, all_devs_counter, dev_id);
													if (pcTeleData) {
														vSendInputMessage(1, xInputMessage.uiSrcObject, mainTELEMETRY_MESSAGE, getMC_Object(), 0,
															0, (void*) pcTeleData, strlen(pcTeleData), 0);
													}
												}
												// vSendInputMessage (1, nObjDest, mainCOMMAND_MESSAGE, xInputMessage.uiSrcObject, 0, xInputMessage.xSourceDevice, pData, 0, nCmd);
											} else
											if (strcmp(cmd, "memsave") == 0) {
												nCmd = mainCOMMAND_MEMSAVE;
												vSendInputMessage (1, nObjDest, mainCOMMAND_MESSAGE, xInputMessage.uiSrcObject, xInputMessage.xSourceDevice, main_IF_PROGRAM, pData, 0, nCmd);
											}

										}	else 	{
											xprintfMsg("Wrong SSN message version");
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
//		xprintfMsg("\r\nFreeHeap:=%d =INPUTTSK===", xPortGetFreeHeapSize());
//		taskYIELD();
	} // while
}


/* The poll Middle Rate sensors task. */
static void prvCheckSensorMRTask(void *pvParameters) {
	/* Just to avoid compiler warnings. */
	(void) pvParameters;
	portTickType xLastWakeTime;
	uint16_t j;
	uint8_t ut;
	int32_t res;
//	uint32_t nLastVal;
	(void) res;
//	uint8_t channel_array[16];
	xSensorMessage sSensorMsg;
	sDevice * pDev;

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
#ifdef  M_DS18B20
		uint8_t flag18b20_conv = 0;
#endif
		RTC_t rtc;
		rtc_gettime(&rtc);

		for (j = 1; j <= all_devs_counter; j++) {
			pDev = getDevByNo(j);
			if (pDev) {
				if (!(pDev->nFlag & DEV_DISABLE_FLAG)) {
					ut = pDev->ucType;
					sSensorMsg.pDev = pDev;
					switch (ut) {

#ifdef  M_DS18B20
					case device_TYPE_DS18B20:
						if (pDev->pDevStruct) {
// check ds18b20 conversation status on this line (it's make only for one (first) device - if set for this group, than skip):
							if (flag18b20_conv != pDev->pGroup->uiGroup) {
								res = ds_start_convert_all(pDev->pGroup, j); // start temperature measuring
								flag18b20_conv = pDev->pGroup->uiGroup;
							}
// if value changed send message into Sensors queue for next processing:
							if (abs(
									((ds18b20_device*) pDev->pDevStruct)->iDevValue
											- ((ds18b20_device*) pDev->pDevStruct)->nDevPrevValue)
									>= pDev->uiDeltaValue) {
								((ds18b20_device*) pDev->pDevStruct)->nDevPrevValue = ((ds18b20_device*) pDev->pDevStruct)->iDevValue;
								sSensorMsg.nDevCmd = 0;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);
//										res = xQueueSend(xSensorsQueue, (void*)&devArray[j], 0);
								if (!res) {
									xprintfMsg(
											"\r\nError sending into Sensors Queue!");
								}
							}
						}
						break;
#endif

#ifdef  M_DHT
					case device_TYPE_DHT22:
						if (pDev->pDevStruct) {
//									taskENTER_CRITICAL(); {
							res = dht_get_data(pDev);
//									} taskEXIT_CRITICAL();
							if ((abs(
									((DHT_data_t*) pDev->pDevStruct)->humidity
											- ((DHT_data_t*) pDev->pDevStruct)->nPrevHumidity)
									>= ((DHT_data_t*) pDev->pDevStruct)->uiDeltaHumidity)) {
								((DHT_data_t*) pDev->pDevStruct)->nPrevHumidity = ((DHT_data_t*) pDev->pDevStruct)->humidity;
								sSensorMsg.nDevCmd = 1;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);
							}
							if (abs(
									((DHT_data_t*) pDev->pDevStruct)->temperature
											- ((DHT_data_t*) pDev->pDevStruct)->nPrevTemperature)
									>= pDev->uiDeltaValue) {
								((DHT_data_t*) pDev->pDevStruct)->nPrevTemperature = ((DHT_data_t*) pDev->pDevStruct)->temperature;
								sSensorMsg.nDevCmd = 0;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);
//											res = xQueueSend(xSensorsQueue, (void*)&devArray[j], 0);
							}
						}
						break;
#endif
#ifdef  M_BMP180
					case device_TYPE_BMP180:
						if (pDev->pDevStruct) {
							res = bmp180_get_data(pDev);
							if (abs(
									((BMP180_data_t*) pDev->pDevStruct)->iTemperature
											- ((BMP180_data_t*) pDev->pDevStruct)->iPrevTemperature)
									>= pDev->uiDeltaValue) {
								// save previous values
								((BMP180_data_t*) pDev->pDevStruct)->iPrevTemperature = ((BMP180_data_t*) pDev->pDevStruct)->iTemperature;

								sSensorMsg.nDevCmd = 0;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);

							}
							if (abs(
									((BMP180_data_t*) pDev->pDevStruct)->uiPressure
											- ((BMP180_data_t*) pDev->pDevStruct)->uiPrevPressure)
									>= ((BMP180_data_t*) pDev->pDevStruct)->uiDeltaPressure) {
								((BMP180_data_t*) pDev->pDevStruct)->uiPrevPressure = ((BMP180_data_t*) pDev->pDevStruct)->uiPressure;
								sSensorMsg.nDevCmd = 1;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);
							}
						}
						break;
#endif
#ifdef  M_BME280
					case device_TYPE_BME280:
						if (pDev->pDevStruct) {
							s32 v_uncomp_pressure_s32;
							s32 v_uncomp_temperature_s32;
							s32 v_uncomp_humidity_s32;

							if (((bme280_t*) pDev->pDevStruct)->oss == 0) {
								// normal mode
								res =
										bme280_read_uncomp_pressure_temperature_humidity(
												pDev, &v_uncomp_pressure_s32,
												&v_uncomp_temperature_s32,
												&v_uncomp_humidity_s32);
							} else {
								// force mode
								res =
										bme280_get_forced_uncomp_pressure_temperature_humidity(
												pDev, &v_uncomp_pressure_s32,
												&v_uncomp_temperature_s32,
												&v_uncomp_humidity_s32);
							}

							if (res) {
								pDev->uiLastUpdate = rtc_get_counter_val();

								((bme280_t*) pDev->pDevStruct)->iTemperature =
										bme280_compensate_temperature_int32(
												pDev, v_uncomp_temperature_s32);
								((bme280_t*) pDev->pDevStruct)->uiPressure =
										bme280_compensate_pressure_int32(pDev,
												v_uncomp_pressure_s32);
								((bme280_t*) pDev->pDevStruct)->uiHumidity =
										bme280_compensate_humidity_int32(pDev,
												v_uncomp_humidity_s32);

//								res = bme280_read_pressure_temperature_humidity(pDev,
//								&((bme280_t*) pDev->pDevStruct)->uiPressure,
//								&((bme280_t*) pDev->pDevStruct)->iTemperature,
//								&((bme280_t*) pDev->pDevStruct)->uiHumidity);
//							}

//							if (res) {
							if (abs(
									((bme280_t*) pDev->pDevStruct)->iTemperature
											- ((bme280_t*) pDev->pDevStruct)->iPrevTemperature)
									>= pDev->uiDeltaValue) {
								((bme280_t*) pDev->pDevStruct)->iPrevTemperature = ((bme280_t*) pDev->pDevStruct)->iTemperature;
								sSensorMsg.nDevCmd = 0;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);

							}
							if (abs(
									((bme280_t*) pDev->pDevStruct)->uiPressure
											- ((bme280_t*) pDev->pDevStruct)->uiPrevPressure)
									>= ((bme280_t*) pDev->pDevStruct)->uiDeltaPressure) {
								((bme280_t*) pDev->pDevStruct)->uiPrevPressure = ((bme280_t*) pDev->pDevStruct)->uiPressure;
								sSensorMsg.nDevCmd = 1;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);
							}
							if (abs(
									((bme280_t*) pDev->pDevStruct)->uiHumidity
											- ((bme280_t*) pDev->pDevStruct)->uiPrevHumidity)
									>= ((bme280_t*) pDev->pDevStruct)->uiDeltaHumidity) {
								((bme280_t*) pDev->pDevStruct)->uiPrevHumidity = ((bme280_t*) pDev->pDevStruct)->uiHumidity;
								sSensorMsg.nDevCmd = 2;
								res = xQueueSend(xSensorsQueue, &sSensorMsg, 0);
							}
							} else {
								xprintfMsg("\r\nError reading BME280 data! Try to reinit device");
								res = bme280DeviceInit(pDev);
							}
						}
						break;
#endif
					case device_TYPE_BB1BIT_IO_AI:
						if (pDev->pDevStruct) {
							// ADC calculation:
							adc_set_regular_sequence(ADC1,
									pDev->pGroup->iDevQty,
									((sADC_data_t*) pDev->pDevStruct)->nChannelArray);
							adc_start_conversion_direct(ADC1);
//								adc_start_conversion_regular(ADC1);
//								while (!adc_eoc(ADC1));
//								nADCch_counter = 0;
//							pDev->nFlag &= 0b11111110;

							for (uint8_t nch = 0; nch < pDev->pGroup->iDevQty;
									nch++) {
//								uint16_t nTmpValue;
								adc_set_regular_sequence(ADC1, 1,
										((((sADC_data_t*) pDev->pDevStruct)->nChannelArray)
												+ nch));
								adc_start_conversion_direct(ADC1);
								while (!adc_eoc(ADC1))
									;
//									nADCch_counter = 0;
//								nTmpValue =
//										((sADC_data_t*) pDev->pDevStruct)->nADCValueArray[nch];
								((sADC_data_t*) pDev->pDevStruct)->nADCValueArray[nch] =
										adc_read_regular(ADC1);
								if (abs(
										((sADC_data_t*) pDev->pDevStruct)->nADCValueArray[nch]
												- ((sADC_data_t*) pDev->pDevStruct)->nADCValueArrayPrev[nch])
										>= pDev->uiDeltaValue) {
									((sADC_data_t*) pDev->pDevStruct)->nADCValueArrayPrev[nch] = ((sADC_data_t*) pDev->pDevStruct)->nADCValueArray[nch];
//									pDev->nFlag |= 0b00000001; // value is changed
									sSensorMsg.nDevCmd = nch;
									res = xQueueSend(xSensorsQueue, &sSensorMsg,
											0);
								}
								//((sADC_data_t*)devArray[j]->pDevStruct)->uiLastUpdate = rtc_get_counter_val();
								pDev->uiLastUpdate = rtc_get_counter_val();
							}
//									if (devArray[j]->nFlag && 0b00000001) {
//										res = xQueueSend(xSensorsQueue, (void*)&devArray[j], 0);
//									}
						}
						break;
#ifdef  M_MHZ19
						case device_TYPE_MHZ19:
							res = mhz19DeviceSendCommandReadCO2(pDev);
						break;
#endif

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
			}
		}
		taskYIELD()
		;
		vTaskDelayUntil(&xLastWakeTime, mainSensorRateMR);
	}
}

/* The process High Rate sensors data task. */
static void prvCheckSensorHRTask(void *pvParameters) {
	(void) pvParameters;
	volatile uint8_t res;
	uint8_t ut;
	uint16_t j;
	xSensorMessage sSensorMsg;
	sDevice * pDev;

	while (1) {
		for (j = 1; j <= all_devs_counter; j++) {
			if (pDev) {
				pDev = getDevByNo(j);
				ut = pDev->ucType;
				switch (ut) {
				case device_TYPE_BB1BIT_IO_INPUT:
					res = (int8_t) bb_read_wire_data_bit(&pDev->pGroup->GrpDev);
					if (res != pDev->nLastPinValue) {
						pDev->nLastPinValue = res;
						pDev->uiLastUpdate = rtc_get_counter_val();

						// res = scanDevActions (devArray[j]);
						// xQueueSend(xSensorsQueue, &devArray[j], 0);
						sSensorMsg.nDevCmd = 0;
						sSensorMsg.pDev = pDev;
						xQueueSendToFront(xSensorsQueue, &sSensorMsg, 0); // send to head of queue!
//									xQueueSendToFront(xSensorsQueue, &devArray[j], 0); // send to head of queue!
					}
					break;
					/*
					 case device_TYPE_STEPMOTOR:
					 if (getMainTimerTick() > (((stepmotor_data_t*) (devArray[j]->pDevStruct))->uiLastTick) +
					 ((stepmotor_data_t*) (devArray[j]->pDevStruct))->uiStepTime) {
					 StepMotorNextStep(devArray[j]);
					 }
					 break;
					 */
				case device_TYPE_BB1BIT_IO_PP:
				case device_TYPE_BB1BIT_IO_OD:
					break;
				}
//					}
			}
		}
//		taskYIELD();
		vTaskDelay(mainSensorRateHR);
	}
}

#ifdef M_ETHERNET
#ifdef ETHERNET_TCP
/* Process Ethernet TCP connection */
static void prvEthernetTCPTask(void *pvParameters) {
	(void) pvParameters;
	sSSNPDU* xSSNPDU;
//	  uint8 pc_ip[4]={192,168,1,101};
//	  uint16 port=ETHERNET_SSN_SERVER_PORT;
//	  uint16 anyport=30001;
	  uint16 len=0;
	  uint16 len_recv;
//	  uint8 buffer[2048];
	  uint8* buffer = 0;
	  uint8 nRes;

	while (1) {
		//		taskYIELD();
		nCurrentInternetState = getSn_SR(0);
		switch (nCurrentInternetState)
		{
		case SOCK_INIT:
			nRes = listen(0);
			xprintfMsg("\r\nTCP server socket init: %d, port: %d", nRes, port);
			if (nRes == 0) {
				ethernetInit(); // to do...
			}
//			connect(0, pc_ip, port);
			break;
		case SOCK_LISTEN:
			break;
		case SOCK_ESTABLISHED:
			if (getSn_IR(0) & Sn_IR_CON) {
				setSn_IR(0, Sn_IR_CON);
			}

			len = getSn_RX_RSR(0);
			if (len > 0) {
				buffer = pvPortMalloc(len + 1);
				if (buffer) {
					len_recv = recv(0, buffer, len);
					if (len_recv > 0) {
						buffer[len_recv] = 0;
						xprintfMsg("\r\nTCP receive: %s", buffer);
						xSSNPDU = parseSSNPDU((char*) buffer);
						if (xSSNPDU) {
							/* Write the received message to common Input queue. */
							vSendInputMessage(1, xSSNPDU->obj_dest,
									xSSNPDU->message_type, xSSNPDU->obj_src, 0,
									0, (void*) xSSNPDU->buffer,
									xSSNPDU->nDataSize, 0);
							// vPortFree(xSSNPDU->buffer);
							vPortFree(xSSNPDU);
						}
//				send(0,buffer,len);
					}
					vPortFree(buffer);
				}
			}
			break;
		case SOCK_CLOSE_WAIT:
			xprintfMsg("\r\nTCP socket close wait");
			close(0);
			break;
		case SOCK_CLOSED:
			ethernetInit(); // to do...
			socket(0, Sn_MR_TCP, port, Sn_MR_ND);
			xprintfMsg("\r\nTCP socket closed");
			break;
	    default :
			xprintfMsg("\r\nTCP state: %d", nCurrentInternetState);
	        break;
		}
		vTaskDelay(mainEthernetRateTCP);
	}
}
#endif

#ifdef ETHERNET_UDP
/* Process Ethernet UDP connection */
static void prvEthernetUDPTask(void *pvParameters) {
	(void) pvParameters;
//	volatile uint8_t res;

	while (1) {
		vTaskDelay(mainEthernetRateUDP);
	}
}
#endif
#endif

static uint32_t	getMainTimerTick()
{
	return uiMainTick;
}

/* The cron function (invoke by timer) */
static void prvCronFunc( void *pvParameters )
{
	( void ) pvParameters;
//	uint16_t i;
//	RTC_t rtc;
	int32_t res;
	uint32_t uiCurrentTick;
//	sAction* pAct;
	(void) res;

		uiMainTick++;
//		rtc_gettime(&rtc);

		// check SSN protocol USART timeout
		if ((uiMainTick > (xSSNPDU.uiLastTick + SSN_TIMEOUT/100))
				&& ((xSSNPDU.state == SSN_STATE_LOADING) || (xSSNPDU.state == SSN_STATE_DATA)))
		{
			xSSNPDU.state = SSN_STATE_ERROR;
			xprintfMsg("\r\nSSN receive data timeout error!\r\nCnt: %d\r\nbuf: %s", xSSNPDU.counter, xSSNPDU.cSSNBuffer);
			vPortFree(xSSNPDU.buffer);
		}

#ifdef  WATCHDOG
		iwdg_reset();	// reset watchdog timer
#endif

		sDevice* pDev = getDevByNo(0); // get virtual time device
		// scan thru actions with time events:
		scanDevActions (pDev);
		uiCurrentTick = rtc_get_counter_val();
		if (uiCurrentTick > (uiLastSaveMemoryTick + mainMEMORY_DEV_SAVE_PERIOD))
		{
			res = storeAllMemDevs();
		}

// heartbeat tick:
		if (uiCurrentTick > (uiLastHeartBeatTS + mainHEARTBEAT_PERIOD))
		{
			heartBeatSend(uiLastHeartBeatTick, uiCurrentTick);
			uiLastHeartBeatTS = uiCurrentTick;
			uiLastHeartBeatTick++;
		}

		logAction(0, 0, 0, 0); // check timeout for log submitting
}

/* The process sensors data task. */
static void prvProcSensorTask( void *pvParameters )
{
	sDevice* dev;
	( void ) pvParameters;
	xSensorMessage sSensorMsg;

	while (1)
	{
		/* Wait for a message from Sensors queue */
//		while ( xQueueReceive(xSensorsQueue, &dev, portMAX_DELAY)	!= pdPASS) ;
		while ( xQueueReceive(xSensorsQueue, &sSensorMsg, portMAX_DELAY)	!= pdPASS) ;
		dev = sSensorMsg.pDev;
		if (dev) {
				switch (dev->ucType) {
#ifdef  M_DS18B20
				case device_TYPE_DS18B20:
					if (dev->pDevStruct) {
						logAction(0, dev->nId, sSensorMsg.nDevCmd, ((ds18b20_device*) dev->pDevStruct)->iDevValue);
					}
					break;
#endif
#ifdef  M_DHT
				case device_TYPE_DHT22:
					if (dev->pDevStruct) {
						if (sSensorMsg.nDevCmd == 0)
							logAction(0, dev->nId, 0, ((DHT_data_t*) dev->pDevStruct)->temperature);
						else
							logAction(0, dev->nId, 1, ((DHT_data_t*) dev->pDevStruct)->humidity);
					}
					break;
#endif
#ifdef  M_BMP180
				case device_TYPE_BMP180:
					if (dev->pDevStruct) {
						if (sSensorMsg.nDevCmd == 0)
							logAction(0, dev->nId, 0, ((BMP180_data_t*) dev->pDevStruct)->iTemperature);
						else
							logAction(0, dev->nId, 1, ((BMP180_data_t*) dev->pDevStruct)->uiPressure);
					}
					break;
#endif
#ifdef  M_BME280
				case device_TYPE_BME280:
					if (dev->pDevStruct) {
						if (sSensorMsg.nDevCmd == 0)
							logAction(0, dev->nId, 0, ((bme280_t*) dev->pDevStruct)->iTemperature);
						else if (sSensorMsg.nDevCmd == 1)
							logAction(0, dev->nId, 1, ((bme280_t*) dev->pDevStruct)->uiPressure);
						else
							logAction(0, dev->nId, 2, ((bme280_t*) dev->pDevStruct)->uiHumidity);
					}
					break;

#endif
#ifdef  M_MHZ19
				case device_TYPE_MHZ19:
					logAction(0, dev->nId, 0, (uint32_t) dev->pDevStruct);
				break;
#endif
					case device_TYPE_BB1BIT_IO_AI:
					if (dev->pDevStruct) {
//						for (uint8_t ch=0; ch < dev->pGroup->iDevQty; ch++) {
						logAction(0, dev->nId, sSensorMsg.nDevCmd, ((sADC_data_t*)dev->pDevStruct)->nADCValueArray[sSensorMsg.nDevCmd]);
//							logAction(0, dev->nId, ch, ((sADC_data_t*)dev->pDevStruct)->nADCValueArray[ch]);
//						}
					}
					break;
				case device_TYPE_BB1BIT_IO_INPUT:
					logAction(0, dev->nId, 0, dev->nLastPinValue);
					break;
				}
				// scan thru actions linked with this device:
				scanDevActions (dev);

		}
			taskYIELD();
	}
}


// ----------------------------------------------

static void prvUSARTEchoTask( void *pvParameters )
{
signed char cChar = 0;
//char cTmpBuf[mainMAX_MSG_LEN];
char cTmpBuf[6];
uint8_t nScanCnt;
uint16_t calcCRC;

	/* Just to avoid compiler warnings. */
	(void) pvParameters;
	nScanCnt = 0;
	xSSNPDU.buffer = 0;
	xSSNPDU.state = SSN_STATE_INIT;

	while (1) {
		/* Block to wait for a character to be received on mainBASECOM */
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
								xprintfMsg("\r\nSSN buffer allocation error!");
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
				if ((cChar== 0x0A)&&(SSN_SKIP_0A)) { // skip 0x0A chars
					break;
				}
				if ((cChar== 0x0D)&&(SSN_SKIP_0D)) { // skip 0x0D chars
					break;
				}
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
								vSendInputMessage (1, xSSNPDU.obj_dest, xSSNPDU.message_type, xSSNPDU.obj_src, 0, 0, (void*) xSSNPDU.buffer, xSSNPDU.nDataSize,0);
							} else {
								xSSNPDU.state = SSN_STATE_ERROR;
								xprintfMsg("\r\nSSN data CRC error! (calc=%04x, msg=%04x)", calcCRC, xSSNPDU.crc16);
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
static void prvLogOutTask( void *pvParameters )
{
	( void ) pvParameters;
	char buf[mainMAX_MSG_LEN+1];

	while (1) {
		/* Wait for a message from Debug queue */
		while(xQueueReceive( xLogOutQueue, &buf, portMAX_DELAY ) != pdPASS);
		sendBaseOut (buf);
	}
}

static void prvBaseOutTask( void *pvParameters )
{
	( void ) pvParameters;
	  portBASE_TYPE xReturn;
//	char buf[mainMAX_MSG_LEN+1];
	char *buf = pvPortMalloc(mainMAX_MSG_LEN+1);
	memset (buf,0,mainMAX_MSG_LEN+1);
	while (1) {
		/* Wait for a message from base out queue */
		while( (xQueueReceive( xBaseOutQueue, buf, portMAX_DELAY ) != pdPASS) && (xSSNPDU.state != SSN_STATE_LOADING) && (xSSNPDU.state != SSN_STATE_DATA));
		if (lComState == pdPASS) {
			// check minimum send after receive timeout and make some delay if needed:
			if ((getMainTimerTick() - xSSNPDU.uiLastTick) < SSN_MIN_SEND_TIMEOUT) {
				vTaskDelay((getMainTimerTick() - xSSNPDU.uiLastTick) + SSN_MIN_SEND_TIMEOUT / portTICK_RATE_MS);
			}
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

static void vMainTimerFunctionInterval1(void* pParam) {
	sEvtElm* pEvtElm = (sEvtElm*) pvTimerGetTimerID(pParam);
	sActElm* pActElm;
	sAction* pAction;
	int32_t nSetDevValue;
	int32_t res = pdTRUE;
//	char cPassMessage[ mainMAX_MSG_LEN ];
	// search all Action elements:
	while (pEvtElm) {
		if (pEvtElm->nElmType == eElmAction) {
			pActElm = (sActElm*) pEvtElm->nElmData1;
			if (pActElm) {
				pAction = pActElm->pAction;

				res = processActionStack(pActElm->nElmDataIn, &nSetDevValue);
				if (res) {
					if (!(pAction->nFlags & devACTION_FLAG_NOLOG)) {
						if (pActElm->nElmDataType != eElmString) {
							logAction(pAction->nActId, pActElm->nDevId, pActElm->nActCmdIn, nSetDevValue);
						}
					}
					setDevValueByID(nSetDevValue, pActElm->nActCmdIn, pActElm->nDevId, pActElm->nElmDataType);

					if (pAction->xActTimerOneShot) {
						xTimerStart(pAction->xActTimerOneShot, 0);
					}
				}
			}
		}
		pEvtElm = (sEvtElm*) pEvtElm->pPrevElm;
	}
}

static void vMainTimerFunctionInterval2(void* pParam) {
	sEvtElm* pEvtElm = (sEvtElm*) pvTimerGetTimerID(pParam);
	sActElm* pActElm;
	sAction* pAction;
	int32_t nSetDevValue;
	int32_t res = pdTRUE;
//	char cPassMessage[ mainMAX_MSG_LEN ];
	// search all Action elements:
	while (pEvtElm) {
		if (pEvtElm->nElmType == eElmAction) {
			pActElm = (sActElm*) pEvtElm->nElmData1;
			if (pActElm) {
				pAction = pActElm->pAction;

				res = processActionStack(pActElm->nElmDataOut, &nSetDevValue);
				if (res) {
					if (!(pAction->nFlags && devACTION_FLAG_NOLOG)) {

						if (pActElm->nElmDataType != eElmString) {
							logAction(pAction->nActId, pActElm->nDevId, pActElm->nActCmdOut, nSetDevValue);
						}
					}
					setDevValueByID(nSetDevValue, pActElm->nActCmdOut, pActElm->nDevId, pActElm->nElmDataType);
				}
			}
		}
		pEvtElm = (sEvtElm*) pEvtElm->pPrevElm;
	}
}

void vMainStartTimer(sAction* pAct)
{
	if (pAct) {
		if (pAct->xActTimer)
			xTimerStart(pAct->xActTimer, 0);
	}

}

#ifdef  M_GSM
uint32_t vMainStartGSMTask(void* pParam)
{
	uint32_t nRes;
	nRes = xTaskCreate( prvStartGSMTask, ( char * ) "StartGSMTask", configMINIMAL_STACK_SIZE, pParam, gsmGSM_TASK_START_PRIORITY, NULL);
	return nRes;
}
#endif

xTimerHandle mainTimerCreate(char* pcTimerName, uint32_t nPeriod, uint32_t isPeriodic, sEvtElm* pEvtElm)
{
	return xTimerCreate(pcTimerName, nPeriod / portTICK_RATE_MS, isPeriodic, pEvtElm, vMainTimerFunctionInterval1);
}

xTimerHandle mainTimerCreateOneShot(char* pcTimerName, uint32_t nPeriod, sEvtElm* pEvtElm)
{
	return xTimerCreate(pcTimerName, nPeriod / portTICK_RATE_MS, pdFALSE, pEvtElm, vMainTimerFunctionInterval2);
}

//void adc1_2_isr(void)
//{
//	long xHigherPriorityTaskWoken = pdFALSE;
//	if (nADCch_counter < 16) {
//		if (pADCValueArray) {
//			pADCValueArray[nADCch_counter++] = adc_read_regular(ADC1);
//		}
//		if (pADCLastUpdate) {
//			*pADCLastUpdate = rtc_get_counter_val();
//		}
//	}
//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}

#ifdef DEBUG_S
static void prvDebugStatTask(void* pParam)
{
	(void) pParam;
	while (1) {

			xprintfMsg("\r\n\r\nTaskStat info: \r\nName	State	Priority	Stack	Num\r\n");
			vTaskList((char*)&cDebugBuf);
			sendBaseOut(cDebugBuf);
			taskYIELD();

			vTaskDelay(mainDEBUG_STAT_RATE);
		}
}
#endif
