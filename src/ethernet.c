/*
 * ethernet.c
 *
 *  Created on: 13 мар. 2019 г.
 *      Author: eric
 */

#define M_ETHERNET // to do: remove!!!

#ifdef  M_ETHERNET

#include "../inc/ssn.h"
#include "ethernet.h"
#include "w5500/socket.h"
#include "utils.h"
#include "w5500/w5500.h"
#include "w5500/SPI2.h"
#include "device.h"
#include "sockutil.h"
#include "commands.h"

//extern uint8 txsize[];
//extern uint8 rxsize[];

uint8 txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
uint8 rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};

extern uint8 nCurrentInternetState;
extern uint8 mac[6];
extern uint8 lip[4];
extern uint8 sub[4];
extern uint8 gw[4];

// to do !!!
void Reset_W5500(void)
{
	gpio_clear(GPIOA, WIZ_RESET);
	delay_nus2(50);
	gpio_set(GPIOA, WIZ_RESET);
	delay_nus2(160000);

}

uint8 getEthernetState(void)
{
	return nCurrentInternetState;
}

// to do !!!
void EthernetHardwareInit(void)
{

	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, WIZ_RESET);
	gpio_set(GPIOA, WIZ_RESET);

	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, WIZ_INT);

}

void ethernetInit()
{

//	  uint8 mac[6]=ETHERNET_LOCAL_MAC;
//	  uint8 lip[4]=ETHERNET_SSN_SIP;
//	  uint8 sub[4]=ETHERNET_SSN_SN;
//	  uint8 gw[4]=ETHERNET_SSN_GW;
	  uint8 ip[4];

	  xprintfMsg("\r\nW5500 EVB initialization over.");

	  Reset_W5500();
	  WIZ_SPI_Init();
	  xprintfMsg("\r\nW5500 initialized!");

	  setSHAR(mac);
	  setSUBR(sub);
	  setGAR(gw);
	  setSIPR(lip);

	    //Init. TX & RX Memory size of w5500
	  sysinit(txsize, rxsize);

	  setRTR(2000);
	  setRCR(3);


	  getSIPR (ip);
	  xprintfMsg("IP : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
	  getSUBR(ip);
	  xprintfMsg("SN : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
	  getGAR(ip);
	  xprintfMsg("GW : %d.%d.%d.%d\r\n", ip[0],ip[1],ip[2],ip[3]);
	  xprintfMsg("Network is ready.\r\n");

}

#endif
