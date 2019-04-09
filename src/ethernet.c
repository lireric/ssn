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


//  GPIO_InitTypeDef GPIO_InitStructure;
//  // Port B output
//  GPIO_InitStructure.GPIO_Pin = WIZ_SCS;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_SetBits(GPIOB, WIZ_SCS);
//
//  // Port A output
//  GPIO_InitStructure.GPIO_Pin =LED0 | LED1 | LED2 | LED3;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  GPIO_ResetBits(GPIOA, LED0);
//  GPIO_ResetBits(GPIOA, LED1);
//  GPIO_SetBits(GPIOA, LED2); // led off
//  GPIO_SetBits(GPIOA, LED3); // led off
//
//  // SPI 1
//  /* Configure SPIy pins: SCK, MISO and MOSI */
//  GPIO_InitStructure.GPIO_Pin = WIZ_SCLK | WIZ_MISO | WIZ_MOSI;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//  // Port B output;
//  GPIO_InitStructure.GPIO_Pin = WIZ_RESET ;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_SetBits(GPIOB, WIZ_RESET);
//
//  // Port C input
//  GPIO_InitStructure.GPIO_Pin = WIZ_INT;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//  /* TIM2 clock enable */
//  RCC_APB1PeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2 | RCC_APB1Periph_SPI2, ENABLE);
//
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
//  											|RCC_APB2Periph_AFIO  | RCC_APB2Periph_USART1, ENABLE);
//  /* Enable the TIM2 global Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//   /* Enable the TIM2 global Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//  /* Set WWDG interrupt vector Preemption Priority to 1 */
//  NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

}

void ethernetInitDHCP()
{

//	  uint8 pc_ip[4]={192,168,1,104};
//	  uint16 pc_port=6001;
//	  uint16 len=0;
//	  uint8 rIP[4];
//	  uint16 rPort;
	  uint8 mac[6]={0x00,0x08,0xdc,0x11,0x11,0x12};
	  uint8 lip[4]={192,168,1,111};
	  uint8 sub[4]={255,255,255,0};
	  uint8 gw[4]={192,168,1,5};
	  uint8 ip[4];
//	  uint8 test[10] = "test";
//	  uint32 counter=0;

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
