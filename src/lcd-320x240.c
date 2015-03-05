/****************************************************************************
 *
 * Project: -----
 *
 * Copyright by Olimex Ltd. All rights reserved.
 *
 * File: lcd-320x240.c
 * Description: Driver for the FPC-K320QVB-V1-O1 TFT LCD based on ILI9320 IC.
 * Developer: Dimitar Dimitrov ( dimitar,olimex.com )
 *
 * Last change: $Date$
 * Revision: $Revision: 29 $
 * Id: $Id$
 *
 ****************************************************************************/

//#include <stdlib.h>
#include <stdarg.h>
//#include <stdio.h>
#include "xprintf.h"

#include <assert.h>
#include <math.h>
//#include "stm32f10x.h"
//#include "usb_type.h"
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/fsmc.h>
#include "fsmc_func.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_fsmc.h"
#include "../inc/lcd-320x240.h"
#include "../inc/fonts.h"

#include "FreeRTOS.h"
#include "semphr.h"

/**
 * @file
 * Driver for the FPC-K320QVB-V1-O1 TFT LCD based on ILI9320 IC.
 *
 */



/* AHB clock period in nanoseconds */
#define T_HCK	14



//#define LCDBUS_RSLOW_ADDR	((vuint16_t *)0x60000000)
//#define LCDBUS_RSHIGH_ADDR	((vuint16_t *)(0x60000000 | (1<<(19+1))))

#define LCDBUS_RSLOW_ADDR	((__IO uint16_t *)0x60000000)
#define LCDBUS_RSHIGH_ADDR	((__IO uint16_t *)(0x60000000 | (1<<(19+1))))

static void Delay(volatile int i)
{
	for(;i;--i) {
		volatile int j;
		for(j=0; j<100;++j);
	}
}

static void lcdreset_set(int val)
{
	if(val) {
//		GPIO_SetBits(GPIOE, GPIO_Pin_2);
		gpio_set(GPIOE, GPIO2);

	} else {
//		GPIO_ResetBits(GPIOE, GPIO_Pin_2);
		gpio_clear(GPIOE, GPIO2);
	}
}

static void lcdled_set(int val)
{
	if(!val) {
//		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		gpio_set(GPIOD, GPIO13);
	} else {
//		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		gpio_clear(GPIOD, GPIO13);
	}
}

static void lcdreset_init(void)
{
	  gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_2_MHZ,
			      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);

//GPIO_InitTypeDef GPIO_InitStructure;
//
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_Init(GPIOE, &GPIO_InitStructure);

  
  lcdreset_set(1);
}

static void lcdled_init(void)
{
	  gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_2_MHZ,
			      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

//GPIO_InitTypeDef GPIO_InitStructure;
//
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  lcdled_set(1);
}


/* Initialize the FSMC bus used for driving the LCD */
static void FSMC_LCDBUS_Init(void)
{

	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_GPIOG);
	rcc_periph_clock_enable(RCC_FSMC);


FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  pw,pr;
//  GPIO_InitTypeDef GPIO_InitStructure;
    
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOE |
//                         RCC_APB2Periph_GPIOF, ENABLE);
  
/*-- GPIO Configuration ------------------------------------------------------*/
  /* SRAM Data lines configuration */
  gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0 | GPIO1 | GPIO8 | GPIO9 | GPIO10 | GPIO14 | GPIO15);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
//                                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
//                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
//                                GPIO_Pin_15;
//  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  //TODO - get rid of address lines!!!
  /* SRAM Address lines configuration */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
//                                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 |
//                                GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_Init(GPIOF, &GPIO_InitStructure);
  gpio_set_mode(GPIOF, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
//                                GPIO_Pin_4 | GPIO_Pin_5;
//  GPIO_Init(GPIOG, &GPIO_InitStructure);
  gpio_set_mode(GPIOG, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5);
  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
  gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO11 | GPIO12 | GPIO13);
   
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_Init(GPIOE, &GPIO_InitStructure);
  gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO3);
   
  /* NOE and NWE configuration */  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
  gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4 | GPIO5);
  
  /* NE1 configuration */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
  gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);
  
 
  /* NBL0, NBL1 configuration */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//  GPIO_Init(GPIOE, &GPIO_InitStructure);
  gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_50_MHZ,
		  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0 | GPIO1);
  
/*-- FSMC Configuration ------------------------------------------------------*/
  pr.FSMC_AddressSetupTime = (5/T_HCK+1);
  pr.FSMC_AddressHoldTime = (5/T_HCK+1);
  pr.FSMC_DataSetupTime = (100/T_HCK+1);
  pr.FSMC_BusTurnAroundDuration = 0;
  pr.FSMC_CLKDivision = 0;
  pr.FSMC_DataLatency = 0;
  pr.FSMC_AccessMode = FSMC_AccessMode_A;

  pw.FSMC_AddressSetupTime = (5/T_HCK+1);
  pw.FSMC_AddressHoldTime = (5/T_HCK+1);
  pw.FSMC_DataSetupTime = ((20+15)/T_HCK+1);
  pw.FSMC_BusTurnAroundDuration = 0;
  pw.FSMC_CLKDivision = 0;
  pw.FSMC_DataLatency = 0;
  pw.FSMC_AccessMode = FSMC_AccessMode_A;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Enable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &pr;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &pw;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  /* Enable FSMC Bank1_SRAM Bank */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);  
}


#if 0
static void LCDBUS_Write(uint16_t val, int rs)
{
	if(rs) {
		*LCDBUS_RSHIGH_ADDR = val;
	} else {
		*LCDBUS_RSLOW_ADDR = val;
	}
}


static uint16_t LCDBUS_Read(int rs)
{
	u32 val;
	
	if(rs) {
		val = *LCDBUS_RSHIGH_ADDR;
	} else {
		val = *LCDBUS_RSLOW_ADDR;
	}
	
	return val;
}
#endif


static void LCDBUS_WriteReg(uint16_t regn, uint16_t val)
{
	*LCDBUS_RSLOW_ADDR = regn;
	*LCDBUS_RSHIGH_ADDR = val;
}


static uint16_t LCDBUS_ReadReg(uint16_t regn)
{
	uint32_t val;
	
	*LCDBUS_RSLOW_ADDR = regn;
	val = *LCDBUS_RSHIGH_ADDR;
	
	return val;
}

int LCD_LcdIdOk(void)
{
	return (LCDBUS_ReadReg(0) == 0x9320);
}

void LCD_SetCursor(unsigned int x, unsigned int y)
{
	unsigned int addr;
	
	addr = y*0x100 + x;
	
	LCDBUS_WriteReg(0x20, addr&0xff);	/* low addr */
	LCDBUS_WriteReg(0x21, (addr>>8)&0x1ff);	/* high addr */
	*LCDBUS_RSLOW_ADDR = 0x22;		/* data reg in IR */
}

void LCD_WriteRAM(uint16_t color)
{
	*LCDBUS_RSHIGH_ADDR = color;
}

// x=[0-240] , y=[0-320]
void LCD_SetPixel(unsigned int x, unsigned int y, uint16_t color)
{
	LCD_SetCursor(x,y);
	LCD_WriteRAM(color);
}

// x=[0-240] , y=[0-320]
uint16_t LCD_GetPixel(unsigned int x, unsigned int y)
{
	unsigned int addr;
//	static volatile uint16_t RRR;
	
	addr = y*0x100 + x;
	
	LCDBUS_WriteReg(0x20, addr&0xff);	/* low addr */
	LCDBUS_WriteReg(0x21, (addr>>8)&0x1ff);	/* high addr */
	LCDBUS_ReadReg(0x22);			/* data latch */
	return *LCDBUS_RSHIGH_ADDR;		/* data */
}


void LCD_WriteMem(unsigned int addr, const uint16_t *data, unsigned int nitems)
{
	LCDBUS_WriteReg(0x20, addr&0xff);	/* low addr */
	LCDBUS_WriteReg(0x21, (addr>>8)&0x1ff);	/* high addr */
	*LCDBUS_RSLOW_ADDR = 0x22;
	while(nitems--) {
		*LCDBUS_RSHIGH_ADDR = *data++;
	}
}

void LCD_FillMem(unsigned int addr, const uint16_t data, unsigned int nitems)
{
	LCDBUS_WriteReg(0x20, addr&0xff);	/* low addr */
	LCDBUS_WriteReg(0x21, (addr>>8)&0x1ff);	/* high addr */
	*LCDBUS_RSLOW_ADDR = 0x22;
	while(nitems--) {
		*LCDBUS_RSHIGH_ADDR = data;
	}
}


void LCD_Disable(void)
{
	lcdreset_set(0);
	LCD_SetBacklight(0);

}
void LCD_Init(void)
{
	FSMC_LCDBUS_Init();
	lcdreset_init();
	lcdled_init();
	
	lcdled_set(1);
	
	lcdreset_set(1);
	Delay(1000);
	lcdreset_set(0);
	Delay(1000);
	lcdreset_set(1);
	Delay(1000);
	
	Delay(500); /* delay 50 ms */
/* Start Initial Sequence ----------------------------------------------------*/
	LCDBUS_WriteReg(0x00E5,0x8000); /* Set the internal vcore voltage */
	LCDBUS_WriteReg(0x0000, 0x0001); /* Start internal OSC. */
	Delay(500);
	LCDBUS_WriteReg(0x00a4, 0x0001); /* calb */
	LCDBUS_WriteReg(0x0007, 0x0000); /* display control */
	Delay(500);
	LCDBUS_WriteReg(0x0001, 0x0100); /* set SS and SM bit */
	LCDBUS_WriteReg(0x0002, 0x0700); /* set 1 line inversion */
	LCDBUS_WriteReg(0x0003, 0x1030); /* set GRAM write direction and BGR=1. */
	LCDBUS_WriteReg(0x0004, 0x0000); /* Resize register */
	LCDBUS_WriteReg(0x0008, 0x0202); /* set the back porch and front porch */
	LCDBUS_WriteReg(0x0009, 0x0000); /* set non-display area refresh cycle ISC[3:0] */
#if 0
	LCDBUS_WriteReg(0x000a, 0x0000); /* FMARK function */
	LCDBUS_WriteReg(0x000c, 0x0000); /* RGB interface setting */
	LCDBUS_WriteReg(0x000d, 0x0000); /* Frame marker Position */
	LCDBUS_WriteReg(0x000f, 0x0000); /* RGB interface polarity */
#endif
/* Power On sequence ---------------------------------------------------------*/
	LCDBUS_WriteReg(0x0007, 0x0101); /* power control 1 BT, AP */
	LCDBUS_WriteReg(0x0017, 0x0001); /* ????? */

	LCDBUS_WriteReg(0x0010, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
	LCDBUS_WriteReg(0x0011, 0x0007); /* DC1[2:0], DC0[2:0], VC[2:0] */
	LCDBUS_WriteReg(0x0012, 0x0000); /* VREG1OUT voltage */
	LCDBUS_WriteReg(0x0013, 0x0000); /* VDV[4:0] for VCOM amplitude */
	Delay(2000);                 /* Dis-charge capacitor power voltage (200ms) */
	LCDBUS_WriteReg(0x0010, 0x16B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
	LCDBUS_WriteReg(0x0011, 0x0037); /* DC1[2:0], DC0[2:0], VC[2:0] */
	Delay(500);                  /* Delay 50 ms */
	LCDBUS_WriteReg(0x0012, 0x013e); /* VREG1OUT voltage */
	Delay(500);                  /* Delay 50 ms */
	LCDBUS_WriteReg(0x0013, 0x1a00); /* VDV[4:0] for VCOM amplitude */
	LCDBUS_WriteReg(0x0029, 0x000f); /* VCM[4:0] for VCOMH */
	Delay(500);                  /* Delay 50 ms */
	LCDBUS_WriteReg(0x0020, 0x0000); /* GRAM horizontal Address */
	LCDBUS_WriteReg(0x0021, 0x0000); /* GRAM Vertical Address */

/* Set GRAM area -------------------------------------------------------------*/
	LCDBUS_WriteReg(0x0050, 0x0000); /* Horizontal GRAM Start Address */
	LCDBUS_WriteReg(0x0051, 0x00EF); /* Horizontal GRAM End Address */
	LCDBUS_WriteReg(0x0052, 0x0000); /* Vertical GRAM Start Address */
	LCDBUS_WriteReg(0x0053, 0x013F); /* Vertical GRAM End Address */
		
	LCDBUS_WriteReg(0x0060, 0x2700); /* Gate Scan Line */
	LCDBUS_WriteReg(0x0061, 0x0001); /* NDL,VLE, REV */
	LCDBUS_WriteReg(0x006a, 0x0000); /* set scrolling line */

/* Panel Control -------------------------------------------------------------*/
	LCDBUS_WriteReg(0x0090, 0x0010);
	LCDBUS_WriteReg(0x0092, 0x0000);
	LCDBUS_WriteReg(0x0093, 0x0000);
#if 0
	LCDBUS_WriteReg(0x0095, 0x0110);
	LCDBUS_WriteReg(0x0097, 0x0000);
	LCDBUS_WriteReg(0x0098, 0x0000);
#endif

/* Adjust the Gamma Curve ----------------------------------------------------*/
	LCDBUS_WriteReg(0x0030, 0x0007);
	LCDBUS_WriteReg(0x0031, 0x0403);
	LCDBUS_WriteReg(0x0032, 0x0404);
	LCDBUS_WriteReg(0x0035, 0x0002);
	LCDBUS_WriteReg(0x0036, 0x0707);
	LCDBUS_WriteReg(0x0037, 0x0606);
	LCDBUS_WriteReg(0x0038, 0x0106);
	LCDBUS_WriteReg(0x0039, 0x0007);
	LCDBUS_WriteReg(0x003c, 0x0700);
	LCDBUS_WriteReg(0x003d, 0x0707);
  
#if 0
/* Partial Display Control ---------------------------------------------------*/
	LCDBUS_WriteReg(0x0080, 0x0000);
	LCDBUS_WriteReg(0x0081, 0x0000);
	LCDBUS_WriteReg(0x0082, 0x0000);
	LCDBUS_WriteReg(0x0083, 0x0000);
	LCDBUS_WriteReg(0x0084, 0x0000);
	LCDBUS_WriteReg(0x0085, 0x0000);
#endif
	LCDBUS_WriteReg(0x0007, 0x0173); /* 262K color and display ON */
	
	
#if 0
	//TODO - remove me! 
	// -> hardware test pattern
	for(;;) {
		*LCDBUS_RSHIGH_ADDR = 0xAAAA;
		*LCDBUS_RSLOW_ADDR = 0x5555;
		(void)*LCDBUS_RSHIGH_ADDR;
		lcdreset_set(0);
		lcdreset_set(1);
	}
#endif
}


void LCD_ExitSleep(void)
{
	//*************Power On sequence ******************//
	LCDBUS_WriteReg(0x0010, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
	LCDBUS_WriteReg(0x0011, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
	LCDBUS_WriteReg(0x0012, 0x0000); /* VREG1OUT voltage */
	LCDBUS_WriteReg(0x0013, 0x0000); /* VDV[4:0] for VCOM amplitude */
	Delay(2000); 			/* Dis-charge capacitor power voltage */
	LCDBUS_WriteReg(0x0010, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
	LCDBUS_WriteReg(0x0011, 0x0147); /* DC1[2:0], DC0[2:0], VC[2:0] */
	Delay(500); 
	LCDBUS_WriteReg(0x0012, 0x013C); /* VREG1OUT voltage */
	Delay(500); 
	LCDBUS_WriteReg(0x0013, 0x0E00); /* VDV[4:0] for VCOM amplitude */
	LCDBUS_WriteReg(0x0029, 0x0009); /* VCM[4:0] for VCOMH */
	Delay(500); 
	LCDBUS_WriteReg(0x0007, 0x0173); /* 262K color and display ON */
}


void LCD_EnterSleep(void)
{
	LCDBUS_WriteReg(0x0007, 0x0000); /* display OFF */
	//************* Power OFF sequence **************//
	LCDBUS_WriteReg(0x0010, 0x0000); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
	LCDBUS_WriteReg(0x0011, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
	LCDBUS_WriteReg(0x0012, 0x0000); /* VREG1OUT voltage */
	LCDBUS_WriteReg(0x0013, 0x0000); /* VDV[4:0] for VCOM amplitude */
	Delay(2000); /* Dis-charge capacitor power voltage */
	LCDBUS_WriteReg(0x0010, 0x0002); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
}


void LCD_EnterDeepStandby(void)
{
	LCDBUS_WriteReg(0x0007, 0x0000); /* display OFF */
	//************* Power OFF sequence **************//
	LCDBUS_WriteReg(0x0010, 0x0000); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
	LCDBUS_WriteReg(0x0011, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
	LCDBUS_WriteReg(0x0012, 0x0000); /* VREG1OUT voltage */
	LCDBUS_WriteReg(0x0013, 0x0000); /* VDV[4:0] for VCOM amplitude */
	Delay(2000); /* Dis-charge capacitor power voltage */
	LCDBUS_WriteReg(0x0010, 0x0004); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
}



void LCD_SetBacklight(int st)
{
	lcdled_set(st);
}

void LCD_SetTestPattern(void)
{
	const unsigned int chunk = (320*240)/10;
	unsigned int i;
	
	LCD_SetCursor(0,0);
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_WHITE);
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_BLACK);
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_GRAY);
	
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_BLUE);
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_GREEN);
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_RED);
	
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_YELLOW);
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_MAGENTA);
	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_CYAN);

	for(i=0; i<chunk; ++i) 
		LCD_WriteRAM(LCD_COLOR_WHITE);
}

void LCD_SetRandomPattern(void)
{
//	unsigned int i;
	LCD_SetCursor(0,0);
//	for(i=0; i<(320*240); ++i)
//		LCD_WriteRAM(rand() & 0xffff);
}

void LCD_Clear(uint16_t bgcolor)
{
	LCD_FillMem(0, bgcolor, 320*240);
}


void LCD_DrawChar(unsigned int row, unsigned int col, char c, uint16_t bgcolor, uint16_t fgcolor)
{
	unsigned int x,y;
	int i,j;
	uint16_t *font_char;

	x = 16 * col;
	if(x > (240-16))
		x = 240-16;
	y = 24 * row;
	if(y > (320-24))
		y = 320-24;

	c -= 32;
	if(c<=0) c = 0;
	if(c>94) c = 94;

	font_char = &ASCII_Table[c*24];
	LCD_SetCursor(x,y);
	for(i=0; i<24; i++) {
		for(j=0; j<16; ++j) {
			if(*font_char & (1u<<j))
				LCD_WriteRAM(fgcolor);
			else
				LCD_WriteRAM(bgcolor);
		}
		font_char++;
		LCD_SetCursor(x,y+i);
	}
}

#define SWAP_INT(A,B)	do {int __x; __x = (A); (A) = (B); (B) = __x;} while(0)

void LCD_DrawLine_norm(int x0, int y0, int x1, int y1, uint16_t color, bool swap)
{
	const int mul = (1<<12);
	int a, b, y;

	assert(x0 < x1);
	assert(y0 != y1);

	a = ((y1 - y0) * mul) / (x1 - x0);
	b = ((y0 * x1 - y1 * x0) * mul) / (x1 - x0);

	while(x0 <= x1) {
		y = a * x0 + b;
		y = (y + mul/2) / mul;
		if(!swap) {
			LCD_SetPixel(x0, y, color);
		} else {
			LCD_SetPixel(y, x0, color);
		}
		x0++;
	}
}

void LCD_DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
	if(x0 == x1) {
		LCD_SetPixel(x0, y0, color);
		while(y0 != y1) {
			if(y0<y1)
				y0++;
			else
				y0--;
			LCD_SetPixel(x0, y0, color);
		}
		return;
	}
	if(y0 == y1) {
		LCD_SetPixel(x0, y0, color);
		while(x0 != x1) {
			if(x0<x1)
				x0++;
			else
				x0--;
			LCD_SetPixel(x0, y0, color);
		}
		return;
	}
/*	if(labs(x1-x0) > labs(y1-y0)) {
		if(x0 > x1) {
			SWAP_INT(x0, x1);
			SWAP_INT(y0, y1);
		}
		LCD_DrawLine_norm(x0,y0,x1,y1, color, FALSE);
	} else {
		if(y0 > y1) {
			SWAP_INT(x0, x1);
			SWAP_INT(y0, y1);
		}
		LCD_DrawLine_norm(y0,x0,y1,x1, color, TRUE);
	}*/
}


void LCD_DrawCircle(int x, int y, int r, uint16_t color)
{
	int i, j;

	for(i=0; i<=r; i++) {
		j = (int)sqrt(r*r - i*i);
		LCD_SetPixel(x+i, y+j, color);
		LCD_SetPixel(x-i, y+j, color);
		LCD_SetPixel(x+i, y-j, color);
		LCD_SetPixel(x-i, y-j, color);
	}
	for(j=0; j<=r; j++) {
		i = (int)sqrt(r*r - j*j);
		LCD_SetPixel(x+i, y+j, color);
		LCD_SetPixel(x-i, y+j, color);
		LCD_SetPixel(x+i, y-j, color);
		LCD_SetPixel(x-i, y-j, color);
	}
}

#ifndef M_PI
  #define M_PI		3.14159265358979323846
#endif

void LCD_RotatePoint(int *px, int *py, int x, int y, int rot_deg)
{
	int rel_px = *px - x;
	int rel_py = *py - y;
	float polar_angle, polar_r;

	polar_r = sqrt(rel_px*rel_px + rel_py*rel_py);
	polar_angle = asin((float)rel_py / polar_r);
	if(rel_px < 0) {
		if(polar_angle >= 0.0) {
			polar_angle += M_PI/2;
		} else {
			polar_angle -= M_PI/2;
		}
	}

	polar_angle += ((float)rot_deg * M_PI) / 180.0;

	*px = (int)floor(cos(polar_angle) * polar_r) + x;
	*py = (int)floor(sin(polar_angle) * polar_r) + y;
}

void LCD_DrawRect(int x, int y, int w, int h, uint16_t color)
{
	int i;

	for(i=0; i<h; i++, y++) {
		uint32_t addr = y*0x100 + x;
		LCD_FillMem(addr, color, w);
	}
}


/*-------------------------------------------------------------------------*/
static uint16_t LCD_ConBgColor;
static uint16_t LCD_ConFgColor;
static unsigned int LCD_ConRow;
static unsigned int LCD_ConCol;

void LCD_ConSetColor(uint16_t bgcolor, uint16_t fgcolor)
{
	LCD_ConBgColor = bgcolor;
	LCD_ConFgColor = fgcolor;
}


void LCD_ConSetPos(unsigned int row, unsigned int col)
{
	LCD_ConRow = row;
	LCD_ConCol = col;
}


void LCD_Printf(xSemaphoreHandle xMutexLCD, const char *fmt, ...)
{
	static char buf[256];
	char *p;
	va_list lst;

	if (xMutexLCD) xSemaphoreTake( xMutexLCD, portMAX_DELAY );

	va_start(lst, fmt);
//	vsprintf(buf, fmt, lst);
	xsprintf(buf, fmt, lst);
	va_end(lst);

	p = buf;
	while(*p) {
		if(*p == '\n') {
			while(LCD_ConCol < 15) {
				LCD_DrawChar(LCD_ConRow, LCD_ConCol++, ' ', 
					    LCD_ConBgColor, LCD_ConFgColor);
			}

			LCD_ConRow++;
			LCD_ConCol = 0;
		} else {
			if(LCD_ConCol >= 15) {
				LCD_ConRow++;
				LCD_ConCol = 0;
			}
			LCD_DrawChar(LCD_ConRow, LCD_ConCol++, *p, 
					LCD_ConBgColor, LCD_ConFgColor);
		}
		p++;
	}
	if (xMutexLCD) xSemaphoreGive( xMutexLCD );
}


