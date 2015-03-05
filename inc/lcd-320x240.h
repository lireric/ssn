/****************************************************************************
 *
 * Project: -----
 *
 * Copyright by Olimex Ltd. All rights reserved.
 *
 * File: lcd-320x240.h
 * Description: Driver for the FPC-K320QVB-V1-O1 TFT LCD based on ILI9320 IC.
 * Developer: Dimitar Dimitrov ( dimitar,olimex.com )
 *
 * Last change: $Date$
 * Revision: $Revision: 29 $
 * Id: $Id$
 *
 ****************************************************************************/

#ifndef LCD_320X240_H
#define LCD_320X240_H

//#include "stm32f10x.h"
//#include "../inc/stm32f10x_lib.h"
#include "arm_comm.h"
#include <stdint.h>                           /* Include standard types */
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif


#define LCD_COLOR_BLACK		0x0000
#define LCD_COLOR_WHITE		0xffff
#define LCD_COLOR_GRAY		0x8410

#define LCD_COLOR_BLUE		0x001f
#define LCD_COLOR_GREEN		0x07e0
#define LCD_COLOR_RED		0xf800
	
#define LCD_COLOR_LIGHTRED	0xfc10

#define LCD_COLOR_YELLOW	0xffe0
#define LCD_COLOR_MAGENTA	0xf81f
#define LCD_COLOR_CYAN		0x07ff

/* Form an LCD 16-bit color out of the given 8-bit Red/Green/Blue components. */
#define LCD_RGB_TO_LCDCOLOR(R,G,B)	\
	( (((R)>>3)&0x1f)<<11 | (((G)>>2)&0x3f)<<5 | (((B)>>3)&0x1f)<<0)

extern void LCD_Init(void);
extern void LCD_EnterSleep(void);
extern void LCD_ExitSleep(void);
extern void LCD_EnterDeepStandby(void);
extern void LCD_SetBacklight(int st);
extern int LCD_LcdIdOk(void);
extern void LCD_SetTestPattern(void);
extern void LCD_SetRandomPattern(void);
extern void LCD_Clear(uint16_t bgcolor);

extern void LCD_SetCursor(unsigned int x, unsigned int y);
extern void LCD_WriteRAM(uint16_t color);

extern void LCD_SetPixel(unsigned int x, unsigned int y, uint16_t color);
extern void LCD_WriteMem(unsigned int addr, const uint16_t *data, unsigned int nitems);
extern void LCD_FillMem(unsigned int addr, const uint16_t data, unsigned int nitems);

extern void LCD_DrawChar(unsigned int row, unsigned int col, char c, uint16_t bgcolor, uint16_t fgcolor);
extern void LCD_DrawLine(int x0, int y0, int x1, int y1, uint16_t color);
extern void LCD_DrawCircle(int x, int y, int r, uint16_t color);
extern void LCD_RotatePoint(int *px, int *py, int x, int y, int rot_deg);
extern void LCD_DrawRect(int x, int y, int w, int h, uint16_t color);

extern void LCD_ConSetColor(uint16_t bgcolor, uint16_t fgcolor);
extern void LCD_ConSetPos(unsigned int row, unsigned int col);
extern void LCD_Printf(xSemaphoreHandle xMutexLCD, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif	/* LCD_320X240_H */


