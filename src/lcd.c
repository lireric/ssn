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
 * lcd.c
 *
 *  Created on: 22 дек. 2014 г.
 *      Author: eric
 */

#include "lcd.h"

xSemaphoreHandle xMutexLCD = NULL;

// ************************************************************************************************
/* The touch screen task. */
	static void prvTSTask( void *pvParameters );

/*-----------------------------------------------------------*/


void lcd_tscal_cb(int x, int y)
{
		int color = LCD_COLOR_YELLOW;

		LCD_Clear(LCD_COLOR_BLACK);
		if(x)
			x = 238;
		if(y)
			y = 318;
		LCD_SetPixel(x,y, color);
		LCD_SetPixel(x+1,y, color);
		LCD_SetPixel(x,y+1, color);
		LCD_SetPixel(x+1,y+1, color);
}



int32_t lcd_init()
{
xTaskHandle hndTSTask;

int32_t xReturn = pdFALSE;

	LCD_Init();
	if (LCD_LcdIdOk()) {

		xMutexLCD = xSemaphoreCreateMutex();

		LCD_Clear(LCD_COLOR_BLACK);
		LCD_ConSetColor(LCD_COLOR_BLACK, LCD_COLOR_GREEN);
		LCD_ConSetPos(0, 0);
		ts_init();
		LCD_Clear(LCD_COLOR_BLUE);
		calib_set(334, 1790, 1734, 1793, 348, 240, 1745, 281);
		//	xReturn = ts_calibrate(lcd_tscal_cb);
		xReturn = pdTRUE;

		if (xReturn) {
			LCD_ConSetColor(LCD_COLOR_BLUE, LCD_COLOR_RED);
			LCD_ConSetPos(0, 0);
			LCD_Printf(xMutexLCD, "TOUCHSCREEN\nERROR");
		}

//		LCD_Printf(xMutexLCD, APP_NAME);
		LCD_ConSetColor(LCD_COLOR_BLACK, LCD_COLOR_WHITE);
//		LCD_Printf(xMutexLCD, "Initializing\nsystem\ncomponents...\n");

		xReturn = xTaskCreate(prvTSTask, (char *) "TSTask",	configMINIMAL_STACK_SIZE, NULL, mainTS_TASK_PRIORITY, &hndTSTask);

	}
	return xReturn;
}

/* The touch screen task. */
static void prvTSTask(void *pvParameters) {
	int x, y;
	int pressed;
	const uint16_t lcd_brush_colors[] = { LCD_COLOR_BLACK, LCD_COLOR_BLUE,
			LCD_COLOR_RED, LCD_COLOR_GREEN, LCD_COLOR_WHITE, LCD_COLOR_GRAY };
	enum {
		ST_GPS_INFO, ST_GSM_INFO, ST_DRAWING,
	} cstate = ST_DRAWING;
	int color_i = 0;
	int prev_pendown = 0;
	int lcd_on = 1;
	int lcd_backlight_on = 1;
	portTickType xLastWakeTime;
	/* Just to avoid compiler warnings. */
	(void) pvParameters;

	while (1) {

		ts_poll(&x, &y, &pressed);

		if (pressed) {
			if (!lcd_on) {
				lcd_on = 1;
				LCD_ExitSleep();
				LCD_SetBacklight(1);
			}
			if (cstate == ST_DRAWING) {
				if (y < 320)
					LCD_SetPixel(x, y, lcd_brush_colors[color_i]);
			} else {
				if (!prev_pendown && (y < 100)) {
					lcd_backlight_on = !lcd_backlight_on;
					LCD_SetBacklight(lcd_backlight_on);
				}
			}

			/* "Home" pressed. */
			if ((x > (48 * 0 + 10)) && (x < (48 * 0 + 38)) && (y >= 325)) {
				LCD_ConSetPos(0,0);
				LCD_ConSetColor(LCD_COLOR_BLACK, LCD_COLOR_WHITE);
				LCD_Printf(xMutexLCD, "home\n");
				LCD_Printf(xMutexLCD, "s:%d",uxTaskGetStackHighWaterMark(NULL));
				/*
				 struct gps_gprmc info;
				 if(!gps_get_gprmc(&info)) {
				 home_pos = info.pos;
				 home_pos_valid = true;
				 }
				 */
			}

			/* "Message" pressed. */
			if ((x > (48 * 1 + 10)) && (x < (48 * 1 + 38)) && (y >= 325)) {
//				if (!prev_pendown) {
					LCD_Printf(xMutexLCD, "Message\n");
//					LCD_Printf("i:%l",ulIdleCycleCount);
//				}
			}

			/* "Address Book" pressed. */
			if ((x > (48 * 2 + 10)) && (x < (48 * 2 + 38)) && (y >= 325)) {
				LCD_Clear(LCD_COLOR_BLACK);
				cstate = ST_GPS_INFO;
				LCD_Printf(xMutexLCD, "Address Book\n");
//				LCD_SetBacklight(1);
			}

			/* "Phone" pressed. */
			if ((x > (48 * 3 + 10)) && (x < (48 * 3 + 38)) && (y >= 325)) {
				LCD_Clear(LCD_COLOR_BLUE);
				cstate = ST_GSM_INFO;
				LCD_Printf(xMutexLCD, "Phone\n");
//				LCD_SetBacklight(0);
			}

			/* "Music" pressed. */
			if ((x > (48 * 4 + 10)) && (x < (48 * 4 + 38)) && (y >= 325)) {
				LCD_Printf(xMutexLCD, "Music\n");
				if (cstate != ST_DRAWING)
					LCD_Clear(LCD_COLOR_WHITE);
				cstate = ST_DRAWING;
				if (!prev_pendown) {
					if ((++color_i) > ARRAY_NUMELEM(lcd_brush_colors))
						color_i = 0;
				}
			}
		}
		vTaskDelayUntil(&xLastWakeTime,  mainTSRate );
	}
}
