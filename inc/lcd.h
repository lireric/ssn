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
 * lcd.h
 *
 *  Created on: 22 дек. 2014 г.
 *      Author: eric
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lcd-320x240.h"
#include "touchscreen.h"
#include "utils.h"

#define mainTSRate				( ( portTickType ) 100 / portTICK_RATE_MS )
#define mainTS_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )

#endif /* INC_LCD_H_ */
