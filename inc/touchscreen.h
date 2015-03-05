/****************************************************************************
 *
 * Project: ------
 *
 * Copyright by Olimex Ltd. All rights reserved.
 *
 * File: touchscreen.h
 * Description: Driver for the FPC-K320QVB-V1-O1 touchscreen.
 * Developer: Dimitar Dimitrov ( dimitar,olimex.com )
 *
 * Last change: $Date$
 * Revision: $Revision: 29 $
 * Id: $Id$
 *
 ****************************************************************************/

#ifndef TOUCHSCREEN_H
#define TOUCHSCREEN_H

//#include "stm32f10x_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ts_point {
	int x;
	int y;
};

struct ts_calib_data {
	struct ts_point xl_yu;
	struct ts_point xr_yu;
	struct ts_point xl_yd;
	struct ts_point xr_yd;
};

 void calib_set (int xl1, int yu1, int xr2, int yu2, int xl3, int yd3, int xr4, int yd4);

void ts_init(void);

int ts_calibrate(void (*tscal_ui_cb)(int x, int y));

void ts_poll(int *x, int *y, int *pressed);


#ifdef __cplusplus
}
#endif

#endif	/* TOUCHSCREEN_H */

