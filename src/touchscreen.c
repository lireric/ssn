/****************************************************************************
 *
 * Project: ------
 *
 * Copyright by Olimex Ltd. All rights reserved.
 *
 * File: touchscreen.h
 * Description: Generic touchscreen routines.
 * Developer: Dimitar Dimitrov ( dimitar,olimex.com )
 *
 * Last change: $Date$
 * Revision: $Revision: 29 $
 * Id: $Id$
 *
 ****************************************************************************/

/* Modifications for SSN project: using libopencm3 */

#include <assert.h>
#include <limits.h>

#include "../inc/touchscreen-hw.h"
#include "../inc/touchscreen.h"

/*
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
*/

struct ts_coefficients {
	int a_x;
	int b_x;
	int a_y;
	int b_y;
};
static struct ts_coefficients coef = {0,0,0,0};

/* This may be filled in by a calibration routine. Five-point
 * calibration would be much nicer. */
static struct ts_calib_data calib = {
	.xl_yu = {0x00000118l, 0x000006fel},
	.xr_yu = {0x06c5, 0x06e1},
	.xl_yd = {0x0158, 0x00d5},
	.xr_yd = {0x06eb, 0x0100},
};


static void calc_coef(int val_low, int val_high, int min, int max, int *a, int *b)
{
	/*
	 * f(val) = a * val + b
	 * f(val_low) = min
	 * f(val_high) = max
	 *
	 * a = ?
	 * b = ?
	 */
	if(val_high == val_low) {
		assert(0);
		return;
	}
	*a = (max - min) / (val_high - val_low);
	*b = (min * val_high - max * val_low) / (val_high - val_low);
}


static void ts_recalculate_calibdata(void)
{
	int a, b;

	calc_coef((calib.xl_yu.x + calib.xl_yd.x)/2, 
			(calib.xr_yu.x + calib.xr_yd.x)/2,
			0, 
			240*1024,
			&a,
			&b);
	coef.a_x = a;
	coef.b_x = b;

	calc_coef((calib.xl_yu.y + calib.xr_yu.y)/2, 
			(calib.xl_yd.y + calib.xr_yd.y)/2,
			0, 
			320*1024,
			&a,
			&b);
	coef.a_y = a;
	coef.b_y = b;
}


void ts_init(void)
{
	tshw_init();
	ts_recalculate_calibdata();
}

void calib_set (int xl1, int yu1, int xr2, int yu2, int xl3, int yd3, int xr4, int yd4) {
			calib.xl_yu.x = xl1;
			calib.xl_yu.y = yu1;
			calib.xr_yu.x = xr2;
			calib.xr_yu.y = yu2;
			calib.xl_yd.x = xl3;
			calib.xl_yd.y =	yd3;
			calib.xr_yd.x = xr4;
			calib.xr_yd.y = yd4;
		ts_recalculate_calibdata();
	};


void ts_get_sample(uint16_t *adc_x, uint16_t *adc_y)
{
	int pressed;
	uint16_t tmpx, tmpy;

	do {
		tshw_poll(adc_x, adc_y, &pressed);
	} while(!pressed);
	do {
		tshw_poll(&tmpx, &tmpy, &pressed);
		/*if(pressed) {
			*adc_x = tmpx;
			*adc_y = tmpy;
		}*/
	} while(pressed);
}

int ts_calibrate(void (*tscal_ui_cb)(int x, int y))
{
	uint16_t adc_x, adc_y;

	assert(tscal_ui_cb);

	tscal_ui_cb(0,0);
	ts_get_sample(&adc_x, &adc_y);
	calib.xl_yu.x = adc_x;
	calib.xl_yu.y = adc_y;

	tscal_ui_cb(INT_MAX,0);
	ts_get_sample(&adc_x, &adc_y);
	calib.xr_yu.x = adc_x;
	calib.xr_yu.y = adc_y;

	tscal_ui_cb(INT_MAX,INT_MAX);
	ts_get_sample(&adc_x, &adc_y);
	calib.xr_yd.x = adc_x;
	calib.xr_yd.y = adc_y;

	tscal_ui_cb(0,INT_MAX);
	ts_get_sample(&adc_x, &adc_y);
	calib.xl_yd.x = adc_x;
	calib.xl_yd.y = adc_y;
	
	ts_recalculate_calibdata();

	return 0;
}


void ts_poll(int *x, int *y, int *pressed)
{
	const unsigned int num_avg = 2;
	uint16_t adc_x = 0, adc_y = 0;
	int i;

	for(i=0; i<num_avg; ++i) {
		uint16_t tmp_x, tmp_y;
		tshw_poll(&tmp_x, &tmp_y, pressed);
		adc_x += tmp_x;
		adc_y += tmp_y;
		if(!*pressed)
			break;
	}

	adc_x /= num_avg;
	adc_y /= num_avg;
	if(*pressed) {
		*x = (coef.a_x * adc_x + coef.b_x) / 1024;
		*y = (coef.a_y * adc_y + coef.b_y) / 1024;
	}
}


