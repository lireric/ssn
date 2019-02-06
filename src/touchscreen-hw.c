/****************************************************************************
 *
 * Project: ------
 *
 * Copyright by Olimex Ltd. All rights reserved.
 *
 * File: touchscreen-hw.h
 * Description: Driver for the FPC-K320QVB-V1-O1 touchscreen.
 * Developer: Dimitar Dimitrov ( dimitar,olimex.com )
 *
 * Last change: $Date$
 * Revision: $Revision: 29 $
 * Id: $Id$
 *
 ****************************************************************************/

/* Modifications for SSN project: using libopencm3 */

#include "touchscreen-hw.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>

#define TS_PIN_XR	GPIO3
#define TS_PIN_YU	GPIO1
#define TS_PIN_XL	GPIO2
#define TS_PIN_YD	GPIO0
#define TS_PORT		GPIOC

#define TS_ADCCH_XR	ADC_CHANNEL13
#define TS_ADCCH_YU	ADC_CHANNEL11
#define TS_ADCCH_XL	ADC_CHANNEL12
#define TS_ADCCH_YD	ADC_CHANNEL10
#define TS_ADC		ADC3


static void delay(volatile int i)
{
	for(;i;--i);
}


void tshw_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

	/* Make sure the ADC doesn't run during config. */
//	adc_off(TS_ADC);
    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
    adc_set_single_conversion_mode(TS_ADC);
//    adc_enable_trigger(TS_ADC, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(TS_ADC);

	tshw_prepare_wait();
	delay(100);
}

static void GPIO_SetAnalog(uint16_t pin)
{
	gpio_set_mode(TS_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_ANALOG, pin);
}


static void GPIO_SetOutput(uint16_t pin, int val)
{
	gpio_set_mode(TS_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, pin);

  
	if(val) {
		gpio_set(TS_PORT, pin);
	} else {
		gpio_clear(TS_PORT, pin);
	}
}

static void GPIO_SetInputPulledUp(uint16_t pin)
{
	gpio_set_mode(TS_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, pin);
}

static int GPIO_GetInputState(uint16_t pin)
{
	return gpio_get(TS_PORT, pin);
}


static uint16_t ADC_Measure(uint16_t ch)
{
	uint16_t val;
	uint8_t channel_array[16];

	/* Make sure the ADC doesn't run during config. */
//	adc_off(TS_ADC);
	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(TS_ADC);
	adc_set_single_conversion_mode(TS_ADC);
	adc_disable_external_trigger_regular(TS_ADC);
	adc_set_right_aligned(TS_ADC);
	
	/* ADC regular channel14 configuration */ 
//    adc_set_sample_time(TS_ADC, ch, ADC_SMPR_SMP_55DOT5CYC);
	adc_set_sample_time_on_all_channels(TS_ADC, ADC_SMPR_SMP_55DOT5CYC);
//	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
//	ADC_RegularChannelConfig(TS_ADC, ch, 1, ADC_SampleTime_55Cycles5);
	
	/* Enable ADC */
//	ADC_Cmd(TS_ADC, ENABLE);
    adc_power_on(TS_ADC);
	delay(100);
	
#if 1
	/* Enable ADC reset calibaration register */   
    adc_reset_calibration(TS_ADC);
//	ADC_ResetCalibration(TS_ADC);
	/* Check the end of ADC reset calibration register */
//	while(ADC_GetResetCalibrationStatus(TS_ADC));
	
	/* Start ADC calibaration */
//	ADC_StartCalibration(TS_ADC);
//    adc_calibration(TS_ADC);
    adc_calibrate_async(TS_ADC);
	/* Check the end of ADC calibration */
//	while(ADC_GetCalibrationStatus(TS_ADC));
#endif     

    /* Select the channel we want to convert. */
	channel_array[0] = ch;
	adc_set_regular_sequence(TS_ADC, 1, channel_array);

	/* Start ADC Software Conversion */ 
//    adc_start_conversion_regular(TS_ADC);
	adc_start_conversion_direct(TS_ADC);
	
	while(!adc_eoc(TS_ADC));

//    val = adc_read_regular(TS_ADC);
    val = ADC_DR(TS_ADC);
	
//    adc_off(TS_ADC);

	return val;
}

//static
void tshw_get_raw(uint16_t *adc_x, uint16_t *adc_y)
{
	uint32_t val1, val2;
	
	/*
	 * YD=0 YU=1, measure XL and XR
	 * YD=1 YU=0, measure XL and XR
	 * The average of the previous four samples is the X value.
	 *
	 * XL=0 XR=1, measure YD and YU
	 * XL=1 XR=0, measure YD and YU
	 * The average of the previous four samples is the Y value.
	 *
	 * A total of 8 ADC measurements are needed!
	 */
	
	GPIO_SetAnalog(TS_PIN_XL | TS_PIN_XR);
	GPIO_SetOutput(TS_PIN_YD, 0);
	GPIO_SetOutput(TS_PIN_YU, 1);
	delay(100);
	val1 = (ADC_Measure(TS_ADCCH_XL) + ADC_Measure(TS_ADCCH_XR))/2;
	
	GPIO_SetOutput(TS_PIN_YD, 1);
	GPIO_SetOutput(TS_PIN_YU, 0);
	delay(100);
	val2 = (ADC_Measure(TS_ADCCH_XL) + ADC_Measure(TS_ADCCH_XR))/2;
	
	*adc_y = (val1+((1<<12)-val2))/4;
	
	GPIO_SetAnalog(TS_PIN_YD | TS_PIN_YU);
	GPIO_SetOutput(TS_PIN_XL, 0);
	GPIO_SetOutput(TS_PIN_XR, 1);
	delay(100);
	val1 = (ADC_Measure(TS_ADCCH_YD) + ADC_Measure(TS_ADCCH_YU))/2;

	GPIO_SetOutput(TS_PIN_XL, 1);
	GPIO_SetOutput(TS_PIN_XR, 0);
	delay(100);
	val2 = (ADC_Measure(TS_ADCCH_YD) + ADC_Measure(TS_ADCCH_YU))/2;
	
	*adc_x = (val1+((1<<12)-val2))/4;
}

//static
void tshw_prepare_wait(void)
{
	/* 
	 * Drive Y low and enable a pull-up on X. When touchscreen is
	 * pressed X will be pulled low by Y and an interrupt will be
	 * generated
	 */
	GPIO_SetOutput(TS_PIN_YD, 0);
	GPIO_SetOutput(TS_PIN_YU, 0);
	GPIO_SetInputPulledUp(TS_PIN_XL | TS_PIN_XR);
}

void tshw_poll(uint16_t *adc_x, uint16_t *adc_y, int *pressed)
{
	if(GPIO_GetInputState(TS_PIN_XL) == 0) {
		*pressed = 1;
		tshw_get_raw(adc_x,adc_y);
		tshw_prepare_wait();
		delay(100);
	} else {
		*pressed = 0;
	}
}
