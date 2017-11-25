/*
 * i2c_impl.c
 *
 *  Created on: 15 окт. 2017 г.
 *      Author: eric
 */

#include "i2c_impl.h"
#include "soft_i2c.h"


void i2c_setup(sGrpDev* pGrpDev)
{
	uint32_t i2c = I2C1; // to do...

//	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
//	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C1EN);
//	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

	/* Enable clocks for I2C1 and AFIO. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);

	i2c_peripheral_disable(i2c);

	soft_i2c_init(pGrpDev);
	soft_i2c_clear_wire_SDA(pGrpDev);
	soft_i2c_clear_wire_SCL(pGrpDev);
	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
	soft_i2c_set_wire_SCL(pGrpDev);		// SETSCL

	/* Set alternate functions for the SCL and SDA pins of I2C1. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C1_SCL | GPIO_I2C1_SDA);

	/* Disable the I2C before changing any configuration. */
//	i2c_peripheral_disable(i2c);




	//configure ANFOFF DNF[3:0] in CR1
//	i2c_enable_analog_filter(I2C1);
//	i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
	/* HSI is at 8Mhz */
//	i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);


	/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_36MHZ);

	/* 400KHz - I2C Fast Mode */
//	i2c_set_fast_mode(I2C1);
	i2c_set_standard_mode(i2c);

	/*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
	i2c_set_ccr(i2c, 0x1e);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
	 * Incremented by 1 -> 11.
	 */
	i2c_set_trise(i2c, 0x0b);
//	 i2c_100khz_i2cclk8mhz(i2c);
	/*
	 * This is our slave address - needed only if we want to receive from
	 * other masters.
	 */
//	i2c_set_own_7bit_slave_address(i2c, 0x32);
	i2c_enable_ack(i2c);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(i2c);
	i2c_reset(i2c);
}


int i2c_WriteBufferAddress (sGrpDev* pGrpDev,  uint8_t chipAddress,  uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer )
{

	uint32_t i2c = I2C1; // to do...
	uint32_t reg32 __attribute__((unused));

//	write_i2c(i2c, chipAddress, registerAddress, sizeOfBuffer, buffer);

	/* Send START condition. */
	i2c_send_start(i2c);
	i2c_enable_ack(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Send destination address. */
	i2c_send_7bit_address(i2c, chipAddress, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Sending the data. */

	i2c_send_data(i2c, registerAddress);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
    while ( sizeOfBuffer > 0 ) {
    	i2c_send_data(i2c, *buffer);
    	if (sizeOfBuffer == 1) {
    		/* After the last byte we have to wait for TxE too. */
    		while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
    	} else {
    		while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
    	}
        buffer++;
        sizeOfBuffer--;
    }
	/* Send STOP condition. */
	i2c_send_stop(i2c);

	return pdTRUE;
}



uint32_t i2c_ReadBufferAddress (sGrpDev* pGrpDev, uint8_t chipAddress, uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer)
{
	uint32_t i2c = I2C1; // to do...

//	read_i2c(i2c, chipAddress, registerAddress, sizeOfBuffer, buffer);

	uint32_t reg32 __attribute__((unused));

//	   if((I2C_SR2(i2c) & (I2C_SR2_BUSY)))  {// проверяю, занята ли шина I2C (взведён ли флаг BUSY)
//       soft_i2c_stop(pGrpDev);



	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	/* Yes, WRITE is correct - for selecting register in chip */
	i2c_send_7bit_address(i2c, chipAddress, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	i2c_send_data(i2c, registerAddress); /* temperature register */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/*
	 * Now we transferred that we want to ACCESS the chip register.
	 * Now we send another START condition (repeated START) and then
	 * transfer the destination but with flag READ.
	 */

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(i2c, chipAddress, I2C_READ);

	/* 2-byte receive is a special case. See datasheet POS bit. */
	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Cleaning I2C_SR1_ACK. */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
    while ( sizeOfBuffer != 0 ) {
        *buffer = i2c_get_data(i2c);
        buffer++;
        sizeOfBuffer--;
        if ( sizeOfBuffer == 0 ) {
        	/*
        	 * Yes they mean it: we have to generate the STOP condition before
        	 * saving the 1st byte.
        	 */
        	i2c_send_stop(i2c);
            break;
        }
        else
        	i2c_nack_next(i2c);
    }

	/* Original state. */
	I2C_CR1(i2c) &= ~I2C_CR1_POS;

	return pdTRUE;

}

