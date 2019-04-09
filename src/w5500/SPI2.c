//#include "stm32f10x.h"
//#include "config.h"
#include <libopencm3/stm32/spi.h>
#include "w5500/socket.h"
#include "w5500/w5500.h"
#include "utils.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void WIZ_SPI_Init(void)
{
	rcc_periph_clock_enable(RCC_SPI1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_NSS | GPIO_SPI1_SCK | GPIO_SPI1_MISO | GPIO_SPI1_MOSI);
//	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
//			GPIO_CNF_INPUT_FLOAT, GPIO_SPI1_MISO);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, WIZ_SCS);

	spi_reset(SPI1);

	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_set_full_duplex_mode(SPI1);
	spi_set_nss_high(SPI1);
//	spi_enable_crc(SPI1);
	spi_enable_software_slave_management(SPI1);
//	spi_disable_software_slave_management(SPI1);
	spi_set_standard_mode(SPI1, 0);

	spi_enable(SPI1);
//	SPI_InitTypeDef   SPI_InitStructure;
//
//	  /* SPI Config -------------------------------------------------------------*/
//	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//	  SPI_InitStructure.SPI_CRCPolynomial = 7;

//	  SPI_Init(SPI2, &SPI_InitStructure);
	  
	  /* Enable SPI */
//	  SPI_Cmd(SPI2, ENABLE);

}

// Connected to Data Flash
void WIZ_CS(uint8_t val)
{
	if (val == LOW) {
//   		GPIO_ResetBits(GPIOB, WIZ_SCS);
		gpio_clear(GPIOA, WIZ_SCS);
	} else if (val == HIGH){
//   		GPIO_SetBits(GPIOB, WIZ_SCS);
		gpio_set(GPIOA, WIZ_SCS);
	}
}


uint8_t SPI2_SendByte(uint8_t byte)
{
//	  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//	spi_send(SPI2, byte);

//	  SPI_I2S_SendData(SPI2, byte);
          
//	  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
          
//	  return SPI_I2S_ReceiveData(SPI2);
	return spi_xfer(SPI1, byte);
}

/*
void SPI1_TXByte(uint8_t byte)
{
	  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);       

	  SPI_I2S_SendData(SPI1, byte);	
}
*/
