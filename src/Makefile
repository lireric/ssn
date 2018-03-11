include ../Makefile.common

OBJS+=newlib_stubs.o
OBJS+=main.o
#OBJS+=drv_pcd8544.o
#OBJS+=drv_pcd8544_l.o
OBJS+=Lucida_Console_6_6x6.o
OBJS+=lcd-320x240.o
OBJS+=touchscreen.o
OBJS+=touchscreen-hw.o
OBJS+=utils.o
OBJS+=STM32_USART.o
OBJS+=bb_device.o
OBJS+=DS18B20.o
OBJS+=device.o
OBJS+=rtc_func.o
OBJS+=fsmc_func.o
OBJS+=i2c_eeprom.o
OBJS+=soft_i2c.o
OBJS+=RTC_DS1307.o
OBJS+=DHT22.o
OBJS+=cJSON.o 
OBJS+=gsm.o 
OBJS+=commands.o
OBJS+=xprintf.o 
OBJS+=aes.o
OBJS+=base64.o
OBJS+=lcd.o
OBJS+=crc16.o
OBJS+=stack.o
OBJS+=ini.o
OBJS+=processor.o
OBJS+=bmp180.o
OBJS+=stepmotor.o
OBJS+=i2c_impl.o
OBJS+=dev_memory.o
OBJS+=bme280dev.o
OBJS+=bme280.o
OBJS+=mh-z19b.o

all: src

src: app.a

app.a: $(OBJS)
	$(AR) cr app.a $(OBJS)

.PHONY: src clean tshow

clean:
	rm -f app.a $(OBJS)

tshow:
	@echo "######################################################################################################"
	@echo "######## optimize settings: $(InfoTextLib), $(InfoTextSrc)"
	@echo "######################################################################################################"
	
