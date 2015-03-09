Copyright (C) 2014-2015 Ernold Vasiliev <ericv@mail.ru>

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

# SSN project - Smart-house Sensor Network

Main goal of this project - make tool that permits to automate smart house using customisable framework. Most preferences like hardware sensors connections, actions logic and routing between modules make in runtime without compilation and MC (microcontroller) flashing.
Typical modules (in/out lines, temperature sensors, humidity sensors, gsm modem etc) included out of the box. New custom modules can easy add to application.
Main architecture concept - using distributed MC network without centralised processing unit. Every MC serve local tasks (eg boiler control, floor or room temperature control, communication with internet web service for telemetry uploading and getting remote control etc).
Each module "knows" what tasks must it process and what tasks it must send to other modules (in SSN terminology such modules named "objects". Objects can be MC module or external entity, web service or logging computer, etc):
[![Concept pic](https://github.com/lireric/ssn/blob/master/models/concept.png)](https://github.com/lireric/ssn/blob/master/models/concept.png)

Cheap hardware are permit to split smart-home automation tasks between numerous modules, that provides reliability and keeps total solution cost on quite low level.

By default in project use STM32F103xx MC

###SSN use next internal subprojects:
[crc16](https://github.com/lireric/crc16) - command line crc16 calculator.
[ssn-web](https://github.com/lireric/ssn-web) - web applications for SSN project

###and external projects:
[FreeRTOS](http://freertos.org)
[libopencm3](https://github.com/libopencm3/libopencm3)

#Configuration examples
Preferenses stored in JSON format and can be uploaded to MC by serial or GPRS (from REST web service) interfaces. Script samples can be find in scripts folder.

##Typical configuration file
(in production all comments and spaces must be removed for save memory reason):

	// message header - protocol version, object-receiver and SSN command 
	{ "ssn":{"v":1,"obj":1,"cmd":"loadprefs", "data": {
	
	// common application preferences
	"app":{
	
	// object for sending logging information
	"logobj":2
	},
	// groups information. All devices (sensors, executing mechanism, etc) grouped in abstract entity named Group. Usually group combined similar devices on one GPIO pin)
	"grp": [
	
	// group must have unique number and can have different attributes according grouped devices. In this example grptimer - hardware processor timer for serving this device, grpport/grppin1 - GPIO porn and pin where device connected
	{"grpnum":1,"grpmaxdevs":15,"grptimer":"TIM3","grpport":"GPIOA","grppin1":1, "devsqty":5, "devs": [
	
	// properties for connected devices - ds18b20 temperature sensors
	  {"devid":2001, "devtype":1, "romid":"28fcc2da020000e5"},
	  {"devid":2002, "devtype":1, "romid":"2898f0da020000fd"},
	  {"devid":2003, "devtype":1, "romid":"28cb98f804000003"}
	]},
	// preferences for GSM modem
	// for sequrity reason in internet exchange use (can be use) AES128 encoding
	// acc - account number. One web service can serve many (65535) group of object (accounts) each one may consist 65536 objects.
	{"grpnum":10,"grpmaxdevs":1, "grptimer":"TIM3", "devsqty":1, "devs": [
	  {"devid":10001,"devtype":10,"v":"SIM900","PortDTR":"GPIOB","PinDTR":14,"PortPwrKey":"GPIOB","PinPwrKey":12,"PortChgCtrl":"GPIOB","PinChgCtrl":15,"PortRTS":"GPIOB","PinRTS":8,"USART":2,"APN":"internet","SrvAddr":"lirclub.ru","SrvPort":80,"SMSNumber":"+79091234567","PriDNS":"8.8.8.8","SecDNS":"8.8.8.8","GUser":"","GUserPswd":"","AESKey":"secret","acc":1}
	]},
	// another ds18b20 sensor, connected to other pin
	 {"grpnum":2,"grpmaxdevs":1,"grptimer":"TIM2","grpport":"GPIOE","grppin1":3, "grppin2":6, "devsqty":1, "devs": [
	  {"devid":1003, "devtype":1, "romid":"280d8af8040000d1"}
	]},
	
	// digital i/o device (1 pin port IO Push-Pull device (digital out), (devtype 4)
	 {"grpnum":7,"grpmaxdevs":1,"grptimer":"TIM5","grpport":"GPIOE","grppin1":1, "devsqty":1, "devs": [
		  {"devid":4004, "devtype":4}
	]},
	// digital i/o device 1 pin port IO Open Drain device, devtype 5)
	 {"grpnum":9,"grpmaxdevs":1,"grptimer":"TIM5","grpport":"GPIOC","grppin1":8, "devsqty":1, "devs": [
	  {"devid":4005, "devtype":5}
	]},
	
	// DHT22 humidity and temperature sensor (devtype 2)
	{"grpnum":8,"grpmaxdevs":1,"grptimer":"TIM4","grpport":"GPIOC","grppin1":9, "devsqty":1, "devs": [
	{ "devid":3001, "devtype":2}
	]}
	],
	
	// routing information. This module can route (own or external) messages to objects 2 and 3 with interfaces USART and GSM modem respectively
	
	"routes":[
	{"obj":2,"if":1},
	{"obj":3,"if":6}
	],
	
	// information about events and actions. All situations on served by this MC object can be described in human readable language like "formulas"
	// This formulas "compile" to Reverse Polish Notation (RPN) and not requred many processor resources in runtime.
	// all conventions of this "language":
	// "aid": action ID unique number of action (by it action can be modified later)
	// d(device_id,index): 'd' - device (any sensor can return value), device_id - unique device ID (according hardware preferences above), index - number of value type in case multi types sensors (eg DHT22 return temperature info - index 0, and humidity info - index 1)
	// tt - daily timestamp (can be also in integer falue - seconds from 00:00:00 time)
	// ty - year, tm - month, td - day of month, tw - day of week
	// i(period,duration) - interval timers. May be single (attribute - "s":"1") or periodical. Timer must be single event in action
	// a(device_id, command_in, value_in, command_out, value_out): device_id - unique device ID (according hardware preferences above). command/value_in - command and value to set on device in event occure (eg a(10001,2,'Alarm') - send SMS 'Alarm' in GSM modem device). command/value_out - command and value to set on device after event fire condition over.
	// "nolog":"1" attribute make skip logging this event 
	"logic":[
	{"aid":1,"astr":"d(2002,0)>260=a(4004,0,0)"},
	{"aid":2,"astr":"d(2002,0)+10>=270=a(10001,2,'Alarm: t<2')","arep":20},
	{"aid":3,"astr":"d(2002,1)<250=a(4004,0,1),a(4005,0,0),a(10001,2,'Alarm!')"},
	{"aid":4,"astr":"(d(2002,1)<900)&&(d(3001,2)>50)=a(4004,0,1,0,0)","nolog":"1"},
	{"aid":5,"astr":"(tt>'7:00:00')&&(tt<'21:00:00')=a(4004,0,0,0,1)"},
	{"aid":6,"astr":"(ty>2015)&&(tm==3)&&(td==8)=a(10001,2,'Greetings!')"},
	{"aid":7,"astr":"(tw>=1)&&(tw<=5)=a(4005,0,1,0,0)"},
	{"aid":8,"astr":"i(3000,100)=a(4004,0,1,0,0)","nolog":"1","s":"1"},
	{"aid":9,"astr":"i(30000,0)=a(10001,5,1)"},
	{"aid":10,"astr":"i(50000,0)=a(10001,6,1)"},
	{"aid":11,"astr":"i(10000,1000)=a(4004,0,1,0,0)"}
	]
	}}}

##to do:
- user web application
- RF interfaces
- smart card reader interface
- stepping motor device
- LCD interface (GUI part)
- Ethernet interface
