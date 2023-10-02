/*
 * ds1307.h
 *
 *  Created on: Oct 1, 2023
 *      Author: mohamed
 */

#ifndef DS1307_H_
#define DS1307_H_

#include "../drivers/Inc/stm32f407xx.h"

/* Application Configurable items */

#define DS1307_I2C				I2C1
#define DS1307_I2C_GPIO_PORT    GPIOB
#define DS1307_I2C_SDA_PIN		GPIO_PIN_NO_7
#define DS1307_I2C_SCL_PIN		GPIO_PIN_NO_6
#define DS1307_I2C_SPEED		I2C_SC_SPEED_SM
#define DS1307_I2C_PUPD			GPIO_PIN_PU // with internal pull-ups

/* Register Addresses (The addresses from DS1307's Data-sheet) */
#define DS1307_ADDR_SEC			0x00
#define DS1307_ADDR_MIN			0x01
#define DS1307_ADDR_HRS			0x02
#define DS1307_ADDR_DAY			0x03
#define DS1307_ADDR_DATE		0x04
#define DS1307_ADDR_MONTH		0x05
#define DS1307_ADDR_YEAR		0x06

#define TIME_FORMAT_12HRS_AM	0
#define TIME_FORMAT_12HRS_PM	1
#define TIME_FORMAT_24HRS		2

#define DS1307_I2C_ADDRESS		0x68 //1101000

#define SUNDAY			1;
#define MONDAY			2;
#define TUESDAY			3;
#define WEDNESDAY	    4;
#define THURSDAY		5;
#define FRIDAY			6;
#define SATURDAY		7;

typedef struct {
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_date_t;

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_time_t;

// Functions Prototypes

uint8_t ds1307_init();

void ds1307_set_current_time(RTC_time_t *);
void ds1307_get_current_time(RTC_time_t *);

void ds1307_set_current_date(RTC_date_t *);
void ds1307_get_current_date(RTC_date_t *);

#endif /* DS1307_H_ */
