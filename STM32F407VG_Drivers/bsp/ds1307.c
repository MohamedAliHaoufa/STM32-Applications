/*
 * ds1307.c
 *
 *  Created on: Oct 1, 2023
 *      Author: mohamed
 */

#include "ds1307.h"

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t bcd_to_binary(uint8_t value);
static uint8_t binary_to_bcd(uint8_t value);

I2C_Handle_t g_ds1307I2cHandle;

/*
 * if it returns 1 : CH = 1 ; means init failed
 * if it returns 0 : CH = 0 ; means init success
 */
uint8_t ds1307_init(){

	// initialize the I2C pins
	ds1307_i2c_pin_config();
	// initialize the I2C peripheral
	ds1307_i2c_config();
	// Enable the I2C peripheral
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	// Make clock halt = 0 ; of the DS1307 device
	ds1307_write(0x00, DS1307_ADDR_SEC);

	// Read back the clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);
	return ((clock_state >> 7) & 0x01);
}

void ds1307_set_current_time(RTC_time_t *rtc_time){

	uint8_t seconds, hrs;
	seconds = binary_to_bcd(rtc_time->seconds);
	seconds &= ~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);

	ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);

	hrs = binary_to_bcd(rtc_time->hours);

	if(rtc_time->time_format == TIME_FORMAT_24HRS){

		hrs &= ~(1 << 6);

	} else {

		hrs |= (1 << 6);
		hrs |= (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? hrs | (1 << 5) : hrs & ~(1 << 5);
	}

	ds1307_write(hrs, DS1307_ADDR_HRS);

}

void ds1307_set_current_date(RTC_date_t *rtc_date){
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);

}

void ds1307_get_current_time(RTC_time_t *rtc_time){
	uint8_t seconds, hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);
	rtc_time->seconds = bcd_to_binary(seconds);

	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	hrs = ds1307_read(DS1307_ADDR_HRS);

	if(hrs & (1 << 6)){
		// 12h format
		rtc_time->time_format = (hrs & (1 << 5)) ? TIME_FORMAT_12HRS_PM : TIME_FORMAT_12HRS_AM;
		hrs &= (0x3 << 5); // clear bit 6 and 5

	} else {
		// 24 format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}
	rtc_time->hours = bcd_to_binary(hrs);

}

void ds1307_get_current_date(RTC_date_t *rtc_date){
	rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
	rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
}

static void ds1307_i2c_pin_config(void){
	GPIO_Handle_t i2c_sda, i2c_scl;

	// All member elements are initialized to 0
	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/*
	 *	I2C_SDA ==> PB6
	 *	I2C_SCL ==> PB7
	 */
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinPinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinPinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_scl);

}

static void ds1307_i2c_config(void){
	g_ds1307I2cHandle.pI2Cx = DS1307_I2C;
	g_ds1307I2cHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_ds1307I2cHandle.I2C_Config.I2C_SCLspeed = DS1307_I2C_SPEED;

	I2C_Init(&g_ds1307I2cHandle);

}

static void ds1307_write(uint8_t value, uint8_t reg_addr){
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;

	I2C_MasterSendData(&g_ds1307I2cHandle, tx, 2, DS1307_I2C_ADDRESS, 0);
}

static uint8_t ds1307_read(uint8_t reg_addr){

	uint8_t data;

	// first send the slave address only to initialize the slave pointer with the register address to send u its content
	I2C_MasterSendData(&g_ds1307I2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, 0);
	I2C_MasterReceiveData(&g_ds1307I2cHandle, &data, 1, DS1307_I2C_ADDRESS, 0);

	return data;
}

/*
 *  0001 0010
 *  m = 10
 *  n = 2
 *  m + n = 12
 */
static uint8_t bcd_to_binary(uint8_t value){
	uint8_t m, n;
	uint8_t binary;
	m = (uint8_t)((value >> 4) * 10) ;
	n = value & (uint8_t)(0x0F);
	binary = m + n ;
	return binary;
}

/*
 * 12
 * m = 1
 * n = 2
 * m << 4 = 10000 | 0010 = 0001 0010
 */
static uint8_t binary_to_bcd(uint8_t value){

	uint8_t m, n;
	uint8_t bcd;
	if(value > 10){
		m = value / 10;
		n = value % 10;
		bcd = (uint8_t)( (m << 4) | n );
	}else {
		bcd = value;
	}
	return bcd;
}

