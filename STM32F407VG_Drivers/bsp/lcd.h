/*
 * lcd.h
 *
 *  Created on: Oct 1, 2023
 *      Author: mohamed
 */

#ifndef LCD_H_
#define LCD_H_

#include "../drivers/Inc/stm32f407xx.h"

/* BSP exposed APIs */
void lcd_init(void);
void lcd_send_command (uint8_t cmd);
void lcd_print_char(uint8_t data);
void lcd_print_string(char* message);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_display_clear(void);
void lcd_display_return_home(void);

/* Application Configurable items */
#define LCD_GPIO_PORT		GPIOD
#define LCD_GPIO_RS			GPIO_PIN_NO_0
#define LCD_GPIO_RW			GPIO_PIN_NO_1
#define LCD_GPIO_EN			GPIO_PIN_NO_2
#define LCD_GPIO_D4			GPIO_PIN_NO_3
#define LCD_GPIO_D5			GPIO_PIN_NO_4
#define LCD_GPIO_D6			GPIO_PIN_NO_5
#define LCD_GPIO_D7			GPIO_PIN_NO_6


/* LCD commands */
#define LCD_CMD_4DL_2N_5X8F		0x28 // used to activate 4bit data length, 2lines(rows), 5x8 font size
#define LCD_CMD_DON_CON			0x0E // used for : Display ON and Cursor ON
#define LCD_CMD_DIS_CLEAR		0x01 // used for : clearing the Display
#define LCD_CMD_INCADD			0x06 // used for the increment of DRAM Address, without shifting the display (0x7 for shift)
#define LCD_CMD_DIS_RETURN_HOME	0x02 // used for : Cursor shifting back to its original position

#endif /* LCD_H_ */
