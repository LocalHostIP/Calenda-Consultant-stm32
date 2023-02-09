/*
 * LCD.h
 *
 *  Created on: Dec 13, 2022
 *      Author: migue
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx_hal.h"
#include "string.h"

// 4 data pins
// 2 RS and E pin

#define LCD_PinType uint16_t
#define LCD_PortType GPIO_TypeDef*

#define LCD_CLEAR_SCREEN 0x01

typedef struct{
	LCD_PortType dataport;
	LCD_PinType *datapin;

	LCD_PortType RS_port;
	LCD_PinType RS_pin;

	LCD_PortType E_port;
	LCD_PinType E_pin;
}LCD_Struct_t;

LCD_Struct_t LCD_Create(
		LCD_PortType port, LCD_PinType pin[],
		LCD_PortType rs_port, LCD_PinType rs_pin,
		LCD_PortType e_port, LCD_PinType e_pin
);

void LCD_Init(LCD_Struct_t *LCD);

void LCD_Command(LCD_Struct_t *LCD,char cmd);

void LCD_Character(LCD_Struct_t *LCD,char cmd);

void LCD_String(LCD_Struct_t *LCD, char *cadena);

void LCD_XY(LCD_Struct_t *LCD, char x, char y);

#endif /* INC_LCD_H_ */
