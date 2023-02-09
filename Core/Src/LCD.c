/*
 * LCD.c
 *
 *  Created on: Dec 13, 2022
 *      Author: migue
 */


#include "LCD.h"

LCD_Struct_t LCD_Create(
		LCD_PortType port, LCD_PinType pin[],
		LCD_PortType rs_port, LCD_PinType rs_pin,
		LCD_PortType e_port, LCD_PinType e_pin
){
	LCD_Struct_t LCD;
	LCD.E_pin = e_pin;
	LCD.E_port = e_port;

	LCD.RS_pin = rs_pin;
	LCD.RS_port = rs_port;

	LCD.dataport = port;
	LCD.datapin = pin;

	//Init Pines
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//Configure data pins (PORTB)
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//Init data pins, (using same ports for all data pins)
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//Configure RS and E pins
	GPIO_InitStruct.Pin = rs_pin | e_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	//Init data pins, (using same ports for all data pins)
	HAL_GPIO_Init(rs_port, &GPIO_InitStruct);

	LCD_Init(&LCD);
	return LCD;
};

void LCD_Init(LCD_Struct_t *LCD)
{
	// 4 bits Configuration
	HAL_Delay(100);
	LCD_Command(LCD,0x30);
	HAL_Delay(5);
	LCD_Command(LCD,0x30);
	HAL_Delay(1);
	LCD_Command(LCD,0x32);
	//5*8
	LCD_Command(LCD,0x28);

	//Activate LCD no cursor mode
	LCD_Command(LCD,0x0C);
	//Clear screen
	LCD_Command(LCD,0x01);
	//Increase from 0 to 40 DDRAM
	LCD_Command(LCD,0x06);
}

void LCD_Character(LCD_Struct_t *LCD,char data)
{
	char dat = 0;
	//For sending characters, always RS=1
	HAL_GPIO_WritePin(LCD->RS_port, LCD->RS_pin, GPIO_PIN_SET);
	// Commands y 4 bits configuration are send in 4 bits and 4 bits
	// Get the first 4 bits of the command (Most Significant Bits)
	dat = (data>>4)&0x0F;
	//Send command to each data pin
	for(uint8_t i=0;i<4;i++){
		HAL_GPIO_WritePin(LCD->dataport,LCD->datapin[i],(dat>>i)&0x01);
	}
	//On data sheet set E pin high wait 1ms and off
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_RESET);

	//Send next 4 bits
	dat = data&0x0F;
	//Send command to each data pin
	for(uint8_t i=0;i<4;i++){
		HAL_GPIO_WritePin(LCD->dataport,LCD->datapin[i],(dat>>i)&0x01);
	}
	//On data sheet set E pin high wait 1ms and off
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_RESET);
}

void LCD_Command(LCD_Struct_t *LCD,char cmd)
{
	char command = 0;
	//For sending commands, always RS=0
	HAL_GPIO_WritePin(LCD->RS_port, LCD->RS_pin, GPIO_PIN_RESET);
	// Commands y 4 bits configuration are send in 4 bits and 4 bits
	// Get the first 4 bits of the command (Most Significant Bits)
	command = (cmd>>4)&0x0F;
	//Send command to each data pin
	for(uint8_t i=0;i<4;i++){
		HAL_GPIO_WritePin(LCD->dataport,LCD->datapin[i],(command>>i) & 0x01);
	}
	//On data sheet set E pin high wait 1ms and off
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_RESET);

	//Send next 4 bits
	command = cmd&0x0F;
	//Send command to each data pin
	for(uint8_t i=0;i<4;i++){
		HAL_GPIO_WritePin(LCD->dataport,LCD->datapin[i],(command>>i)&0x01);
	}
	//On data sheet set E pin high wait 1ms and off
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->E_port, LCD->E_pin, GPIO_PIN_RESET);
}

void LCD_String(LCD_Struct_t *LCD, char *cadena){
	for ( uint8_t i = 0; i < strlen(cadena) ; i++){
		LCD_Character(LCD,cadena[i]);
	}
}

void LCD_XY(LCD_Struct_t *LCD, char x, char y){
	if (x>0){
		//Segunda linea
		LCD_Command(LCD,0xC0+y);
	}
	else{
		LCD_Command(LCD,0x80+y);
	}
}
