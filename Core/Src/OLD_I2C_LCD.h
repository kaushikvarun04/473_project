/*
 * File: I2C_LCD.h
 * Driver Name: [[ I2C_LCD Display ]]
 * SW Layer:   ECUAL
 * Created on: Jan 28, 2024
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include "stdint.h"

#define I2C_LCD_MAX	1	// Maximum Number of I2C_LCD Modules in Your Project
#define I2C_LCD_1	0	// I2C_LCD Instance Number 1 (Add more if you need)

//-----[ Prototypes For All User External Functions ]-----

void I2C_LCD_Init(void);
void I2C_LCD_Clear(void);
void I2C_LCD_Home(void);
void I2C_LCD_SetCursor(uint8_t Col, uint8_t Row);
void I2C_LCD_WriteChar(char Ch);
void I2C_LCD_WriteString( char* Str);

void I2C_LCD_ShiftLeft(void);
void I2C_LCD_ShiftRight(void);

void I2C_LCD_Backlight(void);
void I2C_LCD_NoBacklight(void);
void I2C_LCD_Display(void);
void I2C_LCD_NoDisplay(void);
void I2C_LCD_Cursor(void);
void I2C_LCD_NoCursor(void);
void I2C_LCD_Blink(void);
void I2C_LCD_NoBlink(void);

void I2C_LCD_CreateCustomChar(uint8_t CharIndex, const uint8_t* CharMap);
void I2C_LCD_PrintCustomChar(uint8_t CharIndex);

#endif /* I2C_LCD_H_ */
