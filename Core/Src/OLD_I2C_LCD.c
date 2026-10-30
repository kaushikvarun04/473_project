/*
- File: I2C_LCD.c
- Driver Name: [[ I2C_LCD Display ]]
- SW Layer:   ECUAL
- Created on: Jan 28, 2024
- Author:     Khaled Magdy
- -------------------------------------------
- For More Information, Tutorials, etc.
- Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "I2C_LCD.h"
#include "main.h"

// Millisecond delay using standard STM32 HAL:
#define DELAY_MS(ms) HAL_Delay(ms)


// Bring in external handle if not included in main.h/i2c.h
extern LCD_I2C_HANDLE hi2c1;
#define  LCD_I2C_HANDLE  &hi2c1
#define LCD_I2C_ADDR     (0x3F << 1) // STM32 HAL requires 8-bit left-shifted address
#define LCD_COLS         20
#define LCD_ROWS         4

/*-----------------------[INTERNAL DEFINITIONS]-----------------------*/
// CMD
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80
// DISPLAY ENTRY
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
// DISPLAY CONTROL
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00
// CURSOR MOTION
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00
// FUNCTION SET
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00
// BACKLIGHT CONTROL
#define LCD_BACKLIGHT           0x08
#define LCD_NOBACKLIGHT         0x00
#define EN                      0b00000100  // Enable bit
#define RW                      0b00000010  // Read/Write bit
#define RS                      0b00000001  // Register select bit

/*-----------------------[INTERNAL VARIABLES]-----------------------*/

typedef struct I2C_LCD_InfoParam_s
{
    uint8_t DisplayCtrl;
    uint8_t BacklightVal;
} I2C_LCD_InfoParam_t;

static I2C_LCD_InfoParam_t I2C_LCD_Info;

/*---------------------[STATIC INTERNAL FUNCTIONS]-----------------------*/

static void I2C_LCD_ExpanderWrite(uint8_t DATA)
{
    uint8_t TxData = (DATA) | I2C_LCD_Info.BacklightVal;
    HAL_I2C_Master_Transmit(LCD_I2C_HANDLE, LCD_I2C_ADDR, &TxData, sizeof(TxData), 100);
}

static void I2C_LCD_EnPulse(uint8_t DATA)
{
    I2C_LCD_ExpanderWrite(DATA | EN); // En high
    DELAY_MS(2);                      // Enable pulse must be > 450ns

    I2C_LCD_ExpanderWrite(DATA & ~EN); // En low
    DELAY_MS(50);                      // Commands need > 37us to settle
}

static void I2C_LCD_Write4Bits(uint8_t Val)
{
    I2C_LCD_ExpanderWrite(Val);
    I2C_LCD_EnPulse(Val);
}

static void I2C_LCD_Send(uint8_t Val, uint8_t Mode)
{
    uint8_t HighNib = Val & 0xF0;
    uint8_t LowNib = (Val << 4) & 0xF0;
    I2C_LCD_Write4Bits((HighNib) | Mode);
    I2C_LCD_Write4Bits((LowNib) | Mode);
}

static void I2C_LCD_Cmd(uint8_t CMD)
{
    I2C_LCD_Send(CMD, 0);
}

static void I2C_LCD_Data(uint8_t DATA)
{
    I2C_LCD_Send(DATA, RS);
}

/*-----------------------[USER EXTERNAL FUNCTIONS]-----------------------*/

void I2C_LCD_Init(void)
{
    // Wait at least 40ms after power up before interacting with LCD module
    while(HAL_GetTick() < 50);
    I2C_LCD_Cmd(0x30);
    DELAY_MS(5);  // Delay > 4.1ms
    I2C_LCD_Cmd(0x30);
    DELAY_MS(5);  // Delay > 4.1ms
    I2C_LCD_Cmd(0x30);
    DELAY_MS(150);  // Delay > 100μs
    I2C_LCD_Cmd(0x02);

    // Configure LCD
    I2C_LCD_Cmd(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    I2C_LCD_Cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    I2C_LCD_Cmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    
    I2C_LCD_Info.DisplayCtrl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    I2C_LCD_Info.BacklightVal = LCD_BACKLIGHT;
    
    // Clear LCD
    I2C_LCD_Clear();
}

void I2C_LCD_Clear(void)
{
    I2C_LCD_Cmd(LCD_CLEARDISPLAY);
    DELAY_MS(2);
}

void I2C_LCD_Home(void)
{
    I2C_LCD_Cmd(LCD_RETURNHOME);
    DELAY_MS(2);
}

void I2C_LCD_SetCursor(uint8_t Col, uint8_t Row)
{
    int Row_Offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (Row >= LCD_ROWS)
    {
        Row = LCD_ROWS - 1;
    }
    I2C_LCD_Cmd(LCD_SETDDRAMADDR | (Col + Row_Offsets[Row]));
}

void I2C_LCD_WriteChar(char Ch)
{
    I2C_LCD_Data(Ch);
}

void I2C_LCD_WriteString(char *Str)
{
    while (*Str)
    {
        I2C_LCD_Data(*Str++);
    }
}

void I2C_LCD_ShiftLeft(void)
{
    I2C_LCD_Cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void I2C_LCD_ShiftRight(void)
{
    I2C_LCD_Cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void I2C_LCD_Backlight(void)
{
    I2C_LCD_Info.BacklightVal = LCD_BACKLIGHT;
    I2C_LCD_ExpanderWrite(0);
}

void I2C_LCD_NoBacklight(void)
{
    I2C_LCD_Info.BacklightVal = LCD_NOBACKLIGHT;
    I2C_LCD_ExpanderWrite(0);
}

void I2C_LCD_Display(void)
{
    I2C_LCD_Info.DisplayCtrl |= LCD_DISPLAYON;
    I2C_LCD_Cmd(LCD_DISPLAYCONTROL | I2C_LCD_Info.DisplayCtrl);
}

void I2C_LCD_NoDisplay(void)
{
    I2C_LCD_Info.DisplayCtrl &= ~LCD_DISPLAYON;
    I2C_LCD_Cmd(LCD_DISPLAYCONTROL | I2C_LCD_Info.DisplayCtrl);
}

void I2C_LCD_Cursor(void)
{
    I2C_LCD_Info.DisplayCtrl |= LCD_CURSORON;
    I2C_LCD_Cmd(LCD_DISPLAYCONTROL | I2C_LCD_Info.DisplayCtrl);
}

void I2C_LCD_NoCursor(void)
{
    I2C_LCD_Info.DisplayCtrl &= ~LCD_CURSORON;
    I2C_LCD_Cmd(LCD_DISPLAYCONTROL | I2C_LCD_Info.DisplayCtrl);
}

void I2C_LCD_Blink(void)
{
    I2C_LCD_Info.DisplayCtrl |= LCD_BLINKON;
    I2C_LCD_Cmd(LCD_DISPLAYCONTROL | I2C_LCD_Info.DisplayCtrl);
}

void I2C_LCD_NoBlink(void)
{
    I2C_LCD_Info.DisplayCtrl &= ~LCD_BLINKON;
    I2C_LCD_Cmd(LCD_DISPLAYCONTROL | I2C_LCD_Info.DisplayCtrl);
}

void I2C_LCD_CreateCustomChar(uint8_t CharIndex, const uint8_t* CharMap)
{
    CharIndex &= 0x07;
    I2C_LCD_Cmd(LCD_SETCGRAMADDR | (CharIndex << 3));
    for (int i = 0; i < 8; i++)
    {
        I2C_LCD_Send(CharMap[i], RS);
    }
}

void I2C_LCD_PrintCustomChar(uint8_t CharIndex)
{
    I2C_LCD_Send(CharIndex, RS);
}