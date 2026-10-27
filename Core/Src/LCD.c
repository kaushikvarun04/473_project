#include "LCD.h"
#include <string.h>
#include <stdio.h>

static void sendByte(LCD_Handle *lcd, uint8_t value, bool isData)
{
    HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, isData ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(lcd->hspi, &value, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_SET);
}

static void sendCommand(LCD_Handle *lcd, uint8_t cmd) { sendByte(lcd, cmd, false); }
static void sendData(LCD_Handle *lcd, uint8_t data)   { sendByte(lcd, data, true); }

void LCD_Init(LCD_Handle *lcd, SPI_HandleTypeDef *hspi,
              GPIO_TypeDef *cs_port, uint16_t cs_pin,
              GPIO_TypeDef *rs_port, uint16_t rs_pin)
{
    lcd->hspi = hspi;
    lcd->cs_port = cs_port;
    lcd->cs_pin = cs_pin;
    lcd->rs_port = rs_port;
    lcd->rs_pin = rs_pin;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);   /* CSB idle HIGH */

    HAL_Delay(50);              /* > 40 ms after VDD stable */

    sendCommand(lcd, 0x38);     /* 8-bit, 2-line (20x4), 5x8 font */
    HAL_Delay(1);

    sendCommand(lcd, 0x0C);     /* Display ON, cursor OFF, blink OFF */
    HAL_Delay(1);

    sendCommand(lcd, 0x01);     /* Clear display */
    HAL_Delay(12);

    sendCommand(lcd, 0x06);     /* Entry mode: increment, no shift */
    HAL_Delay(1);

    lcd->displayControl = LCD_DISPLAYON;
    sendCommand(lcd, LCD_DISPLAYCONTROL | lcd->displayControl);
    HAL_Delay(1);
}

void LCD_Clear(LCD_Handle *lcd)
{
    sendCommand(lcd, LCD_CLEARDISPLAY);
    HAL_Delay(2);
}

void LCD_Display(LCD_Handle *lcd, bool turnOn)
{
    if (turnOn) lcd->displayControl |= LCD_DISPLAYON;
    else        lcd->displayControl &= ~LCD_DISPLAYON;
    sendCommand(lcd, LCD_DISPLAYCONTROL | lcd->displayControl);
}

void LCD_CursorMode(LCD_Handle *lcd, bool underline, bool blink)
{
    if (underline) lcd->displayControl |= LCD_CURSORON;
    else           lcd->displayControl &= ~LCD_CURSORON;

    if (blink) lcd->displayControl |= LCD_BLINKON;
    else       lcd->displayControl &= ~LCD_BLINKON;

    sendCommand(lcd, LCD_DISPLAYCONTROL | lcd->displayControl);
}

void LCD_SetCursor(LCD_Handle *lcd, uint8_t col, uint8_t row)
{
    static const uint8_t rowOffsets[LCD_ROWS] = {0x00, 0x40, 0x14, 0x54};

    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;

    sendCommand(lcd, LCD_SETDDRAMADDR | (col + rowOffsets[row]));
}

void LCD_PrintChar(LCD_Handle *lcd, char c)
{
    sendData(lcd, (uint8_t)c);
}

void LCD_PrintStr(LCD_Handle *lcd, const char *str)
{
    while (*str) {
        sendData(lcd, (uint8_t)*str++);
    }
}

void LCD_PrintCharAt(LCD_Handle *lcd, uint8_t col, uint8_t row, char c)
{
    LCD_SetCursor(lcd, col, row);
    LCD_PrintChar(lcd, c);
}

void LCD_PrintStrAt(LCD_Handle *lcd, uint8_t col, uint8_t row, const char *str)
{
    LCD_SetCursor(lcd, col, row);
    LCD_PrintStr(lcd, str);
}

/* Prints at most 20 chars, pads the rest with spaces */
void LCD_PrintLinePadded(LCD_Handle *lcd, uint8_t row, const char *text)
{
    LCD_SetCursor(lcd, 0, row);

    uint8_t i = 0;
    while (text[i] && i < LCD_COLS) {
        sendData(lcd, (uint8_t)text[i]);
        i++;
    }
    for (; i < LCD_COLS; i++) {
        sendData(lcd, ' ');
    }
}

void LCD_DisplaySpeed(LCD_Handle *lcd, float speed_mph, bool has_fix)
{
    LCD_PrintLinePadded(lcd, 0, "== DAQ TELEMETRY ==");   /* 19 chars: fits in 20 */

    if (has_fix) {
        char buf[21];
        snprintf(buf, sizeof(buf), "SPEED: %5.1f MPH", speed_mph);
        LCD_PrintLinePadded(lcd, 1, buf);
        LCD_PrintLinePadded(lcd, 2, "GPS FIX: ACTIVE");
    } else {
        LCD_PrintLinePadded(lcd, 1, "SPEED:   ---.- MPH");
        LCD_PrintLinePadded(lcd, 2, "GPS FIX: SEARCHING..");
    }

    LCD_PrintLinePadded(lcd, 3, "STATUS: LOGGING...");
}
