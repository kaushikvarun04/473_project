#include "LCD.h"
#include <string.h>
#include <stdio.h>

static void sendByte(LCD_Handle *lcd, uint8_t value, bool isData)
{
    HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, isData ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->cs_port, lcd->cs_pin, GPIO_PIN_RESET);
    //HAL_SPI_Transmit(lcd->hspi, &value, 1, HAL_MAX_DELAY);
        HAL_StatusTypeDef st = HAL_SPI_Transmit(lcd->hspi, &value, 1, 100);
if (st != HAL_OK) {
    printf("SPI fail: st=%d state=%d err=0x%lx\r\n",
           st, lcd->hspi->State, lcd->hspi->ErrorCode);
}
//     if (st != HAL_OK) {
//     printf("SPI err %d, state=%d, errcode=0x%lx\r\n",
//            st, lcd->hspi->State, lcd->hspi->ErrorCode);
// }
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
    HAL_Delay(50);                                      /* power-up settle */

    sendCommand(lcd, 0x38);   /* Function set: 8-bit, N=1 (gives 4 rows at 20 cols) */
    HAL_Delay(2);
    sendCommand(lcd, 0x0C);   /* Display ON, cursor off, blink off */
    HAL_Delay(2);
    sendCommand(lcd, 0x01);   /* Clear */
    HAL_Delay(5);
    sendCommand(lcd, 0x06);   /* Entry mode: increment, no shift */
    HAL_Delay(2);
    HAL_Delay(50);

    lcd->displayControl = LCD_DISPLAYON;
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
    LCD_PrintLinePadded(lcd, 0, "CURRENT SPEED");

    if (has_fix) {
        char buf[21];
        uint32_t speed_tenths = (uint32_t)(speed_mph * 10.0f + 0.5f);
        if (speed_tenths > 999999u) speed_tenths = 999999u;

        memcpy(buf, "SPEED: ", 7);
        size_t position = 7;
        uint32_t whole = speed_tenths / 10u;
        uint32_t divisor = 10000u;
        while (divisor > 1u && whole < divisor) divisor /= 10u;
        do {
            buf[position++] = (char)('0' + (whole / divisor) % 10u);
            divisor /= 10u;
        } while (divisor > 0u);
        buf[position++] = '.';
        buf[position++] = (char)('0' + speed_tenths % 10u);
        memcpy(&buf[position], " MPH", sizeof(" MPH"));
        LCD_PrintLinePadded(lcd, 1, buf);
    } else {
        LCD_PrintLinePadded(lcd, 1, "SPEED: WAITING");
    }

    LCD_PrintLinePadded(lcd, 2, "");
    LCD_PrintLinePadded(lcd, 3, "");
}
