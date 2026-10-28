#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RW1063 / HD44780 instruction opcodes */
#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

/* Command flags */
#define LCD_ENTRYLEFT      0x02
#define LCD_DISPLAYON      0x04
#define LCD_CURSORON       0x02
#define LCD_BLINKON        0x01

#define LCD_COLS 20
#define LCD_ROWS 4

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *rs_port;
    uint16_t rs_pin;
    uint8_t displayControl;
} LCD_Handle;

void LCD_Init(LCD_Handle *lcd, SPI_HandleTypeDef *hspi,
              GPIO_TypeDef *cs_port, uint16_t cs_pin,
              GPIO_TypeDef *rs_port, uint16_t rs_pin);

void LCD_Clear(LCD_Handle *lcd);
void LCD_Display(LCD_Handle *lcd, bool turnOn);
void LCD_CursorMode(LCD_Handle *lcd, bool underline, bool blink);
void LCD_SetCursor(LCD_Handle *lcd, uint8_t col, uint8_t row);

void LCD_PrintChar(LCD_Handle *lcd, char c);
void LCD_PrintStr(LCD_Handle *lcd, const char *str);
void LCD_PrintCharAt(LCD_Handle *lcd, uint8_t col, uint8_t row, char c);
void LCD_PrintStrAt(LCD_Handle *lcd, uint8_t col, uint8_t row, const char *str);
void LCD_PrintLinePadded(LCD_Handle *lcd, uint8_t row, const char *text);

void LCD_DisplaySpeed(LCD_Handle *lcd, float speed_mph, bool has_fix);

#ifdef __cplusplus
}
#endif

#endif /* INC_LCD_H_ */
