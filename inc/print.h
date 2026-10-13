/*
 *  print.h
 *
 *  Created on: May 21, 2026
 *      Author: annatai
 * 
 *  enables printf for serial output
 *  
 *  you MUST have these for this to work:
 *  - UART configured in CubeIDE IOC
 *  - add this line to int main() USER CODE BEGIN 2
 *      setvbuf(stdout, NULL, _IONBF, 0); // DISMISS BUFFERING: Forces printf to send data immediately
 */

#include "stm32l4xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef hlpuart1; // adjust for UART channel being used

// Override the GCC standard library write function
int _write(int file, char *ptr, int len) 
{
    // Transmit the entire buffer at once over LPUART1
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}