#include "I2C_LCD_cfg.h"
extern I2C_HandleTypeDef hi2c1;

const I2C_LCD_CfgType I2C_LCD_CfgParam[I2C_LCD_MAX] =
{
	{   /*  Configuration Parameter For I2C_LCD Instance #1   */
		I2C_LCD_1,	/* Index of I2C_LCD Instance #1           */
		&hi2c1,		/* Hardware I2C Module's Handle           */
		0x3F,		/* Hardware I2C_LCD Device Address        */
		20,			/* LCD Columns Count                      */
		4			/* LCD Rows Count                         */
	}
};
