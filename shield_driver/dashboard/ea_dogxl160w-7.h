#ifndef EA_DOGXL160W_7_H
#define EA_DOGXL160W_7_H

#include <stm32f4xx_hal.h>
#include <stdbool.h>
#include "Fonts/fonts.h"

#define LCD_CS_PIN					GPIO_PIN_5
#define LCD_CS_GPIO_PORT 			GPIOC
#define LCD_CS_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOC_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()	__HAL_RCC_GPIOC_CLK_DISABLE()

#define LCD_CS_ON()		HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET);
#define LCD_CS_OFF()	HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET);

#define LCD_CD_PIN					GPIO_PIN_4
#define LCD_CD_GPIO_PORT 			GPIOC
#define LCD_CD_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOC_CLK_ENABLE()
#define LCD_CD_GPIO_CLK_DISABLE()	__HAL_RCC_GPIOC_CLK_DISABLE()

#define LCD_CD_ON()		HAL_GPIO_WritePin(LCD_CD_GPIO_PORT, LCD_CD_PIN, GPIO_PIN_SET);
#define LCD_CD_OFF()	HAL_GPIO_WritePin(LCD_CD_GPIO_PORT, LCD_CD_PIN, GPIO_PIN_RESET);

#define PIXELS_X_AXIS	160
#define PIXELS_Y_AXIS	104

void init_lcd(void);
void paintString(int x_pos, int y_pos, char *str, int scale, sFONT *font);
void paintChar(int x_pos, int y_pos, char c, int scale, sFONT *font);
void paint(void); // writes the frame buffer to the LCD
void line(int x0, int y0, int x1, int y1, bool paint);

#endif /* EA_DOGXL160W_7_H */
