#ifndef LCD_H
#define LCD_H

#include "gpio.h"
#include "stm32g0xx_hal.h"

// Define the GPIO ports and pins for the LCD control signals
#define RS_PORT GPIOB // Port for RS pin, from your GPIO initialization
#define RS_PIN GPIO_PIN_0 // Pin number for RS (from the GPIO initialization)

#define EN_PORT GPIOB // Port for EN pin, from your GPIO initialization
#define EN_PIN GPIO_PIN_1 // Pin number for EN (from the GPIO initialization)

#define D7_PORT GPIOA // Port for D7 pin, from your GPIO initialization
#define D7_PIN GPIO_PIN_7 // Pin number for D7 (from the GPIO initialization)

#define D6_PORT GPIOA // Port for D6 pin, from your GPIO initialization
#define D6_PIN GPIO_PIN_6 // Pin number for D6 (from the GPIO initialization)

#define D5_PORT GPIOB // Port for D5 pin, from your GPIO initialization
#define D5_PIN GPIO_PIN_4 // Pin number for D5 (from the GPIO initialization)

#define D4_PORT GPIOB // Port for D4 pin, from your GPIO initialization
#define D4_PIN GPIO_PIN_5 // Pin number for D4 (from the GPIO initialization)
// Function prototypes
void send_to_lcd(char data, int rs);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_put_cur(int row, int col);
void lcd_init(void);
void lcd_send_string(char *str);
void lcd_clear(void);

#endif // LCD_H
