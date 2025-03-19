#include "gpio.h"
#include <lcd.h>
#include "stm32g0xx_hal.h"
void send_to_lcd(char data, int rs)
{
    // RS pin (pro command/data)
    HAL_GPIO_WritePin(RS_PORT, RS_PIN, rs);  // rs = 1 for data, rs = 0 for command

    /* Write the data to the respective pins */
    HAL_GPIO_WritePin(D7_PORT, D7_PIN, ((data >> 3) & 0x01)); // D7
    HAL_GPIO_WritePin(D6_PORT, D6_PIN, ((data >> 2) & 0x01)); // D6
    HAL_GPIO_WritePin(D5_PORT, D5_PIN, ((data >> 1) & 0x01)); // D5
    HAL_GPIO_WritePin(D4_PORT, D4_PIN, ((data >> 0) & 0x01)); // D4

    /* Toggle EN PIN to send the data */
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_SET); // Set EN to high
    HAL_Delay(20); // Optional delay if needed for timing (e.g., 20 us delay)
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, GPIO_PIN_RESET); // Set EN to low
    HAL_Delay(20); // Optional delay
}
void lcd_send_cmd (char cmd)
{
    char datatosend;
    /* send upper nibble first */
    datatosend = ((cmd>>4)&0x0f);
    send_to_lcd(datatosend,0);  // RS must be while sending command
    /* send Lower Nibble */
    datatosend = ((cmd)&0x0f);
    send_to_lcd(datatosend, 0);
}
void lcd_send_data (char data)
{
    char datatosend;

    /* send higher nibble */
    datatosend = ((data>>4)&0x0f);
    send_to_lcd(datatosend, 1);  // rs =1 for sending data
    /* send Lower nibble */
    datatosend = ((data)&0x0f);
    send_to_lcd(datatosend, 1);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}
void lcd_init (void)
{
    // 4 bit initialisation
    HAL_Delay(50);  // wait for >40ms
    lcd_send_cmd (0x30);
    HAL_Delay(5);  // wait for >4.1ms
    lcd_send_cmd (0x30);
    HAL_Delay(1);  // wait for >100us
    lcd_send_cmd (0x30);
    HAL_Delay(10);
    lcd_send_cmd (0x20);  // 4bit mode
    HAL_Delay(10);

  // dislay initialisation
    lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
    HAL_Delay(1);
    lcd_send_cmd (0x01);  // clear display
    HAL_Delay(1);
    HAL_Delay(1);
    lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    HAL_Delay(1);
    lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}
void lcd_send_string (char * str)
{
	while (*str) lcd_send_data (*str++);
}
void lcd_clear (void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}
