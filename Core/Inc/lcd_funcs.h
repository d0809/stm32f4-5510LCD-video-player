/*
 * lcd_funcs.h
 *
 *  Created on: Apr 16, 2021
 *      Author: regulus
 */

#ifndef INC_LCD_FUNCS_H_
#define INC_LCD_FUNCS_H_



#endif /* INC_LCD_FUNCS_H_ */
void LCD_write_command(SPI_HandleTypeDef *hspi, const uint8_t command);

void LCD_init(SPI_HandleTypeDef *hspi);

void LCD_clear(SPI_HandleTypeDef *hspi);

void LCD_write_byte(SPI_HandleTypeDef *hspi, const uint8_t data);

void LCD_write_bitmap(SPI_HandleTypeDef *hspi, const uint8_t* data);

void LCD_write_BMP(SPI_HandleTypeDef *hspi, FRESULT fres, char * filename);//read .bmp from SD card and draw it on the LCD

void LCD_write_BMP_array(SPI_HandleTypeDef *hspi, const uint8_t * bmparray);//data from bmp stored in array from SD card and draw it on the LCD

void LCD_set_cursor(SPI_HandleTypeDef *hspi, const uint8_t y, const uint8_t x);


