/*
 * lcd_funcs.c
 *
 *  Created on: Apr 16, 2021
 *      Author: regulus
 */
#include "main.h"
#include "fatfs.h"
#include <stdio.h>

void LCD_write_command(SPI_HandleTypeDef *hspi, const uint8_t command){
	  HAL_GPIO_WritePin(GPIOE, DC_Pin, GPIO_PIN_RESET);//command mode

	  HAL_GPIO_WritePin(GPIOE, CE_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(hspi, &command, 1, 100);//send command
	  while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
	  HAL_GPIO_WritePin(GPIOE, CE_Pin, GPIO_PIN_SET);
}

void LCD_init(SPI_HandleTypeDef *hspi){
	  HAL_GPIO_WritePin(GPIOE, RST_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, RST_Pin, GPIO_PIN_RESET);//reset lcd driver
	  HAL_GPIO_WritePin(GPIOE, RST_Pin, GPIO_PIN_SET);

	  uint8_t init[] ={0x21, 0xD0, 0b00100000, 0b00001100};//bytelar dikey yaziliyor
	  //chip active, horizontal addressing, complex instruction set, contrast, basic instructions,
	  for(unsigned int i = 0; i < sizeof(init)/sizeof(uint8_t); ++i){
		  LCD_write_command(hspi, init[i]);
	  }

}

void LCD_clear(SPI_HandleTypeDef *hspi){
	  //48hx84v 48/8=6 6*84=504, clear display
	  uint8_t clear = 0x00;
	  HAL_GPIO_WritePin(GPIOE, DC_Pin, GPIO_PIN_SET);//data mode

	  for(unsigned int i = 0; i < 504; ++i){
		HAL_GPIO_WritePin(GPIOE, CE_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi, &clear, 1, 100);//random data
		while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(GPIOE, CE_Pin, GPIO_PIN_SET);//turn chip off after setup
	  }

}


void LCD_write_byte(SPI_HandleTypeDef *hspi, const uint8_t data){
	  HAL_GPIO_WritePin(GPIOE, DC_Pin, GPIO_PIN_SET);//data mode

	  HAL_GPIO_WritePin(GPIOE, CE_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(hspi, &data, 1, 100);//send data
	  while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
	  HAL_GPIO_WritePin(GPIOE, CE_Pin, GPIO_PIN_SET);
}

void LCD_write_bitmap(SPI_HandleTypeDef *hspi, const uint8_t* data){
	  //LCD_write_command(hspi, 0b00100000);//make sure horizontal addressing is being used
	  	  //HAL_GPIO_WritePin(GPIOE, DC_Pin, GPIO_PIN_SET);//data mode

		  GPIOE->ODR |= DC_Pin;
		  GPIOE->ODR &= ~CE_Pin;//Ce low
		 //HAL_SPI_Transmit(hspi, data, 504, 100);//send data
		  //while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);//sizeof(data)/sizeof(uint8_t)
		  for(uint16_t i = 0; i < 504; ++i){
			  hspi->Instance->DR = data[i];
			  while(!(hspi->Instance->SR & SPI_SR_TXE));//wait until TXE=1
		 }

		  while(hspi->Instance->SR & SPI_SR_BSY);//wait for transmission to end
		  GPIOE->ODR |= CE_Pin;//Ce high
}

void LCD_set_cursor(SPI_HandleTypeDef *hspi, const uint8_t y, const uint8_t x){
	 uint8_t ycoor = y;
	 uint8_t xcoor = x;

	if(ycoor < 0) ycoor = 0;
	if(ycoor > 5) ycoor = 5;
	if(xcoor < 0) xcoor = 0;
	if(xcoor > 83) xcoor = 83;

	ycoor = 0b01000000 | ycoor;
	xcoor = 0b10000000 | xcoor;

	LCD_write_command(hspi, ycoor); //send y cursor command
	LCD_write_command(hspi, xcoor); //send x cursor command
}

void LCD_write_BMP(SPI_HandleTypeDef *hspi, FRESULT fres, char * filename){
		   uint8_t screendata[504] = {0};
	   	   FIL fil; 		//File handle
		   UINT br; //read count

		   fres = f_open(&fil, filename, FA_READ);//open file
		   if (fres != FR_OK) {
		 	printf("f_open error (%i), couldn't open %s\r\n", fres, filename);
		 	while(1);
		   }

		   //printf("Opened '%s' for reading.\r\n", filename);

		    BYTE readBuf[10]  = {0};

		     //fil = bman.bmp, opened previously
		     f_read(&fil, readBuf, 2, &br);//read BMP signature,
		     if(readBuf[0] == 0x42 && readBuf[1] == 0x4D) {
		    	// printf("BMP signature found. \r\n");
		     }
		     else{
		  	   printf("Not a BMP file.\r\n");
		  	   while(1);
		     }
		     uint32_t filesize = read32(&fil, readBuf, &br);
		     //printf("Filesize is %u\r\n", (unsigned int)filesize);

		     read32(&fil, readBuf, &br); //skip 4 reserved bytes

		     uint32_t pixeloffset = read32(&fil, readBuf, &br);//read pixel offset, 4 bytes
		     //printf("Pixel offset is %X.\r\n", (unsigned int)pixeloffset);
		     uint32_t dibsize =read32(&fil, readBuf, &br);
		     //printf("DIB header size is %u\r\n", (unsigned int)dibsize);

		     //uint32_t pixeloffset =0x82;
		     uint8_t bmpdata[576] = {0};//576 byte array, will store bmp pixel array

		     f_lseek(&fil, pixeloffset);//set current address to start of pixel array
		     f_read(&fil, bmpdata, 576, &br);//read image

		   for(unsigned int col = 6; col > 0; --col){//iterate 6 columns
			   for(unsigned int row = 0; row < 8; ++row){//have to read 8*12 bytes to complete a column
		 		   for(unsigned int bytes = 0; bytes < 12; ++bytes){//read row
		 	 		   for(unsigned int bit = 0; bit < 8; ++bit){//iterate bits

		 	 			   //each display column is represented by 12*8=96 bytes in the BMP
		 	 			   if(bytes > 10 || (bytes == 10 && bit >= 4)){
		 	 				   continue;//padding reached, skip
		 	 			   }

		 	 			   unsigned int screenbyte = (col-1)*84+bytes*8+bit;
		 	 			   unsigned int bmpbyte = 96*abs(col-6) + row*12 + bytes;//index of current byte to be read from BMP
		 	 			   if(bmpdata[bmpbyte] & (1<<abs(bit-7))){//check bit, set pixel if 1, bit 0 is msb
		 	 				 screendata[screenbyte] |= 1<<(7-row);
		 	 			   }
		 	 		   }
		 		   }
			   }
		  }
		    f_close(&fil);//close file when done
		    LCD_write_bitmap(hspi, screendata);
}

void LCD_write_BMP_array(SPI_HandleTypeDef *hspi, const uint8_t * bmparray){
	   uint8_t screendata[504] = {0};
	   for(unsigned int col = 6; col > 0; --col){//iterate 6 columns
	   			   for(unsigned int row = 0; row < 8; ++row){//have to read 8*12 bytes to complete a column
	   		 		   for(unsigned int bytes = 0; bytes < 12; ++bytes){//read row
	   		 	 		   for(unsigned int bit = 0; bit < 8; ++bit){//iterate bits

	   		 	 			   //each display column is represented by 12*8=96 bytes in the BMP
	   		 	 			   if(bytes > 10 || (bytes == 10 && bit >= 4)){
	   		 	 				   continue;//padding reached, skip
	   		 	 			   }

	   		 	 			   unsigned int screenbyte = (col-1)*84+bytes*8+bit;
	   		 	 			   unsigned int bmpbyte = 96*abs(col-6) + row*12 + bytes;//index of current byte to be read from BMP
	   		 	 			   if(bmparray[bmpbyte] & (1<<abs(bit-7))){//check bit, set pixel if 1, bit 0 is msb
	   		 	 				 screendata[screenbyte] |= 1<<(7-row);
	   		 	 			   }
	   		 	 		   }
	   		 		   }
	   			   }
	   		  }
	    LCD_write_bitmap(hspi, screendata);

}

