# stm32f4-5510LCD-video-player
A simple STM32F429 program plays video on a Nokia 5510 LCD and plays audio using the internal DAC



This is an STM32F429 program that displays frames(20FPS) read from an SD card on a Nokia 5510 LCD while playing audio that is also read from the SD card. FATFS and the SDIO interface are used to handle the SD card. 

The 5510 LCD is a monochrome 48x80 display meaning each pixel is represented by a single bit. The smallest unit of data accepted by the display is a byte and those are always written vertically. This is a problem as the BMP file format, which is used for the frames, encodes pixels from bottom to top horizontally. Due to this, each frame has to be read and converted before being sent to the display. This is done by the functions LCD_write_BMP and LCD_write_BMP_array in the file lcd_funcs.c. The LCD_write_BMP function accepts a file name, reads the image from the SD card using the file name, checks the BMP header, and writes the image to the display after converting it. The LCD_write_BMP_array function on the other hand only accepts a unit8_t array which is the pixel array of the BMP file, it only converts the pixel data and sends it to the display. This function is used for video playback as opening 20 files per second would have too much overhead.
