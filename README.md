# stm32f4-5510LCD-video-player
**A simple STM32F429 program plays video on a Nokia 5510 LCD and plays audio using the internal DAC
**


This is an STM32F429 program that displays frames(20FPS) read from an SD card on a Nokia 5510 LCD while playing audio that is also read from the SD card. FATFS and the SDIO interface are used to handle the SD card. 

The 5510 LCD is a monochrome 48x80 display meaning each pixel is represented by a single bit. The smallest unit of data accepted by the display is a byte and those are always written vertically. This is a problem as the BMP file format, which is used for the frames, encodes pixels from bottom to top horizontally. Due to this, each frame has to be read and converted before being sent to the display. This is done by the functions LCD_write_BMP and LCD_write_BMP_array in the file lcd_funcs.c. The LCD_write_BMP function accepts a file name, reads the image from the SD card using the file name, checks the BMP header, and writes the image to the display after converting it. The LCD_write_BMP_array function on the other hand only accepts a unit8_t array which is the pixel array of the BMP file, it only converts the pixel data and sends it to the display. This function is used for video playback as opening 20 files per second would have too much overhead.

Due to limitations of the FAT32 filesystem(or potentitally the FATFS library), having thousands of BMP files in a single directory is not feasible as opening files starts to take too long, effectively turning the video to a slideshow after a few seconds. To circumvent this, the BMP files containing the frames are concatenated to a single binary file meaning only a single file needs to be opened to play a video, reducing overhead. The main loop reads 10 frames into a buffer/array. Since the sizes of the BMP files and the offsets of their pixels arrays are known, pixel data of frames by can be read and passed to the LCd functions directly without having to read and process the BMP headers. 

The audio section on the other hand is simpler in comparison. The program reads 22500Hz 8bit audio from the SD card. At the start of the program, a 2048 byte buffer is declared and 2048 bytes are read from the audio file. Timer 2 is configured according to the sample rate of the audio and its interrupt writes new samples to the DAC's output register while also incrementing the audio counter or resetting it when it reached the buffer size. The main loop is responsible for ensuring that the buffer always contains new samples. If the audio counter shows that the first half of the buffer has been played, 1024 new bytes are read and copied to the first half of the audio buffer, if the audio counter reaches the end of the buffer, then 1024 new samples are read and copied to the second half of the buffer. This way the timer interrupt always has new samples to play.

The audio and frame buffers are stored in CCMRAM(core-coupled memory) for extra speed, this was achieved by modifying the linker script and the startup file.

Audacity was used to resample audio and export it as raw 8-bit PCM. FFMPEG and mogrify were used to extract individual frames as BMP files and turn them into 1-bit monochrome images respectively.

This program comes with no guarantees whatsoever and it may contain errors or bugs as I am not a professional programmer or embedded developer. If you notice any issues with it or happen to have a question, please contact me.
