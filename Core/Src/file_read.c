/*
 * file_read.c
 *
 *  Created on: Apr 28, 2021
 *      Author: regulus
 */

#include "main.h"
#include "fatfs.h"

uint32_t read32(FIL* file, BYTE* buff, UINT* br){
	uint32_t retval = 0;
	f_read(file, buff, 4, br);
	 for(unsigned int i = 0; i < 4; ++i){
		   retval |= (buff[i]<<(i*8)); //turn 4 bytes in buffer to a single uint32
	   }
	return retval;
}

uint16_t read16(FIL* file, BYTE* buff, UINT* br){
	uint16_t retval = 0;
	f_read(file, buff, 2, br);
	 for(unsigned int i = 0; i < 2; ++i){
		   retval |= (buff[i]<<(i*8)); //turn 2 bytes in buffer to a single uint16
	   }
	return retval;
}
