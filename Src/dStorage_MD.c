/*
 * dStorage.c
 *
 *  Created on: 23.6.2019
 *      Author: Simos MCmuffin
 */

#include "main.h"
#include <Flash_LL.h>
#include <dStorage_MD.h>

uint8_t readNonVolatileParameters(uint8_t* data, uint16_t length){

	for(uint8_t x=0; x<=8; x++){			//find last written non-volatiles
		if( x == 8)							//if not any data found, return 0
			return 0;

		if( checkFlashAddress( __END_OF_FLASH - (__FLASH_NON_VOL_SIZE * (x + 1)) ) == 0 ){
			readFlash(data, (__END_OF_FLASH - (__FLASH_NON_VOL_SIZE * (x + 1))), length);
			return 1;
		}
	}


	return 0;
}

uint8_t storeNonVolatileParameters(uint8_t* data, uint16_t length){
	uint8_t size = length/8;

	if( (length%8) != 0 )
		size++;

	for(uint8_t x=0; x<=8; x++){			//find first un-written data spot
		if( x == 8 ){						//if all data spots full, then erase first page, write data and then erase second page
			eraseFlashPage(62);

			for(uint8_t y = 0; y < size ; y++){
				writeFlashDword( (__FLASH_PAGE_62 + (y*8)) , (*((y) + (uint64_t*)data)) );
			}

			eraseFlashPage(63);
			return 1;
		}
		else if( checkFlashAddress( (__FLASH_PAGE_62 + (__FLASH_NON_VOL_SIZE * x)) ) ){	//scan for first free spot

			for(uint8_t y = 0; y < size ; y++){
				writeFlashDword( ((__FLASH_PAGE_62 + (__FLASH_NON_VOL_SIZE * x)) + (y*8)) , (*((y) + (uint64_t*)data)) );
			}

			return 1;
		}

	}

	return 0;
}
