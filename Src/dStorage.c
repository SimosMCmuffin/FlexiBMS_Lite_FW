/*
 * dStorage.c
 *
 *  Created on: 23.6.2019
 *      Author: Simos MCmuffin
 */

#include "main.h"
#include "LL_Flash.h"
#include "dStorage.h"

uint8_t loadNonVolatileParameters(nonVolParameters* nonVols){

	for(uint8_t x=0; x<=8; x++){			//find last written non-volatiles
		if( x == 8)							//if not any data found, return 0
			return 0;

		if( checkFlashAddress( __END_OF_FLASH - (__FLASH_NON_VOL_SIZE * (x + 1)) ) == 0 ){
			readFlash(nonVols, (__END_OF_FLASH - (__FLASH_NON_VOL_SIZE * (x + 1))), sizeof(nonVolParameters));
			return 1;
		}
	}


	return 0;
}

uint8_t saveNonVolatileParameters(nonVolParameters* nonVols){
	uint8_t size = sizeof(nonVolParameters)/8;

	if( (sizeof(nonVolParameters)%8) != 0 )
		size++;

	for(uint8_t x=0; x<=8; x++){			//find first un-written data spot
		if( x == 8 ){						//if all data spots full, then erase first page, write data and then erase second page
			eraseFlashPage(62);

			for(uint8_t y = 0; y < size ; y++){
				writeFlashDword( (__FLASH_PAGE_62 + (y*8)) , (*((y) + (uint64_t*)nonVols)) );
			}

			eraseFlashPage(63);
			return 1;
		}
		else if( checkFlashAddress( (__FLASH_PAGE_62 + (__FLASH_NON_VOL_SIZE * x)) ) ){	//scan for first free spot

			for(uint8_t y = 0; y < size ; y++){
				writeFlashDword( ((__FLASH_PAGE_62 + (__FLASH_NON_VOL_SIZE * x)) + (y*8)) , (*((y) + (uint64_t*)nonVols)) );
			}

			return 1;
		}

	}

	return 0;
}
