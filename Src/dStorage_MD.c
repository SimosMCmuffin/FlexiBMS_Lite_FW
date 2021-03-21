/*
	Copyright 2019 - 2021 Simo Sihvonen	"Simos MCmuffin" - simo.sihvonen@gmail.com

	This file is part of the FlexiBMS Lite firmware.

	The FlexiBMS Lite firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The FlexiBMS Lite firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
