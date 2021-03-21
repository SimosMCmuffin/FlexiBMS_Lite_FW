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

#ifndef DSTORAGE_MD_H_
#define DSTORAGE_MD_H_

#define __FLASH_NON_VOL_SIZE (0x200)	//how many bytes are reserved for the non-volatile parameters

#include "main.h"

uint8_t readNonVolatileParameters(uint8_t*, uint16_t);

uint8_t storeNonVolatileParameters(uint8_t*, uint16_t);

#endif /* DSTORAGE_MD_H_ */
