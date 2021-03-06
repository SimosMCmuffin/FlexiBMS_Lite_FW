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

#ifndef FLASH_LL_H_
#define FLASH_LL_H_

#define __FLASH_PAGE_62	(0x0801F000)
#define __FLASH_PAGE_63	(0x0801F800)
#define __END_OF_FLASH	(0x08020000)

uint8_t checkFlashAddress(uint32_t);
uint8_t eraseFlashPage(uint8_t);
uint8_t readFlash(void*, uint32_t, uint32_t);
uint8_t writeFlashDword(uint32_t, uint64_t);

#endif /* FLASH_LL_H_ */
