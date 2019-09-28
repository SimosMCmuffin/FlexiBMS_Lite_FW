/*
 * LL_Flash.c
 *
 *  Created on: Jun 21, 2019
 *      Author: Simos MCmuffin
 */

#include "main.h"
#include "LL_Flash.h"
#include "string.h"

/**
 *
 * @param flashAddr
 * @return
 */
uint8_t checkFlashAddress(uint32_t flashAddr){
	//*(uint32_t*)flashAddr == 0x20010000	//is the data type/value in the pointed address this?
	//POINTERS ARE ALWAYS AS BIG AS NEEDED TO POINT TO THE WHOLE MEMORY SPACE, so 32 bit in this case
	//| this       and | this need to be the same type, or casting error is generated
	uint32_t* data = (uint32_t*)flashAddr;

	if( *data == 0xFFFFFFFF )
		return 1;
	else
		return 0;
}

/**
 *
 * @param flashPage
 * @return
 */
uint8_t eraseFlashPage(uint8_t flashPage){
	while (FLASH->SR & FLASH_SR_BSY);	//wait for Flash not to be busy

	if( !!(FLASH->CR & (1 << 31)) == 1 ){	//is flash locked?
		FLASH->KEYR = 0x45670123;		//write flash unlock keys
		FLASH->KEYR = 0xCDEF89AB;
	}

	//clear ALL error flags
	FLASH->SR |= (1 << 15) | (1 << 14) | (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0);

	FLASH->CR &= ~((1 << 1) | (0xFF << 3));	//reset page erase settings
	FLASH->CR |= (1 << 1) | (flashPage << 3);	//enable page erase and set page number

	FLASH->CR |= (1 << 16);		//start page erase
	while (FLASH->SR & FLASH_SR_BSY);	//wait for page erase to finish

	FLASH->CR &= ~(1 << 1);		//clear page erase enabled bit
	FLASH->CR |= (1 << 31);		//set flash LOCK bit

	if( !!(FLASH->SR & (1 << 0)) == 1 )		//check if End Of Operation flag is up, indicating successful erase
		return 1;
	else
		return 0;
}

uint8_t readFlash(void* data, uint32_t flashAddr, uint32_t dataLen){
//	uint8_t* target = (uint8_t*)data;
//
//	for(uint32_t x=0; x<dataLen; x++){
//		target[x] = *(uint8_t*)(flashAddr + x);
//	}

	memcpy(data, (uint8_t*)flashAddr, dataLen);

	return 0;
}

uint8_t writeFlashDword(uint32_t flashAddr, uint64_t data){
	if( (flashAddr & 0x7) != 0 )	//check if write address is doubleWord/64-bit/8-byte aligned
		return 0;

	while (FLASH->SR & FLASH_SR_BSY);	//wait for Flash not to be busy

	if( !!(FLASH->CR & (1 << 31)) == 1 ){	//is flash locked?
		FLASH->KEYR = 0x45670123;		//write flash unlock keys
		FLASH->KEYR = 0xCDEF89AB;
	}

	//clear ALL error flags
	FLASH->SR |= (1 << 15) | (1 << 14) | (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0);

	FLASH->CR |= (1 << 0);	//enable Flash programming

	*(uint32_t*)flashAddr = data;
	*(uint32_t*)(flashAddr+4) = (data >> 32);

	FLASH->CR &= ~(1 << 0);	//disable Flash programming
	FLASH->CR |= (1 << 31);		//set flash LOCK bit

	if( !!(FLASH->SR & (1 << 0)) == 1 )		//check if End Of Operation flag is up, indicating successful erase
		return 1;
	else
		return 0;
}
