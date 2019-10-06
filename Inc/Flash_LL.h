/*
 * LL_Flash.h
 *
 *  Created on: Jun 21, 2019
 *      Author: Simos MCmuffin
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
