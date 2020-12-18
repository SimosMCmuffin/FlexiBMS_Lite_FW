/*
 * dStorage.h
 *
 *  Created on: 23.6.2019
 *      Author: Simos MCmuffin
 */

#ifndef DSTORAGE_MD_H_
#define DSTORAGE_MD_H_

#define __FLASH_NON_VOL_SIZE (0x200)	//how many bytes are reserved for the non-volatile parameters

#include "main.h"

uint8_t readNonVolatileParameters(uint8_t*, uint16_t);

uint8_t storeNonVolatileParameters(uint8_t*, uint16_t);

#endif /* DSTORAGE_MD_H_ */
