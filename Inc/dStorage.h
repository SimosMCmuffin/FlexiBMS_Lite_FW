/*
 * dStorage.h
 *
 *  Created on: 23.6.2019
 *      Author: Simos MCmuffin
 */

#ifndef DSTORAGE_H_
#define DSTORAGE_H_

#define __FLASH_NON_VOL_SIZE (0x200)	//how many bytes are reserved for the non-volatile parameters

#include "main.h"

uint8_t loadNonVolatileParameters(nonVolParameters*);

uint8_t saveNonVolatileParameters(nonVolParameters*);

#endif /* DSTORAGE_H_ */
