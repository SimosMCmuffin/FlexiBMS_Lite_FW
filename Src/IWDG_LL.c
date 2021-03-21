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
#include <IWDG_LL.h>

void IWDG_init(void){

	if( IWDG_initialized == 0 ){		//check that IWDG is not initialized

		DBGMCU->APB1FZR1 = 0x00001000;	//IDWG stopped when debugging

		IWDG->KR = 0x5555;	//unlock access to IWDG registers
		IWDG->PR = 6;		//prescaler 256, 8ms tick
		IWDG->RLR = 1400;	//1400*8ms = 11200ms

		IWDG_initialized = 1;		//mark IWDG as initialized
	}

}


void IWDG_start(void){	//used to start IWDG

	if( IWDG_initialized == 1){
		IWDG->KR = 0xCCCC;	//start command for IWDG
		for(uint8_t x=0; x<20; x++);
		IWDG->KR = 0xAAAA;		//reset counter command for IWDG
	}

}

void IWDG_zeroCounter(void){	//reset IWDG reset counter

	IWDG->KR = 0xAAAA;		//reset counter command for IWDG

}

