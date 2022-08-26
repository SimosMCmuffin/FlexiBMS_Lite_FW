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

#include "stm32l4xx_hal.h"
#include <SPI_LL.h>

uint16_t SPI_initialized;

void SPI1_init(){

	if( SPI_initialized == 0 ){		//check that SPI is not initialized
		RCC->APB2ENR |= (1 << 12);	//enable SPI1 clock

		SET_CS_PIN;
		GPIOA->MODER &= ~( (3 << 10) | (3 << 12) | (3 << 14) | (3 << 8) );	//configure pins PA5, PA6 & PA7 as alternative function pins, and PA4 as a general output pin
		GPIOA->MODER |= (2 << 10) | (2 << 12) | (2 << 14) | (1 << 8);		//
		//GPIOA->OTYPER |= (1 << 5) | (1 << 6) | (1 << 7) | (1 << 4);			//set PA4, PA5, PA6 & PA7 from push-pull to open-drain

		GPIOA->AFR[0] |= (5 << 20) | (5 << 24) | (5 << 28);		//set alternative function to AF5 (SPI1)

		SPI1->CR1 |= (1 << 9) | (6 << 3) | (1 << 2) | (1 << 1) | (1 << 0);	//SSM, PCLK/64 (@16M -> 250kBaud), Master, CPOL=1, CPHA=1 (SPI mode 3)
		SPI1->CR2 |= (1 << 12) | (1 << 6) | (1 << 2);		//enable CS output (not sure why, but otherwise got MODF status error), enable RX buffer not empty interrupt

		NVIC_EnableIRQ(SPI1_IRQn);			//enable SPI1 interrupt

		SPI1->CR1 |= (1 << 6);		//SPI1 enable

		SPI_initialized = 1;		//mark SPI as initialized
	}

}

void SPI1_deInit(){

	if( SPI_initialized == 1 ){		//check that SPI is initialized
		while( !!(GPIOA->IDR & (1 << 4)) == 0 );	//wait for CS pin to to go high, indicating transaction has ended
		while( !!(SPI1->SR & (1 << 7)) == 1 );	//wait for SPI1 to be not busy
		NVIC_DisableIRQ(SPI1_IRQn);				//disable SPI1 interrupt
		SPI1->CR1 &= ~(1 << 6);					//disable SPI1

		RCC->APB2ENR &= ~(1 << 12);			//disable SPI1 bus clock

		//reset pins to analog state (lowest power consumption)
		GPIOA->MODER &= ~( (3 << 10) | (3 << 12) | (3 << 14) | (3 << 8) );	//configure pins PA4, PA5, PA6 & PA7 as analog pins
		GPIOA->MODER |= (3 << 10) | (3 << 12) | (3 << 14) | (3 << 8);		//pins will float according to external pull-up/-down resistors

		SPI_initialized = 0;		//mark SPI as non-initialized
	}

}

uint8_t SPI1_transmit(uint8_t * data){
	while( !!(SPI1->SR & (1 << 7)) );	//check that SPI bus is not busy or wait

	*(uint8_t *)&SPI1->DR = *data;			//write new data to the TX data register

	while( !!(SPI1->SR & (1 << 0)) == 0);	//wait for transmission to be completed

	uint8_t temp = SPI1->DR;				//read data from RX data register
	return temp;
}

uint8_t SPI1_readyCheck(void){	//return 1 if SPI not doing anything, return 0 if SPI busy
	if( !!(SPI1->SR & (1 << 7)) == 0 ){
		return 1;
	}
	else{
		return 0;
	}
}
