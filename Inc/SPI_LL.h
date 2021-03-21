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

#ifndef SPI_LL_H_
#define SPI_LL_H_

#define SET_CS_PIN	(GPIOA->BSRR |= (1 << 4))
#define CLR_CS_PIN	(GPIOA->BSRR |= (1 << 20))

uint16_t SPI_initialized;

void SPI1_init(void);
void SPI1_deInit(void);
uint8_t SPI1_transmit(uint8_t *);
uint8_t SPI1_readyCheck(void);


#endif /* SPI_LL_H_ */
