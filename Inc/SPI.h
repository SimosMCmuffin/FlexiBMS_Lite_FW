/*
 * SPI.h low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#ifndef SPI_H_
#define SPI_H_

#define SET_CS_PIN	(GPIOA->BSRR |= (1 << 4))
#define CLR_CS_PIN	(GPIOA->BSRR |= (1 << 20))

void SPI1_init(void);
void SPI1_deInit(void);
uint8_t SPI1_transmit(uint8_t *);


#endif /* SPI_H_ */
