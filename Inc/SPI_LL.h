/*
 * SPI.h low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
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
