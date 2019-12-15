/*
 * CAN.h low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#ifndef CAN_LL_H_
#define CAN_LL_H_

void CAN1_init(void);
void CAN1_deInit(void);
uint8_t CAN1_receive(uint32_t*, uint8_t *, uint8_t *);
uint8_t CAN1_transmit(uint32_t, uint8_t *, uint8_t);
uint8_t CAN1_rxAvailable(void);
void CAN1_setupRxFilters(void);
void CAN1_debugEcho(void);

#endif /* CAN_LL_H_ */
