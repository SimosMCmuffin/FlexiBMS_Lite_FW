/*
 * CAN.h low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#ifndef CAN_H_
#define CAN_H_

void CAN1_init(void);
void CAN1_deInit(void);
uint8_t CAN1_transmit(uint32_t, uint8_t *, uint32_t);


#endif /* CAN_H_ */
