/*
 * CAN.c low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#include "stm32l4xx_hal.h"
#include "CAN.h"

void CAN1_init(){

	RCC->APB1ENR1 |= (1 << 25);				//Enable CAN1 bus clock

	GPIOA->MODER = (GPIOA->MODER & ~(3 << 20)) | (1 << 20);		//Configure pin PA10 as OUTPUT for CAN silent mode control
	GPIOA->BSRR |= (1 << 26);									//don't Put CAN into silent mode (refer to TJA1051 datasheet for functional description)

	GPIOB->MODER &= ~( (3 << 16) | (3 << 18) );			//Configure pins PB8 and PB9 alternative functions
	GPIOB->MODER |= (2 << 16) | (2 << 18);
	GPIOB->AFR[1] |= (9 << 0) | (9 << 4);				//set alternative function to AF9(CAN1)

	CAN1->MCR |= (1 << 4);				//no automatic retransmission

	CAN1->MCR |= (1 << 0);				//request initialization
	while( !(CAN1->MSR & (1 << 0)) );		//wait for initialization to be done

	CAN1->BTR = 0;
	CAN1->BTR |= (1 << 24) | (1 << 20) | (4 << 16) | (3 << 0);		//Set timings for 500kHz CAN baud with 16MHz source clock
	CAN1->MCR &= ~(1 << 0);				//request to enter normal mode

	CAN1->MCR &= ~(1 << 1);				//exit sleep mode

}

void CAN1_deInit(){

}

uint8_t CAN1_transmit(uint32_t ID, uint8_t * data, uint32_t length){

	if( !!(CAN1->TSR & (7 << 26)) ){		//Check that at least one TX mailbox is empty
		uint8_t TX_empty = (CAN1->TSR & (3 << 24)) >> 24;	//check which TX mailbox is empty

		CAN1->sTxMailBox[TX_empty].TIR = 0;
		CAN1->sTxMailBox[TX_empty].TIR = (ID << 21);	//set CAN ID
		CAN1->sTxMailBox[TX_empty].TDTR = 0;
		CAN1->sTxMailBox[TX_empty].TDTR = (length << 0);	//set data length

		CAN1->sTxMailBox[TX_empty].TDLR = 0;		//empty lower data byte register
		CAN1->sTxMailBox[TX_empty].TDHR = 0;		//empty higher data byte register

		if( length <= 4 ){							//write data if length <= 4
			for(uint8_t x=0; x<length; x++){
				CAN1->sTxMailBox[TX_empty].TDLR |= (data[x] << x*8);
			}
		}
		else if( length <= 8 ){						//write data if length <= 8
			for(uint8_t x=0; x<4; x++){
				CAN1->sTxMailBox[TX_empty].TDLR |= (data[x] << x*8);
			}
			for(uint8_t x=0; x<(length-4); x++){
				CAN1->sTxMailBox[TX_empty].TDHR |= (data[x+4] << x*8);
			}

		}

		CAN1->sTxMailBox[TX_empty].TIR |= (1 << 0);	//with everything written, request mailbox transmitting


	}
	else{
		return 0;		//return 0 to indicate failure
	}


	return 1;
}
