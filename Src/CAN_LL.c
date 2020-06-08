/*
 * CAN.c low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#include "stm32l4xx_hal.h"
#include <CAN_LL.h>

void CAN1_init(){

	if( CAN_init == 0){					//check that CAN is not initialized
		RCC->APB1ENR1 |= (1 << 25);				//Enable CAN1 bus clock

		GPIOA->MODER = (GPIOA->MODER & ~(3 << 20)) | (1 << 20);		//Configure pin PA10 as OUTPUT for CAN silent mode control
		GPIOA->BSRR |= (1 << 26);									//don't Put CAN into silent mode (refer to TJA1051 datasheet for functional description)

		GPIOB->MODER &= ~( (3 << 16) | (3 << 18) );			//Configure pins PB8 and PB9 alternative functions
		GPIOB->MODER |= (2 << 16) | (2 << 18);
		GPIOB->AFR[1] |= (9 << 0) | (9 << 4);				//set alternative function to AF9(CAN1)

		CAN1->MCR |= (1 << 0);				//request initialization
		while( !!(CAN1->MSR & (1 << 0)) == 0 );		//wait to enter initialization mode

		CAN1->MCR |= (1 << 6) | (1 << 2);		//automatic bus-off management, *no automatic retransmission*, TX order by chronologically

		CAN1->BTR = 0;
		CAN1->BTR |= (3 << 24) | (2 << 20) | (11 << 16) | (1 << 0);		//Set timings for 500kHz CAN baud with 16MHz source clock, 81%
		CAN1->MCR &= ~(1 << 0);				//request to enter normal mode

		CAN1->MCR &= ~(1 << 1);				//exit sleep mode
		while( !!(CAN1->MSR & (1 << 1)) == 1 );			//wait for CAN1 to exit sleep mode

		CAN_init = 1;		//mark CAN as initialized
	}

}

void CAN1_deInit(){

	if( CAN_init == 1){			//check that CAN is initialized
		CAN1->MCR |= (1 << 1);		//request to enter sleep mode
		while( !!(CAN1->MSR & (1 << 1)) == 0 );		//wait for bus activity to finish/stop and enter sleep mode
		RCC->APB1ENR1 &= ~(1 << 25);			//Disable CAN1 bus clock

		GPIOB->MODER &= ~( (3 << 16) | (3 << 18) );			//Configure pins PB8 and PB9 to analog state for low-power usage
		GPIOB->MODER |= (3 << 16) | (3 << 18);

		CAN_init = 0;		//mark CAN as non-initialized
	}

}

uint8_t CAN1_transmit(uint32_t ID, uint8_t * data, uint8_t length){

	if( !!(CAN1->TSR & (7 << 26)) && CAN_init == 1 ){		//Check that at least one TX mailbox is empty and that CAN is iniatilized
		uint8_t TX_empty = 0;// (CAN1->TSR & (3 << 24)) >> 24;	//check which TX mailbox is empty


		if( !!(CAN1->TSR & (1 << 26)) == 1 )
			TX_empty = 0;
		else if( !!(CAN1->TSR & (1 << 27)) == 1 )
			TX_empty = 1;
		else
			TX_empty = 2;


		CAN1->sTxMailBox[TX_empty].TIR = 0;

		if( ID <= 0x7FF ){		//decide whether to use standard or extended ID frame based on the argument ID's size
			CAN1->sTxMailBox[TX_empty].TIR = (ID << 21);	//set CAN ID (standard, 11-bit)
		}
		else if( ID >= 0x800 && ID <= 0x1FFFFFFF){
			CAN1->sTxMailBox[TX_empty].TIR = (ID << 3);	//set CAN ID (extended, 29-bit)
			CAN1->sTxMailBox[TX_empty].TIR |= (1 << 2);	//use extended CAN ID
		}
		else{
			return 0;	//return 0 if ID out of range
		}

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

uint8_t CAN1_rxAvailable(){		//check if CAN RX packets available
	if( (CAN1->RF0R & (3 << 0)) != 0 ){
		return 1;
	}
	else if( (CAN1->RF1R & (3 << 0)) != 0 ){
		return 1;
	}
	else{
		return 0;
	}
}

void CAN1_setupRxFilters(){
	CAN1->FMR = 1;		//filter init mode

	CAN1->FM1R = 0;		//all filters in mask mode
	CAN1->FS1R = 0x3FFF;	//all filters in single 32-bit configuration
	CAN1->FFA1R = 0;	//assing all filters for FIFO 0
	CAN1->FA1R = 1;		//Filter 0 active

	CAN1->sFilterRegister[0].FR1 = 0;	//ID 0
	CAN1->sFilterRegister[0].FR2 = 0;	//set ALL mask bits to DO_NOT_CARE, meaning all messages will be read into RX FIFO

	CAN1->FMR = 0;		//filter active mode
}

void CAN1_debugEcho(){
	//check if CAN messages available in any RX mailbox and send first one back into bus with identical information

	if( CAN1_rxAvailable() ){
		uint8_t length = 0, data[8] = {0};
		uint32_t ID = 0;

		CAN1_receive(&ID, data, &length);
		ID = ID >> 21;

		CAN1_transmit(ID, data, length);
	}

}

uint8_t CAN1_receive(uint32_t * ID, uint8_t * data, uint8_t * length){
	//check if mail available in either RX mailbox, if not return 0

	if( (CAN1->RF0R & (3 << 0)) != 0 ){
		if( !!(CAN1->sFIFOMailBox[0].RIR & (1 << 2)) == 1 ){	//check if standard or extended ID
			*ID = CAN1->sFIFOMailBox[0].RIR >> 3;
		}
		else{
			*ID = CAN1->sFIFOMailBox[0].RIR >> 21;
		}

		*length = CAN1->sFIFOMailBox[0].RDTR & (0xF << 0);	//read/extract data length

		for(uint8_t x=0; x<4; x++){			//read/extract lower 4 bytes into buffer
			data[x] = (CAN1->sFIFOMailBox[0].RDLR >> x*8);
		}

		for(uint8_t x=0; x<4; x++){			//read/extract higher 4 bytes into buffer
			data[x+4] = (CAN1->sFIFOMailBox[0].RDHR >> x*8);
		}

		CAN1->RF0R |= (1 << 5);		//release received message
		return 1;
	}
	else if( (CAN1->RF1R & (3 << 0)) != 0 ){
		if( !!(CAN1->sFIFOMailBox[1].RIR & (1 << 2)) == 1 ){	//check if standard or extended ID
			*ID = CAN1->sFIFOMailBox[1].RIR >> 3;
		}
		else{
			*ID = CAN1->sFIFOMailBox[1].RIR >> 21;
		}

		*length = CAN1->sFIFOMailBox[1].RDTR & (0xF << 0);	//read/extract data length

		for(uint8_t x=0; x<4; x++){			//read/extract lower 4 bytes into buffer
			data[x] = (CAN1->sFIFOMailBox[1].RDLR >> x*8);
		}

		for(uint8_t x=0; x<4; x++){			//read/extract higher 4 bytes into buffer
			data[x+4] = (CAN1->sFIFOMailBox[1].RDHR >> x*8);
		}

		CAN1->RF1R |= (1 << 5);		//release received message
		return 1;
	}
	else{
		return 0;
	}

}

uint8_t CAN1_enableLoopBackMode(void){

	if( !!(CAN1->BTR & (1 << 30)) == 0 ){	//check if loop back mode is enabled yet

		if( !!(CAN1->MSR & (1 << 0)) == 0 ){	//check if CAN1 is in initialization mode, if not put it in init mode
			CAN1->MCR |= (1 << 0);				//request initialization
			while( !(CAN1->MSR & (1 << 0)) );		//wait for initialization to be done
		}

		CAN1->BTR |= (1 << 30);		//enable loop back mode

		CAN1->MCR &= ~(1 << 0);		//request to enter back to normal mode

	}
	else{
		return 0;	//return 0 if loop back mode already enabled
	}

	return 1; 	//loop back mode enabled

}

uint8_t CAN1_disableLoopBackMode(void){

	if( !!(CAN1->BTR & (1 << 30)) == 1 ){	//check if loop back mode is disabled yet

		if( !!(CAN1->MSR & (1 << 0)) == 0 ){	//check if CAN1 is in initialization mode, if not put it in init mode
			CAN1->MCR |= (1 << 0);				//request initialization
			while( !(CAN1->MSR & (1 << 0)) );		//wait for initialization to be done
		}

		CAN1->BTR &= ~(1 << 30);		//disable loop back mode

		CAN1->MCR &= ~(1 << 0);		//request to enter back to normal mode

	}
	else{
		return 0;	//return 0 if loop back mode already disabled
	}

	return 1; 	//loop back mode disabled

}
