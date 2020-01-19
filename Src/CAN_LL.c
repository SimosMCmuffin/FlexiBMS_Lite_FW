/*
 * CAN.c low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#include <stm32l4xx_hal.h>
#include <stdio.h>
#include <string.h>
#include "CAN_LL.h"
#include "commands.h"
#include "config.h"
#include "crc.h"
#include "datatypes.h"

#define CAN_TRANSMIT_MAX_ATTEMPTS 100

static unsigned int rx_buffer_last_id;

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

uint8_t CAN1_attempt_transmit(uint32_t ID, uint8_t * data, uint8_t length){

	if( !!(CAN1->TSR & (7 << 26)) ){		//Check that at least one TX mailbox is empty
		uint8_t TX_empty = (CAN1->TSR & (3 << 24)) >> 24;	//check which TX mailbox is empty

		CAN1->sTxMailBox[TX_empty].TIR = 0;
		CAN1->sTxMailBox[TX_empty].TIR = (ID << 3) | (1 << 2);	//set CAN ID
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

uint8_t CAN1_transmit(uint32_t ID, uint8_t * data, uint8_t length){
	for (uint8_t i = 0; i < CAN_TRANSMIT_MAX_ATTEMPTS; i++) {
		if (CAN1_attempt_transmit(ID, data, length))
			return 1;
	}
	return 0;
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

uint8_t CAN1_receive(uint32_t * ID, uint8_t * data, uint8_t * length){
	//check if mail available in either RX mailbox, if not return 0

	if( (CAN1->RF0R & (3 << 0)) != 0 ){
		if (CAN1->sFIFOMailBox[0].RIR & (1 << 2))
			*ID = CAN1->sFIFOMailBox[0].RIR >> 3;  // extended ID
		else
			*ID = CAN1->sFIFOMailBox[0].RIR >> 21;  // standard ID

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
		if (CAN1->sFIFOMailBox[1].RIR & (1 << 2))
			*ID = CAN1->sFIFOMailBox[1].RIR >> 3;  // extended ID
		else
			*ID = CAN1->sFIFOMailBox[1].RIR >> 21;  // standard ID

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

void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint8_t ind = 0;
		send_buffer[ind++] = CAN_ID;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		CAN1_transmit(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			CAN1_transmit(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			CAN1_transmit(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = CAN_ID;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		CAN1_transmit(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	comm_can_send_buffer(rx_buffer_last_id, data, len, 1);
}


void CAN1_process_message() {
	if (!CAN1_rxAvailable())
		return;

	uint8_t length = 0, data[8] = {0};
	uint32_t id = 0;
	CAN1_receive(&id, data, &length);

	uint8_t controller_id = id & 0xFF;
	CAN_PACKET_ID cmd = id >> 8;

	if (controller_id != CAN_ID || cmd != CAN_PACKET_PROCESS_SHORT_BUFFER)
		return;

	if (length < 3)
		return;

	rx_buffer_last_id = data[0];
	uint8_t commands_send = data[1];

	if (commands_send == 0) {
		commands_process_packet(data + 2, length - 2, send_packet_wrapper);
	}

	// Nothing to do for commands_send==1 (forward) and commands_send==2 (process without responding) for now.
}
