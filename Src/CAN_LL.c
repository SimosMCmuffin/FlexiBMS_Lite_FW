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
#include <main.h>

#define CAN_TRANSMIT_MAX_ATTEMPTS 100

#define RX_BUFFER_SIZE 512
static uint8_t rx_buffer[RX_BUFFER_SIZE];

#define TX_BUFFER_SIZE 16
struct txBuffer_struct{
	uint32_t ID;
	uint8_t data[8];
	uint8_t len;
};
static struct txBuffer_struct txBuffer[TX_BUFFER_SIZE];
static uint8_t queueIndex = 0, txIndex = 0, ISR_running = 0;

extern nonVolParameters nonVolPars;
extern runtimeParameters runtimePars;

void CAN1_init(){

	if( CAN_initialized == 0){					//check that CAN is not initialized
		RCC->APB1ENR1 |= (1 << 25);				//Enable CAN1 bus clock

		GPIOA->MODER = (GPIOA->MODER & ~(3 << 20)) | (1 << 20);		//Configure pin PA10 as OUTPUT for CAN silent mode control
		GPIOA->BSRR |= (1 << 26);									//don't Put CAN into silent mode (refer to TJA1051 datasheet for functional description)

		GPIOB->MODER &= ~( (3 << 16) | (3 << 18) );			//Configure pins PB8 and PB9 alternative functions
		GPIOB->MODER |= (2 << 16) | (2 << 18);
		GPIOB->AFR[1] |= (9 << 0) | (9 << 4);				//set alternative function to AF9(CAN1)

		CAN1->MCR |= (1 << 0);				//request initialization
		while( !!(CAN1->MSR & (1 << 0)) == 0 );		//wait to enter initialization mode

		CAN1->MCR |= (1 << 6) | (1 << 4) | (1 << 2);		//automatic bus-off management, *no automatic retransmission*, TX order by chronologically

		CAN1->IER |= (1 << 0) | (1 << 1);		//enable TX interrupt, RX mailbox 0 interrupt

		CAN1->BTR = 0;
		CAN1->BTR |= (3 << 24) | (2 << 20) | (11 << 16) | (1 << 0);		//Set timings for 500kHz CAN baud with 16MHz source clock, 75%
		CAN1->MCR &= ~(1 << 0);				//request to enter normal mode

		CAN1->MCR &= ~(1 << 1);				//exit sleep mode
		while( !!(CAN1->MSR & (1 << 1)) == 1 );			//wait for CAN1 to exit sleep mode

		NVIC_EnableIRQ(CAN1_TX_IRQn);		//enable CAN1 TX interrupt
		NVIC_EnableIRQ(CAN1_RX0_IRQn);		//enable CAN1 RX mailbox 0 interrupt

		CAN_initialized = 1;		//mark CAN as initialized
	}

}

void CAN1_deInit(){

	if( CAN_initialized == 1 ){			//check that CAN is initialized
		CAN1->MCR |= (1 << 1);		//request to enter sleep mode
		while( !!(CAN1->MSR & (1 << 1)) == 0 || ISR_running == 1 );		//wait for bus activity to finish/stop and enter sleep mode
		RCC->APB1ENR1 &= ~(1 << 25);			//Disable CAN1 bus clock

		GPIOB->MODER &= ~( (3 << 16) | (3 << 18) );			//Configure pins PB8 and PB9 to analog state for low-power usage
		GPIOB->MODER |= (3 << 16) | (3 << 18);

		CAN_initialized = 0;		//mark CAN as non-initialized
	}

}

uint8_t CAN1_attempt_transmit(uint32_t ID, uint8_t * data, uint8_t length){

	if( !!(CAN1->TSR & (1 << 26)) && CAN_initialized == 1 ){		//Check that TX mailbox 0 is empty and that CAN is iniatilized
		uint8_t TX_empty = 0;//(CAN1->TSR & (3 << 24)) >> 24;	//check which TX mailbox is empty

		CAN1->sTxMailBox[TX_empty].TIR = 0;
		CAN1->sTxMailBox[TX_empty].TIR = (ID << 3) | (1 << 2);	//set CAN ID, extended frame

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
		if (CAN1_enqueueTxBuffer(ID, data, length))
			return 1;
	}
	return 0;
}

//used to send CAN-frames from TX buffer to TX mailbox
uint8_t CAN1_sendTxBuffer(void){
	//check that there's available frames waiting in the tx-buffer
	if( queueIndex == txIndex ){
		CAN1->TSR |= (1 << 0);	//clear the TX mailbox 0 empty flag, as there isn't anything to send at the moment. Otherwise gets stuck in the ISR.
		ISR_running = 0;
		return 0;
	}

	//attempt to enter a new CAN frame to the TX mailbox
	if(	CAN1_attempt_transmit(	txBuffer[txIndex].ID,
								txBuffer[txIndex].data,
								txBuffer[txIndex].len) == 0){
		return 0;
	}
	else
		ISR_running = 1;

	txIndex++;
	//check that queueIndex stays in range
	if( txIndex == TX_BUFFER_SIZE )
		txIndex = 0;

	return 1;
}

//used to put CAN-frames into a TX buffer
uint8_t CAN1_enqueueTxBuffer(uint32_t ID, uint8_t * data, uint8_t length){
	//first check that were not rolling over the queue index on unsent frames
	if( ((queueIndex+1)%TX_BUFFER_SIZE) == txIndex )
		return 0;	//buffer full

	//insert data to the buffer
	txBuffer[queueIndex].ID = ID;
	memcpy(txBuffer[queueIndex].data, data, 8);
	txBuffer[queueIndex].len = length;

	queueIndex++;
	//check that queueIndex stays in range
	if( queueIndex == TX_BUFFER_SIZE )
		queueIndex = 0;

	//if TX mailbox 0 is empty, then most likely the TX ISR is not running, prime it by putting the first frame into a mailbox from outside the ISR.
	//Once the first mailbox empties it'll trigger the TX ISR where we can check if there are more waiting TX frames and send them all
	//from the ISR, meanwhile we can queue more frames for it to send out on it's own
	NVIC_DisableIRQ(CAN1_TX_IRQn);		//disable CAN1 TX interrupt
	if( ISR_running == 0 ){
		CAN1_sendTxBuffer();
	}
	NVIC_EnableIRQ(CAN1_TX_IRQn);		//enable CAN1 TX interrupt

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

	//CAN1->sFilterRegister[0].FR1 = 0;	//ID 0
	CAN1->sFilterRegister[0].FR1 = 0 | (nonVolPars.genParas.canID << 3) | (1 << 2);	//ID [7:0] = canID and must use extended CAN ID
	//CAN1->sFilterRegister[0].FR2 = 0;	//set ALL mask bits to DO_NOT_CARE, meaning all messages will be read into RX FIFO
	CAN1->sFilterRegister[0].FR2 = 0 | (0xFF << 3) | (1 << 2);	//set mask bits as "care" for first 8 bits of extended ID and that the frame is using extended ID

	CAN1->FMR = 0;		//filter active mode
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


void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint8_t ind = 0;
		send_buffer[ind++] = nonVolPars.genParas.canID;
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
		send_buffer[ind++] = nonVolPars.genParas.canID;
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

static void send_packet_wrapper(uint8_t to, uint8_t *data, unsigned int len) {
	comm_can_send_buffer(to, data, len, 1);
}


void CAN1_process_message() {
	//CAN1_sendTxBuffer();	//check if there is available CAN-frames in the tx-buffer and see if there are available TX mailboxes

	if (!CAN1_rxAvailable())
		return;

	uint8_t length = 0, data[8] = {0};
	uint32_t id = 0;
	CAN1_receive(&id, data, &length);

	uint8_t controller_id = id & 0xFF;
	CAN_PACKET_ID cmd = id >> 8;

	if (controller_id != nonVolPars.genParas.canID)
		return;

	int ind = 0;
	uint8_t from = 0;
	uint8_t* packet_data = NULL;
	unsigned int packet_length = 0;
	uint8_t commands_send = 0;
	switch (cmd) {
	case CAN_PACKET_PROCESS_SHORT_BUFFER:
		from = data[ind++];
		commands_send = data[ind++];
		packet_data = data + ind;
		packet_length = length - ind;
		break;
	case CAN_PACKET_FILL_RX_BUFFER:
		memcpy(rx_buffer + data[0], data + 1, length - 1);
		return;
	case CAN_PACKET_FILL_RX_BUFFER_LONG:
		{
			unsigned int rxbuf_ind = (unsigned int)data[0] << 8;
			rxbuf_ind |= data[1];
			if (rxbuf_ind < RX_BUFFER_SIZE) {
				memcpy(rx_buffer + rxbuf_ind, data + 2, length - 2);
			}
		}
		return;
	case CAN_PACKET_PROCESS_RX_BUFFER:
		from = data[ind++];
		commands_send = data[ind++];
		unsigned int rxbuf_len = (unsigned int)data[ind++] << 8;
		rxbuf_len |= (unsigned int)data[ind++];
		if (rxbuf_len > RX_BUFFER_SIZE) {
			return;
		}
		uint8_t crc_high = data[ind++];
		uint8_t crc_low = data[ind++];

		if (crc16(rx_buffer, rxbuf_len)
				!= ((unsigned short) crc_high << 8
						| (unsigned short) crc_low)) {
			return;
		}
		packet_data = rx_buffer;
		packet_length = rxbuf_len;
		break;
	default:
		return;
	}

	if (commands_send == 0) {
		commands_process_packet(from, packet_data, packet_length, send_packet_wrapper);
	}
	// Nothing to do for commands_send==1 (forward) and commands_send==2 (process without responding) for now.
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

//CAN1 TX ISR
void CAN1_TX_IRQHandler(void){
	static uint8_t retryAttempts = 0;

	if( !!(CAN1->TSR & (1 << 0)) == 1 ){	//TX mailbox 0 transmit request complete

		if( !!(CAN1->TSR & (1 << 1)) == 0 ){	//check if the mailbox message was transmitted successfully and retry is failed
			retryAttempts++;
			if(retryAttempts < 10){				//only allow a fixed amount of re-transmit attempts, as to not get stuck
				CAN1->sTxMailBox[0].TIR |= (1 << 0);		//request new TX attempt with old mailbox content, aka retry
				return;
			}
		}

		retryAttempts = 0;
		CAN1_sendTxBuffer();

	}

}

//CAN1 RX mailbox 0 ISR
void CAN1_RX0_IRQHandler(void){

	if( !!(CAN1->RF0R & (3 << 0)) == 1 ){	//RX mailbox 0 not empty

		if( nonVolPars.genParas.canActivityTick == 1 ){	//CAN activity cyan flick
			__GREEN_LED_ON;
			__RED_LED_OFF;
			__BLUE_LED_ON;
		}

		CAN1_process_message();

		//CAN RX activeTimer refresh
		if( nonVolPars.genParas.canRxRefreshActive != 0 ){	//check if parameter is enabled, before going into a bigger if-statement
			if( (runtimePars.activeTick - (nonVolPars.genParas.canRxRefreshActive * __TIME_HOUR_TICKS)) <= HAL_GetTick() ){
				runtimePars.activeTick = HAL_GetTick() + (nonVolPars.genParas.canRxRefreshActive * __TIME_HOUR_TICKS);
			}
		}

	}

}
