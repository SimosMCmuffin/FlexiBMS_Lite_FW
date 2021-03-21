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

#ifndef CAN_LL_H_
#define CAN_LL_H_

uint16_t CAN_initialized;

void CAN1_init(void);
void CAN1_deInit(void);
uint8_t CAN1_receive(uint32_t*, uint8_t *, uint8_t *);
uint8_t CAN1_transmit(uint32_t, uint8_t *, uint8_t);
uint8_t CAN1_rxAvailable(void);
uint8_t CAN1_sendTxBuffer(void);	//used to send CAN-frames from TX buffer to TX mailbox
uint8_t CAN1_enqueueTxBuffer(uint32_t, uint8_t *, uint8_t);	//used to put CAN-frames into a TX buffer
void CAN1_process_message(void);
void CAN1_setupRxFilters(void);
void CAN1_debugEcho(void);
uint8_t CAN1_enableLoopBackMode(void);
uint8_t CAN1_disableLoopBackMode(void);

#endif /* CAN_LL_H_ */
