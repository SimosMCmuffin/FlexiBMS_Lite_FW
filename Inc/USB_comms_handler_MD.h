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

#ifndef USB_COMMS_HANDLER_MD_H_
#define USB_COMMS_HANDLER_MD_H_

#include "main.h"

typedef enum
{
	packCellCount = 0,
	maxChgCurr,
	termCurr,
	minCellVolt,
	maxCellVolt,
	minChgVolt,
	maxChgVolt,
	minPackVolt,
	maxPackVolt,
	termCellVolt,
	termPackVolt,
	cellBalVolt,
	cellDiffVolt,
	minNTCtemp,
	maxNTCtemp,
	minBMStemp,
	maxBMStemp,
	alwaysBalancing,
	refreshWaitTime,

	ADC_chan_gain_0,
	ADC_chan_offset_0,
	ADC_chan_gain_1,
	ADC_chan_offset_1,
	ADC_chan_gain_2,
	ADC_chan_offset_2,
	ADC_chan_gain_3,
	ADC_chan_offset_3,
	ADC_chan_gain_4,
	ADC_chan_offset_4,
	extNTCbetaValue,
	AdcOversampling,

	stayActiveTime,
	keep5ValwaysOn,

	balTempRatio,
	storageCellVoltage,
	timeToStorageDischarge,
	canActivityTick,
	canID,

	duringStandby5vOn,
	canRxRefreshActive,
	canWakeUp,
	parallelPackCount,
	currentVoltageRatio,

	restartChargTime,

	numberOfElements
}_parameter_ID;

typedef enum
{
	error_invalidMessageID = 0,
	error_invalidValue,
	error_invalidCommand
}_error_ID;

void USB_checkForNewMessages(void);

void set_defaults(void);
void set_parameter(float *, _parameter_ID);

void set_ADC_chan_gain(float*, uint8_t);
void set_ADC_chan_offset(float*, uint8_t);
void set_extNTCbetaValue(float*);

void set_stayActiveTime(float*);

void report_state(void);
void report_statePrint(void);
void report_faults(void);
void zero_faults(void);
void report_loadDefaults(void);
void report_firmware(void);
void report_hardware(void);
void report_UID(void);
void report_help(void);
void report_error(_error_ID);
void report_save(uint16_t);
void report_load(uint16_t);
void report_parameters(_parameter_ID, uint8_t);

float readFloat(uint8_t*, uint32_t*, uint8_t);
void appendUint16(uint8_t*, uint16_t, uint16_t*);
void appendUint64(uint8_t*, uint64_t, uint16_t*);
void appendHex32(uint8_t*, uint32_t, uint16_t*);
void appendUID(uint8_t*, uint16_t*);
void appendStringFromMemory(uint8_t*, uint8_t*, uint8_t, uint16_t*);
void appendFloat(uint8_t*, float, uint16_t*);
void appendString(uint8_t*, const uint8_t*, uint16_t*);
void appendParameter(uint8_t*, uint16_t, uint16_t*);
void appendChargingState(uint8_t*, uint16_t, uint16_t*);
void appendChargingEndFlag(uint8_t*, uint16_t, uint16_t*);
void appendFault(uint8_t*, uint16_t, uint16_t*);


#endif /* USB_COMMS_HANDLER_MD_H_ */
