/*
 * USB_comms_handler.h
 *
 *  Created on: 20.7.2019
 *      Author: Simos MCmuffin
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
	tickInterval,

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

	numberOfElements
}_parameter_ID;

typedef enum
{
	error_invalidMessageID = 0,
	error_invalidValue,
	error_invalidCommand
}_error_ID;

void checkForNewMessages(void);

void set_defaults(void);
void set_parameter(float *, _parameter_ID);

void set_ADC_chan_gain(float*, uint8_t);
void set_ADC_chan_offset(float*, uint8_t);
void set_extNTCbetaValue(float*);

void set_stayActiveTime(float*);

void report_state(void);
void report_faults(void);
void report_firmware(void);
void report_help(void);
void report_error(_error_ID);
void report_save(uint16_t);
void report_load(uint16_t);
void report_parameters(_parameter_ID, uint8_t);

float readFloat(uint8_t*, uint32_t*, uint8_t);
void appendUint16(uint8_t*, uint16_t, uint16_t*);
void appendFloat(uint8_t*, float, uint16_t*);
void appendString(uint8_t*, const uint8_t*, uint16_t*);
void appendParameter(uint8_t*, uint16_t, uint16_t*);
void appendFault(uint8_t*, uint16_t, uint16_t*);


#endif /* USB_COMMS_HANDLER_MD_H_ */
