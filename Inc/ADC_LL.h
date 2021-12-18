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

#ifndef ADC_LL_H_
#define ADC_LL_H_

typedef enum
{
	ADCchannel_batteryVoltage = 6,
	ADCchannel_chargerVoltage = 16,
	ADCchannel_currentSense = 8,
	ADCchannel_externalTempSense = 7,
	ADCchannel_mcuInternalTemp = 17
}_ADC_channel;

typedef enum
{
	batteryVoltage = 0,
	chargerVoltage,
	chargeCurrent,
	externalTemp,
	mcuInternalTemp
}_ADC_result;

volatile uint16_t ADC_results[5], ADC_convertedResults[5], ADC_HighMin[3][2], ADC_conversionIndex, ADC_initialized;
volatile uint16_t TS_CAL1, TS_CAL2;
float intTempStep, temp30Cvoltage, HWmult[3];

void ADC_init(void);
void ADC_deInit(void);

void ADC_sampleChannel(uint8_t);
uint16_t ADC_readChannelAvg(uint8_t);

void convertADCresults(void);
uint16_t ADC_startConversion(uint8_t);

void ADC_setupSequence(void);
uint8_t ADC_runSequence(void);

void ADC_start(void);
void ADC_stop(void);
void ADC_clearMaxMin(void);
uint8_t ADC_busyCheck(void);
uint8_t ADC_isStopped(void);


#endif /* ADC_LL_H_ */
