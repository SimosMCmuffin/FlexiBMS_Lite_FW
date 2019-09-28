/*
 * ADC.h low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#ifndef ADC_H_
#define ADC_H_

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

volatile uint16_t ADC_results[5], ADC_convertedResults[5], ADC_conversionNumber;
volatile uint16_t TS_CAL1, TS_CAL2;
float intTempStep, temp30Cvoltage;

void ADC_init(void);
void ADC_deInit(void);

void ADC_sampleChannel(uint8_t);
uint16_t ADC_readChannelAvg(uint8_t);

void convertADCresults(void);
uint16_t ADC_startConversion(uint8_t);

void ADC_setupSequence(void);
void ADC_runSequence(void);


#endif /* ADC_H_ */
