/*
 * ADC.c low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#include "main.h"
#include "math.h"
#include "ADC.h"

extern nonVolParameters nonVolPars;

void ADC_init(void){
	RCC->CCIPR |= (3 << 28);	//ADC input clock SYSCLK
	RCC->AHB2ENR |= (1 << 13);	//enable ADC bus clock

	GPIOA->MODER &= ~( (3 << 2) | (3 << 4) | (3 << 6) );
	GPIOA->MODER |= (3 << 2) | (3 << 4) | (3 << 6);		//pins PA1, PA2, PA3 into analog mode
	GPIOB->MODER &= ~(3 << 2);
	GPIOB->MODER |= (3 << 2);							//pin PB1 to analog mode

	ADC1_COMMON->CCR |= (3 << 16);		//clock -> HCLK/4
	ADC1_COMMON->CCR |= (1 << 23);		//enable internal temperature sensor

	ADC1->CR &= ~(1 << 29);			//exit deep-power-down state
	ADC1->CR |= (1 << 28);			//enable ADC voltage regulator

	for(uint32_t x=0; x<500; x++);		//datasheet specs MAX time of 20 �s for ADC voltage regulator boot time, just wait long enough

	ADC1->CR |= (1 << 31);					//start ADC calibration
	while( !!(ADC1->CR & (1 << 31)) );		//wait for calibration to finish

	for(uint8_t x=0; x<5; x++){			//clear ADC_result registers
		ADC_results[x] = 0;
		ADC_convertedResults[x] = 0;
	}

	//TS_CAL1 & TS_CAL2 are factory calibration ADC results for 30C and 130C respectively
	TS_CAL1 = *(uint16_t*)0x1FFF75A8, TS_CAL2 = *(uint16_t*)0x1FFF75CA;
	temp30Cvoltage = TS_CAL1 * (3000.0 / 4095.0);
	intTempStep = (TS_CAL2 - TS_CAL1) * (3000.0 / 4095.0) / 100;	//calculate the voltage difference for 1C step

	ADC1->CR |= (1 << 0);			//enable ADC1
}

void ADC_deInit(){

	while( !(ADC1->ISR & (1 << 0)) );		//wait if ADRDY flag is not set, indicating ADC is not ready
	ADC1->CR |= (1 << 1);					//set ADDIS (adc disable) bit
	while( !!(ADC1->ISR & (1 << 1)) );		//wait if ADDIS flag is set, indicating ADC is not yet disabled

	ADC1->CR |= (1 << 29);				//enter deep-power-down state, this also disables ADVREGEN bit - ADC voltage regulator

	RCC->AHB2ENR &= ~(1 << 13);			//disable ADC bus clock
	RCC->CCIPR &= ~(3 << 28);			//ADC input clock NONE

	//No need to touch GPIO configurations as analog mode is the preferred mode for low-power

}

void convertADCresults(void){
	const float LSBres = (3300.0 / 4095.0);	//ADC LSB voltage in milliVolts (mV)

	//battery voltage, mV
	ADC_convertedResults[batteryVoltage] =
			( ADC_results[batteryVoltage] * LSBres * (114.7/4.7) * nonVolPars.adcParas.ADC_chan_gain[batteryVoltage] ) + nonVolPars.adcParas.ADC_chan_offset[batteryVoltage];

	//charger voltage, mV
	ADC_convertedResults[chargerVoltage] =
			( ADC_results[chargerVoltage] * LSBres * (117.2/4.7) * nonVolPars.adcParas.ADC_chan_gain[chargerVoltage] ) + nonVolPars.adcParas.ADC_chan_offset[chargerVoltage];

	//current sense, mA
	ADC_convertedResults[chargeCurrent] =
			( ADC_results[chargeCurrent] * LSBres * 4 * nonVolPars.adcParas.ADC_chan_gain[chargeCurrent] ) + nonVolPars.adcParas.ADC_chan_offset[chargeCurrent];

	//external temperature sense, Kelvin
	//
	ADC_convertedResults[externalTemp] =
			(nonVolPars.adcParas.extNTCbetaValue * 298.15) / (nonVolPars.adcParas.extNTCbetaValue + (298.15 * log(( 10000 * ( 4095.0 / ADC_results[externalTemp] - 1) ) / 10000)));

	//internal MCU temperature sense, Kelvin
	ADC_convertedResults[mcuInternalTemp] =
			( ADC_results[mcuInternalTemp] * LSBres / 2.5 * nonVolPars.adcParas.ADC_chan_gain[mcuInternalTemp] ) + nonVolPars.adcParas.ADC_chan_offset[mcuInternalTemp];

}

//DON'T USE!
//Decrepit, old manual locking function for single channel ADC conversion
uint16_t ADC_startConversion(uint8_t ADCchannel){

	ADC1->SQR1 &= ~(0b11111 << 6);		//clear old converted channel
	ADC1->SQR1 |= (ADCchannel << 6);	//write new channel to convert

	//ADC1->ISR |= (1 << 2);		//clear EOC flag
	if( !!(ADC1->ISR & (1 << 0)) ){
		ADC1->CR |= (1 << 2);		//start adc conversion

		while( !(ADC1->ISR & (1 << 2)) );		//wait for EOC flag

		return ADC1->DR;
	}
	else
		return 0;

}

void ADC_setupSequence(void){

	//Enable ADC regular conversion oversampling, 16x oversampling, shift result automatically 4 bits (=divide by 16)
	if( nonVolPars.adcParas.AdcOversampling == 1 ){
		ADC1->CFGR2 = 0;
	}
	else{
		switch(nonVolPars.adcParas.AdcOversampling){
		case 2:
			ADC1->CFGR2 = (1 << 0) | (0 << 2) | (1 << 5);
			break;
		case 4:
			ADC1->CFGR2 = (1 << 0) | (1 << 2) | (2 << 5);
			break;
		case 8:
			ADC1->CFGR2 = (1 << 0) | (2 << 2) | (3 << 5);
			break;
		default:		//default to 16x oversampling
			ADC1->CFGR2 = (1 << 0) | (3 << 2) | (4 << 5);
			break;
		}
	}

	//use discontinuous mode for higher reliability
	ADC1->CFGR |= (1 << 16);

	//setup ADC_channel sampling time for ALL channels to 92.5 ADC clocks
	ADC1->SMPR1 = (5 << 0) | (5 << 3) | (5 << 6) | (5 << 9) | (5 << 12) | (5 << 15) | (5 << 18) | (5 << 21) | (5 << 24) | (5 << 27);
	ADC1->SMPR2 = (5 << 0) | (5 << 3) | (5 << 6) | (5 << 9) | (5 << 12) | (5 << 15) | (5 << 18) | (5 << 21) | (5 << 24);

	//setup regular conversion sequence, 5 conversions, setup ADC_channels
	ADC1->SQR1 = (4 << 0) | (ADCchannel_batteryVoltage << 6) | (ADCchannel_chargerVoltage << 12) | (ADCchannel_currentSense << 18) | (ADCchannel_externalTempSense << 24);
	ADC1->SQR2 = (ADCchannel_mcuInternalTemp << 0);

	//enable interrupts for End-Of-Conversion and End-Of-Sequence
	ADC1->IER = (1 << 2) | (1 << 3);
	ADC1->ISR = (1 << 2) | (1 << 3);	//clear interrupt bits
	NVIC_EnableIRQ(ADC1_IRQn);			//enable ADC1 interrupt

}

void ADC_runSequence(void){

	ADC_conversionNumber = 0;			//zero sequence index
	ADC1->ISR = (1 << 2) | (1 << 3);	//clear interrupt bits

	while( !(ADC1->ISR & (1 << 0)) );		//wait if ADRDY flag is not set, indicating ADC is not ready
	ADC1->CR |= (1 << 2);				//start regular channel conversion/sequence

}

void ADC1_IRQHandler(){

	if( !!(ADC1->ISR & (1 << 2)) ){		//End-Of-Conversion (EOC)
		ADC_results[ADC_conversionNumber] = ADC1->DR;	//save conversion result, this also clear EOC flag by reading the data register
		ADC_conversionNumber++;			//increment sequence index

		if( ADC_conversionNumber == 5 ){
			convertADCresults();			//convert RAW ADC results to actual units
			ADC_runSequence();
		}
		else{
			ADC1->CR |= (1 << 2);			//start next sampling+conversion
		}
	}

	if( !!(ADC1->ISR & (1 << 3)) ){		//End-Of-Sequence (EOS)
		ADC1->ISR = (1 << 3);			//clear interrupt flag
		//convertADCresults();			//convert RAW ADC results to actual units
	}

}
