/*
 * LTC6803_3.c low level peripheral driver for STM32L433 for in use with FlexiBMS 0.2
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#include "stm32l4xx_hal.h"
#include <LTC6803_3_DD.h>
#include <SPI_LL.h>
#include <main.h>

extern runtimeParameters runtimePars;

void LTC6803_init(void){
	LTC_data.CFGR[0] = 0xE1;
	for(uint8_t x=1; x<6; x++){
		LTC_data.CFGR[x] = 0;
	}
	for(uint8_t x=0; x<18; x++){
		LTC_data.Cells[x] = 0;
	}
	for(uint8_t x=0; x<12; x++){
		LTC_data.cVoltages[x] = 0;
	}

	LTC_data.DIAG[0] = 0;
	LTC_data.DIAG[1] = 0;
	LTC_data.PEC = 0x41;
}

void LTC6803_writeConfiguration(void){
	SPI_message = 1;
	SPI_index = 0;

	CLR_CS_PIN;

	LTC_data.PEC = 0x41;
	LTC_data.PEC = calculatePEC(LTC_data.PEC, LTC_data.CFGR[0]);
	LTC_data.PEC = calculatePEC(LTC_data.PEC, LTC_data.CFGR[1]);
	LTC_data.PEC = calculatePEC(LTC_data.PEC, LTC_data.CFGR[2]);
	LTC_data.PEC = calculatePEC(LTC_data.PEC, LTC_data.CFGR[3]);
	LTC_data.PEC = calculatePEC(LTC_data.PEC, LTC_data.CFGR[4]);
	LTC_data.PEC = calculatePEC(LTC_data.PEC, LTC_data.CFGR[5]);

	*(uint8_t *)&SPI1->DR = 0x01;

}

void LTC6803_startCellMeasurements(void){
	SPI_message = 2;
	SPI_index = 0;

	CLR_CS_PIN;

	*(uint8_t *)&SPI1->DR = 0x10;
}

void LTC6803_startOpenCellMeasurements(void){
	SPI_message = 3;
	SPI_index = 0;

	CLR_CS_PIN;

	*(uint8_t *)&SPI1->DR = 0x20;
}

void LTC6803_startInternalTemperatureMeas(void){
	SPI_message = 5;
	SPI_index = 0;

	CLR_CS_PIN;

	*(uint8_t *)&SPI1->DR = 0x33;
}

void LTC6803_readCellMeasurements(void){		//Start SPI communication to retrieve measurement results for cell voltages
	SPI_message = 4;
	SPI_index = 0;

	CLR_CS_PIN;

	*(uint8_t *)&SPI1->DR = 0x04;

}

void LTC6803_readInternalTemperature(void){		//Start SPI communication to retrieve measurement result for ltc6803 internal temperature sensor
	SPI_message = 6;
	SPI_index = 0;

	CLR_CS_PIN;

	*(uint8_t *)&SPI1->DR = 0x0E;

}

void LTC6803_convertCellVoltages(void){		//convert raw normal cell measurement results from ltc6803 to milliVolts
	for(uint8_t x=0; x<6; x++){				//un-pack the conversion results which are 12-bit long
		LTC_data.cVoltages[(x*2)] = LTC_data.Cells[(x*3)] | (LTC_data.Cells[(x*3)+1] << 8);
		LTC_data.cVoltages[(x*2)] &= 0x0FFF;
		LTC_data.cVoltages[(x*2)+1] = (LTC_data.Cells[(x*3)+2] << 4) | (LTC_data.Cells[(x*3)+1] >> 4);
		LTC_data.cVoltages[(x*2)+1] &= 0x0FFF;
	}

	for(uint8_t x=0; x<12; x++){			//convert results into millivolts. Aka 3,856V = 3856mV
		if(LTC_data.cVoltages[x] > 512)
			LTC_data.cVoltages[x] -= 512;
		else
			LTC_data.cVoltages[x] = 0;

		LTC_data.cVoltages[x] *= 1.5;
	}
}

void LTC6803_convertOpenCellVoltages(void){	//convert raw open cell measurement results from ltc6803 to milliVolts
	for(uint8_t x=0; x<6; x++){				//un-pack the conversion results which are 12-bit long
		LTC_data.oVoltages[(x*2)] = LTC_data.Cells[(x*3)] | (LTC_data.Cells[(x*3)+1] << 8);
		LTC_data.oVoltages[(x*2)] &= 0x0FFF;
		LTC_data.oVoltages[(x*2)+1] = (LTC_data.Cells[(x*3)+2] << 4) | (LTC_data.Cells[(x*3)+1] >> 4);
		LTC_data.oVoltages[(x*2)+1] &= 0x0FFF;
	}

	for(uint8_t x=0; x<12; x++){			//convert results into millivolts. Aka 3,856V = 3856mV
		if(LTC_data.oVoltages[x] > 512)
			LTC_data.oVoltages[x] -= 512;
		else
			LTC_data.oVoltages[x] = 0;

		LTC_data.oVoltages[x] *= 1.5;
	}
}

void LTC6803_convertTemperatureVoltages(void){	//convert raw internal temperature measurement results from ltc6803 to Kelvins
	LTC_data.internalTemperature = (( LTC_data.temperatureVoltages[3] | ((LTC_data.temperatureVoltages[4] & 0x0F) << 8) ) - 512 ) * 1.5 / 8;
}


void LTC6803_setCellDischarge(uint8_t cell, uint8_t state){		//toggle bleed resistors on or off for balancing
	if(cell < 8){
		if(state != 1){
			LTC_data.CFGR[1] &= ~(1 << cell);
		}
		else{
			LTC_data.CFGR[1] |= (1 << cell);
		}
	}
	else if(cell < 12){
		if(state != 1){
			LTC_data.CFGR[2] &= ~(1 << (cell-8));
		}
		else{
			LTC_data.CFGR[2] |= (1 << (cell-8));
		}
	}
}

uint16_t LTC6803_getCellVoltage(uint8_t cell){				//return specific cell voltage

	return LTC_data.cVoltages[cell];
}

uint16_t LTC6803_getTemperature(void){
	return LTC_data.internalTemperature;
}

uint8_t calculatePEC(uint8_t PEC, uint8_t incoming){		//calculate PEC (Packet Error Correction) function
	uint8_t IN0, IN1, IN2, DIN;

	for(uint8_t x=0; x<8; x++){
		DIN = !!(incoming & (0x80 >> x));
		IN0 = !!(PEC & (1 << 7)) ^ DIN;
		IN1 = !!(PEC & (1 << 0)) ^ IN0;
		IN2 = !!(PEC & (1 << 1)) ^ IN0;

		IN1 = IN1 << 1;
		IN2 = IN2 << 2;

		PEC = PEC << 1;
		PEC &= ~0b00000111;
		PEC |= IN0 | IN1 | IN2;
	}
	return PEC;
}

void LTC6803_runEnable(void){
	runtimePars.LTC6803runState = 1;
}

void LTC6803_runDisable(void){
	runtimePars.LTC6803runState = 0;
}

void LTC6803_transactionHandler(uint64_t* LTC6803tick){
	static uint64_t LTC6803resetTick = 0;
	static uint8_t LTC6803commsState = 0;

	if( LTC6803resetTick == 0)
		LTC6803resetTick = HAL_GetTick();

	if( SPI_message == 0 && runtimePars.LTC6803runState == 1 ){
		LTC6803resetTick = HAL_GetTick();

		switch( LTC6803commsState ){

		case 0:
			LTC6803_convertTemperatureVoltages();
			LTC6803_writeConfiguration();
			LTC6803commsState = 1;
			break;

		case 1:
			LTC6803_startCellMeasurements();
			*LTC6803tick = HAL_GetTick() + 20;
			LTC6803commsState = 2;
			break;

		case 2:
			LTC6803_readCellMeasurements();
			LTC6803commsState = 3;
			break;

		case 3:
			LTC6803_convertCellVoltages();
			LTC6803_startOpenCellMeasurements();
			*LTC6803tick = HAL_GetTick() + 20;
			LTC6803commsState = 4;
			break;

		case 4:
			LTC6803_readCellMeasurements();
			LTC6803commsState = 5;
			break;

		case 5:
			LTC6803_convertOpenCellVoltages();
			LTC6803_startInternalTemperatureMeas();
			LTC6803commsState = 6;
			break;

		case 6:
			LTC6803_readInternalTemperature();
			LTC6803commsState = 0;
			break;

		default:
			break;
		}
	}

}

void SPI1_IRQHandler(){

	if( !!(SPI1->SR & (1 << 0)) ){
		uint8_t data = SPI1->DR;

		switch(SPI_message){
		case 1:								//write configuration registers
			if( SPI_index == 8 ){
				SET_CS_PIN;
				SPI_message = 0;
				break;
			}
			if( SPI_index == 7 ){
				*(uint8_t *)&SPI1->DR = LTC_data.PEC;
				SPI_index++;
				break;
			}
			if( SPI_index >= 1 ){
				*(uint8_t *)&SPI1->DR = LTC_data.CFGR[SPI_index-1];
				SPI_index++;
				break;
			}
			*(uint8_t *)&SPI1->DR = 0xC7;
			SPI_index++;
			break;

		case 2:								//start normal cell measurement
			if( SPI_index == 1 ){
				SET_CS_PIN;
				SPI_message = 0;
				break;
			}
			*(uint8_t *)&SPI1->DR = 0xB0;
			SPI_index++;
			break;

		case 3:								//start open cell measurement
			if( SPI_index == 1 ){
				SET_CS_PIN;
				SPI_message = 0;
				break;
			}
			*(uint8_t *)&SPI1->DR = 0x20;
			SPI_index++;
			break;

		case 4:								//read cell voltage measurement results
			if( SPI_index == 20 ){
				LTC_data.PEC = data;
				SET_CS_PIN;
				SPI_message = 0;
				break;
			}
			if( SPI_index >= 2 ){
				LTC_data.Cells[SPI_index-2] = data;
				*(uint8_t *)&SPI1->DR = 0xFF;
				SPI_index++;
				break;
			}
			if(SPI_index == 1){
				*(uint8_t *)&SPI1->DR = 0xFF;
				SPI_index++;
				break;
			}
			*(uint8_t *)&SPI1->DR = 0xDC;
			SPI_index++;
			break;

		case 5:								//start LTC6803 internal temperature sensor measurement
			if( SPI_index == 1 ){
				SET_CS_PIN;
				SPI_message = 0;
				break;
			}
			*(uint8_t *)&SPI1->DR = 0x59;
			SPI_index++;
			break;

		case 6:								//read temperature measurement results
			if( SPI_index == 7 ){
				LTC_data.PEC = data;
				SET_CS_PIN;
				SPI_message = 0;
				break;
			}
			if( SPI_index >= 2 ){
				LTC_data.temperatureVoltages[SPI_index-2] = data;
				*(uint8_t *)&SPI1->DR = 0xFF;
				SPI_index++;
				break;
			}
			if(SPI_index == 1){
				*(uint8_t *)&SPI1->DR = 0xFF;
				SPI_index++;
				break;
			}
			*(uint8_t *)&SPI1->DR = 0xEA;
			SPI_index++;
			break;

		default: break;
		}
	}

}
