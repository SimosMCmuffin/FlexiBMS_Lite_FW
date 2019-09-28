/*
 * AuxFunctions.c
 *
 *  Created on: 27.5.2019
 *      Author: Simos MCmuffin
 */

#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "AuxFunctions.h"
#include "LTC6803_3.h"
#include "ADC.h"
#include "main.h"
#include "dStorage.h"

extern USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev);
extern nonVolParameters nonVolPars;
extern runtimeParameters runtimePars;

void initNonVolatiles(nonVolParameters* nonVols, uint8_t loadDefaults){

	if( loadNonVolatileParameters(nonVols) == 0 || loadDefaults == 1){	//if no parameters stored in FLASH, load defaults

		for(uint8_t x=0; x<5; x++){
			nonVols->adcParas.ADC_chan_gain[x] = 1.0;
			nonVols->adcParas.ADC_chan_offset[x] = 0;
		}
		nonVols->adcParas.extNTCbetaValue = 3380;
		nonVols->adcParas.AdcOversampling = 16;

		nonVols->genParas.stayActiveTime = 1000;

		nonVols->chgParas.packCellCount = 12;
		nonVols->chgParas.cellBalVolt = 4150;
		nonVols->chgParas.cellDiffVolt = 10;
		nonVols->chgParas.termCellVolt = 4180;
		nonVols->chgParas.termPackVolt = 50400;
		nonVols->chgParas.maxChgCurr = 6500;
		nonVols->chgParas.termCurr = 300;

		nonVols->chgParas.minCellVolt = 2000;
		nonVols->chgParas.maxCellVolt = 4250;
		nonVols->chgParas.minChgVolt= 10000;
		nonVols->chgParas.maxChgVolt = 55000;
		nonVols->chgParas.minPackVolt= 7000;
		nonVols->chgParas.maxPackVolt= 52000;

		nonVols->chgParas.maxBMStemp = 330;
		nonVols->chgParas.minBMStemp = 260;

		nonVols->chgParas.maxNTCtemp = 0;
		nonVols->chgParas.minNTCtemp = 0;

		nonVols->chgParas.tickInterval = 5000;

	}

}

void initRuntimeParameters(runtimeParameters* runtimePars){
	runtimePars->ADCrunState = 0;
	runtimePars->statusLive = 1;
	runtimePars->statusTick = 1000;
	runtimePars->usbConnected = 0;
	runtimePars->chargingState = 0;
	runtimePars->faults = 0;
	runtimePars->buck5vEnabled = 0;
	runtimePars->charging = 0;
	runtimePars->balancing = 0;
}

uint8_t countCells(void){			//Count how many cells are above
	uint8_t cellCount = 0;

	for(uint8_t x=0; x<12; x++){
		if( LTC6803_getCellVoltage(x) >= nonVolPars.chgParas.minCellVolt && LTC6803_getCellVoltage(x) >= nonVolPars.chgParas.maxCellVolt )
			cellCount++;
	}

	return cellCount;
}

uint16_t highestCell(uint8_t cellCount){
	uint16_t temp = 0;

	for(uint8_t x=0; x<cellCount; x++){
		if(LTC6803_getCellVoltage(x) > temp)
			temp = LTC6803_getCellVoltage(x);
	}

	return temp;
}
uint16_t lowestCell(uint8_t cellCount){
	uint16_t temp = 60000;

	for(uint8_t x=0; x<cellCount; x++){
		if(LTC6803_getCellVoltage(x) < temp)
			temp = LTC6803_getCellVoltage(x);
	}

	return temp;
}

uint8_t usbPowerPresent(void){		//Start and stop USB service depending on if 5V is detected on the USB connector, indicating a connected cable
	static uint8_t usb_state = 0;

	if( !!(GPIOB->IDR & (1 << 7)) && usb_state == 0 ){	//if USB_Detect signal is high (USB 5V present), return 1
		MX_USB_DEVICE_Init();
		runtimePars.usbConnected = 1;
		usb_state = 1;
		//__BLUE_LED_ON;
	}
	else if( !(GPIOB->IDR & (1 << 7)) && usb_state == 1 ){
		USBD_DeInit(&hUsbDeviceFS);		//Stop usb service
		runtimePars.usbConnected = 0;
		usb_state = 0;
		//__BLUE_LED_OFF;
	}

	return usb_state;
}


/**
 * @brief  chargeControl : used to switch and monitor the charging
 * @param  allowCharging : 0 - stop charging, 1 - start/continue charging if possible
 * @retval 0 - not charging, 1 - fault, 2 - Charging
 */
uint8_t chargeControl(){
	static uint8_t chargingState = 0;
	static uint64_t chargeTick = 0;

	__ENABLE_CHARGER_VOLTAGE;
	//check if charger detected
	if( ADC_convertedResults[chargerVoltage] > nonVolPars.chgParas.minChgVolt ){
		__ENABLE_5V_BUCK;

		checkChargerVoltageFault();
		checkBMStemperatureFault();
		checkNTCtemperatureFault();
		checkCellVoltageFaults();

		if( nonVolPars.chgParas.packCellCount == 0 ){
			checkPackVoltageFault();
			__ENABLE_BAT_VOLTAGE;
		}

		if( runtimePars.faults == 0 && chargingState == 0){
			//allow charging to start
			chargingState = 1;
		}

	}

	if( chargingState != 0 && runtimePars.faults != 0 ){
		chargingState = 0;
	}

	//balancing
	if( chargingState != 0 && nonVolPars.chgParas.packCellCount != 0 ){	//allow balancing if chargingState != 0

		runtimePars.balancing = 0;
		for(uint8_t x=0; x < nonVolPars.chgParas.packCellCount; x++){
			if( 	(LTC6803_getCellVoltage(x) >= nonVolPars.chgParas.cellBalVolt) &&						//if cell voltage above balance voltage
					(LTC6803_getCellVoltage(x) > (lowestCell(nonVolPars.chgParas.packCellCount) + nonVolPars.chgParas.cellDiffVolt)) ){	//and cell difference greater than allowed compared to the lowest cell
				LTC6803_setCellDischarge(x, 1);
				runtimePars.balancing = 1;
			}
			else
				LTC6803_setCellDischarge(x, 0);
		}

	}
	else{												//if chargingState == 0, stop balancing
		for(uint8_t x=0; x<12; x++){
			LTC6803_setCellDischarge(x, 0);
		}
	}


	switch(chargingState){
	case 0:
		__DISABLE_5V_BUCK;
		__DISABLE_BAT_VOLTAGE;
		__DISABLE_CHG;
		runtimePars.balancing = 0;
		runtimePars.charging = 0;
		break;

	case 1:
		//start charging, monitor current and voltages real-time for a couple hundred milliseconds
		//before enabling charge path, check that cell and pack voltages below termination voltage
		if( nonVolPars.chgParas.packCellCount != 0 ){
			for( uint8_t x=0; x<nonVolPars.chgParas.packCellCount; x++){
				if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.termCellVolt ){	//if cell voltage above termination voltage, jump directly end/wait state
					chargingState = 3;
					break;
				}
			}
		}
		else{
			if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.termPackVolt ){
				chargingState = 3;
				break;
			}
		}
		chargeTick = HAL_GetTick() + 250;
		__ENABLE_CHG;
		runtimePars.charging = 1;

		chargingState = 2;
		break;

	case 2:
		if( chargeTick <= HAL_GetTick() ){

			if( 	ADC_convertedResults[chargeCurrent] > nonVolPars.chgParas.maxChgCurr ||
					ADC_convertedResults[chargeCurrent] < nonVolPars.chgParas.termCurr ){	//if charging current too high or below termination current, stop
				chargingState = 3;
				__DISABLE_CHG;
				break;
			}

			if( nonVolPars.chgParas.packCellCount != 0 ){
				for( uint8_t x=0; x<nonVolPars.chgParas.packCellCount; x++){
					if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.termCellVolt ){	//if cell voltage above termination voltage, jump directly end/wait state
						chargingState = 3;
						__DISABLE_CHG;
						break;
					}
				}
			}
			else{
				if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.termPackVolt ){
					chargingState = 3;
					__DISABLE_CHG;
					break;
				}
			}

		}
		else{

			if( nonVolPars.chgParas.packCellCount != 0 ){
				for( uint8_t x=0; x<nonVolPars.chgParas.packCellCount; x++){
					if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.termCellVolt ){	//if cell voltage above termination voltage, jump directly end/wait state
						chargingState = 3;
						break;
					}
				}
			}
			else{
				if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.termPackVolt ){
					chargingState = 3;
					break;
				}
			}

		}

		break;

	case 3:
		//end or pause charging,
		__DISABLE_CHG;
		runtimePars.charging = 0;
		if( ADC_convertedResults[chargerVoltage] < nonVolPars.chgParas.minChgVolt ){
			__DISABLE_BAT_VOLTAGE;
			__DISABLE_5V_BUCK;
			chargeTick = HAL_GetTick() + 250;
			chargingState = 4;
		}
		break;

	case 4:
		if( chargeTick <= HAL_GetTick() )
			chargingState = 0;
		break;

	default:
		break;
	}


	/*
	switch(0){
	case 0:						//stop charging, shut off Vsenses and turn off MOSFET bridge
		__DISABLE_CHG;
		__DISABLE_BAT_VOLTAGE;
		chargingState = 0;
		chargingStatus = 0;
		break;

	case 1:

		switch( chargingState ){
		case 0:							//Enable 5V buck and battery Vsense
			if( waitTime != 0 )
				waitTime--;
			else if( ADC_convertedResults[chargerVoltage] >= 5000 ){
				if( nonVolPars.chgParas.packCellCount == countCells() ){	//if detect as many cells as configured
					__ENABLE_5V_BUCK;
					__ENABLE_BAT_VOLTAGE;
					chargingState = 1;
					chargingStatus = 2;
					waitTime = 100;
				}
			}

			break;

		case 1:
			if( waitTime != 0 )
				waitTime--;
			else if( (ADC_convertedResults[batteryVoltage] >= (nonVolPars.chgParas.packCellCount * 2500)) ){	//if battery voltage higher than 2,5V average cell voltage
				__ENABLE_CHG;
				chargingState = 2;
				waitTime = 20;
			}

			break;

		case 2:
			if( waitTime != 0 )
				waitTime--;
			else if( (ADC_convertedResults[chargeCurrent] >= nonVolPars.chgParas.termCurr) &&		//if current not below termination current
					(ADC_convertedResults[chargeCurrent] <= nonVolPars.chgParas.maxChgCurr) &&		//and below max charging current
					(highestCell(nonVolPars.chgParas.packCellCount) <= nonVolPars.chgParas.termCellVolt) ){	//and highest cell not above termination cell voltage

				uint8_t balancing = 0;
				for(uint8_t x=0; x<nonVolPars.chgParas.packCellCount; x++){
					if( 	(LTC6803_getCellVoltage(x) >= nonVolPars.chgParas.cellBalVolt) &&						//if cell voltage at allowed balance voltage
							(LTC6803_getCellVoltage(x) > (lowestCell(nonVolPars.chgParas.packCellCount) + nonVolPars.chgParas.cellDiffVolt)) ){	//and cell difference greater than allowed compared to the lowest cell
						LTC6803_setCellDischarge(x, 1);
						balancing++;
					}
					else
						LTC6803_setCellDischarge(x, 0);
				}

				if( balancing > 0 )
					chargingStatus = 3;
				else
					chargingStatus = 2;

			}
			else{
				__DISABLE_CHG;
				chargingState = 3;
				chargingStatus = 0;
			}

			break;

		case 3:
			if( ADC_convertedResults[chargerVoltage] <= 500 ){
				__DISABLE_BAT_VOLTAGE;
				chargingState = 0;
				chargingStatus = 0;
				waitTime = 50;
			}

			break;

		default:
			break;
		}

		break;

		default:
			break;
	}
	 */

	//status led control
//	switch( chargingStatus ){
//	case 0:		//no charging going on
//		__GREEN_LED_OFF;
//		__RED_LED_OFF;
//		break;
//	case 1:		//error
//		__GREEN_LED_OFF;
//		__RED_LED_ON;
//		break;
//	case 2:		//charging
//		__GREEN_LED_ON;
//		__RED_LED_OFF;
//		break;
//	case 3:		//balancing and possibly charging
//		__GREEN_LED_ON;
//		__RED_LED_ON;
//		break;
//	default:
//		break;
//	}

	runtimePars.chargingState = chargingState;

	return chargingState;
}

void statusLed(void){
	static uint64_t statusLedTick = 0;
	static uint8_t state = 0;

	if( statusLedTick < HAL_GetTick() ){
		statusLedTick = HAL_GetTick() + 1000;

		if( runtimePars.usbConnected == 1 && (runtimePars.charging || runtimePars.balancing) ){
			if( state == 0 )
				state = 1;
			else
				state = 0;
		}
		else{
			if( runtimePars.usbConnected == 1 ){
				state = 0;
			}
			else{
				state = 1;
			}
		}
	}

	if( state == 0 ){
		__RED_LED_OFF;
		__GREEN_LED_OFF;
		if( runtimePars.usbConnected == 1 ){
			__BLUE_LED_ON;
		}
		else{
			__BLUE_LED_OFF;
		}
	}
	else{
		__BLUE_LED_OFF;
		if( runtimePars.balancing == 1 ){
			__GREEN_LED_ON;
			__RED_LED_ON;
		}
		else if( runtimePars.charging == 1 ){
			__GREEN_LED_ON;
			__RED_LED_OFF;
		}
		else{
			__GREEN_LED_OFF;
			__RED_LED_OFF;
		}


	}

}

void checkChargerVoltageFault(){

	//if charger voltage over max
	if( ADC_convertedResults[chargerVoltage] > nonVolPars.chgParas.maxChgVolt ){
		runtimePars.faults |= (1 << fault_highChargerVoltage);
	}
	else{
		runtimePars.faults &= ~(1 << fault_highChargerVoltage);
	}

}

void checkBMStemperatureFault(){

	//if BMS temperature low
	if( 	ADC_convertedResults[mcuInternalTemp] < nonVolPars.chgParas.minBMStemp ||
			LTC6803_getTemperature() < nonVolPars.chgParas.minBMStemp ){
		runtimePars.faults |= (1 << fault_lowBMStemp);
	}
	else{
		runtimePars.faults &= ~(1 << fault_lowBMStemp);
	}

	//if BMS temperature high
	if( 	ADC_convertedResults[mcuInternalTemp] > nonVolPars.chgParas.maxBMStemp ||
			LTC6803_getTemperature() > nonVolPars.chgParas.maxBMStemp ){
		runtimePars.faults |= (1 << fault_highBMStemp);
	}
	else{
		runtimePars.faults &= ~(1 << fault_highBMStemp);
	}

}

void checkNTCtemperatureFault(){

	//if NTC temperature low and enabled
	if( ADC_convertedResults[externalTemp] < nonVolPars.chgParas.minPackVolt && nonVolPars.chgParas.minNTCtemp != 0 ){
		runtimePars.faults |= (1 << fault_lowNTCtemp);
	}
	else{
		runtimePars.faults &= ~(1 << fault_lowNTCtemp);
	}

	//if NTC temperature high and enabled
	if( ADC_convertedResults[externalTemp] > nonVolPars.chgParas.maxPackVolt && nonVolPars.chgParas.maxNTCtemp != 0 ){
		runtimePars.faults |= (1 << fault_highNTCtemp);
	}
	else{
		runtimePars.faults &= ~(1 << fault_highNTCtemp);
	}

}

void checkCellVoltageFaults(){

	//if cell sensing enabled
	if( nonVolPars.chgParas.packCellCount != 0 ){
		for( uint8_t x=0; x<12; x++){
			if( LTC6803_getCellVoltage(x) < nonVolPars.chgParas.minCellVolt && x < nonVolPars.chgParas.packCellCount ){	//if cell voltage below limit
				runtimePars.faults |= (1 << (fault_lowCellVoltage0 + (2*x)) );
			}
			else{
				runtimePars.faults &= ~(1 << (fault_lowCellVoltage0 + (2*x)) );
			}

			if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.maxCellVolt && x < nonVolPars.chgParas.packCellCount ){	//if cell voltage above limit
				runtimePars.faults |= (1 << (fault_lowCellVoltage0 + (2*x)) );
			}
			else{
				runtimePars.faults &= ~(1 << (fault_highCellVoltage0 + (2*x)) );
			}
		}
	}
	else{	//else zero all cell voltage faults
		for( uint8_t x=0; x<12; x++){
			runtimePars.faults &= ~(1 << (fault_lowCellVoltage0 + (2*x)) );
			runtimePars.faults &= ~(1 << (fault_highCellVoltage0 + (2*x)) );
		}
	}

}

void checkPackVoltageFault(){

	//if pack voltage low
	if( ADC_convertedResults[batteryVoltage] < nonVolPars.chgParas.minPackVolt ){
		runtimePars.faults |= (1 << fault_lowPackVoltage);
	}
	else{
		runtimePars.faults &= ~(1 << fault_lowPackVoltage);
	}

	//if pack voltage high
	if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.maxPackVolt ){
		runtimePars.faults |= (1 << fault_highPackVoltage);
	}
	else{
		runtimePars.faults &= ~(1 << fault_highPackVoltage);
	}

}

void checkMaxCurrentFault(){

	//if charging current over max
	if( ADC_convertedResults[chargeCurrent] > nonVolPars.chgParas.maxChgCurr ){
		runtimePars.faults |= (1 << fault_highChargingCurrent);
	}
	else{
		runtimePars.faults &= ~(1 << fault_highChargingCurrent);
	}

}
