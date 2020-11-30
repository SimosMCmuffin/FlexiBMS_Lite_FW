/*
 * AuxFunctions.c
 *
 *  Created on: 27.5.2019
 *      Author: Simos MCmuffin
 */

#include "stm32l4xx_hal.h"
#include <usb_device_ST.h>
#include "AuxFunctions.h"
#include <LTC6803_3_DD.h>
#include <ADC_LL.h>
#include "main.h"
#include <dStorage_MD.h>
#include "config.h"
#include <CAN_LL.h>
#include <SPI_LL.h>

extern USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev);
extern nonVolParameters nonVolPars;
extern runtimeParameters runtimePars;

void initNonVolatiles(nonVolParameters* nonVols, uint8_t loadDefaults){

	//if no parameters stored in FLASH or if they were stored with different version number, load defaults
	if( loadNonVolatileParameters(nonVols) == 0 || loadDefaults == 1 || nonVols->FW_version != ((FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR) ){

		for(uint8_t x=0; x<5; x++){
			nonVols->adcParas.ADC_chan_gain[x] = 1.0;
			nonVols->adcParas.ADC_chan_offset[x] = 0;
		}
		nonVols->adcParas.extNTCbetaValue = 3380;
		nonVols->adcParas.AdcOversampling = 16;

		nonVols->genParas.stayActiveTime = 100;
		nonVols->genParas.alwaysBalancing = 0;
		nonVols->genParas.always5vRequest = 0;
		nonVols->genParas.duringActive5vOn = 0;
		nonVols->genParas.storageCellVoltage = 3800;
		nonVols->genParas.timeToStorageDischarge = 0;

		nonVols->chgParas.packCellCount = 12;
		nonVols->chgParas.cellBalVolt = 4150;
		nonVols->chgParas.cellDiffVolt = 10;
		nonVols->chgParas.termCellVolt = 4180;
		nonVols->chgParas.termPackVolt = 50400;
		nonVols->chgParas.maxChgCurr = 6500;
		nonVols->chgParas.termCurr = 300;
		nonVols->chgParas.balTempRatio = 3;

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

		nonVols->chgParas.refreshWaitTime = 30;
		nonVols->genParas.canActivityTick = 0;
		nonVols->genParas.canID = CAN_ID;
		nonVols->genParas.canRxRefreshActive = 0;

		nonVols->FW_version = (FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR;

	}

}

void initRuntimeParameters(runtimeParameters* runtimePars){
	runtimePars->statePrintout = 0;
	runtimePars->statusTick = 1000;
	runtimePars->ADCrunState = 0;
	runtimePars->LTC6803runState = 0;
	runtimePars->usbConnected = 0;
	runtimePars->chargerConnected = 0;
	runtimePars->optoActive = 0;
	runtimePars->chargingState = notCharging;
	runtimePars->currentRunMode = 1;
	runtimePars->activeFaults = 0;
	runtimePars->latchedFaults = 0;

	runtimePars->buck5vEnabled = 0;
	runtimePars->buck5vRequest = 0;
	runtimePars->buck5vRequest |= (nonVolPars.genParas.always5vRequest << always5vRequest);
	runtimePars->packVoltageEnabled = 0;
	runtimePars->packVoltageRequest = 0;
	runtimePars->chargerVoltageEnabled = 0;
	runtimePars->chargerVoltageRequest = 0;

	runtimePars->charging = 0;
	runtimePars->balancing = 0;
	runtimePars->storageDischarged = 1;
	runtimePars->storageTimerState = 1;

	runtimePars->activeTick = HAL_GetTick() + ( nonVolPars.genParas.stayActiveTime * __TIME_HOUR_TICKS ) ;
	runtimePars->storageTick = HAL_GetTick() + ( nonVolPars.genParas.timeToStorageDischarge * __TIME_HOUR_TICKS );
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
		runtimePars.usbConnected = 1;
		usb_state = 1;
		runtimePars.buck5vRequest |= (1 << usb5vRequest);
	}
	else if( !(GPIOB->IDR & (1 << 7)) && usb_state == 1 ){
		runtimePars.usbConnected = 0;
		usb_state = 0;
		runtimePars.buck5vRequest &= ~(1 << usb5vRequest);
	}

	return usb_state;
}


/**
 * @brief  chargeControl : used to switch and monitor the charging
 */
void chargeControl(void){
	static uint64_t chargeTick = 0, auxTick = 0;
	static uint16_t OCevent = 0;

	//check if charger detected
	if( runtimePars.chargerConnected == 1 ){
		runtimePars.buck5vRequest |= (1 << charging5vRequest);		//request 5V buck

		if( runtimePars.chargingState == notCharging ){
			runtimePars.activeFaults = 0;		//clear active faults
		}

		checkChargerVoltageFault();
		checkBMStemperatureFault();
		checkNTCtemperatureFault();
		checkCellVoltageFaults();
		checkPackVoltageFault();

		if( runtimePars.activeFaults == 0 && runtimePars.chargingState == notCharging && runtimePars.buck5vEnabled == 1){
			//allow charging to start
			runtimePars.chargingState = chargingStarting;
			chargeTick = HAL_GetTick() + 250;
		}
	}
	else if( runtimePars.chargerConnected == 0 && runtimePars.chargingState != charging && runtimePars.chargingState != startingCurrent ){
		runtimePars.chargingState = notCharging;
		runtimePars.buck5vRequest &= ~(1 << charging5vRequest);
	}

	if( runtimePars.chargingState != notCharging && runtimePars.chargingState != faultState && runtimePars.activeFaults != 0 ){
		//if faults found, set state machine to fault wait state
		runtimePars.chargingState = faultState;
		chargeTick = HAL_GetTick() + ( nonVolPars.chgParas.refreshWaitTime * 1000 );
	}

	balanceControl();	//control/monitor cell balancing


	switch(runtimePars.chargingState){
	case notCharging:
		__DISABLE_CHG;
		runtimePars.charging = 0;
		break;

	case chargingStarting:
		if( chargeTick <= HAL_GetTick() ){
			//start charging, monitor current and voltages real-time for a couple hundred milliseconds
			//before enabling charge path, check that cell and pack voltages below termination voltage
			if( nonVolPars.chgParas.packCellCount != 0 ){
				for( uint8_t x=0; x<nonVolPars.chgParas.packCellCount; x++){
					if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.termCellVolt ){	//if cell voltage above termination voltage, jump directly end/wait state
						runtimePars.chargingState = chargingEnd;
						break;
					}
				}
			}
			else{
				if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.termPackVolt ){
					runtimePars.chargingState = chargingEnd;
					break;
				}

			}

			if( runtimePars.activeFaults != 0 ){	//before opening charging FETs, check that there aren't faults
				runtimePars.chargingState = faultState;
			}
			else{
				chargeTick = HAL_GetTick() + 300;
				auxTick = HAL_GetTick();

				__ENABLE_CHG;
				runtimePars.charging = 1;

				OCevent = 0;
				runtimePars.chargingState = startingCurrent;
			}

		}
		break;

	case startingCurrent:
		//monitor the current, allow max 250ms of overcurrent period and stop charging if the current isn't above the termination current limit in 500ms at the end
		if( (auxTick + 1) <= HAL_GetTick() ){	//check roughly every 1 ms
			auxTick++;


			if( ADC_convertedResults[chargeCurrent] > nonVolPars.chgParas.maxChgCurr ){	//check for overcurrent event
				OCevent++;
			}

			if( OCevent > 35 ){	//if overcurrent event has been happening long enough (35ms), terminate charge
				runtimePars.activeFaults |= ((uint64_t)1 << fault_highChargingCurrent);
				runtimePars.chargingState = chargingEnd;
				__DISABLE_CHG;
				break;
			}


			if( chargeTick <= HAL_GetTick() ){	//if at the end of the starting period the current is below the termination current, go to end-of-charge
				if( ADC_convertedResults[chargeCurrent] > nonVolPars.chgParas.termCurr ){
					runtimePars.chargingState = charging;
				}
				else{
					runtimePars.chargingState = chargingEnd;
				}
			}
		}
		break;

	case charging:
		if( chargeTick <= HAL_GetTick() ){

			if( 	checkMaxCurrentFault() ||
					ADC_convertedResults[chargeCurrent] < nonVolPars.chgParas.termCurr ){	//if charging current too high or below termination current, stop
				runtimePars.chargingState = chargingEnd;
				__DISABLE_CHG;
				break;
			}

			if( nonVolPars.chgParas.packCellCount != 0 ){
				for( uint8_t x=0; x<nonVolPars.chgParas.packCellCount; x++){
					if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.termCellVolt ){	//if cell voltage above termination voltage, jump directly end/wait state
						runtimePars.chargingState = chargingEnd;
						__DISABLE_CHG;
						break;
					}
				}
			}
			else{
				if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.termPackVolt ){
					runtimePars.chargingState = chargingEnd;
					__DISABLE_CHG;
					break;
				}
			}

		}
		else{

			if( nonVolPars.chgParas.packCellCount != 0 ){
				for( uint8_t x=0; x<nonVolPars.chgParas.packCellCount; x++){
					if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.termCellVolt ){	//if cell voltage above termination voltage, jump directly end/wait state
						runtimePars.chargingState = chargingEnd;
						break;
					}
				}
			}
			else{
				if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.termPackVolt ){
					runtimePars.chargingState = chargingEnd;
					break;
				}
			}

		}

		break;

	case chargingEnd:
		//end of charging,
		__DISABLE_CHG;
		runtimePars.charging = 0;

		if( runtimePars.chargerConnected == 0 ){	//check if charger disconnected
			runtimePars.packVoltageRequest &= ~(1 << 0);
			chargeTick = HAL_GetTick() + 250;
			runtimePars.chargingState = chargerDisconnected;
		}
		break;

	case chargerDisconnected:
		if( chargeTick <= HAL_GetTick() ){
			runtimePars.buck5vRequest &= ~(1 << charging5vRequest);
			runtimePars.chargingState = notCharging;
			runtimePars.activeFaults = 0;
		}
		break;

	case faultState:
		//fault triggered charging stop,
		__DISABLE_CHG;
		runtimePars.charging = 0;

		if( runtimePars.chargerConnected == 0 ){	//check if charger disconnected
			runtimePars.packVoltageRequest &= ~(1 << 0);
			chargeTick = HAL_GetTick() + 250;
			runtimePars.chargingState = chargerDisconnected;
		}
		else if( chargeTick <= HAL_GetTick() ){
			//wait for the refreshWaitTime to pass and try to start charging again
			runtimePars.chargingState = notCharging;
		}
		break;

	default:
		break;
	}


}

void detectCharger(void){

	if( ADC_convertedResults[chargerVoltage] > nonVolPars.chgParas.minChgVolt ){
		if( runtimePars.chargerConnected == 0 ){	//clear latched faults if charger re-connected
			runtimePars.latchedFaults = 0;
		}
		runtimePars.chargerConnected = 1;
	}
	else{
		runtimePars.chargerConnected = 0;
	}
}

void balanceControl(void){
	//balancing
	if( 	nonVolPars.chgParas.packCellCount != 0 && runtimePars.activeFaults == 0 &&
			(nonVolPars.genParas.alwaysBalancing == 1 || runtimePars.chargingState != notCharging) ){

		uint8_t cellIndices[MAX_CELLS];
		sortCellsByVoltage(cellIndices);

		//calculate how many balancing resistors can be used based on the "balTempRatio", maxBMStemp and current BMS temp
		int8_t balTempRatio_allowed = 0;
		if( nonVolPars.chgParas.balTempRatio != 0){	//check if balTempRatio is used

			if( LTC6803_getTemperature() >= ADC_convertedResults[mcuInternalTemp] ){	//use the higher PCB temperature
				balTempRatio_allowed = nonVolPars.chgParas.maxBMStemp - LTC6803_getTemperature();
			}
			else{
				balTempRatio_allowed = nonVolPars.chgParas.maxBMStemp - ADC_convertedResults[mcuInternalTemp];
			}
			balTempRatio_allowed /= nonVolPars.chgParas.balTempRatio;	//divide the difference between current BMS temp and max BMS temp for max allowed resistors

		}
		else{										//otherwise use the static max (5)
			balTempRatio_allowed = MAX_CELLS_BALANCING;
		}


		runtimePars.balancing = 0;

		for(uint8_t x=0; x < nonVolPars.chgParas.packCellCount; x++){

			uint8_t i = cellIndices[x];
			if( 	(LTC6803_getCellVoltage(i) >= nonVolPars.chgParas.cellBalVolt) &&						//if cell voltage above balance voltage
					(LTC6803_getCellVoltage(i) > (lowestCell(nonVolPars.chgParas.packCellCount) + nonVolPars.chgParas.cellDiffVolt)) ){	//and cell difference greater than allowed compared to the lowest cell

				if( runtimePars.balancing < balTempRatio_allowed ){	//allow max of 6 resistors to balance, to help reduce the thermal generation
					LTC6803_setCellDischarge(i, 1);
				}
				else{
					LTC6803_setCellDischarge(i, 0);
				}
				runtimePars.balancing++;
			}
			else
				LTC6803_setCellDischarge(i, 0);

		}

	}
	else if( 	runtimePars.storageDischarged == 0 && runtimePars.storageTimerState == 0 &&
				nonVolPars.chgParas.packCellCount != 0 && LTC6803_getCellVoltage(0) != 0 ){	//storage discharge balance control
		uint8_t cellIndices[MAX_CELLS];
		sortCellsByVoltage(cellIndices);

		//calculate how many balancing resistors can be used based on the "balTempRatio", maxBMStemp and current BMS temp
		int8_t balTempRatio_allowed = 0;
		if( nonVolPars.chgParas.balTempRatio != 0){	//check if balTempRatio is used

			if( LTC6803_getTemperature() >= ADC_convertedResults[mcuInternalTemp] ){	//use the higher PCB temperature
				balTempRatio_allowed = nonVolPars.chgParas.maxBMStemp - LTC6803_getTemperature();
			}
			else{
				balTempRatio_allowed = nonVolPars.chgParas.maxBMStemp - ADC_convertedResults[mcuInternalTemp];
			}
			balTempRatio_allowed /= nonVolPars.chgParas.balTempRatio;	//divide the difference between current BMS temp and max BMS temp for max allowed resistors

		}
		else{										//otherwise use the static max
			balTempRatio_allowed = MAX_CELLS_BALANCING;
		}


		runtimePars.balancing = 0;
		uint8_t notBalanced = 0;

		for(uint8_t x=0; x < nonVolPars.chgParas.packCellCount; x++){

			uint8_t i = cellIndices[x];
			if( LTC6803_getCellVoltage(i) > nonVolPars.genParas.storageCellVoltage ){
				notBalanced++;

				if( runtimePars.balancing < balTempRatio_allowed ){	//allow max of 6 resistors to balance, to help reduce the thermal generation
					LTC6803_setCellDischarge(i, 1);
				}
				else{
					LTC6803_setCellDischarge(i, 0);
				}

				runtimePars.balancing++;
			}
			else
				LTC6803_setCellDischarge(i, 0);

		}

		if( notBalanced == 0){	//if no cell needs balancing, indicating that they're below the storageDischargeVoltage
			runtimePars.storageDischarged = 1;	//flag storageDischarge as completed
		}

	}
	else{												//stop balancing
		runtimePars.balancing = 0;
		for(uint8_t x=0; x<12; x++){
			LTC6803_setCellDischarge(x, 0);
		}
	}

	if( runtimePars.balancing > 0 ){
		runtimePars.buck5vRequest |= (1 << balancing5vRequest);
	}
	else{
		runtimePars.buck5vRequest &= ~(1 << balancing5vRequest);
	}

}

void hwRequestControl(void){

	if( runtimePars.currentRunMode > 0 && (nonVolPars.chgParas.minNTCtemp != 0 || nonVolPars.chgParas.maxNTCtemp != 0) ){
		runtimePars.buck5vRequest |= (1 << ntc5vRequest);
	}
	else{
		runtimePars.buck5vRequest &= ~(1 << ntc5vRequest);
	}

	if( runtimePars.buck5vRequest > 0 ){	//enable 5V buck, if there are requests for the 5V rail
		__ENABLE_5V_BUCK;
		runtimePars.buck5vEnabled = 1;
	}
	else{
		__DISABLE_5V_BUCK;
		runtimePars.buck5vEnabled = 0;
	}

	if( runtimePars.packVoltageRequest > 0 ){	//enable pack voltage resistive divider, if there are requests for pack voltage
		__ENABLE_BAT_VOLTAGE;
		runtimePars.packVoltageEnabled = 1;
	}
	else{
		__DISABLE_BAT_VOLTAGE;
		runtimePars.packVoltageEnabled = 0;
	}

	if( runtimePars.chargerVoltageRequest > 0 ){	//enable charger voltage resistive divider, if there are requests for charger voltage
		__ENABLE_CHARGER_VOLTAGE;
		runtimePars.chargerVoltageEnabled = 1;
	}
	else{
		__DISABLE_CHARGER_VOLTAGE;
		runtimePars.chargerVoltageEnabled = 0;
	}

}

void statusLed(void){
	static uint64_t statusLedTick = 0;
	static uint8_t state = 0, bootDone = 0;

	if( bootDone <= 7 ){	//Boot LED color animation
		runtimePars.buck5vRequest |= (1 << led5vRequest);	//request 5V buck on, so can use LED

		if( statusLedTick < HAL_GetTick() ){
			statusLedTick = HAL_GetTick() + 300;

			switch(bootDone){
			case 0: __GREEN_LED_ON; __RED_LED_OFF; __BLUE_LED_OFF; bootDone++; break;	//green
			case 1: __GREEN_LED_OFF; __RED_LED_ON; __BLUE_LED_OFF; bootDone++; break;	//red
			case 2: __GREEN_LED_OFF; __RED_LED_OFF; __BLUE_LED_ON; bootDone++; break;	//blue

			case 3: __GREEN_LED_ON; __RED_LED_ON; __BLUE_LED_OFF; bootDone++; break;	//yellow
			case 4: __GREEN_LED_OFF; __RED_LED_ON; __BLUE_LED_ON; bootDone++; break;	//magenta
			case 5: __GREEN_LED_ON; __RED_LED_OFF; __BLUE_LED_ON; bootDone++; break;	//cyan

			case 6: __GREEN_LED_ON; __RED_LED_ON; __BLUE_LED_ON; bootDone++; break;		//white

			default: __GREEN_LED_OFF; __RED_LED_OFF; __BLUE_LED_OFF; bootDone++;		//off
						runtimePars.buck5vRequest &= ~(1 << led5vRequest); break;		//remove 5V buck request
			}
		}

	}
	else{				//Normal LED operation

		if( statusLedTick < HAL_GetTick() ){
			statusLedTick = HAL_GetTick() + 1000;

			if( runtimePars.usbConnected == 1 && 	(runtimePars.activeFaults || runtimePars.charging ||
									runtimePars.balancing || runtimePars.chargingState == faultState) ){
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
			if( nonVolPars.genParas.canActivityTick == 1 && CAN1_rxAvailable() == 1 ){	//CAN activity cyan flick
				__GREEN_LED_ON;
				__RED_LED_OFF;
				__BLUE_LED_ON;
			}
			else if( runtimePars.usbConnected == 1 ){	//USB connected
				__BLUE_LED_ON;
			}
			else{
				__BLUE_LED_OFF;
			}
		}
		else{
			__BLUE_LED_OFF;
			if( nonVolPars.genParas.canActivityTick == 1 && CAN1_rxAvailable() == 1 ){	//CAN activity cyan flick
				__GREEN_LED_ON;
				__RED_LED_OFF;
				__BLUE_LED_ON;
			}
			else if( runtimePars.activeFaults != 0 || runtimePars.chargingState == faultState ){	//if active faults found -> status LED = RED
				__GREEN_LED_OFF;
				__RED_LED_ON;
			}
			else if( runtimePars.balancing > 0 ){	//if balancing -> status LED = YELLOW
				__GREEN_LED_ON;
				__RED_LED_ON;
			}
			else if( runtimePars.charging == 1 ){	//if charging -> status LED = GREEN
				__GREEN_LED_ON;
				__RED_LED_OFF;
			}
			else if( runtimePars.latchedFaults > 0 ){	//if there are latched faults -> status LED = MAGENTA
				__GREEN_LED_OFF;
				__RED_LED_ON;
				__BLUE_LED_ON;
			}
			else{
				__GREEN_LED_OFF;
				__RED_LED_OFF;
			}
		}

	}

}

void updateActiveTimer(void){

	if( 	runtimePars.usbConnected == 1 || runtimePars.charging == 1 || runtimePars.balancing == 1 ||
			runtimePars.optoActive == 1 || runtimePars.chargerConnected == 1 ){
		runtimePars.activeTick = HAL_GetTick() + ( nonVolPars.genParas.stayActiveTime * __TIME_HOUR_TICKS ) + 500;
	}

	if( runtimePars.activeTick <= HAL_GetTick() && runtimePars.storageDischarged == 1 ){
		runtimePars.activeTimerState = 0;	//active time window passed

		if( nonVolPars.genParas.duringActive5vOn == 1 ){	//if duringActive5vOn enabled, clear 5V request
			runtimePars.buck5vRequest &= ~(1 << active5vRequest);
		}
	}
	else{
		runtimePars.activeTimerState = 1;	//active time window not-passed

		if( nonVolPars.genParas.duringActive5vOn == 1 ){	//if duringActive5vOn enabled, set 5V request
			runtimePars.buck5vRequest |= (1 << active5vRequest);
		}
	}
}

void updateStorageTimer(void){

	if( runtimePars.chargerConnected == 1 ){
		runtimePars.storageTick = HAL_GetTick() + ( nonVolPars.genParas.timeToStorageDischarge * __TIME_HOUR_TICKS );
	}

	if( runtimePars.storageTick <= HAL_GetTick() && nonVolPars.genParas.timeToStorageDischarge != 0 ){
		if( runtimePars.storageTimerState == 1 ){
		runtimePars.storageTimerState = 0;	//storage time window passed
		runtimePars.storageDischarged = 0;	//clear storageDischarge state when starting storagedischarging
		}
	}
	else{
		if( runtimePars.storageTimerState == 0 ){
		runtimePars.storageTimerState = 1;	//storage time window not-passed
		runtimePars.storageDischarged = 1;	//set storageDischarged to true if time window not-passed
		}
	}
}

void jumpToBootloader(void){

	//Stop and de-init USB stack
	USBD_DeInit(&hUsbDeviceFS);
	__disable_irq();

	//de-init all peripherals
	SysTick->CTRL = 4;	//reset systick to default state (no interrupt enabled)

	RCC->AHB2RSTR = ~(0x00000000);		//Reset all peripherals in AHB2 bus
	RCC->AHB2ENR = 0;				//disable all peripheral clocks in AHB2 bus
	RCC->AHB2RSTR = 0;			//Clear reset all peripherals in AHB2 bus

	RCC->APB1RSTR1 = ~(0x00000400);		//Reset all peripherals in APB1 bus, except RTC peripheral
	RCC->APB1ENR1 = 0x00000400;			//disable all peripheral clocks in APB1 bus
	RCC->APB1RSTR1 = 0;			//Clear reset all peripherals in APB1 bus

	RCC->APB2RSTR = ~(0x00000000);		//Reset all peripherals in APB2 bus
	RCC->APB2ENR = 0;			//disable all peripheral clocks in APB2 bus
	RCC->APB2RSTR = 0;			//Clear reset all peripherals in APB2 bus


	//check oscillator status, MSI as main clock @ 4 MHz and everything else off
	//check if USB's 48 MHz internal oscillator on
	if( (RCC->CRRCR & (1 << 0)) != 0 ){	//HSI48_ON bit
		RCC->CRRCR = 0;					//disable
	}
	if( ((RCC->CR & (0xF << 4)) >> 4) != 6 ){	//check if MSI_range something else than 6 (4 MHz speed)
		RCC->CR = (RCC->CR & ~(0xF << 4)) | (6 << 4);	//set MSI_range to 6 (4 MHz speed)
	}

	for(uint32_t x=0; x<1400000; x++);

	//jump to STM bootloader
	asm("ldr r0, =0x1FFF0000"); //load R0 register with constant value 0x1FFF 0000, in this case the beginning of STM bootloader
	asm("ldr sp, [r0, #0]");	//load SP with value that R0 points to	*0x1FFF 0000 (End of stack)
	asm("ldr r0, [r0, #4]");	//load R0 with value R0 + 4 points to *0x1FFF 0004 (Reset Vector)
	asm("bx r0");				//jump there
}

void restartFW(void){
	//Stop and de-init USB stack
	USBD_DeInit(&hUsbDeviceFS);
	__disable_irq();

	//de-init all peripherals
	SysTick->CTRL = 4;	//reset systick to default state (no interrupt enabled)

	RCC->AHB2RSTR = ~(0x00000000);		//Reset all peripherals in AHB2 bus
	RCC->AHB2ENR = 0;				//disable all peripheral clocks in AHB2 bus
	RCC->AHB2RSTR = 0;			//Clear reset all peripherals in AHB2 bus

	RCC->APB1RSTR1 = ~(0x00000400);		//Reset all peripherals in APB1 bus, except RTC peripheral
	RCC->APB1ENR1 = 0x00000400;			//disable all peripheral clocks in APB1 bus
	RCC->APB1RSTR1 = 0;			//Clear reset all peripherals in APB1 bus

	RCC->APB2RSTR = ~(0x00000000);		//Reset all peripherals in APB2 bus
	RCC->APB2ENR = 0;			//disable all peripheral clocks in APB2 bus
	RCC->APB2RSTR = 0;			//Clear reset all peripherals in APB2 bus

	//check oscillator status, MSI as main clock @ 4 MHz and everything else off
	//check if USB's 48 MHz internal oscillator on
	if( (RCC->CRRCR & (1 << 0)) != 0 ){	//HSI48_ON bit
		RCC->CRRCR = 0;					//disable
	}
	if( ((RCC->CR & (0xF << 4)) >> 4) != 6 ){	//check if MSI_range something else than 6 (4 MHz speed)
		RCC->CR = (RCC->CR & ~(0xF << 4)) | (6 << 4);	//set MSI_range to 6 (4 MHz speed)
	}

	for(uint32_t x=0; x<1400000; x++);

	SCB->AIRCR = (0x5FA << 16) | (1 << 2);	//request system reset from the System Control Block (SCB) in the M4-cortex core

}

void checkChargerVoltageFault(){

	//if charger voltage over max
	if( ADC_convertedResults[chargerVoltage] > nonVolPars.chgParas.maxChgVolt ){
		runtimePars.activeFaults |= ((uint64_t)1 << fault_highChargerVoltage);
		//runtimePars.latchedFaults |= ((uint64_t)1 << fault_highChargerVoltage);
	}
	/*
	else{
		runtimePars.activeFaults &= ~((uint64_t)1 << fault_highChargerVoltage);
	}
	*/


}

void checkBMStemperatureFault(){

	//if BMS temperature low
	if( 	ADC_convertedResults[mcuInternalTemp] < nonVolPars.chgParas.minBMStemp ||
			LTC6803_getTemperature() < nonVolPars.chgParas.minBMStemp ){
		runtimePars.activeFaults |= ((uint64_t)1 << fault_lowBMStemp);
		//runtimePars.latchedFaults |= ((uint64_t)1 << fault_lowBMStemp);
	}
	/*
	else{
		runtimePars.activeFaults &= ~((uint64_t)1 << fault_lowBMStemp);
	}
	*/


	//if BMS temperature high
	if( 	ADC_convertedResults[mcuInternalTemp] > nonVolPars.chgParas.maxBMStemp ||
			LTC6803_getTemperature() > nonVolPars.chgParas.maxBMStemp ){
		runtimePars.activeFaults |= ((uint64_t)1 << fault_highBMStemp);
		//runtimePars.latchedFaults |= ((uint64_t)1 << fault_highBMStemp);
	}
	/*
	else{
		runtimePars.activeFaults &= ~((uint64_t)1 << fault_highBMStemp);
	}
	*/

}

void checkNTCtemperatureFault(){

	//if NTC temperature low and enabled
	if( ADC_convertedResults[externalTemp] < nonVolPars.chgParas.minPackVolt && nonVolPars.chgParas.minNTCtemp != 0 ){
		runtimePars.activeFaults |= ((uint64_t)1 << fault_lowNTCtemp);
		//runtimePars.latchedFaults |= ((uint64_t)1 << fault_lowNTCtemp);
	}
	/*
	else{
		runtimePars.activeFaults &= ~((uint64_t)1 << fault_lowNTCtemp);
	}
	*/

	//if NTC temperature high and enabled
	if( ADC_convertedResults[externalTemp] > nonVolPars.chgParas.maxPackVolt && nonVolPars.chgParas.maxNTCtemp != 0 ){
		runtimePars.activeFaults |= ((uint64_t)1 << fault_highNTCtemp);
		//runtimePars.latchedFaults |= ((uint64_t)1 << fault_highNTCtemp);
	}
	/*
	else{
		runtimePars.activeFaults &= ~((uint64_t)1 << fault_highNTCtemp);
	}
	*/

}

void checkCellVoltageFaults(){

	//if cell sensing enabled
	if( nonVolPars.chgParas.packCellCount != 0 ){
		for( uint8_t x=0; x<12; x++){
			if( LTC6803_getCellVoltage(x) < nonVolPars.chgParas.minCellVolt && x < nonVolPars.chgParas.packCellCount ){	//if cell voltage below limit
				runtimePars.activeFaults |= ((uint64_t)1 << (fault_lowCellVoltage0 + (2*x)) );
				//runtimePars.latchedFaults |= ((uint64_t)1 << (fault_lowCellVoltage0 + (2*x)) );
			}
			/*
			else{
				runtimePars.activeFaults &= ~((uint64_t)1 << (fault_lowCellVoltage0 + (2*x)) );
			}
			*/

			if( LTC6803_getCellVoltage(x) > nonVolPars.chgParas.maxCellVolt && x < nonVolPars.chgParas.packCellCount ){	//if cell voltage above limit
				runtimePars.activeFaults |= ((uint64_t)1 << (fault_highCellVoltage0 + (2*x)) );
				//runtimePars.latchedFaults |= ((uint64_t)1 << (fault_highCellVoltage0 + (2*x)) );
			}
			/*
			else{
				runtimePars.activeFaults &= ~((uint64_t)1 << (fault_highCellVoltage0 + (2*x)) );
			}
			*/

			if( LTC6803_getCellVoltage(x) >= nonVolPars.chgParas.minCellVolt && x >= nonVolPars.chgParas.packCellCount){
				runtimePars.activeFaults |= ((uint64_t)1 << (fault_voltageErrorCell0 + x));
				//runtimePars.latchedFaults |= ((uint64_t)1 << (fault_voltageErrorCell0 + x));
			}
		}
	}
	/*
	else{	//else zero all cell voltage faults
		for( uint8_t x=0; x<12; x++){
			runtimePars.faults &= ~(1 << (fault_lowCellVoltage0 + (2*x)) );
			runtimePars.faults &= ~(1 << (fault_highCellVoltage0 + (2*x)) );
		}
	}
	*/

}

void checkPackVoltageFault(){

	//if pack voltage low
	if( nonVolPars.chgParas.packCellCount == 0 ){
		if( ADC_convertedResults[batteryVoltage] < nonVolPars.chgParas.minPackVolt ){
			runtimePars.activeFaults |= ((uint64_t)1 << fault_lowPackVoltage);
			//runtimePars.latchedFaults |= ((uint64_t)1 << fault_lowPackVoltage);
		}
		/*
		else{
			runtimePars.activeFaults &= ~((uint64_t)1 << fault_lowPackVoltage);
		}
		*/

		//if pack voltage high
		if( ADC_convertedResults[batteryVoltage] > nonVolPars.chgParas.maxPackVolt ){
			runtimePars.activeFaults |= ((uint64_t)1 << fault_highPackVoltage);
			//runtimePars.latchedFaults |= ((uint64_t)1 << fault_highPackVoltage);
		}
		/*
		else{
			runtimePars.activeFaults &= ~((uint64_t)1 << fault_highPackVoltage);
		}
		*/

	}

}

uint8_t checkMaxCurrentFault(){

	//if charging current over max
	if( ADC_convertedResults[chargeCurrent] > nonVolPars.chgParas.maxChgCurr ){
		runtimePars.activeFaults |= ((uint64_t)1 << fault_highChargingCurrent);
		//runtimePars.latchedFaults |= ((uint64_t)1 << fault_highChargingCurrent);
		return 1;
	}
	else{
		//runtimePars.activeFaults &= ~((uint64_t)1 << fault_highChargingCurrent);
		return 0;
	}

}

void changeRunMode(){

	switch( runtimePars.currentRunMode ){

	case 0: 			//LOW-POWER STANDBY MODE
		if( runtimePars.optoActive == 1 || runtimePars.chargerConnected == 1 || runtimePars.usbConnected == 1 || runtimePars.activeTimerState == 1 ){
																				//(opto_enable found || charger_detected || usb_found)
			PWR->CR1 &= ~(1 << 14);			//disable low-power run mode
			while( !!(PWR->SR2 & (1 << 9)) );	//wait for voltage regulator to settle
			changeMSIfreq(8);				//switch MSI to 16MHz

			SysTick_Config(16000);		//update systick to 1000Hz tick rate with 16MHz clock
			InitPeripherals();


			runtimePars.chargerVoltageRequest |= (1 << 0);
			runtimePars.packVoltageRequest |= (1 << 0);
			runtimePars.currentRunMode = 1;
		}
		break;

	case 1: 			//RUN MODE
		if( runtimePars.usbConnected == 1 ){	//if USB 5V found, start 48MHz oscillator
			RCC->CRRCR |= (1 << 0);			//enable HSI48 clock, used for USB
			while( !(RCC->CRRCR & (1 << 1)) );	//wait for HSI48 to stabilize
			MX_USB_DEVICE_Init();

			runtimePars.currentRunMode = 2;
		}
		else if( runtimePars.optoActive == 0 && runtimePars.chargerConnected == 0 && runtimePars.activeTimerState == 0 ){
																				//opto_enable not found && charger not detected && stayActiveTimer expired
			changeMSIfreq(0);					//switch MSI to 100kHz
			PWR->CR1 |= (1 << 14);					//Go into low-power run mode

			SysTick_Config(100);			//update systick to 1000Hz tick rate with 100kHz clock
			deInitPeripherals();

			runtimePars.chargerVoltageRequest &= ~(1 << 0);
			runtimePars.packVoltageRequest &= ~(1 << 0);
			runtimePars.currentRunMode = 0;
		}
		break;

	case 2:				//RUN MODE W/ USB
		if( runtimePars.usbConnected == 0 ){	//if USB 5V not found, then shutdown 48MHz oscillator and switch to run mode 1
			USBD_DeInit(&hUsbDeviceFS);		//Stop usb service
			RCC->CRRCR &= ~(1 << 0);			//disable HSI48 clock, used for USB

			runtimePars.currentRunMode = 1;
		}
		break;

	}

}

void sortCellsByVoltage(uint8_t indices[]) {  // return indices of the cells sorted by voltage, highest to lowest
	// initialize
	for (uint8_t i=0; i < nonVolPars.chgParas.packCellCount; i++)
		indices[i] = i;

	// bubble sort
	uint8_t swapped = 1;
	while (swapped) {
		swapped = 0;
		for (uint8_t i=0; i < nonVolPars.chgParas.packCellCount - 1; i++) {
			if (LTC6803_getCellVoltage(indices[i]) < LTC6803_getCellVoltage(indices[i + 1])) {
				uint8_t tmp = indices[i];
				indices[i] = indices[i+1];
				indices[i+1] = tmp;
				swapped = 1;
			}
		}
	}
}

uint8_t extractUID(uint8_t pos){

	uint32_t number = 0;
	uint8_t tempNumber = 0;

	if(pos <= 3 ){
		number = *(uint32_t*)(0x1FFF7590);
    
		tempNumber = number >> (8*pos);
	}
	else if( pos >= 4 && pos <= 7){
		number = *(uint32_t*)(0x1FFF7590+4);
		tempNumber = number >> (8*(pos-4));
	}
	else{
		number = *(uint32_t*)(0x1FFF7590+8);
		tempNumber = number >> (8*(pos-8));
	}

	return tempNumber;
}

uint8_t updateOptoState(void){		//update the state of the opto-isolator
	if( !!(GPIOA->IDR & (1 << 9)) == 0 ){	//if PA9 pin low -> opto-isolator active
		runtimePars.optoActive = 1;
		runtimePars.buck5vRequest |= (1 << opto5vRequest);
		return 1;
	}
	else{
		runtimePars.optoActive = 0;
		runtimePars.buck5vRequest &= ~(1 << opto5vRequest);
		return 0;
	}
}

void changeMSIfreq(uint8_t freq){
	while ( !(RCC->CR & RCC_CR_MSIRDY));				//check that MSI ready before changing frequencies
	RCC->CR &= ~(0xF << 4);								//set MSI clock frequency to 100kHz
	RCC->CR |= (freq << 4);
	RCC->CR |= (1 << 3);								//activate new MSI clock frequency
	while ( !(RCC->CR & RCC_CR_MSIRDY));				//wait for MSI to be ready
}

void InitPeripherals(void){
	//USB init is taken care of when USB 5V is detected
	SPI1_init();					//init SPI1 low level HW
	LTC6803_init();					//init LTC6803 device driver
	LTC6803_runEnable();
	ADC_init();						//init ADC low level HW
	ADC_start();
	CAN1_init();					//init CAN low level HW
	CAN1_setupRxFilters();			//init CAN RX ID filters

	/*
	if( USART_ENABLED)
	//INIT USART
	else if( I2C ENABLED
	//INIT I2C
	 */
}

void deInitPeripherals(void){
	ADC_stop();
	ADC_deInit();
	LTC6803_runDisable();
	SPI1_deInit();
	CAN1_deInit();

	/*
		if( USART_ENABLED)
		//DE-INIT USART
		else if( I2C ENABLED
		//DE-INIT I2C
		 */
}

//trim MSI oscillator against HSI16
void trimOscillator(void){
	/*
	//system clock output on PA8 pin
	GPIOA->MODER &= ~(3 << 16);						//CHARGE_ENABLE
	GPIOA->MODER |= (2 << 16);

	RCC->CFGR |= (2 << 24);		//MCO on MSI
	*/

	uint8_t index = 0;

	RCC->CR |= (1 << 8);		//enable HSI16
	while( !!(RCC->CR & (1 << 10)) == 0 );	//wait for HSI16 to be ready

	HAL_Delay(10);

	RCC->CCIPR |= (2 << 20);	//set LPTIM2 clock source to HSI16

	RCC->APB1ENR1 |= (1 << 31);		//enable LPTIM1 bus clock
	RCC->APB1ENR2 |= (1 << 5);		//enable LPTIM2 bus clock

	LPTIM1->CR |= (1 << 0);			//enable LPTIM1
	LPTIM2->CR |= (1 << 0);			//enable LPTIM2

	LPTIM1->ARR = 0xFFFF;		//set top of counters to max (16-bit)
	LPTIM2->ARR = 0xFFFF;

	while(1){

	LPTIM1->CR |= (1 << 1);		//trigger LPTIM1 in single mode
	LPTIM2->CR |= (1 << 1);		//trigger LPTIM2 in single mode

	volatile uint16_t lptim1_CNT = 0, lptim2_CNT = 0;

	while( lptim1_CNT < 0xB000 )		//wait for the counter values to go up
		lptim1_CNT = LPTIM1->CNT;

	lptim1_CNT = LPTIM1->CNT;			//read
	lptim2_CNT = LPTIM2->CNT;
	lptim1_CNT = LPTIM1->CNT;
	lptim2_CNT = LPTIM2->CNT;


	if( lptim1_CNT > lptim2_CNT ){	//counts close enough to eachother
		break;
	}
	else{
		index++;
		RCC->ICSCR = (RCC->ICSCR & ~(0xFF << 8)) | (index << 8);
	}
	HAL_Delay(10);
	}


	LPTIM1->CR &= ~(1 << 0);		//disable LPTIM1
	LPTIM2->CR &= ~(1 << 0);		//disable LPTIM2

	RCC->APB1ENR1 &= ~(1 << 31);	//disable LPTIM1 bus clock
	RCC->APB1ENR2 &= ~(1 << 5);		//disable LPTIM2 bus clock

	RCC->CR &= ~(1 << 8);		//disable HSI16
	RCC->CCIPR &= ~(3 << 20);	//set LPTIM2 clock source to default
}
