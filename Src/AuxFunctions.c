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
		nonVols->genParas.alwaysBalancing = 0;
		nonVols->genParas.always5vRequest = 0;

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
	runtimePars->statePrintout = 0;
	runtimePars->statusTick = 1000;
	runtimePars->usbConnected = 0;
	runtimePars->chargingState = 0;
	runtimePars->faults = 0;

	runtimePars->buck5vEnabled = 0;
	runtimePars->buck5vRequest = 0;
	runtimePars->buck5vRequest |= (nonVolPars.genParas.always5vRequest << always5vRequest);
	runtimePars->packVoltageEnabled = 0;
	runtimePars->packVoltageRequest = 0;
	runtimePars->chargerVoltageEnabled = 0;
	runtimePars->chargerVoltageRequest = 0;

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
		runtimePars.buck5vRequest |= (1 << usb5vRequest);
	}
	else if( !(GPIOB->IDR & (1 << 7)) && usb_state == 1 ){
		USBD_DeInit(&hUsbDeviceFS);		//Stop usb service
		runtimePars.usbConnected = 0;
		usb_state = 0;
		runtimePars.buck5vRequest &= ~(1 << usb5vRequest);
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

	runtimePars.chargerVoltageRequest |= (1 << 0);
	//check if charger detected
	if( ADC_convertedResults[chargerVoltage] > nonVolPars.chgParas.minChgVolt ){
		runtimePars.buck5vRequest |= (1 << charging5vRequest);		//request 5V buck


		if( runtimePars.buck5vEnabled == 1 ){						//if 5V buck enabled
			checkChargerVoltageFault();
			checkBMStemperatureFault();
			checkNTCtemperatureFault();
			checkCellVoltageFaults();

			if( nonVolPars.chgParas.packCellCount == 0 ){
				checkPackVoltageFault();
				runtimePars.packVoltageRequest |= (1 << 0);
			}

			if( runtimePars.faults == 0 && chargingState == 0){
				//allow charging to start
				runtimePars.packVoltageRequest |= (1 << 0);
				chargingState = 1;
			}
		}

	}

	if( chargingState != 0 && runtimePars.faults != 0 ){
		chargingState = 0;
	}

	//balancing
	if( 	(chargingState != 0 && nonVolPars.chgParas.packCellCount != 0) ||
			(nonVolPars.genParas.alwaysBalancing == 1 && nonVolPars.chgParas.packCellCount != 0) ){

		uint8_t cellIndices[MAX_CELLS];
		sortCellsByVoltage(cellIndices);

		runtimePars.balancing = 0;
		for(uint8_t x=0; x < nonVolPars.chgParas.packCellCount; x++){
			uint8_t i = cellIndices[x];
			if( 	(LTC6803_getCellVoltage(i) >= nonVolPars.chgParas.cellBalVolt) &&						//if cell voltage above balance voltage
					(LTC6803_getCellVoltage(i) > (lowestCell(nonVolPars.chgParas.packCellCount) + nonVolPars.chgParas.cellDiffVolt)) ){	//and cell difference greater than allowed compared to the lowest cell

				if( runtimePars.balancing <= 5 ){	//allow max of 5 resistors to balance, to help reduce the thermal generation
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

		if( runtimePars.balancing > 0 ){
			runtimePars.buck5vRequest |= (1 << balancing5vRequest);
		}
		else{
			runtimePars.buck5vRequest &= ~(1 << balancing5vRequest);
		}

	}
	else{												//stop balancing
		runtimePars.balancing = 0;
		for(uint8_t x=0; x<12; x++){
			LTC6803_setCellDischarge(x, 0);
		}
	}


	switch(chargingState){
	case 0:
		runtimePars.packVoltageRequest &= ~(1 << 0);
		__DISABLE_CHG;
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
			runtimePars.packVoltageRequest &= ~(1 << 0);
			chargeTick = HAL_GetTick() + 250;
			chargingState = 4;
		}
		break;

	case 4:
		if( chargeTick <= HAL_GetTick() )
			runtimePars.buck5vRequest &= ~(1 << charging5vRequest);
			chargingState = 0;
		break;

	default:
		break;
	}

	runtimePars.chargingState = chargingState;

	return chargingState;
}

void hwRequestControl(void){

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
			if( runtimePars.balancing > 0 ){
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

void changeRunMode(uint8_t runMode){

	switch(runMode){

	case 0: 			//LOW-POWER STANDBY MODE
		if( usbPowerPresent() ){		//opto_enable found || charger_detected)
			PWR->CR1 &= ~(1 << 14);			//disable low-power run mode
			while( !!(PWR->SR2 & (1 << 9)) );	//wait for voltage regulator to settle
			while ( !(RCC->CR & RCC_CR_MSIRDY));
			RCC->CR = (RCC->CR & ~(0xF << 4)) | (8 << 4);	//set MSI clock frequency to 16MHz
			RCC->CR |= (1 << 3);					//activate new MSI clock frequency


			__ENABLE_5V_BUCK;			//turn on 5V buck SMPS
			//TODO ENABLE PERIPHERALS

			runMode = 1;
		}
		break;

	case 1: 			//RUN MODE
		if( usbPowerPresent() ){
			PWR->CR1 = (PWR->CR1 & ~(3 << 9)) | (1 << 9);		//set voltage scaling to range 1 (high frequency)
			while( !!(PWR->SR2 & (1 << 10)) );					//wait for voltage regulator to settle
			FLASH->ACR = (FLASH->ACR & ~(7 << 0)) | (0 << 0);	//decrease flash memory wait states

			RCC->CRRCR |= (1 << 0);			//enable HSI48 clock, used for USB
			while( !(RCC->CRRCR & (1 << 1)) );	//wait for HSI48 to stabilize
			RCC->APB1ENR1 |= (1 << 26);		//enable USB FS bus clock
			USBD_Start(&hUsbDeviceFS);		//Start usb service

			runMode = 2;
		}
		else if( 0 ){	//opto_enable not found && charger not detected
			while ( !(RCC->CR & RCC_CR_MSIRDY));
			RCC->CR &= ~(0xF << 4);								//set MSI clock frequency to 100kHz
			RCC->CR |= (1 << 3);								//activate new MSI clock frequency

			//PWR->CR1 |= (1 << 14);					//Go into low-power run mode

			//TODO DISABLE PERIPHERALS
			__DISABLE_5V_BUCK;			//turn off 5V buck SMPS

			runMode = 0;
		}
		break;

	case 2:				//RUN MODE WITH USB
		if(1){

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
