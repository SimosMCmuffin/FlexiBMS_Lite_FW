/*
 * USB_comms_handler.c
 *
 *  Created on: 21.7.2019
 *      Author: Simos MCmuffin
 */

#include <USB_comms_handler_MD.h>
#include <usbd_cdc_if_ST.h>
#include <dStorage_MD.h>
#include <ADC_LL.h>
#include <LTC6803_3_DD.h>
#include <config.h>
#include <AuxFunctions.h>


#define APP_RX_DATA_SIZE  64

extern void jumpToStmBootloader(void);
extern void restartFW(void);

extern const uint8_t FW_VERSION[];
extern nonVolParameters nonVolPars;
extern runtimeParameters runtimePars;

void USB_checkForNewMessages(){
	static uint64_t liveTick = 0;
	uint32_t length = 0;
	uint8_t data[APP_RX_DATA_SIZE] = {0};

	if( VCP_retrieveInputData(data, &length) ){		//check if messages available

		for( uint8_t x=0; x<length; x++ ){		//find message starting character '$'
			if( data[x] == '$' ){
				x++;
				uint16_t parameter_ID = 0;

				switch( data[x] ){				//check what next CHAR is
				case '?':
					report_help();
					break;
				case '$':
					report_parameters(0, 1);
					break;
				case 'R':
					report_statePrint();
					break;
				case 'F':
					report_faults();
					break;
				case 'B':
					jumpToStmBootloader();
					break;
				case 'N':
					jumpToCustomBootloader();
					break;
				case 'K':
					restartFW();
					break;
				case 'W':
					report_firmware();
					break;
				case 'H':
					report_hardware();
					break;
				case 'C':
					report_UID();
					break;
				case 'S':
					saveNonVolatileParameters(&nonVolPars, 1);
					report_save(0);
					break;
				case 'L':
					report_load( initNonVolatiles(&nonVolPars, 0) );
					break;
				case 'D':
					report_loadDefaults();
					initNonVolatiles(&nonVolPars, 1);
					break;
				default:

					if( data[x] < '0' || data[x] > '9' ){	//check if unknown command
						report_error(error_invalidCommand);
						break;
					}

					while( data[x] >= '0' && data[x] <= '9' && x < length ){	//check how many numbers after '$' and get message ID from that
						parameter_ID *= 10;
						parameter_ID += data[x] - '0';
						x++;
					}

					if( parameter_ID >= 0 && parameter_ID < numberOfElements && x <= length ){	//sanity check

						if( data[x] == '=' ){									//if write
							float value = readFloat(data, &length, x);			//get data from message

							set_parameter(&value, parameter_ID);
						}
						report_parameters(parameter_ID, 0);					//echo new value back

						break;
					}
					else{
						report_error(error_invalidMessageID);
					}

					break;
				}
				break;
			}

			else{
				report_error(error_invalidCommand);
			}

		}
	}

	if( runtimePars.statePrintout == 1 && runtimePars.usbConnected == 1 ){
		if( liveTick <= HAL_GetTick() ){
			liveTick = HAL_GetTick() + runtimePars.statusTick;
			report_state();
		}

	}

}

void set_parameter(float* value, _parameter_ID parameterID){

	switch(parameterID){
	case packCellCount:
		nonVolPars.chgParas.packCellCount = (uint16_t)*value;
		break;
	case maxChgCurr:
		nonVolPars.chgParas.maxChgCurr = (uint16_t)*value;
		break;
	case termCurr:
		nonVolPars.chgParas.termCurr = (uint16_t)*value;
		break;
	case minCellVolt:
		nonVolPars.chgParas.minCellVolt = (uint16_t)*value;
		break;
	case maxCellVolt:
		nonVolPars.chgParas.maxCellVolt = (uint16_t)*value;
		break;
	case minChgVolt:
		nonVolPars.chgParas.minChgVolt = (uint16_t)*value;
		break;
	case maxChgVolt:
		nonVolPars.chgParas.maxChgVolt = (uint16_t)*value;
		break;
	case minPackVolt:
		nonVolPars.chgParas.minPackVolt = (uint16_t)*value;
		break;
	case maxPackVolt:
		nonVolPars.chgParas.maxPackVolt = (uint16_t)*value;
		break;
	case termCellVolt:
		nonVolPars.chgParas.termCellVolt = (uint16_t)*value;
		break;
	case termPackVolt:
		nonVolPars.chgParas.termPackVolt = (uint16_t)*value;
		break;
	case cellBalVolt:
		nonVolPars.chgParas.cellBalVolt = (uint16_t)*value;
		break;
	case cellDiffVolt:
		nonVolPars.chgParas.cellDiffVolt = (uint16_t)*value;
		break;
	case minNTCtemp:
		nonVolPars.chgParas.minNTCtemp = (uint16_t)*value;
		break;
	case maxNTCtemp:
		nonVolPars.chgParas.maxNTCtemp = (uint16_t)*value;
		break;
	case minBMStemp:
		nonVolPars.chgParas.minBMStemp = (uint16_t)*value;
		break;
	case maxBMStemp:
		nonVolPars.chgParas.maxBMStemp = (uint16_t)*value;
		break;
	case refreshWaitTime:
		nonVolPars.chgParas.refreshWaitTime = (uint16_t)*value;
		break;
	case ADC_chan_gain_0:
		nonVolPars.adcParas.ADC_chan_gain[batteryVoltage] = *value;
		break;
	case ADC_chan_offset_0:
		nonVolPars.adcParas.ADC_chan_offset[batteryVoltage] = *value;
		break;
	case ADC_chan_gain_1:
		nonVolPars.adcParas.ADC_chan_gain[chargerVoltage] = *value;
		break;
	case ADC_chan_offset_1:
		nonVolPars.adcParas.ADC_chan_offset[chargerVoltage] = *value;
		break;
	case ADC_chan_gain_2:
		nonVolPars.adcParas.ADC_chan_gain[chargeCurrent] = *value;
		break;
	case ADC_chan_offset_2:
		nonVolPars.adcParas.ADC_chan_offset[chargeCurrent] = *value;
		break;
	case ADC_chan_gain_3:
		nonVolPars.adcParas.ADC_chan_gain[externalTemp] = *value;
		break;
	case ADC_chan_offset_3:
		nonVolPars.adcParas.ADC_chan_offset[externalTemp] = *value;
		break;
	case ADC_chan_gain_4:
		nonVolPars.adcParas.ADC_chan_gain[mcuInternalTemp] = *value;
		break;
	case ADC_chan_offset_4:
		nonVolPars.adcParas.ADC_chan_offset[mcuInternalTemp] = *value;
		break;
	case extNTCbetaValue:
		nonVolPars.adcParas.extNTCbetaValue = (uint16_t)*value;
		break;
	case AdcOversampling:
		nonVolPars.adcParas.AdcOversampling = (uint16_t)*value;
		break;
	case stayActiveTime:
		nonVolPars.genParas.stayActiveTime = (uint16_t)*value;
		break;
	case alwaysBalancing:
		nonVolPars.genParas.alwaysBalancing = (uint8_t)*value;
		break;
	case keep5ValwaysOn:
		nonVolPars.genParas.always5vRequest = (uint8_t)*value;
		runtimePars.buck5vRequest = ((uint16_t)*value << always5vRequest);
		break;
	case balTempRatio:
		nonVolPars.chgParas.balTempRatio = (uint16_t)*value;
		break;
	case storageCellVoltage:
		nonVolPars.genParas.storageCellVoltage = (uint16_t)*value;
		break;
	case timeToStorageDischarge:
		nonVolPars.genParas.timeToStorageDischarge = (uint16_t)*value;
		break;
	case canActivityTick:
		nonVolPars.genParas.canActivityTick = (uint8_t)*value;
		break;
	case canID:
		nonVolPars.genParas.canID = (uint8_t)*value;
		break;
	case duringActive5vOn:
		nonVolPars.genParas.duringActive5vOn = (uint8_t)*value;
		break;
	case canRxRefreshActive:
		nonVolPars.genParas.canRxRefreshActive = (uint16_t)*value;
		break;
	default:
		report_error(error_invalidMessageID);
		break;
	}
}

void set_ADC_chan_gain(float* value, uint8_t channel){
	nonVolPars.adcParas.ADC_chan_gain[channel] = *value;
}

void set_ADC_chan_offset(float* value, uint8_t channel){
	nonVolPars.adcParas.ADC_chan_offset[channel] = *value;
}

void set_extNTCbetaValue(float* value){
	nonVolPars.adcParas.extNTCbetaValue = (uint16_t)*value;
}

void report_faults(void){
	uint8_t text[256] = {};
	uint16_t pos = 0;

	static const uint8_t Ftext1[] = {"\r\nLatched faults:"};
	static const uint8_t Ftext2[] = {"\r\nActive faults:"};
	appendString(text, Ftext1, &pos);

	for(uint8_t x=0; x<fault_numberOfElements; x++){
		if( !!(runtimePars.latchedFaults & ((uint64_t)1 << x)) ){
			appendFault(text, x, &pos);
		}
	}

	appendString(text, Ftext2, &pos);

	for(uint8_t x=0; x<fault_numberOfElements; x++){
		if( !!(runtimePars.activeFaults & ((uint64_t)1 << x)) ){
			appendFault(text, x, &pos);
		}
	}

	text[pos] = '\r';
	pos++;
	text[pos] = '\n';
	pos++;

	while( CDC_Transmit_FS(text, pos) );
}

void report_statePrint(void){
	static const uint8_t Ftext1[] = {"\r\nStatus reporting ON:\r\n"};
	static const uint8_t Ftext2[] = {"\r\nStatus reporting OFF:\r\n"};

	if( runtimePars.statePrintout == 0 ){
		runtimePars.statePrintout = 1;
		CDC_Transmit_FS(Ftext1, sizeof(Ftext1)-1);
	}
	else{
		runtimePars.statePrintout = 0;
		CDC_Transmit_FS(Ftext2, sizeof(Ftext2)-1);
	}

}

void report_loadDefaults(void){
	uint8_t text[64] = {};
	uint16_t pos = 0;

	static const uint8_t Ftext1[] = {"\r\nDefault values loaded!\r\n"};
	appendString(text, Ftext1, &pos);

	CDC_Transmit_FS(text, pos);
}

void report_firmware(void){
	uint8_t text[64] = {};
	uint16_t pos = 0;

	static const uint8_t Ftext1[] = {"FW version: "};
	appendString(text, Ftext1, &pos);
	appendUint16(text, FW_VERSION_MAJOR, &pos);
	text[pos] = '.';
	pos++;
	appendUint16(text, FW_VERSION_MINOR, &pos);

	text[pos] = '\r';
	pos++;
	text[pos] = '\n';
	pos++;

	CDC_Transmit_FS(text, pos);

}

void report_hardware(void){
	uint8_t text[64] = {};
	uint16_t pos = 0;
	uint8_t* memoryLocation = (uint8_t*)0x1FFF7000;

	static const uint8_t Ftext1[] = {"Board: "HW_NAME" "};
	appendString(text, Ftext1, &pos);
	appendStringFromMemory(text, memoryLocation, 0, &pos);

	text[pos] = '\r';
	pos++;
	text[pos] = '\n';
	pos++;

	CDC_Transmit_FS(text, pos);
}

void report_UID(void){
	uint8_t text[64] = {};
	uint16_t pos = 0;

	static const uint8_t Ftext1[] = {"Board UID: "};
	appendString(text, Ftext1, &pos);
	appendUID(text, &pos);

	text[pos] = '\r';
	pos++;
	text[pos] = '\n';
	pos++;

	CDC_Transmit_FS(text, pos);

}

void report_state(void){
	uint8_t text[128] = {};
	uint16_t pos = 0;

	static const uint8_t Ftext1[] = {"State:"};
	appendString(text, Ftext1, &pos);

	for(uint8_t x=0; x<12; x++){
		appendUint16(text, LTC6803_getCellVoltage(x), &pos);
		text[pos] = ':';
		pos++;
	}

	appendUint16(text, ADC_convertedResults[batteryVoltage], &pos);
	text[pos] = ':';
	pos++;

	appendUint16(text, ADC_convertedResults[chargerVoltage], &pos);
	text[pos] = ':';
	pos++;

	appendUint16(text, ADC_convertedResults[chargeCurrent], &pos);
	text[pos] = ':';
	pos++;

	appendUint16(text, ADC_convertedResults[mcuInternalTemp], &pos);
	text[pos] = ':';
	pos++;

	appendUint16(text, LTC6803_getTemperature(), &pos);
	text[pos] = ':';
	pos++;

	appendUint16(text, ADC_convertedResults[externalTemp], &pos);
	text[pos] = ':';
	pos++;

	appendChargingState(text, runtimePars.chargingState, &pos);

	text[pos] = '\r';
	pos++;
	text[pos] = '\n';
	pos++;

	CDC_Transmit_FS(text, pos);
}

void report_help(){
	static const uint8_t text[] = 	{
							"\r\n"
							"$? (command list)\r\n"
							"$$ (view configurable parameters)\r\n"
							"$R (turn BMS state report ON/OFF)\r\n"
							"$F (view BMS faults)\r\n"
							"$W (print FW version)\r\n"
							"$H (print HW version)\r\n"
							"$C (print board Unique ID)\r\n"
							"$B (jump into bootloader)\r\n"
							"$K (reboot FW)\r\n"
							"$S (save parameters to EEPROM)\r\n"
							"$L (load parameters from EEPROM)\r\n"
							"$D (load default parameters)\r\n\r\n"
							};

	CDC_Transmit_FS(text, sizeof(text)-1);
}

void report_error(_error_ID errorCode){
	uint8_t text[128] = {};
	uint16_t pos = 0;

	static const uint8_t Ftext1[] = {"\r\n\r\n"};
	appendString(text, Ftext1, &pos);

	switch(errorCode){
	case error_invalidMessageID:;
		static const uint8_t etext1[] = {"Invalid parameter ID\r\n"};
		appendString(text, etext1, &pos);
		break;
	case error_invalidValue:;
		static const uint8_t etext2[] = {"Invalid parameter value\r\n"};
		appendString(text, etext2, &pos);
		break;
	case error_invalidCommand:;
		static const uint8_t etext3[] = {"Invalid command, type \"$?\" for command list\r\n"};
		appendString(text, etext3, &pos);
		break;
	default:
		break;
	}

	CDC_Transmit_FS(text, pos);
}

void report_save(uint16_t status){
	static const uint8_t text[] = 	{
			"\r\n"
			"Non-volatile parameters saved to FLASH\r\n"
	};

	CDC_Transmit_FS(text, sizeof(text)-1);
}

void report_load(uint16_t status){
	if(status){
		static const uint8_t text[] = 	{
				"\r\n"
				"Non-volatile parameters loaded from FLASH\r\n"
		};
		CDC_Transmit_FS(text, sizeof(text)-1);
	}
	else{
		static const uint8_t text[] = 	{
				"\r\n"
				"No Parameters found in FLASH! Save current ones with \"$S\" or load default values with \"$D\"\r\n"
		};
		CDC_Transmit_FS(text, sizeof(text)-1);
	}

}

void report_parameters(_parameter_ID parameter, uint8_t reportAll){
	uint8_t text[1024] = {};
	uint16_t pos = 0;

	static const uint8_t Ftext1[] = {"\r\n"};

	for(uint16_t x=0; x<numberOfElements; x++){

		if( reportAll == 0)
			x = parameter;
		if( (x % 10) == 0 )
			appendString(text, Ftext1, &pos);

		appendParameter(text, x, &pos);

		if( reportAll == 0)
			break;

		if( pos > 512){
			while( CDC_Transmit_FS(text, pos) );
			pos = 0;
		}
	}
	while( CDC_Transmit_FS(text, pos) );

}

void appendParameter(uint8_t* text, uint16_t indexNo, uint16_t* pos){
	text[*pos] = '$'; *pos += 1;
	appendUint16(text, indexNo, pos);
	text[*pos] = '='; *pos += 1;

	switch(indexNo){
	case packCellCount:
		appendUint16(text, nonVolPars.chgParas.packCellCount, pos);
		static const uint8_t description0[] = {" (Pack cell count; number of series cells in the battery pack, set to 0 to disable cell sensing, Uint)\r\n"};
		appendString(text, description0, pos);
		break;
	case maxChgCurr:
		appendUint16(text, nonVolPars.chgParas.maxChgCurr, pos);
		static const uint8_t description1[] = {" (Max charging current; mA (milliAmps), maximum current allowed to flow to battery, Uint)\r\n"};
		appendString(text, description1, pos);
		break;
	case termCurr:
		appendUint16(text, nonVolPars.chgParas.termCurr, pos);
		static const uint8_t description2[] = {" (Charging termination current; mA (milliAmps), stop charging when current drops below this, Uint)\r\n"};
		appendString(text, description2, pos);
		break;
	case minCellVolt:
		appendUint16(text, nonVolPars.chgParas.minCellVolt, pos);
		static const uint8_t description3[] = {" (Minimum cell voltage; mV (milliVolts), minimum allowed cell voltage, no charging allowed if cell voltage below this, Uint)\r\n"};
		appendString(text, description3, pos);
		break;
	case maxCellVolt:
		appendUint16(text, nonVolPars.chgParas.maxCellVolt, pos);
		static const uint8_t description4[] = {" (Maximum cell voltage; mV (milliVolts), maximum allowed cell voltage, no charging allowed if cell voltage above this, Uint)\r\n"};
		appendString(text, description4, pos);
		break;
	case minChgVolt:
		appendUint16(text, nonVolPars.chgParas.minChgVolt, pos);
		static const uint8_t description5[] = {" (Minimum charger voltage; mV (milliVolts), minimum allowed charger voltage, no charging allowed if charger voltage below this, Uint)\r\n"};
		appendString(text, description5, pos);
		break;
	case maxChgVolt:
		appendUint16(text, nonVolPars.chgParas.maxChgVolt, pos);
		static const uint8_t description6[] = {" (Maximum charger voltage; mV (milliVolts), maximum allowed charger voltage, no charging allowed if charger voltage above this, Uint)\r\n"};
		appendString(text, description6, pos);
		break;
	case minPackVolt:
		appendUint16(text, nonVolPars.chgParas.minPackVolt, pos);
		static const uint8_t description28[] = {" (Minimum pack voltage; mV (milliVolts), minimum allowed pack voltage, no charging allowed if pack voltage below this, Uint)\r\n"};
		appendString(text, description28, pos);
		break;
	case maxPackVolt:
		appendUint16(text, nonVolPars.chgParas.maxPackVolt, pos);
		static const uint8_t description29[] = {" (Maximum pack voltage; mV (milliVolts), maximum allowed pack voltage, no charging allowed if pack voltage above this, Uint)\r\n"};
		appendString(text, description29, pos);
		break;
	case termCellVolt:
		appendUint16(text, nonVolPars.chgParas.termCellVolt, pos);
		static const uint8_t description7[] = {" (Charging cell termination voltage; mV (milliVolts), don't allow any cell to go above this voltage when charging, Uint)\r\n"};
		appendString(text, description7, pos);
		break;
	case termPackVolt:
		appendUint16(text, nonVolPars.chgParas.termPackVolt, pos);
		static const uint8_t description30[] = {" (Charging pack termination voltage; mV (milliVolts), don't allow pack to go above this voltage when charging, Uint)\r\n"};
		appendString(text, description30, pos);
		break;
	case cellBalVolt:
		appendUint16(text, nonVolPars.chgParas.cellBalVolt, pos);
		static const uint8_t description8[] = {" (Cell balancing voltage; mV (milliVolts), allow balancing once a cell goes above this voltage, Uint)\r\n"};
		appendString(text, description8, pos);
		break;
	case cellDiffVolt:
		appendUint16(text, nonVolPars.chgParas.cellDiffVolt, pos);
		static const uint8_t description9[] = {" (Allowed difference between cell groups; mV (milliVolts), maximum allowed voltage difference between cell groups, balance if difference bigger, Uint)\r\n"};
		appendString(text, description9, pos);
		break;
	case minNTCtemp:
		appendUint16(text, nonVolPars.chgParas.minNTCtemp, pos);
		static const uint8_t description10[] = {" (Minimum external NTC thermistor temperature; K (Kelvin), set to 0 to disable, if enabled, the minimum temperature above which charging is allowed, Uint)\r\n"};
		appendString(text, description10, pos);
		break;
	case maxNTCtemp:
		appendUint16(text, nonVolPars.chgParas.maxNTCtemp, pos);
		static const uint8_t description11[] = {" (Maximum external NTC thermistor temperature; K (Kelvin), set to 0 to disable, if enabled, the maximum temperature below which charging is allowed, Uint)\r\n"};
		appendString(text, description11, pos);
		break;
	case minBMStemp:
		appendUint16(text, nonVolPars.chgParas.minBMStemp, pos);
		static const uint8_t description12[] = {" (Minimum PCB temperature; K (Kelvin), the minimum temperature above which charging is allowed, Uint)\r\n"};
		appendString(text, description12, pos);
		break;
	case maxBMStemp:
		appendUint16(text, nonVolPars.chgParas.maxBMStemp, pos);
		static const uint8_t description13[] = {" (Maximum PCB temperature; K (Kelvin), the maximum temperature below which charging is allowed, Uint)\r\n"};
		appendString(text, description13, pos);
		break;
	case refreshWaitTime:
		appendUint16(text, nonVolPars.chgParas.refreshWaitTime, pos);
		static const uint8_t description14[] = {" (Fault wait time; s (seconds), How long to wait after fault state before trying to start charging again, Uint)\r\n"};
		appendString(text, description14, pos);
		break;
	case ADC_chan_gain_0:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_gain[batteryVoltage], pos);
		static const uint8_t description15[] = {" (Gain for Battery voltage ADC conversion, Float)\r\n"};
		appendString(text, description15, pos);
		break;
	case ADC_chan_offset_0:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_offset[batteryVoltage], pos);
		static const uint8_t description16[] = {" (Offset for Battery voltage ADC conversion, Float)\r\n"};
		appendString(text, description16, pos);
		break;
	case ADC_chan_gain_1:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_gain[chargerVoltage], pos);
		static const uint8_t description17[] = {" (Gain for Charger voltage ADC conversion, Float)\r\n"};
		appendString(text, description17, pos);
		break;
	case ADC_chan_offset_1:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_offset[chargerVoltage], pos);
		static const uint8_t description18[] = {" (Offset for Charger voltage ADC conversion, Float)\r\n"};
		appendString(text, description18, pos);
		break;
	case ADC_chan_gain_2:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_gain[chargeCurrent], pos);
		static const uint8_t description19[] = {" (Gain for Current sense ADC conversion, Float)\r\n"};
		appendString(text, description19, pos);
		break;
	case ADC_chan_offset_2:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_offset[chargeCurrent], pos);
		static const uint8_t description20[] = {" (Offset for Current sense ADC conversion, Float)\r\n"};
		appendString(text, description20, pos);
		break;
	case ADC_chan_gain_3:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_gain[externalTemp], pos);
		static const uint8_t description21[] = {" (Gain for External NTC temperature probe conversion, Float)\r\n"};
		appendString(text, description21, pos);
		break;
	case ADC_chan_offset_3:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_offset[externalTemp], pos);
		static const uint8_t description22[] = {" (Offset for External NTC temperature probe conversion, Float)\r\n"};
		appendString(text, description22, pos);
		break;
	case ADC_chan_gain_4:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_gain[mcuInternalTemp], pos);
		static const uint8_t description23[] = {" (Gain for Internal MCU temperature conversion, Float)\r\n"};
		appendString(text, description23, pos);
		break;
	case ADC_chan_offset_4:
		appendFloat(text, nonVolPars.adcParas.ADC_chan_offset[mcuInternalTemp], pos);
		static const uint8_t description24[] = {" (Offset for Internal MCU temperature conversion, Float)\r\n"};
		appendString(text, description24, pos);
		break;
	case extNTCbetaValue:
		appendUint16(text, nonVolPars.adcParas.extNTCbetaValue, pos);
		static const uint8_t description26[] = {" (External NTC probe Beta-value; external NTC sensor's beta value, Uint)\r\n"};
		appendString(text, description26, pos);
		break;
	case AdcOversampling:
		appendUint16(text, nonVolPars.adcParas.AdcOversampling, pos);
		static const uint8_t description27[] = {" (ADC's oversampling setting; allowed values (1, 2, 4, 8, 16), Uint)\r\n"};
		appendString(text, description27, pos);
		break;
	case stayActiveTime:
		appendUint16(text, nonVolPars.genParas.stayActiveTime, pos);
		static const uint8_t description25[] = {" (h (Hours), how long to stay in active mode, Uint)\r\n"};
		appendString(text, description25, pos);
		break;
	case alwaysBalancing:
		appendUint16(text, nonVolPars.genParas.alwaysBalancing, pos);
		static const uint8_t description31[] = {" (0/1, allow cell balancing outside of charging, Boolean)\r\n"};
		appendString(text, description31, pos);
		break;
	case keep5ValwaysOn:
		appendUint16(text, nonVolPars.genParas.always5vRequest, pos);
		static const uint8_t description32[] = {" (0/1, force 5V regulator always on when battery connected, Boolean)\r\n"};
		appendString(text, description32, pos);
		break;
	case balTempRatio:
		appendUint16(text, nonVolPars.chgParas.balTempRatio, pos);
		static const uint8_t description34[] = {" (balancing temperature ratio, dynamically adjusts the max allowed balancing resistors based on BMS temperature, set to 0 to use static maximum, Uint)\r\n"};
		appendString(text, description34, pos);
		break;
	case storageCellVoltage:
		appendUint16(text, nonVolPars.genParas.storageCellVoltage, pos);
		static const uint8_t description35[] = {" (Storage discharge voltage; mV (milliVolts), if storage discharge enabled, then pack will be discharged to this voltage, Uint)\r\n"};
		appendString(text, description35, pos);
		break;
	case timeToStorageDischarge:
		appendUint16(text, nonVolPars.genParas.timeToStorageDischarge, pos);
		static const uint8_t description36[] = {" (h (Hours), how long to wait from last CHARGING event to start discharging the pack to the storage voltage, set to 0 to disable, Uint)\r\n"};
		appendString(text, description36, pos);
		break;
	case canActivityTick:
		appendUint16(text, nonVolPars.genParas.canActivityTick, pos);
		static const uint8_t description37[] = {" (0/1, CAN activity status LED tick, good for testing that the BMS is receiving CAN traffic, Boolean)\r\n"};
		appendString(text, description37, pos);
		break;
	case canID:
		appendUint16(text, nonVolPars.genParas.canID, pos);
		static const uint8_t description38[] = {" (CAN ID number for this BMS unit, if using multi-BMS setups, all BMS' need to have unique CAN ID, uint16_t)\r\n"};
		appendString(text, description38, pos);
		break;
	case duringActive5vOn:
		appendUint16(text, nonVolPars.genParas.duringActive5vOn, pos);
		static const uint8_t description39[] = {" (0/1, if set to 1, keeps 5V regulator on if activeTimer is not expired even if USB, charger or Opto not active, Boolean)\r\n"};
		appendString(text, description39, pos);
		break;
	case canRxRefreshActive:
		appendUint16(text, nonVolPars.genParas.canRxRefreshActive, pos);
		static const uint8_t description40[] = {" (h (Hours), up to how many hours a CAN-frame reception can extend activeTimer, set to 0 to disable, uint16_t)\r\n"};
		appendString(text, description40, pos);
		break;
	default:;
		static const uint8_t description33[] = {" (parameter error)\r\n"};
		appendString(text, description33, pos);
		break;
	}

}

void appendChargingState(uint8_t* text, uint16_t state, uint16_t* pos){

	switch(state){
	case notCharging:;
		static const uint8_t description0[] = {"notCharging:"};
		appendString(text, description0, pos);
		break;
	case chargingStarting:;
		static const uint8_t description1[] = {"chargingStarting:"};
		appendString(text, description1, pos);
		break;
	case startingCurrent:;
		static const uint8_t description6[] = {"startingCurrent:"};
		appendString(text, description6, pos);
		break;
	case charging:;
		static const uint8_t description2[] = {"charging:"};
		appendString(text, description2, pos);
		break;
	case chargingEnd:;
		static const uint8_t description3[] = {"chargingEnd:"};
		appendString(text, description3, pos);
		break;
	case chargerDisconnected:;
		static const uint8_t description4[] = {"chargerDisconnected:"};
		appendString(text, description4, pos);
		break;
	case faultState:;
		static const uint8_t description5[] = {"faultState:"};
		appendString(text, description5, pos);
		break;
	default:
		break;
	}

}

void appendFault(uint8_t* text, uint16_t indexNo, uint16_t* pos){	//fault related strings

	switch(indexNo){
	case fault_lowPackVoltage:;
		static const uint8_t description0[] = {"LOW_PACK_VOLT:"};
		appendString(text, description0, pos);
		break;
	case fault_highPackVoltage:;
		static const uint8_t description1[] = {"HIGH_PACK_VOLT:"};
		appendString(text, description1, pos);
		break;
	case fault_lowCellVoltage0:;
		static const uint8_t description2[] = {"CELL1_LOW_VOLT:"};
		appendString(text, description2, pos);
		break;
	case fault_highCellVoltage0:;
		static const uint8_t description3[] = {"CELL1_HIGH_VOLT:"};
		appendString(text, description3, pos);
		break;
	case fault_lowCellVoltage1:;
		static const uint8_t description4[] = {"CELL2_LOW_VOLT:"};
		appendString(text, description4, pos);
		break;
	case fault_highCellVoltage1:;
		static const uint8_t description5[] = {"CELL2_HIGH_VOLT:"};
		appendString(text, description5, pos);
		break;
	case fault_lowCellVoltage2:;
		static const uint8_t description6[] = {"CELL3_LOW_VOLT:"};
		appendString(text, description6, pos);
		break;
	case fault_highCellVoltage2:;
		static const uint8_t description7[] = {"CELL3_HIGH_VOLT:"};
		appendString(text, description7, pos);
		break;
	case fault_lowCellVoltage3:;
		static const uint8_t description8[] = {"CELL4_LOW_VOLT:"};
		appendString(text, description8, pos);
		break;
	case fault_highCellVoltage3:;
		static const uint8_t description9[] = {"CELL4_HIGH_VOLT:"};
		appendString(text, description9, pos);
		break;
	case fault_lowCellVoltage4:;
		static const uint8_t description10[] = {"CELL5_LOW_VOLT:"};
		appendString(text, description10, pos);
		break;
	case fault_highCellVoltage4:;
		static const uint8_t description11[] = {"CELL5_HIGH_VOLT:"};
		appendString(text, description11, pos);
		break;
	case fault_lowCellVoltage5:;
		static const uint8_t description12[] = {"CELL6_LOW_VOLT:"};
		appendString(text, description12, pos);
		break;
	case fault_highCellVoltage5:;
		static const uint8_t description13[] = {"CELL6_HIGH_VOLT:"};
		appendString(text, description13, pos);
		break;
	case fault_lowCellVoltage6:;
		static const uint8_t description14[] = {"CELL7_LOW_VOLT:"};
		appendString(text, description14, pos);
		break;
	case fault_highCellVoltage6:;
		static const uint8_t description15[] = {"CELL7_HIGH_VOLT:"};
		appendString(text, description15, pos);
		break;
	case fault_lowCellVoltage7:;
		static const uint8_t description16[] = {"CELL8_LOW_VOLT:"};
		appendString(text, description16, pos);
		break;
	case fault_highCellVoltage7:;
		static const uint8_t description17[] = {"CELL8_HIGH_VOLT:"};
		appendString(text, description17, pos);
		break;
	case fault_lowCellVoltage8:;
		static const uint8_t description18[] = {"CELL9_LOW_VOLT:"};
		appendString(text, description18, pos);
		break;
	case fault_highCellVoltage8:;
		static const uint8_t description19[] = {"CELL9_HIGH_VOLT:"};
		appendString(text, description19, pos);
		break;
	case fault_lowCellVoltage9:;
		static const uint8_t description20[] = {"CELL10_LOW_VOLT:"};
		appendString(text, description20, pos);
		break;
	case fault_highCellVoltage9:;
		static const uint8_t description21[] = {"CELL10_HIGH_VOLT:"};
		appendString(text, description21, pos);
		break;
	case fault_lowCellVoltage10:;
		static const uint8_t description22[] = {"CELL11_LOW_VOLT:"};
		appendString(text, description22, pos);
		break;
	case fault_highCellVoltage10:;
		static const uint8_t description23[] = {"CELL11_HIGH_VOLT:"};
		appendString(text, description23, pos);
		break;
	case fault_lowCellVoltage11:;
		static const uint8_t description24[] = {"CELL12_LOW_VOLT:"};
		appendString(text, description24, pos);
		break;
	case fault_highCellVoltage11:;
		static const uint8_t description25[] = {"CELL12_HIGH_VOLT:"};
		appendString(text, description25, pos);
		break;
	case fault_voltageErrorCell0:;
		static const uint8_t description32[] = {"CELL1_VOLT_ERROR:"};
		appendString(text, description32, pos);
		break;
	case fault_voltageErrorCell1:;
		static const uint8_t description33[] = {"CELL2_VOLT_ERROR:"};
		appendString(text, description33, pos);
		break;
	case fault_voltageErrorCell2:;
		static const uint8_t description34[] = {"CELL3_VOLT_ERROR:"};
		appendString(text, description34, pos);
		break;
	case fault_voltageErrorCell3:;
		static const uint8_t description35[] = {"CELL4_VOLT_ERROR:"};
		appendString(text, description35, pos);
		break;
	case fault_voltageErrorCell4:;
		static const uint8_t description36[] = {"CELL5_VOLT_ERROR:"};
		appendString(text, description36, pos);
		break;
	case fault_voltageErrorCell5:;
		static const uint8_t description37[] = {"CELL6_VOLT_ERROR:"};
		appendString(text, description37, pos);
		break;
	case fault_voltageErrorCell6:;
		static const uint8_t description38[] = {"CELL7_VOLT_ERROR:"};
		appendString(text, description38, pos);
		break;
	case fault_voltageErrorCell7:;
		static const uint8_t description39[] = {"CELL8_VOLT_ERROR:"};
		appendString(text, description39, pos);
		break;
	case fault_voltageErrorCell8:;
		static const uint8_t description40[] = {"CELL9_VOLT_ERROR:"};
		appendString(text, description40, pos);
		break;
	case fault_voltageErrorCell9:;
		static const uint8_t description41[] = {"CELL10_VOLT_ERROR:"};
		appendString(text, description41, pos);
		break;
	case fault_voltageErrorCell10:;
		static const uint8_t description42[] = {"CELL11_VOLT_ERROR:"};
		appendString(text, description42, pos);
		break;
	case fault_voltageErrorCell11:;
		static const uint8_t description43[] = {"CELL12_VOLT_ERROR:"};
		appendString(text, description43, pos);
		break;
	case fault_highChargerVoltage:;
		static const uint8_t description26[] = {"HIGH_CHARGER_VOLT:"};
		appendString(text, description26, pos);
		break;
	case fault_highChargingCurrent:;
		static const uint8_t description27[] = {"HIGH_CHG_CURR:"};
		appendString(text, description27, pos);
		break;
	case fault_lowBMStemp:;
		static const uint8_t description28[] = {"LOW_BMS_TEMP:"};
		appendString(text, description28, pos);
		break;
	case fault_highBMStemp:;
		static const uint8_t description29[] = {"HIGH_BMS_TEMP:"};
		appendString(text, description29, pos);
		break;
	case fault_lowNTCtemp:;
		static const uint8_t description30[] = {"LOW_NTC_TEMP:"};
		appendString(text, description30, pos);
		break;
	case fault_highNTCtemp:;
		static const uint8_t description31[] = {"HIGH_NTC_TEMP:"};
		appendString(text, description31, pos);
		break;
	default:
		break;
	}

}

void appendString(uint8_t* text, const uint8_t* string, uint16_t* pos){

	for(uint8_t x=0; x<strlen((const char*)string); x++){
		text[*pos] = string[x];
		*pos += 1;
	}

}

void appendUint16(uint8_t* text, uint16_t number, uint16_t* pos){
	uint8_t temp[5] = {0, 0, 0, 0, 0}, index = 0;

	if( number == 0){
		text[*pos] = '0';
		*pos += 1;
	}
	else{
		while(number > 0){
			temp[index] = number % 10;
			number /= 10;
			index++;
		}

		while(index > 0){
			text[*pos] = '0' + temp[index-1];
			*pos += 1;
			index--;
		}
	}

}

void appendUint64(uint8_t* text, uint64_t number, uint16_t* pos){
	uint8_t temp[20] = {0}, index = 0;

	if( number == 0){
		text[*pos] = '0';
		*pos += 1;
	}
	else{
		while(number > 0){
			temp[index] = number % 10;
			number /= 10;
			index++;
		}

		while(index > 0){
			text[*pos] = '0' + temp[index-1];
			*pos += 1;
			index--;
		}
	}

}

void appendHex32(uint8_t* text, uint32_t number, uint16_t* pos){
	uint8_t temp[10] = {'0', 'x', 0, 0, 0, 0, 0, 0, 0, 0};

	for(uint8_t x=9; x>1; x--){
		uint8_t nTemp = number % 16;
		number = number >> 4;

		if( nTemp >= 10 ){
			temp[x] = 'A' + (nTemp - 10);
		}
		else{
			temp[x] = '0' + nTemp;
		}

	}

	for(uint8_t x=0; x<10; x++){
		text[*pos] = temp[x];
		*pos += 1;
	}

}

void appendStringFromMemory(uint8_t* text, uint8_t* location, uint8_t terminationChar, uint16_t* pos){
	//read characters from specified location until specified termination character is met

	while( *location != terminationChar && *location != 0xFF ){
	text[*pos] = *location;
	*pos += 1;
	location += 1;
	}

}

void appendUID(uint8_t* text, uint16_t* pos){
	uint8_t temp[26] = {'0', 'x', 	0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0};

	//extract the 96-bit unique hardware ID and append it as hexadecimals

	uint32_t number = *(uint32_t*)(0x1FFF7590);

	for(uint8_t x=2; x<10; x+=2){
		uint8_t nTemp2 = number % 16;
		number = number >> 4;
		uint8_t nTemp1 = number % 16;
		number = number >> 4;

		if( nTemp1 >= 10 ){
			temp[x] = 'A' + (nTemp1 - 10);
		}
		else{
			temp[x] = '0' + nTemp1;
		}

		if( nTemp2 >= 10 ){
			temp[x+1] = 'A' + (nTemp2 - 10);
		}
		else{
			temp[x+1] = '0' + nTemp2;
		}
	}

	number = *(uint32_t*)(0x1FFF7590+4);

	for(uint8_t x=10; x<18; x+=2){
		uint8_t nTemp2 = number % 16;
		number = number >> 4;
		uint8_t nTemp1 = number % 16;
		number = number >> 4;

		if( nTemp1 >= 10 ){
			temp[x] = 'A' + (nTemp1 - 10);
		}
		else{
			temp[x] = '0' + nTemp1;
		}

		if( nTemp2 >= 10 ){
			temp[x+1] = 'A' + (nTemp2 - 10);
		}
		else{
			temp[x+1] = '0' + nTemp2;
		}
	}

	number = *(uint32_t*)(0x1FFF7590+8);

	for(uint8_t x=18; x<26; x+=2){
		uint8_t nTemp2 = number % 16;
		number = number >> 4;
		uint8_t nTemp1 = number % 16;
		number = number >> 4;

		if( nTemp1 >= 10 ){
			temp[x] = 'A' + (nTemp1 - 10);
		}
		else{
			temp[x] = '0' + nTemp1;
		}

		if( nTemp2 >= 10 ){
			temp[x+1] = 'A' + (nTemp2 - 10);
		}
		else{
			temp[x+1] = '0' + nTemp2;
		}
	}

	for(uint8_t x=0; x<26; x++){
		text[*pos] = temp[x];
		*pos += 1;
	}

}

void appendFloat(uint8_t* text, float number, uint16_t* pos){	//print to 4 decimal places
	uint8_t temp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, index = 0;

	if( number < 0 ){		//check front sign
		text[*pos] = '-';
		*pos += 1;
		number *= -1;
	}

	uint32_t tempValue = (uint32_t)(number * 10000);


	if( tempValue == 0){
		text[*pos] = '0';
		*pos += 1;
		text[*pos] = '.';
		*pos += 1;
		text[*pos] = '0';
		*pos += 1;
	}
	else{
		while(tempValue > 0){
			temp[index] = tempValue % 10;
			tempValue /= 10;
			index++;
		}

		if(index < 4)
			index = 5;

		while(index > 0){
			if(index == 4){
				text[*pos] = '.';
				*pos += 1;
			}
			text[*pos] = '0' + temp[index-1];
			*pos += 1;
			index--;
		}
	}
}

float readFloat(uint8_t* data, uint32_t* length, uint8_t pos){	//read value from string
	uint8_t exp = 0, negative = 0, numbers = 0;
	uint32_t value = 0;

	pos++;
	if( data[pos] == '-' ){		//check front sign
		negative = 1;
		pos++;
	}
	else if( data[pos] == '+' ){
		negative = 0;
		pos++;
	}


	while( pos < *length ){			//get number data and exponent

		if( data[pos] >= '0' && data[pos] <= '9' ){		//read numbers
			value *= 10;
			value += data[pos] - '0';
			pos++;
			numbers++;
		}
		else if( data[pos] == '.' || data[pos] == ',' ){	//detect decimal point
			pos++;
			exp = numbers;
		}
		else											//stop if some other characters found
			break;

	}

	float temp = (float)value;

	if(exp > 0){
		for( ; numbers > exp; exp++ )		//apply that exponent
			temp /= 10;
	}

	if( negative == 1 )
		temp *= -1;


	return temp;
}
