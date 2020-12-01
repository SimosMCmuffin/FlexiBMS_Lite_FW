#include <stdio.h>
#include <string.h>
#include "commands.h"
#include "buffer.h"
#include "config.h"
#include "datatypes.h"
#include "main.h"
#include "AuxFunctions.h"
#include "ADC_LL.h"
#include "LTC6803_3_DD.h"
#include "dStorage_MD.h"

#define CELSIUS_TO_KELVIN_1E1 2732  // add 273.2 to Celsius to get Kelvin

extern runtimeParameters runtimePars;
extern nonVolParameters nonVolPars;

static void(* volatile send_func)(uint8_t to, uint8_t *data, unsigned int len) = 0;

/**
 * Send a packet using the set send function.
 */
void commands_send_packet(uint8_t to, uint8_t *data, unsigned int len) {
	if (send_func) {
		send_func(to, data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 */
void commands_process_packet(uint8_t from, uint8_t *data, unsigned int len,
		void(*reply_func)(uint8_t to, uint8_t *data, unsigned int len)) {

	if (!len) {
		return;
	}

	send_func = reply_func;

	int32_t ind = 0;
	uint8_t send_buffer[128];

	COMM_PACKET_ID packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_FW_VERSION:
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;
		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;
		memcpy(send_buffer + ind, STM32_UUID_8, 12);
		ind += 12;

		commands_send_packet(from, send_buffer, ind);
		break;
	case COMM_GET_VALUES:
		send_buffer[ind++] = COMM_GET_VALUES;

		buffer_append_int32(send_buffer, (int32_t) ADC_convertedResults[batteryVoltage], &ind);  // packVoltage
		buffer_append_int32(send_buffer, (int32_t) ADC_convertedResults[chargeCurrent], &ind);  // packCurrent

		send_buffer[ind++] = 50;  // FIXME: SoC

		uint16_t cell_high = highestCell(nonVolPars.chgParas.packCellCount);
		uint16_t cell_low = lowestCell(nonVolPars.chgParas.packCellCount);
		buffer_append_int32(send_buffer, cell_high, &ind);  // cellVoltageHigh
		buffer_append_int32(send_buffer, (cell_high + cell_low) / 2, &ind);  // cellVoltageAverage (very rough approximation)
		buffer_append_int32(send_buffer, cell_low, &ind);  // cellVoltageLow
		buffer_append_int32(send_buffer, cell_high - cell_low, &ind);  // cellVoltageMisMatch

		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // loCurrentLoadVoltage
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // loCurrentLoadCurrent
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // hiCurrentLoadVoltage
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // hiCurrentLoadCurrent
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // auxVoltage
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // auxCurrent

		buffer_append_int16(send_buffer, (int16_t) (ADC_convertedResults[externalTemp] * 1e1 - CELSIUS_TO_KELVIN_1E1), &ind);  // tempBatteryHigh
		buffer_append_int16(send_buffer, (int16_t) (ADC_convertedResults[externalTemp] * 1e1 - CELSIUS_TO_KELVIN_1E1), &ind);  // tempBatteryAverage

		uint16_t mcu_temp = ADC_convertedResults[mcuInternalTemp];
		uint16_t ltc_temp = LTC6803_getTemperature();
		uint16_t bms_temp_high = (mcu_temp > ltc_temp) ? mcu_temp : ltc_temp;
		uint16_t bms_temp_avg = (mcu_temp + ltc_temp) / 2;
		buffer_append_int16(send_buffer, (int16_t) (bms_temp_high * 1e1 - CELSIUS_TO_KELVIN_1E1), &ind);  // tempBMSHigh
		buffer_append_int16(send_buffer, (int16_t) (bms_temp_avg * 1e1 - CELSIUS_TO_KELVIN_1E1), &ind);  // tempBMSAverage

		// TODO: use runtimePars.chargingState and runtimePars.charging
		send_buffer[ind++] = (uint8_t) OP_STATE_INIT;  // FIXME: operationalState
		send_buffer[ind++] = 0;  // FIXME: chargeBalanceActive (Indicator for charging)

		send_buffer[ind++] = 0;  // Future faultstate

		send_buffer[ind++] = nonVolPars.genParas.canID;
		commands_send_packet(from, send_buffer, ind);

		break;
	case COMM_GET_BMS_CELLS:
		send_buffer[ind++] = COMM_GET_BMS_CELLS;

		send_buffer[ind++] = nonVolPars.chgParas.packCellCount;
		for (int i = 0; i < nonVolPars.chgParas.packCellCount; i++) {
			int sign = LTC6803_getCellDischarge(i) ? -1 : 1;
			buffer_append_int16(send_buffer, (int16_t) (LTC6803_getCellVoltage(i) * sign), &ind);
		}

		send_buffer[ind++] = nonVolPars.genParas.canID;
		commands_send_packet(from, send_buffer, ind);
		break;
	case COMM_SET_BMS_CONF:
		nonVolPars.chgParas.packCellCount = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.maxChgCurr = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.termCurr = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.minCellVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.maxCellVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.minChgVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.maxChgVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.minPackVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.maxPackVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.termCellVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.termPackVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.cellBalVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.cellDiffVolt = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.minNTCtemp = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.maxNTCtemp = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.minBMStemp = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.maxBMStemp = buffer_get_uint16(data, &ind);
		nonVolPars.chgParas.refreshWaitTime = buffer_get_uint16(data, &ind);
		for (size_t i = 0; i < sizeof(nonVolPars.adcParas.ADC_chan_gain) / sizeof(nonVolPars.adcParas.ADC_chan_gain[0]); ++i) {
			nonVolPars.adcParas.ADC_chan_gain[i] = buffer_get_float32_auto(data, &ind);
		}
		for (size_t i = 0; i < sizeof(nonVolPars.adcParas.ADC_chan_offset) / sizeof(nonVolPars.adcParas.ADC_chan_offset[0]); ++i) {
			nonVolPars.adcParas.ADC_chan_offset[i] = buffer_get_float32_auto(data, &ind);
		}
		nonVolPars.adcParas.extNTCbetaValue = buffer_get_uint16(data, &ind);
		nonVolPars.adcParas.AdcOversampling = buffer_get_uint16(data, &ind);
		nonVolPars.genParas.stayActiveTime = buffer_get_uint16(data, &ind);
		nonVolPars.genParas.alwaysBalancing = data[ind++];
		nonVolPars.genParas.always5vRequest = data[ind++];
		nonVolPars.chgParas.balTempRatio = buffer_get_uint16(data, &ind);
		nonVolPars.genParas.storageCellVoltage = buffer_get_uint16(data, &ind);
		nonVolPars.genParas.timeToStorageDischarge = buffer_get_uint16(data, &ind);
		nonVolPars.genParas.canActivityTick = data[ind++];
		nonVolPars.genParas.canID = data[ind++];
		nonVolPars.genParas.duringActive5vOn = data[ind++];
		nonVolPars.genParas.canRxRefreshActive = buffer_get_uint16(data, &ind);
		ind = 0;
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = nonVolPars.genParas.canID;
		commands_send_packet(from, send_buffer, ind);
		break;
	case COMM_GET_BMS_CONF:
		send_buffer[ind++] = packet_id;
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.packCellCount, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.maxChgCurr, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.termCurr, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.minCellVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.maxCellVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.minChgVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.maxChgVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.minPackVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.maxPackVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.termCellVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.termPackVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.cellBalVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.cellDiffVolt, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.minNTCtemp, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.maxNTCtemp, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.minBMStemp, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.maxBMStemp, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.refreshWaitTime, &ind);
		for (size_t i = 0; i < sizeof(nonVolPars.adcParas.ADC_chan_gain) / sizeof(nonVolPars.adcParas.ADC_chan_gain[0]); ++i) {
			buffer_append_float32_auto(send_buffer, nonVolPars.adcParas.ADC_chan_gain[i], &ind);
		}
		for (size_t i = 0; i < sizeof(nonVolPars.adcParas.ADC_chan_offset) / sizeof(nonVolPars.adcParas.ADC_chan_offset[0]); ++i) {
			buffer_append_float32_auto(send_buffer, nonVolPars.adcParas.ADC_chan_offset[i], &ind);
		}
		buffer_append_uint16(send_buffer, nonVolPars.adcParas.extNTCbetaValue, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.adcParas.AdcOversampling, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.genParas.stayActiveTime, &ind);
		send_buffer[ind++] = nonVolPars.genParas.alwaysBalancing;
		send_buffer[ind++] = nonVolPars.genParas.always5vRequest;
		buffer_append_uint16(send_buffer, nonVolPars.chgParas.balTempRatio, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.genParas.storageCellVoltage, &ind);
		buffer_append_uint16(send_buffer, nonVolPars.genParas.timeToStorageDischarge, &ind);
		send_buffer[ind++] = nonVolPars.genParas.canActivityTick;
		send_buffer[ind++] = nonVolPars.genParas.canID;
		send_buffer[ind++] = nonVolPars.genParas.duringActive5vOn;
		buffer_append_uint16(send_buffer, nonVolPars.genParas.canRxRefreshActive, &ind);
		commands_send_packet(from, send_buffer, ind);
		break;
	case COMM_STORE_BMS_CONF:
		saveNonVolatileParameters(&nonVolPars);
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = nonVolPars.genParas.canID;
		commands_send_packet(from, send_buffer, ind);
		break;
	case COMM_REBOOT:
		restartFW();
		break;
	default:
		break;
	}
}
