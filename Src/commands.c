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

#define CELSIUS_TO_KELVIN_1E1 2732  // add 273.2 to Celsius to get Kelvin

extern runtimeParameters runtimePars;
extern nonVolParameters nonVolPars;

static void(* volatile send_func)(unsigned char *data, unsigned int len) = 0;

/**
 * Send a packet using the set send function.
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 */
void commands_process_packet(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {

	if (!len) {
		return;
	}

	send_func = reply_func;

	int32_t ind = 0;
	uint8_t send_buffer[128];

	COMM_PACKET_ID packet_id = data[0];

	switch (packet_id) {
	case COMM_FW_VERSION:
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;
		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;
		//memcpy(send_buffer + ind, STM32_UUID_8, 12);
		//ind += 12;

		commands_send_packet(send_buffer, ind);
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

		send_buffer[ind++] = CAN_ID;
		commands_send_packet(send_buffer, ind);

		break;
	case COMM_GET_BMS_CELLS:
		send_buffer[ind++] = COMM_GET_BMS_CELLS;

		send_buffer[ind++] = nonVolPars.chgParas.packCellCount;
		for (int i = 0; i < nonVolPars.chgParas.packCellCount; i++) {
			int sign = LTC6803_getCellDischarge(i) ? -1 : 1;
			buffer_append_int16(send_buffer, (int16_t) (LTC6803_getCellVoltage(i) * sign), &ind);
		}

		send_buffer[ind++] = CAN_ID;
		commands_send_packet(send_buffer, ind);
		break;
	default:
		break;
	}
}
