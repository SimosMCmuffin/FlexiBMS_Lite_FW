#include <stdio.h>
#include <string.h>
#include "commands.h"
#include "buffer.h"
#include "config.h"
#include "datatypes.h"
#include "ADC_LL.h"
#include "LTC6803_3_DD.h"

//extern runtimeParameters runtimePars;

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

		buffer_append_float32(send_buffer, ADC_convertedResults[batteryVoltage], 1e3, &ind);  // FIXME: packVoltage
		buffer_append_float32(send_buffer, ADC_convertedResults[chargeCurrent], 1e3, &ind);  // FIXME: packCurrent

		send_buffer[ind++] = 50;  // FIXME: SoC

		buffer_append_float32(send_buffer, 4.0, 1e3, &ind);  // FIXME: cellVoltageHigh
		buffer_append_float32(send_buffer, 3.9, 1e3, &ind);  // FIXME: cellVoltageAverage
		buffer_append_float32(send_buffer, 3.8, 1e3, &ind);  // FIXME: cellVoltageLow
		buffer_append_float32(send_buffer, 0.2, 1e3, &ind);  // FIXME: cellVoltageMisMatch

		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // loCurrentLoadVoltage
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // loCurrentLoadCurrent
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // hiCurrentLoadVoltage
		buffer_append_float16(send_buffer, 0.0, 1e2, &ind);  // hiCurrentLoadCurrent
		buffer_append_float16(send_buffer, ADC_convertedResults[chargeCurrent], 1e2, &ind);  // FIXME: auxVoltage
		buffer_append_float16(send_buffer, ADC_convertedResults[chargeCurrent], 1e2, &ind);  // FIXME: auxCurrent

		// TODO: where does ADC_convertedResults[mcuInternalTemp] fit?
		// TODO: convert temperatures to Celsius
		buffer_append_float16(send_buffer, ADC_convertedResults[externalTemp], 1e1, &ind);  // FIXME: tempBatteryHigh
		buffer_append_float16(send_buffer, ADC_convertedResults[externalTemp], 1e1, &ind);  // FIXME: tempBatteryAverage
		buffer_append_float16(send_buffer, LTC6803_getTemperature(), 1e1, &ind);  // FIXME: tempBMSHigh
		buffer_append_float16(send_buffer, LTC6803_getTemperature(), 1e1, &ind);  // FIXME: tempBMSAverage

		// TODO: use runtimePars.chargingState and runtimePars.charging
		send_buffer[ind++] = (uint8_t) OP_STATE_INIT;  // FIXME: operationalState
		send_buffer[ind++] = 0;  // FIXME: chargeBalanceActive (Indicator for charging)

		send_buffer[ind++] = 0;  // Future faultstate

		send_buffer[ind++] = CAN_ID;
		commands_send_packet(send_buffer, ind);

		break;
	case COMM_GET_BMS_CELLS:
		send_buffer[ind++] = COMM_GET_BMS_CELLS;

		send_buffer[ind++] = MAX_CELLS;
		for (int i = 0; i < MAX_CELLS; i++) {
			// TODO: multiply by -1 if balancing the cell
			buffer_append_uint16(send_buffer, LTC6803_getCellVoltage(i), &ind);
		}

		send_buffer[ind++] = CAN_ID;
		commands_send_packet(send_buffer, ind);
		break;
	default:
		break;
	}
}
