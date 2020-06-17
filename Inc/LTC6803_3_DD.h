/*
 * LTC6803_3.h
 *
 *  Created on: 30.5.2018
 *      Author: Simos MCmuffin
 */

#ifndef LTC6803_3_DD_H_
#define LTC6803_3_DD_H_

#include <stdint.h>

struct LTC6803_3{
	uint8_t CFGR[6];				//LTC6803 configuration values
	uint8_t Cells[18];				//raw LTC6803 cell measurement results
	uint16_t cVoltages[12];			//mV, normal cell voltages
	uint16_t oVoltages[12];			//mV, open cell voltages
	uint8_t temperatureVoltages[5];	//raw LTC6803 temperature measurement results
	uint16_t internalTemperature;	//Kelvin, internal temperature
	uint8_t DIAG[2];
	uint8_t PEC;
} LTC_data;

uint8_t SPI_message, SPI_index;

void LTC6803_init(void);

void LTC6803_writeConfiguration(void);
void LTC6803_startCellMeasurements(void);
void LTC6803_startInternalTemperatureMeas(void);
void LTC6803_startOpenCellMeasurements(void);
void LTC6803_readCellMeasurements(void);
void LTC6803_readInternalTemperature(void);

void LTC6803_convertCellVoltages(void);
void LTC6803_convertOpenCellVoltages(void);
void LTC6803_convertTemperatureVoltages(void);

void LTC6803_transactionHandler(uint64_t*);

void LTC6803_setCellDischarge(uint8_t, uint8_t);
uint8_t LTC6803_getCellDischarge(uint8_t cell);
uint16_t LTC6803_getCellVoltage(uint8_t);
uint16_t LTC6803_getTemperature(void);
uint8_t calculatePEC(uint8_t, uint8_t);

void LTC6803_runEnable(void);
void LTC6803_runDisable(void);

#endif /* LTC6803_3_DD_H_ */
