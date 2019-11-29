/*
 * AuxFunctions.h
 *
 *  Created on: 27.5.2019
 *      Author: Simos MCmuffin
 */

#ifndef AUXFUNCTIONS_H_
#define AUXFUNCTIONS_H_


uint8_t countCells(void);
uint16_t highestCell(uint8_t);
uint16_t lowestCell(uint8_t);
uint8_t usbPowerPresent(void);
uint8_t chargeControl(void);
void hwRequestControl(void);
void statusLed(void);
void changeRunMode(uint8_t);
void jumpToBootloader(void);

void checkChargerVoltageFault(void);
void checkBMStemperatureFault(void);
void checkNTCtemperatureFault(void);
void checkCellVoltageFaults(void);
void checkPackVoltageFault(void);
void checkMaxCurrentFault(void);


#endif /* AUXFUNCTIONS_H_ */
