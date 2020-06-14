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
void chargeControl(void);
void detectCharger(void);
void balanceControl(void);
void hwRequestControl(void);
void statusLed(void);
void changeRunMode();
void jumpToBootloader(void);

uint8_t checkFaults(void);
void checkChargerVoltageFault(void);
void checkBMStemperatureFault(void);
void checkNTCtemperatureFault(void);
void checkCellVoltageFaults(void);
void checkPackVoltageFault(void);
uint8_t checkMaxCurrentFault(void);

uint8_t getHWversion(void);

void updateActiveTimer(void);

void sortCellsByVoltage(uint8_t*);

uint8_t extractUID(uint8_t);
uint8_t updateOptoState(void);
void changeMSIfreq(uint8_t);
void forceRunMode(uint8_t);
void InitPeripherals(void);
void deInitPeripherals(void);
void trimOscillator(void);

#endif /* AUXFUNCTIONS_H_ */
