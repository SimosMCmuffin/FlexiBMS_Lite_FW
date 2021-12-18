/*
	Copyright 2019 - 2021 Simo Sihvonen	"Simos MCmuffin" - simo.sihvonen@gmail.com

	This file is part of the FlexiBMS Lite firmware.

	The FlexiBMS Lite firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The FlexiBMS Lite firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef AUXFUNCTIONS_H_
#define AUXFUNCTIONS_H_

#include <main.h>


uint8_t countCells(void);
uint16_t highestCell(uint8_t);
uint16_t lowestCell(uint8_t);
uint8_t usbPowerPresent(void);
void chargeControl(void);
void sendHeartBeat(void);
void detectCharger(void);
void balanceControl(void);
void hwRequestControl(void);
void statusLed(void);
void changeRunMode();
void jumpToStmBootloader(void);
void jumpToCustomBootloader(void);
void restartFW(void);

void checkChargerVoltageFault(void);
void checkBMStemperatureFault(void);
void checkNTCtemperatureFault(void);
void checkCellVoltageFaults(void);
void checkPackVoltageFault(void);
uint8_t checkMaxCurrentFault(void);

uint8_t getHWversion(void);
void saveNonVolatileParameters(nonVolParameters*, uint8_t);
void packNonVolatileParameters(nonVolParameters*, uint8_t*, int32_t*);
void unpackNonVolatileParameters(nonVolParameters*, uint8_t*);

void updateActiveTimer(void);
void updateStorageTimer(void);

void sortCellsByVoltage(uint8_t*);

uint8_t extractUID(uint8_t);
uint8_t updateOptoState(void);
void changeMSIfreq(uint8_t);
void forceRunMode(uint8_t);
void InitPeripherals(void);
void deInitPeripherals(void);
void trimOscillator(void);

#endif /* AUXFUNCTIONS_H_ */
