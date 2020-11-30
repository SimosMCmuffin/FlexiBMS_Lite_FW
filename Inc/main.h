/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define __ENABLE_5V_BUCK ( GPIOB->PUPDR = (GPIOB->PUPDR & ~(3 << 0)) | (1 << 0) )
#define __DISABLE_5V_BUCK ( GPIOB->PUPDR = (GPIOB->PUPDR & ~(3 << 0)) | (1 << 1) )

#define __ENABLE_CHG ( GPIOA->BSRR = (1 << 8) )
#define __DISABLE_CHG ( GPIOA->BSRR = (1 << 24) )

#define __ENABLE_BAT_VOLTAGE ( GPIOB->BSRR = (1 << 2) )
#define __DISABLE_BAT_VOLTAGE ( GPIOB->BSRR = (1 << 18) )

#define __ENABLE_CHARGER_VOLTAGE ( GPIOB->BSRR = (1 << 12) )
#define __DISABLE_CHARGER_VOLTAGE ( GPIOB->BSRR = (1 << 28) )

#define __BLUE_LED_ON ( GPIOB->BSRR |= (1 << 31) )
#define __BLUE_LED_OFF ( GPIOB->BSRR |= (1 << 15) )

#define __GREEN_LED_ON ( GPIOB->BSRR |= (1 << 30) )
#define __GREEN_LED_OFF ( GPIOB->BSRR |= (1 << 14) )

#define __RED_LED_ON ( GPIOB->BSRR |= (1 << 29) )
#define __RED_LED_OFF ( GPIOB->BSRR |= (1 << 13) )

#define __TIME_HOUR_TICKS ( 3600000 )

#include "stm32l4xx_hal.h"

typedef enum
{
	fault_lowPackVoltage = 0,
	fault_highPackVoltage,
	fault_lowCellVoltage0,
	fault_highCellVoltage0,
	fault_lowCellVoltage1,
	fault_highCellVoltage1,
	fault_lowCellVoltage2,
	fault_highCellVoltage2,
	fault_lowCellVoltage3,
	fault_highCellVoltage3,
	fault_lowCellVoltage4,
	fault_highCellVoltage4,
	fault_lowCellVoltage5,
	fault_highCellVoltage5,
	fault_lowCellVoltage6,
	fault_highCellVoltage6,
	fault_lowCellVoltage7,
	fault_highCellVoltage7,
	fault_lowCellVoltage8,
	fault_highCellVoltage8,
	fault_lowCellVoltage9,
	fault_highCellVoltage9,
	fault_lowCellVoltage10,
	fault_highCellVoltage10,
	fault_lowCellVoltage11,
	fault_highCellVoltage11,
	fault_voltageErrorCell0,
	fault_voltageErrorCell1,
	fault_voltageErrorCell2,
	fault_voltageErrorCell3,
	fault_voltageErrorCell4,
	fault_voltageErrorCell5,
	fault_voltageErrorCell6,
	fault_voltageErrorCell7,
	fault_voltageErrorCell8,
	fault_voltageErrorCell9,
	fault_voltageErrorCell10,
	fault_voltageErrorCell11,
	fault_highChargerVoltage,
	fault_highChargingCurrent,
	fault_lowBMStemp,
	fault_highBMStemp,
	fault_lowNTCtemp,
	fault_highNTCtemp,
	fault_numberOfElements
}_fault_ID;

typedef enum
{
	usb5vRequest = 0,
	charging5vRequest,
	ntc5vRequest,
	led5vRequest,
	balancing5vRequest,
	always5vRequest,
	opto5vRequest,
	active5vRequest
}_5vRequest_ID;

enum
{
	notCharging = 0,
	chargingStarting,
	startingCurrent,
	charging,
	chargingEnd,
	chargerDisconnected,
	faultState
}_chargingState_ID;

typedef struct _chargingParameters {
	uint16_t packCellCount;	//# number of series cells in the battery pack
	uint16_t maxChgCurr;	//mA (milliAmps), maximum current allowed to flow to battery
	uint16_t termCurr;		//mA (milliAmps), stop charging when current drops below this

	uint16_t minCellVolt;	//mV (milliVolts), minimum allowable cell voltage, no charging allowed if cell voltage below
	uint16_t maxCellVolt;	//mV (milliVolts), maximum allowable cell voltage, no charging allowed if cell voltage above
	uint16_t minChgVolt;	//mV (milliVolts), minimum allowable charger voltage, no charging allowed if charger voltage below
	uint16_t maxChgVolt;	//mV (milliVolts), minimum allowable charger voltage, no charging allowed if charger voltage above
	uint16_t minPackVolt;	//mV (milliVolts), minimum allowable pack voltage, no charging allowed if cell voltage below
	uint16_t maxPackVolt;	//mV (milliVolts), maximum allowable pack voltage, no charging allowed if cell voltage above

	uint16_t termCellVolt;	//mV (milliVolts), don't allow any cell to go above this voltage
	uint16_t termPackVolt;	//mV (milliVolts), don't allow pack to go above this voltage
	uint16_t cellBalVolt;	//mV (milliVolts), allow balancing once a cell goes above this voltage
	uint16_t cellDiffVolt;	//mV (milliVolts), maximum allowed voltage difference between cell groups, balance if difference bigger

	uint16_t balTempRatio;	//K (Kelvin), balancing temperature ratio of how many simultaneous resistors can be on at any time

	uint16_t minNTCtemp;		//K (Kelvin), if NTC probe enabled, the minimum temperature above which charging is allowed
	uint16_t maxNTCtemp;		//K (Kelvin), if NTC probe enabled, the maximum temperature below which charging is allowed
	uint16_t minBMStemp;		//K (Kelvin), PCB temperature, the minimum temperature above which charging is allowed
	uint16_t maxBMStemp;		//K (Kelvin), PCB temperature, the maximum temperature below which charging is allowed

	uint16_t refreshWaitTime;	//s (seconds), wait time
} chargingParameters;

typedef struct _ADCparameters {
	float ADC_chan_gain[5];		//gain adjustments for ADC_channels
	float ADC_chan_offset[5];	//offset adjustments ADC_channels
	uint16_t extNTCbetaValue;	//external NTC sensors beta value
	uint16_t AdcOversampling;	//oversampling setting for ADC
} ADCparameters;

typedef struct _generalParameters {
	uint16_t stayActiveTime;	//h (Hours), how long to stay in active mode
	uint8_t alwaysBalancing;	//1 or 0, allow balancing even when not charging
	uint8_t always5vRequest;	//1 or 0, force 5V buck always on
	uint8_t duringActive5vOn;	//1 or 0, whether to keep 5V regulator on during active time, even if USB, charger or Opto not active

	uint16_t storageCellVoltage;	//mV (milliVolts), what voltage to discharge cells if storage voltage discharge enabled
	uint16_t timeToStorageDischarge;	//h (hours), how long to wait after active state to start discharging cells to storage voltage

	uint8_t canActivityTick;	//1 or 0, ticks the status led in magenta when can packets are received
	uint8_t canID;				//canID that the BMS uses to recognize as itself
	uint16_t canRxRefreshActive;	//h (hours),Receiving CAN messages refresh activeTimer up to this length, 0 to disable
} generalParameters;

typedef struct _nonVolParameters {
	uint16_t FW_version;
	chargingParameters chgParas;
	ADCparameters adcParas;
	generalParameters genParas;
} nonVolParameters;

typedef struct _runtimeParameters {
	uint16_t statePrintout;
	uint16_t statusTick;
	uint16_t ADCrunState;
	uint16_t LTC6803runState;
	uint16_t usbConnected;
	uint16_t chargerConnected;
	uint16_t optoActive;
	uint16_t activeTimerState;
	uint16_t storageTimerState;
	uint16_t storageDischarged;
	uint16_t chargingState;
	uint16_t currentRunMode;
	uint64_t activeFaults;
	uint64_t latchedFaults;

	uint16_t buck5vEnabled;
	uint16_t buck5vRequest;
	uint16_t packVoltageEnabled;
	uint16_t packVoltageRequest;
	uint16_t chargerVoltageEnabled;
	uint16_t chargerVoltageRequest;

	uint16_t charging;
	uint16_t balancing;

	uint64_t activeTick;
	uint64_t storageTick;

} runtimeParameters;

void Error_Handler(void);
extern void initNonVolatiles(nonVolParameters*, uint8_t);
extern void initRuntimeParameters(runtimeParameters*);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
