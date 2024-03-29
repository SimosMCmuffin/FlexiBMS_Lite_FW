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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include <usb_device_ST.h>
#include <SPI_LL.h>
#include <LTC6803_3_DD.h>
#include <ADC_LL.h>
#include <CAN_LL.h>
#include <IWDG_LL.h>
#include "AuxFunctions.h"
#include <dStorage_MD.h>
#include <USB_comms_handler_MD.h>

void SystemClock_Config(void);
static void GPIO_Init(void);

extern USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev);

nonVolParameters nonVolPars;
runtimeParameters runtimePars;

int main(void)
{

	initNonVolatiles(&nonVolPars, 0);		//init/load non-volatile parameters
	initRuntimeParameters(&runtimePars);	//init default runtime parameters

	HAL_Init();						//init ST's HAL core

	SystemClock_Config();			//init oscillators

	GPIO_Init();					//Init GPIO in-/outputs
	InitPeripherals();				//function to init all peripherals

	runtimePars.chargerVoltageRequest |= (1 << 0);	//start with charger and pack voltage measuring enabled
	runtimePars.packVoltageRequest |= (1 << 0);

	HAL_Delay(500);

	trimOscillator();

	uint64_t systemTick = HAL_GetTick(), LTC6803tick = HAL_GetTick();
	runtimePars.activeTick =  HAL_GetTick() + ( nonVolPars.genParas.stayActiveTime * __TIME_HOUR_TICKS ) + 5000;


	while (1)
	{

		if( systemTick + 25 <= HAL_GetTick() ){		//go in here at max every 25ms
			systemTick = HAL_GetTick();

			statusLed();		//Control status LED

			usbPowerPresent();		//check if 5V is detected from the USB connector
			updateOptoState();		//read state of the Opto-isolator
			updateActiveTimer();	//update activeTimer flags
			updateStorageTimer();	//update storageDischargeTimer flags
			changeRunMode();		//change running modes based on some flags

		}

		saveNonVolatileParameters(&nonVolPars, 0);

		hwRequestControl();		//disable/enable 5V buck and ADC channels based on software requests

		detectCharger();
		sendHeartBeat();
		chargeControl();	//charging algorithm, everything charging control related is done through here

		USB_checkForNewMessages();		//check for new messages from USB and send State printout if enabled

		if( LTC6803tick <= HAL_GetTick() ){		//all LTC6803 SPI communications are started from this tick function
			LTC6803tick = HAL_GetTick() + 5;
			LTC6803_transactionHandler(&LTC6803tick);
		}


		if( runtimePars.currentRunMode == 0 ){	//special loop for low-power running
			uint8_t cycles = 0;

			while( runtimePars.currentRunMode == 0 ){
				//most of time do nothing, but check opto and USB
				//occasionally enable ADC for couple conversion cycles to update analog readings
				cycles++;

				if( cycles == 30 ){
					cycles = 0;
					//__GREEN_LED_ON;
					runtimePars.chargerVoltageRequest |= (1 << 0);
					runtimePars.packVoltageRequest |= (1 << 0);
					if( nonVolPars.genParas.canWakeUp == 1 ){	//If CAN activity allowed to wake-up the BMS from sleep
						GPIOB->MODER &= ~(3 << 16);		//Set CAN1_RX pin as input
						runtimePars.buck5vRequest |= (1 << always5vRequest);
					}
					hwRequestControl();		//disable/enable 5V buck and ADC channels based on software requests
					HAL_Delay(20);

					ADC_init();						//init ADC low level HW
					ADC_start();

					uint8_t state = !!(GPIOB->IDR & (1 << 8));
					uint64_t startTick = HAL_GetTick();
					uint16_t changes = 0;
					while( startTick+120 > HAL_GetTick() ){
						if( state != !!(GPIOB->IDR & (1 << 8)) ){
							state = !!(GPIOB->IDR & (1 << 8));
							changes++;
						}

					}

					if(changes > 3){
						//__RED_LED_OFF;
						runtimePars.canActivity = 1;
					}
					else{
						//__RED_LED_ON;
						runtimePars.canActivity = 0;
					}

					//HAL_Delay(200);
					ADC_stop();
					ADC_deInit();
					runtimePars.chargerVoltageRequest &= ~(1 << 0);
					runtimePars.packVoltageRequest &= ~(1 << 0);
					if( nonVolPars.genParas.canWakeUp == 1 )
						runtimePars.buck5vRequest &= ~(1 << always5vRequest);
					hwRequestControl();		//disable/enable 5V buck and ADC channels based on software requests
					//__GREEN_LED_OFF;
				}

				usbPowerPresent();		//check if 5V is detected from the USB connector
				updateOptoState();		//read state of the Opto-isolator
				updateActiveTimer();	//update activeTimer flags
				runtimePars.canActivity = 0;
				updateStorageTimer();	//update storageDischargeTimer flags
				detectCharger();		//check if charger connected
				changeRunMode();		//change running modes based on some flags
				HAL_Delay(160);			//wait 160ms
			}

			HAL_Delay(320);
		}

	}
}


/** System Clock Configuration
 */
void SystemClock_Config(void){

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
	{
		Error_Handler();
	}
}
/** Pinout Configuration
 */
static void GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIOB->MODER = GPIOB->MODER & ~(3 << 0);	//5V_BUCK_ENABLE
	//__ENABLE_5V_BUCK;
	//__DISABLE_5V_BUCK;

	GPIOB->MODER &= ~((3 << 24) | (3 << 4));		//PMON_ENABLE & BAT_VOLTAGE_ENABLE
	GPIOB->MODER |= (1 << 24) | (1 << 4);
	//__ENABLE_BAT_VOLTAGE;
	//__ENABLE_CHARGER_VOLTAGE;

	GPIOB->MODER &= ~( (3 << 26) | (3 << 28) | (3 << 30) );		//Status led pins PB13, PB14, PB15
	GPIOB->MODER |=  (1 << 26) | (1 << 28) | (1 << 30);
	GPIOB->BSRR |=  (1 << 13) | (1 << 14) | (1 << 15);


	GPIOA->MODER &= ~(3 << 16);						//CHARGE_ENABLE
	GPIOA->MODER |= (1 << 16);
	__DISABLE_CHG;

	GPIOB->MODER = (GPIOB->MODER & ~(3 << 14));		//USB_DETECT
	GPIOB->PUPDR |= (2 << 14);						//enable PB7/USB_DETECT pull-down resistor

	GPIOA->MODER &= ~(3 << 18);					//OPTO_enable to input mode

}
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
