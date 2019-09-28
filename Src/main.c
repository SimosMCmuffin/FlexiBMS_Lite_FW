/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2019 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *ä
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "SPI.h"
#include "LTC6803_3.h"
#include "ADC.h"
#include "CAN.h"
#include "AuxFunctions.h"
#include "dStorage.h"
#include "USB_comms_handler.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void changeRunMode(void);

extern USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev);


volatile uint8_t runMode = 0;

nonVolParameters nonVolPars;
runtimeParameters runtimePars;

int main(void)
{
	initNonVolatiles(&nonVolPars, 0);
	initRuntimeParameters(&runtimePars);

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_USB_DEVICE_Init();
	SPI1_init();
	LTC6803_init();
	ADC_init();
	CAN1_init();

	USBD_DeInit(&hUsbDeviceFS);		//Stop usb service

	/*
	//USART test configuration
	RCC->APB1ENR1 |= (1 << 18);		//enable USART3 bus clock
	USART3->BRR = 278;				//bus clock 16MHz -> 57600 baud
	USART3->CR1 |= (1 << 3);		//enable transmitter
	USART3->CR1 |= (1 << 0);		//enable USART3
	 */

	/*
	USBD_Stop(&hUsbDeviceFS);	//Stop usb service
	RCC->CRRCR &= ~(1 << 0);	//disable HSI48 clock, used for USB
	RCC->APB1ENR1 &= ~(1 << 26);	//disable USB FS bus clock


	FLASH->ACR = (FLASH->ACR & ~(7 << 0)) | (2 << 0);	//Increase flash memory wait states
	PWR->CR1 = (PWR->CR1 & ~(3 << 9)) | (2 << 9);		//set voltage scaling to range 2 (medium frequency)

	while ( !(RCC->CR & RCC_CR_MSIRDY));
	RCC->CR &= ~(0xF << 4);								//set MSI clock frequency to 100kHz
	RCC->CR |= (1 << 3);								//activate new MSI clock frequency


	//PWR->CR1 |= (1 << 1) | (1 << 8);		//Go in low-power mode to stop 2. Enable RTC and backup domain registers
	//RCC->APB1ENR1 |= (1 << 10);				//enable RTC bus clock
	//RCC->BDCR |= (2 << 8) | (1 << 15);			//LSI selected as a clock source for RTC. RTC clock enabled
	//RCC->CSR |= (1 << 0);					//enable LSI (32kHz)
	//while ( !(RCC->CSR & RCC_CSR_LSIRDY));	//wait for LSI start-up

	PWR->CR1 |= (1 << 14);					//Go into low-power run mode

	while(1);
	*/


	/*
	SCB->SCR = (SCB->SCR & ~((1 << 1) | (1 << 4))) | (1 << 2);	//clear SEVONPEND & SLEEPONEXIT, set DEEPSLEEP

	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	//RTC->CR |= (1 << 0);		//set Wake-up timer prescaler to 8x
	RTC->CR |= (1 << 10);		//enable wake-up timer

	//HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
	//asm("WFI");
	 */

	//GPIOA->BSRR |= (1 << 8);

	//uint8_t CANdata[8] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
	//uint32_t CANid = 0x55, CANlength = 8;

	//CAN1_transmit(CANid, CANdata, CANlength);

	if( runtimePars.ADCrunState == 0 ){
		ADC_setupSequence();
		ADC_runSequence();
	}

	uint64_t systemTick = HAL_GetTick(), LTC6803tick = HAL_GetTick(), LTC6803resetTick = HAL_GetTick();
	uint8_t LTC6803commsState = 0;

	while (1)
	{

		if( systemTick + 20 <= HAL_GetTick() ){
			systemTick = HAL_GetTick();

			usbPowerPresent();

		}

		statusLed();

		chargeControl();

		checkForNewMessages();

//		if( ADC_conversionNumber == 5 )
//			ADC_runSequence();

		if( LTC6803tick <= HAL_GetTick() ){		//all LTC6803 SPI communications are started from this tick function
			LTC6803tick = HAL_GetTick() + 5;

			if( SPI_message == 0 ){
				LTC6803resetTick = HAL_GetTick();

				switch( LTC6803commsState ){

				case 0:
					LTC6803_convertTemperatureVoltages();
					LTC6803_writeConfiguration();
					LTC6803commsState = 1;
					break;

				case 1:
					LTC6803_startCellMeasurements();
					LTC6803tick = HAL_GetTick() + 20;
					LTC6803commsState = 2;
					break;

				case 2:
					LTC6803_readCellMeasurements();
					LTC6803commsState = 3;
					break;

				case 3:
					LTC6803_convertCellVoltages();
					LTC6803_startOpenCellMeasurements();
					LTC6803tick = HAL_GetTick() + 20;
					LTC6803commsState = 4;
					break;

				case 4:
					LTC6803_readCellMeasurements();
					LTC6803commsState = 5;
					break;

				case 5:
					LTC6803_convertOpenCellVoltages();
					LTC6803_startInternalTemperatureMeas();
					LTC6803commsState = 6;
					break;

				case 6:
					LTC6803_readInternalTemperature();
					LTC6803commsState = 0;
					break;

				default:
					break;
				}
			}
		}


		/*
		if( !!(RTC->ISR & (1 << 10)) ){		//check if wake-up timer set
			asm("nop");
			asm("nop");
			RTC->ISR &= ~(1 << 10);			//reset wake-up timer flag

		}
		 */

	}

}


void changeRunMode(void){

	switch(runMode){

	case 0: 			//LOW-POWER STANDBY MODE
		if( usbPowerPresent() ){		//opto_enable found || charger_detected)
			PWR->CR1 &= ~(1 << 14);			//disable low-power run mode
			while( !!(PWR->SR2 & (1 << 9)) );	//wait for voltage regulator to settle
			while ( !(RCC->CR & RCC_CR_MSIRDY));
			RCC->CR = (RCC->CR & ~(0xF << 4)) | (8 << 4);	//set MSI clock frequency to 16MHz
			RCC->CR |= (1 << 3);					//activate new MSI clock frequency


			__ENABLE_5V_BUCK;			//turn on 5V buck SMPS
			//TODO ENABLE PERIPHERALS

			runMode = 1;
		}
		break;

	case 1: 			//RUN MODE
		if( usbPowerPresent() ){
			PWR->CR1 = (PWR->CR1 & ~(3 << 9)) | (1 << 9);		//set voltage scaling to range 1 (high frequency)
			while( !!(PWR->SR2 & (1 << 10)) );					//wait for voltage regulator to settle
			FLASH->ACR = (FLASH->ACR & ~(7 << 0)) | (0 << 0);	//decrease flash memory wait states

			RCC->CRRCR |= (1 << 0);			//enable HSI48 clock, used for USB
			while( !(RCC->CRRCR & (1 << 1)) );	//wait for HSI48 to stabilize
			RCC->APB1ENR1 |= (1 << 26);		//enable USB FS bus clock
			USBD_Start(&hUsbDeviceFS);		//Start usb service

			runMode = 2;
		}
		else if( 0 ){	//opto_enable not found && charger not detected
			while ( !(RCC->CR & RCC_CR_MSIRDY));
			RCC->CR &= ~(0xF << 4);								//set MSI clock frequency to 100kHz
			RCC->CR |= (1 << 3);								//activate new MSI clock frequency

			//PWR->CR1 |= (1 << 14);					//Go into low-power run mode

			//TODO DISABLE PERIPHERALS
			__DISABLE_5V_BUCK;			//turn off 5V buck SMPS

			runMode = 0;
		}
		break;

	case 2:				//RUN MODE WITH USB
		if(1){

		}
		break;

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	//HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	//HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Pinout Configuration
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIOB->MODER = GPIOB->MODER & ~(3 << 0);	//5V_BUCK_ENABLE
	__ENABLE_5V_BUCK;
	//__DISABLE_5V_BUCK;

	/*
	GPIOB->MODER &= ~((3 << 20) | (3 << 22));		//USART3
	GPIOB->MODER |= (2 << 20) | (2 << 22);
	GPIOB->AFR[1] |= (7 << 8) | (7 << 12);
	*/

	GPIOB->MODER &= ~((3 << 24) | (3 << 4));		//PMON_ENABLE & BAT_VOLTAGE_ENABLE
	GPIOB->MODER |= (1 << 24) | (1 << 4);
	__ENABLE_BAT_VOLTAGE;
	__ENABLE_CHARGER_VOLTAGE;

	GPIOB->MODER &= ~( (3 << 26) | (3 << 28) | (3 << 30) );		//Status led pins PB13, PB14, PB15
	GPIOB->MODER |=  (1 << 26) | (1 << 28) | (1 << 30);
	GPIOB->BSRR |=  (1 << 13) | (1 << 14) | (1 << 15);


	GPIOA->MODER &= ~(3 << 16);						//CHARGE_ENABLE
	GPIOA->MODER |= (1 << 16);
	__DISABLE_CHG;

	GPIOB->MODER = (GPIOB->MODER & ~(3 << 14));		//USB_DETECT


}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
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
