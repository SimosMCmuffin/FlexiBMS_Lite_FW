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
#include <usb_device_ST.h>
#include <SPI_LL.h>
#include <LTC6803_3_DD.h>
#include <ADC_LL.h>
#include <CAN_LL.h>
#include "AuxFunctions.h"
#include <dStorage_MD.h>
#include <USB_comms_handler_MD.h>

void SystemClock_Config(void);
static void GPIO_Init(void);

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

	GPIO_Init();
	MX_USB_DEVICE_Init();
	SPI1_init();
	LTC6803_init();
	ADC_init();
	CAN1_init();

	USBD_DeInit(&hUsbDeviceFS);		//Stop usb service

	if( runtimePars.ADCrunState == 0 ){	//
		ADC_setupSequence();
		ADC_runSequence();
	}

	uint64_t systemTick = HAL_GetTick(), LTC6803tick = HAL_GetTick();

	while (1)
	{

		if( systemTick + 20 <= HAL_GetTick() ){
			systemTick = HAL_GetTick();

			usbPowerPresent();

		}

		statusLed();

		chargeControl();

		checkForNewMessages();

		if( LTC6803tick <= HAL_GetTick() ){		//all LTC6803 SPI communications are started from this tick function
			LTC6803tick = HAL_GetTick() + 5;
			LTC6803_transactionHandler(&LTC6803tick);
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
}

/** Pinout Configuration
 */
static void GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIOB->MODER = GPIOB->MODER & ~(3 << 0);	//5V_BUCK_ENABLE
	__ENABLE_5V_BUCK;
	//__DISABLE_5V_BUCK;

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
