/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 * Author			  : Simo Sihvonen (Simos MCmuffin)
 ******************************************************************************

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

const uint8_t FW_VERSION[] = {"FW_0002_WIP_DEBUG"};

nonVolParameters nonVolPars;
runtimeParameters runtimePars;

int main(void)
{

	initNonVolatiles(&nonVolPars, 0);		//init/load non-volatile parameters
	initRuntimeParameters(&runtimePars);	//init default runtime parameters

	HAL_Init();						//init ST's HAL core

	SystemClock_Config();			//init oscillators

	GPIO_Init();					//Init GPIO in-/outputs
	MX_USB_DEVICE_Init();			//init ST's CDC (VCP) USB stack
	SPI1_init();					//init SPI1 low level HW
	LTC6803_init();					//init LTC6803 device driver
	ADC_init();						//init ADC low level HW
	CAN1_init();					//init CAN low level HW
	CAN1_setupRxFilters();			//init CAN RX ID filters

	USBD_DeInit(&hUsbDeviceFS);		//Stop usb service

	if( runtimePars.ADCrunState == 0 ){	//setup and start ADC sampling and conversion
		ADC_setupSequence();
		ADC_runSequence();
	}

	uint64_t systemTick = HAL_GetTick(), LTC6803tick = HAL_GetTick();


	while (1)
	{

		if( systemTick + 20 <= HAL_GetTick() ){		//go in here at max every 20ms
			systemTick = HAL_GetTick();

			usbPowerPresent();		//Init/deInit USB based on if 5V is detected from the USB connector

		}

		CAN1_debugEcho();

		statusLed();		//Control status LED

		hwRequestControl();		//disable/enable 5V buck and ADC channels based on software requests

		chargeControl();	//charging algorithm, everything charging control related is done through here

		checkForNewMessages();		//check for new messages from USB and send State printout if enabled

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
