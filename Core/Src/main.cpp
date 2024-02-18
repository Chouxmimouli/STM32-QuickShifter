/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <iostream>
#include <string>
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IgnitionCutTime 200

#define ADC_BUF_LEN 4096
#define UpShiftDeadZone 60
#define DownShiftDeadZone 60

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint16_t adc_buf[ADC_BUF_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool KillSwitch = false; // For debugging
bool GearShiftFail = false;
uint8_t EngagedGear = 0;
uint8_t PrevEngagedGear = 0;
uint16_t CalibratedShifterPosition = 0;
uint16_t ShifterPosition = 0; // For debugging

class Shifter {
public:

	uint16_t GetPosition(){
	     uint32_t AdcAverage = 0;
	     for (int i = 0; i < ADC_BUF_LEN; i++) {
	         AdcAverage +=  adc_buf[i];
	     }
	     AdcAverage /= ADC_BUF_LEN;
	     ShifterPosition = AdcAverage; // For debugging
	     return AdcAverage;
	}

	void Calibrate() {
		CalibratedShifterPosition = GetPosition();
	}

	void TryRecalibrate() {
	  	if (__HAL_TIM_GET_COUNTER(&htim3) >= 15000){
	  		Calibrate();
	  		HAL_TIM_Base_Stop(&htim3);
	  		TIM3->CNT = 0;
	  	}
	}
};

class Gear {
private:
	void Reset() {
		HAL_GPIO_WritePin(OutGear0_GPIO_Port, OutGear0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(OutGear1_GPIO_Port, OutGear1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(OutGear2_GPIO_Port, OutGear2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(OutGear3_GPIO_Port, OutGear3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(OutGear4_GPIO_Port, OutGear4_Pin, GPIO_PIN_RESET);
	}

public:

	void Get() {
		EngagedGear = 99; // default value if none of the pins are low or selector barrel in between 2 gears
	    switch(HAL_GPIO_ReadPin(InGear0_GPIO_Port, InGear0_Pin)) {
	    	case 0: // pin is low, gear 0 is engaged
	    		EngagedGear = 0;
	    		break;
	    	case 1: // pin is high, gear 0 is not engaged
	    switch(HAL_GPIO_ReadPin(InGear1_GPIO_Port, InGear1_Pin)) {
	    	case 0:
	    		EngagedGear = 1;
	    		break;
	    	case 1:
	    switch(HAL_GPIO_ReadPin(InGear2_GPIO_Port, InGear2_Pin)) {
	    	case 0:
	    		EngagedGear = 2;
	    		break;
	    	case 1:
	    switch(HAL_GPIO_ReadPin(InGear3_GPIO_Port, InGear3_Pin)) {
	    	case 0:
	    		EngagedGear = 3;
	    		break;
	    	case 1:
	    switch(HAL_GPIO_ReadPin(InGear4_GPIO_Port, InGear4_Pin)) {
	    	case 0:
	    		EngagedGear = 4;
	            break;
	    	case 1:
	    	break;
	    }
	    break; // once the selected gear is found, break out
	    }
	    break;
	    }
	    break;
	    }
	    break;
	    }
	}

	void Set() {
	  	switch(EngagedGear) {
	  	  case 0:
	  		  Reset();
	  		  HAL_GPIO_WritePin(OutGear0_GPIO_Port, OutGear0_Pin, GPIO_PIN_SET);
	  		  break;
	  	  case 1:
	  		  Reset();
	  		  HAL_GPIO_WritePin(OutGear1_GPIO_Port, OutGear1_Pin, GPIO_PIN_SET);
	  		  break;
	  	  case 2:
	  		  Reset();
	  		  HAL_GPIO_WritePin(OutGear2_GPIO_Port, OutGear2_Pin, GPIO_PIN_SET);
	  		  break;
	  	  case 3:
	  		  Reset();
	  		  HAL_GPIO_WritePin(OutGear3_GPIO_Port, OutGear3_Pin, GPIO_PIN_SET);
	  		  break;
	  	  case 4:
	  		  Reset();
	  		  HAL_GPIO_WritePin(OutGear4_GPIO_Port, OutGear4_Pin, GPIO_PIN_SET);
	  		  break;
	  	}
	}
};


class WaitFor {
public:
	void UpShift(Gear* pGear, Shifter* pShifter) {
		while ((EngagedGear != (PrevEngagedGear + 1)) || (ShifterPosition > (CalibratedShifterPosition - 200))) {
			pGear->Get();
			pShifter->GetPosition();

			// Upshift failed, user shifts back into lower gear
			if (ShifterPosition > (CalibratedShifterPosition + DownShiftDeadZone)) {
				PrevEngagedGear++;
				GearShiftFail = true;
				break;
			}
		}
	}

	void DownShift(Gear* pGear, Shifter* pShifter) {
		while ((EngagedGear != (PrevEngagedGear - 1)) || (ShifterPosition < (CalibratedShifterPosition + 200))) {
			pGear->Get();
			pShifter->GetPosition();

			// Downshift failed, user shifts back into upper gear
			if (ShifterPosition < (CalibratedShifterPosition - UpShiftDeadZone)) {
				PrevEngagedGear--;
				GearShiftFail = true;
				break;
			}
		}
	}

};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	Shifter shifter;
	Gear gear;
	WaitFor waitFor;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  //Set the initial EngagedGear
  gear.Get();
  gear.Set();
  shifter.Calibrate();

  HAL_TIM_Base_Start(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  shifter.TryRecalibrate();
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	  // Up shift
	  if (shifter.GetPosition() < (CalibratedShifterPosition - UpShiftDeadZone)) {
		  PrevEngagedGear = EngagedGear;
	  	  ExitToUpShift:

	  	  if (EngagedGear != 0){
	  		  HAL_GPIO_WritePin(KillSwitch_GPIO_Port, KillSwitch_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  		  KillSwitch = 1;

	  		waitFor.UpShift(&gear, &shifter);
	  		  if (GearShiftFail == 1) {
	  			  GearShiftFail = 0;
	  			  goto ExitToDownShift;
	  		  }

	  		  gear.Get();
	  		  gear.Set();

	  		  HAL_Delay(IgnitionCutTime);
	  		  HAL_GPIO_WritePin(KillSwitch_GPIO_Port, KillSwitch_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  		  KillSwitch = 0;
	  	  }
	  	  else {
	  		  waitFor.UpShift(&gear, &shifter);
	  		  gear.Set();
	  	  }

	  	  while (shifter.GetPosition() < (CalibratedShifterPosition - (UpShiftDeadZone - 10)));

	  	  TIM3->CNT = 0;
	  	  HAL_TIM_Base_Start(&htim3);
	  }

	  // Down shift
	  if (shifter.GetPosition() > (CalibratedShifterPosition + DownShiftDeadZone)){
		  PrevEngagedGear = EngagedGear;
	  	  ExitToDownShift:

		  waitFor.DownShift(&gear, &shifter);
	  	  if (GearShiftFail == 1) {
	  		  GearShiftFail = 0;
	  		  goto ExitToUpShift;
	  	  }

	  	  gear.Set();

	  	  while (shifter.GetPosition() > (CalibratedShifterPosition + (DownShiftDeadZone - 10)));

	  	  TIM3->CNT = 0;
	  	  HAL_TIM_Base_Start(&htim3);
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KillSwitch_Pin|OutGear2_Pin|OutGear1_Pin|OutGear0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OutGear4_Pin|OutGear3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KillSwitch_Pin OutGear2_Pin OutGear1_Pin OutGear0_Pin */
  GPIO_InitStruct.Pin = KillSwitch_Pin|OutGear2_Pin|OutGear1_Pin|OutGear0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : InGear4_Pin InGear3_Pin InGear2_Pin */
  GPIO_InitStruct.Pin = InGear4_Pin|InGear3_Pin|InGear2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : InGear1_Pin InGear0_Pin */
  GPIO_InitStruct.Pin = InGear1_Pin|InGear0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OutGear4_Pin OutGear3_Pin */
  GPIO_InitStruct.Pin = OutGear4_Pin|OutGear3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
