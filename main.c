
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t phy_to_dll_rx_bus;
static uint8_t RxcValue = 0;
static uint8_t TxcValue = 0;
static uint8_t phy_to_dll_rx_bus_valid = 0;
uint8_t dll_to_phy_tx_bus;
static uint8_t dll_to_phy_tx_bus_valid = 0;
static uint8_t phy_tx_busy = 0;
int ArrTx[8]; 
int ArrRx[8];
uint8_t tempbyte = 0x00;
uint8_t temp1;
static uint8_t BitToTransmit = 0;
static uint8_t BitRecieved = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void convertNumberIntoArray(int* Arr,uint8_t number) {
    for(int j=7;j>=0;j--)
	{
				Arr[j] = (number & 1);
        number  = number >> 1;
    } 
    return;
}
uint8_t convertArrayIntoNumber(int* Arr) {
    for(int l=0;l<8;l++)
	{
				temp1 = (*Arr & 1);
				tempbyte = tempbyte | temp1;
				Arr = Arr +1;
				//Byte = Byte | (Arr[j] & 1);
				if(l != 7) tempbyte = tempbyte << 1;
    } 
    return tempbyte;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		dll_layer();
		phy_layer();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TXC_Pin|TXD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TXC_Pin TXD_Pin */
  GPIO_InitStruct.Pin = TXC_Pin|TXD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RXC_Pin */
  GPIO_InitStruct.Pin = RXC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RXC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RXD_Pin */
  GPIO_InitStruct.Pin = RXD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RXD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void dll_layer(){
	dll_TX();
	dll_RX();
}

void phy_layer(){
	phy_TX();
	phy_RX();
}

void dll_TX(){
	static int gap = 0;
	uint8_t RandNum;
	if(!dll_to_phy_tx_bus_valid && !phy_tx_busy)
	{
		if (!gap)
		{
			RandNum = rand()% 256;
			dll_to_phy_tx_bus = RandNum;
			dll_to_phy_tx_bus_valid = 1;
			gap = 20;
		}
		else gap--;
	}
	else return;
	return;
}

void dll_RX(){
	static uint8_t RecievedData = 0;
	if(phy_to_dll_rx_bus_valid)
		{
		RecievedData = phy_to_dll_rx_bus;
		phy_to_dll_rx_bus_valid = 0;
	}
	return;
}

void phy_TX(){
	uint8_t temp;
	static int flagFinished = 0; //Indicates if we finished sending the Byte | 0 - not finished , 1 - finished 
	static uint8_t TxLastClockValue = 0;
	uint8_t CurrentClockValue = 0;
	uint8_t DataToTransmit;
	static int i = 0; // index of array

	if(dll_to_phy_tx_bus_valid == 1)
		{
		DataToTransmit = dll_to_phy_tx_bus; // Saving data from bus to local variable
		convertNumberIntoArray(ArrTx,DataToTransmit);	
		dll_to_phy_tx_bus_valid = 0;  // informing the DLL layer that I took the data from the bus
		phy_tx_busy = 1;  // informing the DLL layer that I am busy sending data
		flagFinished = 0; 
		HAL_TIM_Base_Start_IT(&htim3);	//Start Synchronization Clock
		}
	if(flagFinished == 0){
		TxcValue = HAL_GPIO_ReadPin(TXC_GPIO_Port,TXC_Pin);
		CurrentClockValue = HAL_GPIO_ReadPin(TXC_GPIO_Port,TXC_Pin);  // Reads the current clock value
	if( TxLastClockValue == 0 && CurrentClockValue == 1)
	{
			BitToTransmit = ArrTx[i];
			HAL_GPIO_WritePin(TXD_GPIO_Port,TXD_Pin,BitToTransmit);
			i++;
			if(i == 8)
			{
				phy_tx_busy = 0;
				i = 0;
				flagFinished = 1;
				HAL_TIM_Base_Stop_IT(&htim3); //Stop Synchronization Clock
			}
		}
	TxLastClockValue = CurrentClockValue;
	return;
	}
}

void phy_RX(){
	static int k = 0;
	static uint8_t RxLastClockValue = 0;
	uint8_t RxCurrentClockValue = 0;
	RxcValue = HAL_GPIO_ReadPin(RXC_GPIO_Port,RXC_Pin);
	RxCurrentClockValue = HAL_GPIO_ReadPin(RXC_GPIO_Port,RXC_Pin);  // Reads the current clock value
	if( RxLastClockValue == 1 && RxCurrentClockValue == 0)
	{
		BitRecieved = HAL_GPIO_ReadPin(RXD_GPIO_Port,RXD_Pin);
		ArrRx[k] = BitRecieved;
		k++;
		if(k == 8)
		{
			convertArrayIntoNumber(ArrRx);
			phy_to_dll_rx_bus_valid = 1;
			phy_to_dll_rx_bus = tempbyte;
			tempbyte = 0;
			k = 0;
		}
	}
	RxLastClockValue = RxCurrentClockValue;
}

/*
GPIO_PinState HAL_GPIO_ReadOutPutPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_PinState bitstatus;
	assert_param(IS_GPIO_PIN(GPIO_Pin));
	
	if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
	{
		bitstatus = GPIO_PIN_SET;
	}
	else
	{
		bitstatus = GPIO_PIN_RESET;
	}
	return bitstatus;
}*/


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
		break;
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
