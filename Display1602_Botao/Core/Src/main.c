/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int debounce_factor = 10;
int active = 0;
int btn_count = 0;
int btn_prev = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void debounceKey(GPIO_TypeDef *, uint32_t);
void writeCommandLCD(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
void clearLCD(void);
void cursorLCD(uint16_t, uint16_t);
void homeLCD(void);
void initLCD(void);
void writeLCD(unsigned char);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */

  initLCD();

  unsigned char vector[13];
  vector[0] = 88;
  vector[1] = 111;
  vector[2] = 114;
  vector[3] = 110;
  vector[4] = 111;
  vector[5] = 116;
  vector[6] = 111;
  vector[7] = 114;
  vector[8] = 32;
  vector[9] = 50;
  vector[10] = 48;
  vector[11] = 50;
  vector[12] = 50;

  uint32_t changed = 0;
  uint16_t charcount = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	debounceKey(KEY_GPIO_Port, KEY_Pin);
	if(active == 1 && changed == 0){
		if(charcount < 13){
			writeLCD(vector[charcount]);
			charcount++;
		}
		changed = 1;
	}
	if(active == 0){
		changed = 0;
	}
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
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RW_Pin|EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RW_Pin EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = RW_Pin|EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void debounceKey(GPIO_TypeDef * port, uint32_t pin){
	  int leitura = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
	  if(leitura != btn_prev){
		  if(btn_count <= debounce_factor){
			  btn_count++;
		  }else{
			  if(leitura == 0){
				  active = 1;
			  }else{
				  btn_count = 0;
				  active = 0;
			  }
			  btn_prev = leitura;
		  }
	  }else{
		  btn_count = 0;
	  }
}

void writeCommandLCD(uint32_t RS_Val, uint32_t D7_Val, uint32_t D6_Val, uint32_t D5_Val, uint32_t D4_Val, uint32_t DelayEN){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RS_Val);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, D4_Val);
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, D5_Val);
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, D6_Val);
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, D7_Val);
	HAL_Delay(DelayEN);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);
	HAL_Delay(DelayEN);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
}

void writeLCD(unsigned char c){
	uint16_t bit[8];
	bit[7] = c/128;
	bit[6] = c/64 - ((c/128)*2);
	bit[5] = c/32 - ((c/64)*2);
	bit[4] = c/16 - ((c/32)*2);
	bit[3] = c/8 - ((c/16)*2);
	bit[2] = c/4 - ((c/8)*2);
	bit[1] = c/2 - ((c/4)*2);
	bit[0] = c - ((c/2)*2);
	writeCommandLCD(1, bit[7], bit[6], bit[5], bit[4], 4);
	writeCommandLCD(1, bit[3], bit[2], bit[1], bit[0], 4);
}

void clearLCD(){
	writeCommandLCD(0, 0, 0, 0, 0, 4);
	writeCommandLCD(0, 0, 0, 0, 1, 4);
}

void cursorLCD(uint16_t row, uint16_t column){
	uint16_t addr = 0;
	if(row != 0) addr += 0x40;
	addr += column;
	uint16_t bit[8];
	bit[7] = addr/128;
	bit[6] = addr/64 - ((addr/128)*2);
	bit[5] = addr/32 - ((addr/64)*2);
	bit[4] = addr/16 - ((addr/32)*2);
	bit[3] = addr/8 - ((addr/16)*2);
	bit[2] = addr/4 - ((addr/8)*2);
	bit[1] = addr/2 - ((addr/4)*2);
	bit[0] = addr - ((addr/2)*2);
	writeCommandLCD(0, 1, bit[6], bit[5], bit[4], 4);
	writeCommandLCD(0, bit[3], bit[2], bit[1], bit[0], 4);
}

void homeLCD(){
	writeCommandLCD(0, 0, 0, 0, 0, 4);
	writeCommandLCD(0, 0, 0, 1, 0, 4);
}

void initLCD(){
	//RW 0
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);

	//4 bit data
	writeCommandLCD(0, 0, 0, 1, 0, 4);

	//2 line, font 5x8
	writeCommandLCD(0, 0, 0, 1, 0, 4);
	writeCommandLCD(0, 1, 0, 0, 0, 4);

	//Display ON
	writeCommandLCD(0, 0, 0, 0, 0, 2);
	writeCommandLCD(0, 1, 1, 1, 1, 2);

	//Clear LCD
	writeCommandLCD(0, 0, 0, 0, 0, 2);
	writeCommandLCD(0, 0, 0, 0, 1, 2);

	//Return Home
	writeCommandLCD(0, 0, 0, 0, 0, 2);
	writeCommandLCD(0, 0, 0, 1, 0, 2);
}

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
