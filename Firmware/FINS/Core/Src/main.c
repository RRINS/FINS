/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#include "stdio.h"
#include "application.hpp"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here. This is
     used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint16_t ACCEL_ADDR = 0x18 << 1;
static const uint8_t ACC_CHIP_ID = 0x00;
// write 0x00 for OFF, 0x04 for ON
static const uint8_t ACC_PWR_CTRL = 0x7D;
static const uint8_t ACC_RANGE = 0x41;

static const uint16_t GYRO_ADDR = 0x68 << 1;
static const uint8_t GYRO_CHIP_ID = 0x00;
static const uint8_t GYRO_RANGE = 0x0F;
static const uint16_t GYRO_RANGE_CONVERSION[] = { 2000, 1000, 500, 250, 125 };
static const uint8_t GYRO_BANDWIDTH = 0x10;

HAL_StatusTypeDef ret;
uint8_t buf[12];
int16_t val;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  char textBuffer[256];

  // Short delay for devices to initialize
  HAL_Delay(1000U);

  uint8_t onCommand[] = {ACC_PWR_CTRL, 0x04};
  ret = HAL_I2C_Master_Transmit(&hi2c1, ACCEL_ADDR, onCommand, 2, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {
	printf("Error Tx: %i %s\n", ret, (char*)buf);
  }

  HAL_Delay(50U);


  for(int i=1; i<128; i++)
	{
	  ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
	  if(ret == HAL_OK)
	  {
		  printf("Heard 0x%X; %i\n", i, i);
	  }
	}

  uint8_t accelRange;
  uint8_t gyroRange;
  // Read accel range register
  ret = HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDR, ACC_RANGE, I2C_MEMADD_SIZE_8BIT, &accelRange, 1, HAL_MAX_DELAY);
  if(ret != HAL_OK)
	{
		printf("Error Tx: %i %s\n", ret, (char*)buf);
	}
  printf("accel range: 0x%X\n", accelRange);

  uint8_t newGyroRange = 0x02;
  ret = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDR, GYRO_RANGE, I2C_MEMADD_SIZE_8BIT, &newGyroRange, 1, HAL_MAX_DELAY);
  if(ret != HAL_OK)
	{
		printf("Error Tx: %i %s\n", ret, (char*)buf);
	}

  uint8_t newGyroBandwidth = 0x07;
  ret = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDR, GYRO_BANDWIDTH, I2C_MEMADD_SIZE_8BIT, &newGyroBandwidth, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
  	{
  		printf("Error Tx: %i %s\n", ret, (char*)buf);
  	}

  ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDR, GYRO_RANGE, I2C_MEMADD_SIZE_8BIT, &gyroRange, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
  	{
  		printf("Error Tx: %i %s\n", ret, (char*)buf);
  	}
    float gyroConversion = GYRO_RANGE_CONVERSION[gyroRange] / 32767.0;
    printf("gyro range: 0x%X, conversion %f\n", gyroRange, gyroConversion);

  unsigned long loopCount = 0;
  const unsigned long LOOPDELAY = 100;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    int16_t x, y, z;
    for(int i = 0; i < 6; ++i)
    {
        ret = HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDR, 0x12+i, I2C_MEMADD_SIZE_8BIT, &buf[i], 1, HAL_MAX_DELAY);
        if(ret != HAL_OK)
        {
        	printf("Error Tx: %i %s\n", ret, (char*)buf);
        }
    }

    x = buf[1]*256 + buf[0];
    y = buf[3]*256 + buf[2];
    z = buf[5]*256 + buf[4];

    float accelX, accelY, accelZ;

    accelX = x/32768.0 * pow(2.0, accelRange + 1.0) * 1.5;
    accelY = y/32768.0 * pow(2.0, accelRange + 1.0) * 1.5;
    accelZ = z/32768.0 * pow(2.0, accelRange + 1.0) * 1.5;

    for(int i = 0; i < 6; ++i)
    {
    	ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDR, 0x02+i, I2C_MEMADD_SIZE_8BIT, &buf[i], 1, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			printf("Error Tx: %i %s\n", ret, (char*)buf);
		}
    }

    x = buf[1]*256 + buf[0];
	y = buf[3]*256 + buf[2];
	z = buf[5]*256 + buf[4];

	float gyroX, gyroY, gyroZ;

	gyroX = x * gyroConversion;
	gyroY = y * gyroConversion;
	gyroZ = z * gyroConversion;

    int size = snprintf(textBuffer, sizeof(textBuffer),
    		"{ \"accel\": {\"X\":%f, \"Y\":%f, \"Z\":%f}, \"gyro\": {\"X\":%f, \"Y\":%f, \"Z\":%f}, \"time\": %lu}\n",
    		accelX, accelY, accelZ, gyroX, gyroY, gyroZ, loopCount++ * LOOPDELAY);
    HAL_UART_Transmit(&huart4, textBuffer, size, HAL_MAX_DELAY);

	HAL_Delay(LOOPDELAY);
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
