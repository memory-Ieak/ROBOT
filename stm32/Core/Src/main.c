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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct MPU6050_data
{
  int8_t accelX_MSB, accelX_LSB;
  int8_t accelY_MSB, accelY_LSB;
  int8_t accelZ_MSB, accelZ_LSB;

  int8_t temp_MSB, temp_LSB;

  int8_t gyroX_MSB, gyroX_LSB;
  int8_t gyroY_MSB, gyroY_LSB;
  int8_t gyroZ_MSB, gyroZ_LSB;
};

struct __attribute__ ((packed)) MPU6050_accel_data
{
  uint8_t SOH;
  float accX, accY, accZ;
  uint8_t EOT;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_ADR_WRITE   (0x68 << 1) + 0
#define IMU_ADR_READ    (0x68 << 1) + 1
#define MPU_ACCEL_COEF  (2.0f / 32767.0f)
#define MPU_GYRO_COEF   (250.0f / 32767.0f)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int _write(int fd, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

void send_data(uint8_t *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Read_info();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Read_info()
{
    // Recover all data from the MPU in one call using our stuct
    // Register 59 to 64 Accelerometer (see register map p29)
    // Register 65 to 66 Accelerometer (see register map 30)
    // Register 67 to 72 Accelerometer (see register map 31)
  struct MPU6050_data data;
  HAL_I2C_Mem_Read(&hi2c1, IMU_ADR_READ, 59, 1, (uint8_t*)&data, sizeof(struct MPU6050_data), HAL_MAX_DELAY);
   
  struct MPU6050_accel_data dataToSend;
  // Converting our value to g force(Accel) and to deg/sec(Gyro)
  dataToSend.SOH = 0x1;
  dataToSend.accX = MPU_ACCEL_COEF * (((int16_t)data.accelX_MSB << 8) + (int16_t)data.accelX_LSB);
  dataToSend.accY = MPU_ACCEL_COEF * (((int16_t)data.accelY_MSB << 8) + (int16_t)data.accelY_LSB);
  dataToSend.accZ = MPU_ACCEL_COEF * (((int16_t)data.accelZ_MSB << 8) + (int16_t)data.accelZ_LSB);
  dataToSend.EOT = 0x4;
  /*float gyroX = MPU_GYRO_COEF * (((int16_t)data.gyroX_MSB << 8) + (int16_t)data.gyroX_LSB);
  float gyroY = MPU_GYRO_COEF * (((int16_t)data.gyroY_MSB << 8) + (int16_t)data.gyroY_LSB);
  float gyroZ = MPU_GYRO_COEF * (((int16_t)data.gyroZ_MSB << 8) + (int16_t)data.gyroZ_LSB);*/

  /*printf(">accelX:%f\n", dataToSend.accX);
  printf(">accelY:%f\n", dataToSend.accY);
  printf(">accelZ:%f\n", dataToSend.accZ);*/
  /*printf(">gyroX:%f\n", gyroX);
  printf(">gyroY:%f\n", gyroY);
  printf(">gyroZ:%f\n", gyroZ);*/
  send_data((uint8_t*)&dataToSend, sizeof(struct MPU6050_accel_data));
}
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
  /* USER CODE BEGIN 2 */

  // Init the I2C device
  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADR_WRITE, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    printf("Failed to init device!\n");
    return 1;
  }

  uint8_t data = 0;
  // Set the gyroscope scale range to +/-250 deg/s
  // Register 27 (see register map p14)
  HAL_I2C_Mem_Write(&hi2c1, IMU_ADR_WRITE, 27, 1, &data, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    printf("Failed to setup gyroscope range!\n");
    return 1;
  }

  // Set the accelerometer scale range to +/-2G deg/s
  // Register 28 (see register map p15)
  HAL_I2C_Mem_Write(&hi2c1, IMU_ADR_WRITE, 28, 1, &data, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    printf("Failed to setup accelerometer range!\n");
    return 1;
  }

  // Enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L
  // Register 38 (see register map p15)
  /*HAL_I2C_Mem_Write(&hi2c1, IMU_ADR_WRITE, 38, 1, &data, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    printf("Failed to set FIFO with accelo!\n");
    return 1;
  }*/

  // Enable FIFO_OFLOW_EN
  // Register 60 (see register map p40)
  /*HAL_I2C_Mem_Write(&hi2c1, IMU_ADR_WRITE, 60, 1, &data, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    printf("Failed to wake up the device!\n");
    return 1;
  }*/

  // Exit sleep mode / Wake up the device. We won't receive any data when it is in sleep mode.
  // Register 107 (see register map p40)
  HAL_I2C_Mem_Write(&hi2c1, IMU_ADR_WRITE, 107, 1, &data, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    printf("Failed to wake up the device!\n");
    return 1;
  }

  printf("MPU6050 Initialized correctly\n");
  // Little delay in case the MPU need to do some work before being able to receive data
  HAL_Delay(15);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Read_info();
    HAL_Delay(5);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

#ifdef USE_FULL_ASSERT
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
