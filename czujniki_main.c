/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "crc.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_CS_PIN       GPIO_PIN_1
#define GYRO_CS_PORT      GPIOC

#define ADXL345_ADDR        (0x53 << 1) // 7-bit address shifted for HAL
#define ADXL345_DEVID_REG   0x00
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32

#define gyro_x sensordata[0]
#define gyro_y sensordata[1]
#define gyro_z sensordata[2]
#define acc_x sensordata[3]
#define acc_y sensordata[4]
#define acc_z sensordata[5]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*** WDS ***/
static uint8_t uart_tx_frame[FRAME_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief Gyroscope initialization
 */
void Gyro_Init(void);
/**
 * @brief Read and convert gyroscope data
 * @param x gyroscope's reading on X axis [g]
 * @param y gyroscope's reading on Y axis [g]
 * @param z gyroscope's reading on Z axis [g]
 */
void Gyro_ReadData(float* x, float* y, float* z);
/**
 * @brief Write to gyroscope register
 * @param reg register's index
 * @param data data to be written into the register
 */
void Gyro_WriteReg(uint8_t reg, uint8_t data);
/**
 * @brief Read from gyroscope register
 * @param reg register's index
 */
uint8_t Gyro_ReadReg(uint8_t reg);
/**
 * @brief Read multiple bytes from gyroscope
 * @param reg register's index
 * @param buffer where the multiple-bytes data is stored
 * @param length lenght of the buffer
 */
void Gyro_ReadRegs(uint8_t reg, uint8_t* buffer, uint8_t length);

/**
 * @brief Accelerometer initialization
 */
void ADXL345_Init(void);
/**
 * @brief Read and convert accelerometer data
 * @param x accelerometer's reading on X axis [g]
 * @param y accelerometer's reading on Y axis [g]
 * @param z accelerometer's reading on Z axis [g]
 */
void ADXL345_ReadAccel(float* x, float* y, float* z);
/**
 * @brief Write to accelerometer register
 * @param reg register's index
 * @param data data to be written into the register
 */
void ADXL345_Write(uint8_t reg, uint8_t data);
/**
 * @brief Read from accelerometer's given register
 * @param reg register's index
 * @param buffer where the multiple-bytes data is stored
 * @param length lenght of the buffer
 */
void ADXL345_Read(uint8_t reg, uint8_t* buffer, uint8_t length);
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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Gyro_Init();
  ADXL345_Init();

  float sensordata[6]; //0 gyro_x, 1 gyro_y, 2 gyro_z, 3 acc_x, 4 acc_y, 5 acc_z
  float roll=0, pitch=0, yaw=0;
  float acc_roll, acc_pitch;
  float dt = 0.1f; // 10ms loop // 100ms
  float alpha = 0.98f;
  // kompensacja odchyleń żyro
  float biasX = 0, biasY = 0, biasZ = 0;
  for (int i = 0; i < 1000; ++i) {
	  Gyro_ReadData(&gyro_x, &gyro_y, &gyro_z);
      biasX += gyro_x; biasY += gyro_y; biasZ += gyro_z;
      HAL_Delay(1);
  }
  biasX /= 1000.0f; biasY /= 1000.0f; biasZ /= 1000.0f;

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Gyro_ReadData(&gyro_x, &gyro_y, &gyro_z);
      ADXL345_ReadAccel(&acc_x, &acc_y, &acc_z);

      gyro_x -= biasX;
	  gyro_y -= biasY;
	  gyro_z -= biasZ;
      acc_pitch = atan2f(-acc_x, sqrtf(acc_y*acc_y + acc_z*acc_z)) * 180.0f / M_PI;
	  acc_roll = atan2f(acc_y, acc_z) * 180.0f / M_PI;

	  pitch = alpha * (pitch + gyro_x * dt) + (1 - alpha) * acc_pitch;
	  roll  = alpha * (roll  + gyro_y * dt) + (1 - alpha) * acc_roll;
      // yaw  += gyro_z * dt; // niepotrzebny

   	  HAL_Delay(dt);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Gyroscope initialization
void Gyro_Init(void){
    // CTRL_REG1: 0x0F = Normal mode, all axes enabled, 95 Hz ODR
    Gyro_WriteReg(0x20, 0x0F);
    // CTRL_REG4: 0x20 = 2000 dps full scale
    Gyro_WriteReg(0x23, 0x20);
}

// Read from gyroscope register
uint8_t Gyro_ReadReg(uint8_t reg){
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi5, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi5, &rx, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return rx;
}

// Read multiple bytes from gyroscope
void Gyro_ReadRegs(uint8_t reg, uint8_t* buffer, uint8_t length){
    uint8_t tx = reg | 0xC0; // Set auto-increment and read bits
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi5, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi5, buffer, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

// Read and convert gyroscope data
void Gyro_ReadData(float* x, float* y, float* z){
    uint8_t buffer[6];
    int16_t rawX, rawY, rawZ;
    float sensitivity = 70.0f; // Sensitivity for 2000 dps full scale

    Gyro_ReadRegs(0x28, buffer, 6);

    rawX = (int16_t)(buffer[1] << 8 | buffer[0]);
    rawY = (int16_t)(buffer[3] << 8 | buffer[2]);
    rawZ = (int16_t)(buffer[5] << 8 | buffer[4]);

    // LSB * mg/LSB / 1000 = g
    *x = rawX * sensitivity / 1000.0f;
    *y = rawY * sensitivity / 1000.0f;
    *z = rawZ * sensitivity / 1000.0f;
}

void ADXL345_Init(void){
    uint8_t id;
    ADXL345_Read(ADXL345_DEVID_REG, &id, 1);
    if (id != 0x8)
    {
        // Handle error
        while (1);
    }

    ADXL345_Write(ADXL345_POWER_CTL, 0x08);      // Set measure bit
    ADXL345_Write(ADXL345_DATA_FORMAT, 0x00);    // Set range to ±2g
}

void ADXL345_ReadAccel(float* x, float* y, float* z){
    uint8_t buffer[6];
    int16_t rawX, rawY, rawZ;
    float sensitivity = 0.004f; // 4 mg/LSB

    ADXL345_Read(ADXL345_DATAX0, buffer, 6);

    rawX = (int16_t)(buffer[1] << 8 | buffer[0]);
    rawY = (int16_t)(buffer[3] << 8 | buffer[2]);
    rawZ = (int16_t)(buffer[5] << 8 | buffer[4]);

    *x = rawX * sensitivity;
    *y = rawY * sensitivity;
    *z = rawZ * sensitivity;
}

void ADXL345_Read(uint8_t reg, uint8_t* buffer, uint8_t length){
    HAL_I2C_Mem_Read(&hi2c3, ADXL345_ADDR, reg, 1, buffer, length, HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
