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
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f4xx_hal.h"
#include "gyro.h"
#include "acc_ADXL345.h"
#include <string.h>
#include <math.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAMEBUFFER ((uint16_t *)0xD0000000)
#define MAZE_WIDTH 12
#define MAZE_HEIGHT 16
#define CELL_SIZE 20
#define MAZE_ORIGIN_X 0
#define MAZE_ORIGIN_Y 0

#define SPEED 5.5f


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
typedef struct {
	float x_px;
	float y_px;
	uint8_t tile_x;// column
	uint8_t tile_y;// row
} Ball;

typedef enum {
    GAME_RUNNING,
    GAME_WIN,
    GAME_LOSE
} GameState;

/*0 nothing
  1 wall
  2 goal
  3 danger*/
uint8_t maze1[MAZE_HEIGHT][MAZE_WIDTH] = {
  {1,0,1,1,1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,1,1,1,1,1,0,1},
  {1,0,0,0,1,0,0,0,1,3,0,1},
  {1,0,1,1,1,1,0,0,0,3,0,1},
  {1,0,1,0,1,0,0,1,1,3,0,1},
  {1,0,0,0,3,1,0,0,0,3,0,1},
  {1,0,1,1,1,1,0,1,1,3,0,1},
  {1,0,1,1,0,0,0,0,1,3,0,1},
  {1,0,0,0,0,1,1,1,1,3,0,1},
  {1,3,0,1,0,0,0,0,0,0,0,1},
  {1,3,0,1,1,1,1,1,0,3,0,1},
  {1,3,0,0,0,0,1,3,3,1,0,1},
  {1,1,1,0,1,0,0,0,3,3,0,1},
  {1,0,0,0,1,0,1,3,3,3,0,1},
  {1,1,1,1,1,1,1,1,1,1,2,1},
};


uint8_t maze2[MAZE_HEIGHT][MAZE_WIDTH] = {
	{1,1,1,1,1,1,1,1,1,1,1,1},
	{1,0,0,0,0,1,0,0,0,0,2,1},
	{1,0,1,1,0,1,0,1,1,1,1,1},
	{1,0,1,3,0,1,0,0,0,0,0,1},
	{1,0,1,0,0,1,1,1,1,1,0,1},
	{1,0,1,0,0,0,0,0,0,1,0,1},
	{1,0,1,1,1,1,1,1,0,1,0,1},
	{1,0,0,0,0,0,0,1,0,1,0,1},
	{1,1,1,1,1,1,0,1,0,1,0,1},
	{1,0,0,0,0,1,0,1,0,1,0,1},
	{1,0,1,1,0,1,0,1,0,1,0,1},
	{1,0,3,0,0,1,0,1,0,1,0,1},
	{1,1,1,1,1,1,0,1,0,1,0,1},
	{1,0,0,0,0,0,0,1,0,0,0,1},
	{1,0,1,1,1,1,1,1,1,1,0,1},
	{1,1,1,1,1,1,1,1,1,1,1,1},
};


uint8_t maze3[MAZE_HEIGHT][MAZE_WIDTH] = {
	{1,1,1,1,1,1,1,1,1,1,1,1},
	{1,0,0,0,0,0,0,0,0,0,2,1},
	{1,0,1,1,1,1,1,1,1,0,1,1},
	{1,0,1,3,3,0,0,0,1,0,0,1},
	{1,0,1,0,1,1,1,0,1,1,0,1},
	{1,0,1,0,1,3,1,0,0,1,0,1},
	{1,0,0,0,1,0,1,1,0,1,0,1},
	{1,0,1,1,1,0,1,3,0,1,0,1},
	{1,0,1,1,0,0,0,0,1,1,0,1},
	{1,0,0,0,0,1,1,1,1,3,0,1},
	{1,3,0,1,0,0,0,0,0,3,0,1},
	{1,3,0,1,1,1,1,1,0,3,0,1},
	{1,3,0,0,0,0,1,3,3,1,0,1},
	{1,1,1,0,1,0,0,0,3,1,0,1},
	{1,0,0,0,1,0,1,3,3,1,0,1},
	{1,1,1,1,1,1,1,1,1,1,1,1},
};


uint8_t ballSprite[CELL_SIZE][CELL_SIZE] = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
  {0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void update_ball_tile_position(Ball *ball) {
    ball->tile_x = (uint8_t)(ball->x_px / CELL_SIZE);
    ball->tile_y = (uint8_t)(ball->y_px / CELL_SIZE);
}
void draw_cell(uint8_t tile_x, uint8_t tile_y, uint32_t color)
{
    BSP_LCD_SetTextColor(color);
    BSP_LCD_FillRect(
        MAZE_ORIGIN_X + tile_x * CELL_SIZE,
        MAZE_ORIGIN_Y + tile_y * CELL_SIZE,
        CELL_SIZE,
        CELL_SIZE
    );
}
void draw_ballSprite(float x_px, float y_px)
{
	int top_left_x = (int)(x_px - CELL_SIZE / 2);
	int top_left_y = (int)(y_px - CELL_SIZE / 2);

	for (int py = 0; py < CELL_SIZE; py++) {
		for (int px = 0; px < CELL_SIZE; px++) {
			if (ballSprite[py][px] == 1) {
				BSP_LCD_DrawPixel(top_left_x + px, top_left_y + py, LCD_COLOR_YELLOW);
	        }
	    }
	}
}
void draw_maze(const uint8_t maze_data[MAZE_HEIGHT][MAZE_WIDTH])
{
	for (int y = 0; y < MAZE_HEIGHT; y++) {
	        for (int x = 0; x < MAZE_WIDTH; x++) {
	        	uint32_t color;

	        	switch (maze_data[y][x]) {
	        	case 0:  color = LCD_COLOR_BLACK; break;
	        	case 1:  color = LCD_COLOR_DARKBLUE; break;
	        	case 2:  color = LCD_COLOR_GREEN; break;
	        	case 3:  color = LCD_COLOR_RED; break;
	        	default: color = LCD_COLOR_WHITE; break;
	        	}

	        	draw_cell(x, y, color);
	        }
	    }
}

void wait_for_user_button(void) {
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
    HAL_Delay(200);
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
    HAL_Delay(200);
}

void reset_ball_position(Ball *ball) {
    ball->x_px = 1 * CELL_SIZE + CELL_SIZE / 2;
    ball->y_px = 1 * CELL_SIZE + CELL_SIZE / 2;
    update_ball_tile_position(ball);
}



float* Gyro_CountBias(float sensordata[]);
int __io_putchar(int ch);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Ball ball = {.x_px = 1 * CELL_SIZE + CELL_SIZE / 2, .y_px = 1 * CELL_SIZE + CELL_SIZE / 2,};
const uint8_t (*currMaze)[MAZE_WIDTH] = maze1;

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
  Gyro_Init();
  ADXL345_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_BLACK);


  update_ball_tile_position(&ball);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  GameState currentGameState = GAME_RUNNING;
  uint8_t currTile = 0;
  uint8_t currLevel = 0;



  float sensordata[6]; //0 gyro_x, 1 gyro_y, 2 gyro_z, 3 acc_x, 4 acc_y, 5 acc_z
  float roll=0, pitch=0;
  float acc_roll, acc_pitch, alpha = 0.98f;
  float dt = 0.1f; // 100ms loop
  float* gyro_bias; gyro_bias = Gyro_CountBias(sensordata);   // kompensacja odchyleń żyro

  uint32_t lastTick = HAL_GetTick();

  while (1)
  {
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	  HAL_Delay(17);


	  uint32_t now = HAL_GetTick();
	  dt = (now - lastTick) / 1000.0f; // in seconds
	  lastTick = now;

	  Gyro_ReadData(&gyro_x, &gyro_y, &gyro_z);
	  ADXL345_ReadAccel(&acc_x, &acc_y, &acc_z);

	  gyro_x -= gyro_bias[0];
	  gyro_y -= gyro_bias[1];
	  gyro_z -= gyro_bias[2];
	  acc_pitch = atan2f(-acc_x, sqrtf(acc_y*acc_y + acc_z*acc_z)) * 180.0f / M_PI;
	  acc_roll = atan2f(acc_y, acc_z) * 180.0f / M_PI;

//	  acc_pitch = 0;
//	  acc_roll = 0;

	  if(gyro_x > NOISE_THRESHOLD) pitch = alpha * (pitch + gyro_x * dt) + (1 - alpha) * acc_pitch;
	  if(gyro_y > NOISE_THRESHOLD) roll  = alpha * (roll  + gyro_y * dt) + (1 - alpha) * acc_roll;


	  printf("gyro_x: [%f], \tgyro_y: [%f], \tgyro_z: [%f]\r\n", gyro_x, gyro_y, gyro_z);
	  printf("acc_x: [%f], \tacc_y: [%f], \t acc_z: [%f]\r\n", acc_x, acc_y, acc_z);
	  printf("roll: [%f], pitch: [%f]\r\n", roll, pitch);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
//	  BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"text");

	  /*
	   * ###########################
	   * ####  GAME STATE CHECK  ###
	   * ###########################
	   * */

	  currTile = currMaze[ball.tile_y][ball.tile_x];

	  if (currTile == 2) {
	      currentGameState = GAME_WIN;
	  }
	  else if(currTile == 3){
		  currentGameState = GAME_LOSE;
	  }
	  else{
	  	  currentGameState = GAME_RUNNING;
	  }

	  /*
	  	   * ###########################
	  	   * ######### GAME LOGIC  #####
	  	   * ###########################
	  	   * */

	  if(currentGameState == GAME_RUNNING){
		 draw_maze(currMaze);
		 float vx = /*roll * */SPEED;
		 float vy = /*pitch * */SPEED;

		 float new_x = ball.x_px + vx * dt;
		 int next_tile_x = (int)(new_x / CELL_SIZE);
		 if (currMaze[ball.tile_y][next_tile_x] != 1) {
		     ball.x_px = new_x;
		 }


		 float new_y = ball.y_px + vy * dt;
		 int next_tile_y = (int)(new_y / CELL_SIZE);
		 if (currMaze[next_tile_y][ball.tile_x] != 1) {
		     ball.y_px = new_y;
		 }

		 update_ball_tile_position(&ball);
		 draw_ballSprite(ball.x_px, ball.y_px);
	  }
	  else if(currentGameState == GAME_LOSE){
		  BSP_LCD_Clear(LCD_COLOR_RED);
		  BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"YOU LOSE!", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, LINE(6), (uint8_t *)"Press button", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, LINE(7), (uint8_t *)"to restart", CENTER_MODE);
		  wait_for_user_button();
		  reset_ball_position(&ball);
		  currentGameState = GAME_RUNNING;
	  }
	  else{
		  BSP_LCD_Clear(LCD_COLOR_GREEN);
		  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		  BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"YOU WIN!", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, LINE(6), (uint8_t *)"Press button", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, LINE(7), (uint8_t *)"to advance", CENTER_MODE);
		  wait_for_user_button();


		  currLevel = (currLevel + 1) % 3;
		  switch (currLevel) {
		          case 0: currMaze = maze1;
		          	  	  BSP_LCD_Clear(LCD_COLOR_GREEN);
		          	  	  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		          	  	  BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Level 1", CENTER_MODE);
		          	  	  HAL_Delay(1000);
		          	  	  break;
		          case 1: currMaze = maze2;
		          	  	  BSP_LCD_Clear(LCD_COLOR_GREEN);
		          	  	  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		          	  	  BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Level 2", CENTER_MODE);
		          	  	  HAL_Delay(1000);
		          	  	  break;
		          case 2: currMaze = maze3;
		          	  	  BSP_LCD_Clear(LCD_COLOR_GREEN);
		          	  	  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		          	  	  BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Level 3", CENTER_MODE);
		          	  	  HAL_Delay(1000);
		          	  	  break;
		  }

		  reset_ball_position(&ball);
		  currentGameState = GAME_RUNNING;
	  }


	  uint32_t frameTime = HAL_GetTick() - now;
	  if (frameTime < 17)
	      HAL_Delay(17 - frameTime);



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/* USER CODE BEGIN 4 */
float* Gyro_CountBias(float sensordata[]){
	static float bias[3];
	for (int i = 0; i < 1000; ++i) {
		  Gyro_ReadData(&gyro_x, &gyro_y, &gyro_z);
	      bias[0] += gyro_x; bias[1] += gyro_y; bias[2] += gyro_z;
	      HAL_Delay(1);
	  }
	for(int j=0; j<3; j++)
	  bias[j] /= 1000.0f;

	return bias;
}
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
