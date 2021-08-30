/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  LOCKED = 0,
  SETUP,
  RUN,
  IDLE
} Op_Mode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_REVOLUTIONS		1200
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
Op_Mode mode = IDLE;
uint8_t interrupt_flag = 0;
uint8_t outside_us_interrupt_flag = 0;
uint8_t inside_us_interrupt_flag = 0;
uint8_t locked_mode_buff[] = "Locked\r\n";
uint8_t locked_mode_instr_buff[] = "Open:0 Closed:1\r\n";
uint8_t setup_mode_buff[] = "Setup\r\n";
uint8_t run_mode_buff[] = "Run\r\n";
uint8_t key_buff[1];
uint8_t no_obj_buff[] = "No object detected.";
uint8_t parameter_buff[16];
uint8_t outside_dist_buff[16];
uint8_t inside_dist_buff[16];
uint16_t outside_dist = 0;
uint16_t inside_dist = 0;
uint16_t outside_pulse_start;
uint16_t outside_pulse_width = 0;
uint16_t inside_pulse_start;
uint16_t inside_pulse_width = 0;
uint16_t num_revolutions = 0;
uint8_t p1 = 2;
uint8_t p2 = 2;
uint8_t p3 = 5;
uint8_t p4 = 1;
uint8_t p5 = 1;
uint8_t command_code = 254;
uint8_t clear_code = 1;
uint8_t address_code;
uint8_t opening = 0;
uint8_t closing = 0;
uint8_t collision_occurred = 0;
uint8_t locked_pos = 2; // open: 0, closed: 1, not set: 2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void poll_keypad() {
  key_buff[0] = ' ';
  /* Read from keypad */
  /* Rotate pulses between each row and read from each column */
  for (int i = 0; i < 4; i++) {
	  if (i == 0) { // Row 1
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		  // Read columns
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) { // Column 1
			  key_buff[0] = '1';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) { // Column 2
			  key_buff[0] = '2';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) { // Column 3
			  key_buff[0] = '3';
		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	  } else if (i == 1) { // Row 2
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		  // Read columns
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) { // Column 1
			  key_buff[0] = '4';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) { // Column 2
			  key_buff[0] = '5';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) { // Column 3
			  key_buff[0] = '6';
		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	  } else if (i == 2) { // Row 3
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
		  // Read columns
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) { // Column 1
			  key_buff[0] = '7';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) { // Column 2
			  key_buff[0] = '8';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) { // Column 3
			  key_buff[0] = '9';
		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	  } else if (i == 3) { // Row 4
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
		  // Read columns
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) { // Column 1
			  key_buff[0] = '*';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) { // Column 2
			  key_buff[0] = '0';
		  } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) { // Column 3
			  key_buff[0] = '#';
		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
	  HAL_Delay(5);
  }
}

void print_LCD(uint8_t line, uint8_t col, uint8_t* msg, uint8_t msg_len) {
	if (line == 1) {
		address_code = 128 + col;
	} else if (line == 2) {
		address_code = 192 + col;
	}
	HAL_UART_Transmit(&huart6, &command_code, 1, 1000);
	HAL_UART_Transmit(&huart6, &address_code, 1, 1000);
	HAL_UART_Transmit(&huart6, msg, msg_len, 1000);
}

void clear_LCD() {
	// Clear LCD display
	HAL_UART_Transmit(&huart6, &command_code, 1, 1000);
	HAL_UART_Transmit(&huart6, &clear_code, 1, 1000);
}

/* Spin motor clockwise */
void open_door() {
	// Ramp up to 400 rpm (0-50% duty cycle)
	if (num_revolutions < 50 / p4) {
	  uint32_t pulse = (p4 * (num_revolutions + 1) * 10) - 1;
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}

	// Ramp down to decelerate (50-0% duty cycle)
	if (num_revolutions < MAX_REVOLUTIONS && num_revolutions >= MAX_REVOLUTIONS - 50 / p5) {
	  uint32_t pulse = 499 - (p5 * ((50 / p5) - (MAX_REVOLUTIONS - num_revolutions) + 1) * 10) + 1;
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}

/* Spin motor counterclockwise */
void close_door() {
	// Ramp up to 400 rpm (0-50% duty cycle)
	if (num_revolutions < 50 / p4) {
	  uint32_t pulse = (p4 * (num_revolutions + 1) * 10) - 1;
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	}

	// Ramp down to decelerate (50-0% duty cycle)
	if (num_revolutions < MAX_REVOLUTIONS && num_revolutions >= MAX_REVOLUTIONS - 50 / p5) {
	  uint32_t pulse = 499 - (p5 * ((50 / p5) - (MAX_REVOLUTIONS - num_revolutions) + 1) * 10) + 1;
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	}
}

/* Restrict to 200 rpm after collision */
void close_door_after_collision() {
	// Ramp up to 200 rpm (0-25% duty cycle)
	if (num_revolutions < 25 / p4) {
	  uint32_t pulse = (p4 * (num_revolutions + 1) * 10) - 1;
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	}

	// Ramp down to decelerate (25-0% duty cycle)
	if (num_revolutions < MAX_REVOLUTIONS && num_revolutions >= MAX_REVOLUTIONS - 25 / p5) {
	  uint32_t pulse = 249 - (p5 * ((25 / p5) - (MAX_REVOLUTIONS - num_revolutions) + 1) * 10) + 1;
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	}
}

void stop_door() {
	// Set PWM duty cycle to 100% for both inputs to turn off motor
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  uint8_t finished_setup = 0;
  uint8_t setup_stage = 1;
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
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3); //Start the TIM3 module
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Start PWM on CH1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Start PWM on CH2

  HAL_TIM_Base_Start(&htim2); //Start the TIM2 module
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //Start PWM on CH2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Get operating mode from push buttons and write to LCD display */
	  if (interrupt_flag == 1) {
		  clear_LCD();
		  HAL_Delay(20);
		  if (mode == LOCKED) {
			  print_LCD(1, 0, locked_mode_buff, sizeof(locked_mode_buff));
			  print_LCD(2, 0, locked_mode_instr_buff, sizeof(locked_mode_instr_buff));
		  } else if (mode == SETUP) {
			  finished_setup = 0;
			  setup_stage = 1;
			  print_LCD(1, 0, setup_mode_buff, sizeof(setup_mode_buff));
			  print_LCD(2, 0, parameter_buff, sprintf((char*) parameter_buff, "P%d: %d feet", 1, p1));
		  } else if (mode == RUN) {
			  print_LCD(1, 0, run_mode_buff, sizeof(run_mode_buff));
			  uint32_t pulse = (p4 * 10) - 1;
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
		  }
		  interrupt_flag = 0;
	  }

	  /* Flash Red LED when door is in motion (none of the switches are on and not in setup mode) */
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET &&
			  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET &&
			  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET &&
			  (mode == LOCKED || mode == RUN)) {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
		  HAL_Delay(100);
	  } else {
		  /* Turn off LED */
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  }

	  /* Flash Yellow LED if collision switch is hit */
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET && (mode == LOCKED || mode == RUN)) {
		  collision_occurred = 1;
		  stop_door();
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
		  HAL_Delay(100);
	  } else {
		  /* Turn off LED */
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

		  if (collision_occurred == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET && (mode == LOCKED || mode == RUN)) {
			  if (opening == 1) {
				  num_revolutions = MAX_REVOLUTIONS - num_revolutions;
			  }
			  // Start closing door
			  closing = 1;
			  opening = 0;
		  }
	  }

	  // Get distance from outside sensor
	  if (outside_us_interrupt_flag == 1 && mode == RUN) {
		  outside_dist = outside_pulse_width * 10; // in microseconds
		  if (outside_dist == 38000) { // 38 ms
			  // No object
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		  } else {
			  outside_dist = outside_dist / 58; // cm
			  /* Turn on Green LED1 if outside sensor detects object within P1 distance */
			  if (outside_dist <= p1 * 30.48) {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
				  if (closing == 1) {
					  num_revolutions = MAX_REVOLUTIONS - num_revolutions;
				  }
				  // Open door when object is nearby
				  opening = 1;
				  closing = 0;
			  } else {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			  }
		  }
		  outside_us_interrupt_flag = 0;
	  }

	  // Get distance from inside sensor
	  if (inside_us_interrupt_flag == 1 && mode == RUN) {
		  inside_dist = inside_pulse_width * 10; // in microseconds
		  if (inside_dist == 38000) { // 38 ms
			  // No object
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		  } else {
			  inside_dist = inside_dist / 58; // cm
			  /* Turn on Green LED2 if inside sensor detects object within P2 distance */
			  if (inside_dist <= p2 * 30.48) {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
				  if (closing == 1) {
					  num_revolutions = MAX_REVOLUTIONS - num_revolutions;
				  }
				  // Open door when object is nearby
				  opening = 1;
				  closing = 0;
			  } else {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			  }
		  }
		  inside_us_interrupt_flag = 0;
	  }

	  /* LOCKED mode */
	  if (mode == LOCKED) {
		  // Get input from user (fully open or fully close)
		  poll_keypad();
		  if (key_buff[0] == '0') { // Open position
			  locked_pos = 0;
		  } else if (key_buff[0] == '1') { // Closed position
			  locked_pos = 1;
		  }

		  if (locked_pos == 0) { // Open
			  open_door();
		  } else if (locked_pos == 1) { // Close
			  if (collision_occurred == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {
				  close_door_after_collision();
			  } else if (collision_occurred == 0) {
				  close_door();
			  }
		  }

		  /* Disable door motor power when limit switches are hit (fully closed/fully open) */
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) {
			  locked_pos = 2; // clear position
			  stop_door();
			  collision_occurred = 0;
			  num_revolutions = 0;
		  }
	  }

	  /* SETUP mode: Get settings from user */
	  if (mode == SETUP) {
		  // Disable door motor
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999);

		  if (!finished_setup) {
			  poll_keypad();

			  // Move to next setup stage
			  if (key_buff[0] == '#') {
				  setup_stage = (setup_stage % 5) + 1;
			  }

			  // Finish setup
			  if (key_buff[0] == '*') {
				  finished_setup = 1;
				  clear_LCD();
			  } else {
				  if (setup_stage == 1) { // p1
					  if (key_buff[0] >= '1' && key_buff[0] <= '4') {
						  p1 = (key_buff[0] - 48) * 2;
					  }
					  sprintf((char*) parameter_buff, "P%d: %d feet", setup_stage, p1);
				  } else if (setup_stage == 2) { // p2
					  if (key_buff[0] >= '1' && key_buff[0] <= '4') {
						  p2 = (key_buff[0] - 48) * 2;
					  }
					  sprintf((char*) parameter_buff, "P%d: %d feet", setup_stage, p2);
				  } else if (setup_stage == 3) { // p3
					  if (key_buff[0] >= '1' && key_buff[0] <= '4') {
						  p3 = (key_buff[0] - 48) * 5;
					  }
					  sprintf((char*) parameter_buff, "P%d: %d sec", setup_stage, p3);
				  } else if (setup_stage == 4) { // p4
					  if (key_buff[0] >= '1' && key_buff[0] <= '3') {
						  if (key_buff[0] == '1') {
							  p4 = 1;
						  } else if (key_buff[0] == '2') {
							  p4 = 2;
						  } else if (key_buff[0] == '3') {
							  p4 = 5;
						  }
					  }
					  sprintf((char*) parameter_buff, "P%d: %d\r\n\r\n", setup_stage, p4);
				  } else if (setup_stage == 5) { // p5
					  if (key_buff[0] >= '1' && key_buff[0] <= '3') {
						  if (key_buff[0] == '1') {
							  p4 = 1;
						  } else if (key_buff[0] == '2') {
							  p4 = 2;
						  } else if (key_buff[0] == '3') {
							  p4 = 5;
						  }
					  }
					  sprintf((char*) parameter_buff, "P%d: %d\r\n\r\n", setup_stage, p5);
				  }

				  if (key_buff[0] != ' ') {
					  print_LCD(2, 0, parameter_buff, 16);
				  }
			  }
		  }
	  }

	  if (mode == RUN) {
		  // 1200 revolutions == 8ft (for 150 rev/foot)

		  // Open door (spin motor clockwise)
		  if (opening == 1) {
			  /* Disable door motor power when max limit switch is hit (fully open) */
			  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) {
				  stop_door();
				  opening = 0;
				  // Keep door open for p3 seconds after fully open
				  HAL_Delay(p3 * 1000); // ms
				  closing = 1; // Close door
				  num_revolutions = 0;
			  } else {
				  open_door();
			  }
		  } else if (closing == 1) { // Close door (spin motor counterclockwise)
			  /* Disable door motor power when min limit switch is hit (fully closed) */
			  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
				  stop_door();
				  closing = 0;
				  collision_occurred = 0;
				  num_revolutions = 0;
			  } else {
				  if (collision_occurred == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {
					  close_door_after_collision();
				  } else if (collision_occurred == 0) {
					  close_door();
				  }
			  }
		  }
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Get pulse width from ultrasonic sensors */
	if (GPIO_Pin == GPIO_PIN_1) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) { // Rising edge
			outside_pulse_start = __HAL_TIM_GetCounter(&htim2);
		} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) { // Falling edge
			outside_pulse_width = __HAL_TIM_GetCounter(&htim2) - outside_pulse_start;
			outside_us_interrupt_flag = 1;
		}
	}
	if (GPIO_Pin == GPIO_PIN_2) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) { // Rising edge
			inside_pulse_start = __HAL_TIM_GetCounter(&htim2);
		} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) { // Falling edge
			inside_pulse_width = __HAL_TIM_GetCounter(&htim2) - inside_pulse_start;
			inside_us_interrupt_flag = 1;
		}
	}

	/* Get number of revolutions from motor */
	if (GPIO_Pin == GPIO_PIN_10) {
		num_revolutions++;
	}

	/* Get operating mode from push buttons and write to LCD display */
	if (GPIO_Pin == GPIO_PIN_4) {
		interrupt_flag = 1;
		mode = LOCKED;
	}
	if (GPIO_Pin == GPIO_PIN_5) {
		interrupt_flag = 1;
		mode = SETUP;
	}
	if (GPIO_Pin == GPIO_PIN_6) {
		interrupt_flag = 1;
		mode = RUN;
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
