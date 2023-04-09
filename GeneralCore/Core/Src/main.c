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
#include "collab_util.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LED1_ON	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1)
#define LED1_OFF	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0)
#define LED1_TOGGLE	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
#define LED2_ON	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1)
#define LED2_OFF	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0)
#define LED2_TOGGLE	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)
#define LED3_ON 	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1)
#define LED3_OFF	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0)
#define LED3_TOGGLE	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)

#define LED1_BLINK(n) for(uint8_t i = 0; i < n; i++){LED1_ON; HAL_Delay(150); LED1_OFF; HAL_Delay(150);}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

// instances
Motor_HandleTypeDef cmotor_lf, cmotor_rf, cmotor_lb, cmotor_rb;
JY62_HandleTypeDef himu;
XB_HandleTypeDef hxb;

// game information 1
uint8_t gameStage;		    // 0: pre-match(standby); 1: first half; 2: second half
uint8_t gameStatus;			// 0: standby; 1: running
uint8_t task_mode = 0;          // 0:set battery 1: get packet 2:send packet 3:charge

uint32_t gameStageTimeLimit;		// in ms
uint32_t gameStageTimeSinceStart;	// in ms
Rectangle obstacles[5];			// area that depletes charge faster
Coordinate allyBeacons[3];		// ally charging station coordinate
Coordinate oppoBeacons[3];		// opponent charging station coordinate
Coordinate want_allyBeacons[3];
Coordinate exitpoints[8];         	// record the exit.

// game information 2
Order *delivering[8];		// package picked up but not yet delivered
uint8_t delivering_num = 0;
uint8_t allyBeacons_num = 0;
uint8_t oppoBeacons_num = 0;
extern Order_list orders;

// game information 3
Coordinate myCoord;			// precise coordinate returned by game master
fCoordinate EstiCoord;       // predict coordinate
uint8_t CoordinateUpdate;   // 0 is not Update, 1 is Update
uint8_t overtime = 0;

float angleZ;
double omegaZ, accelY;		// turning speed and linear acceleration
float initangleZ;                  // init angleZ
float myScore;				// current score returned by Master
int32_t myCharge;				// current charge returned by Master

// interchange information 1
uint32_t gameStageTimeLeft;		// in ms
// OLED display buffer
char firstLine[22], secondLine[22], thirdLine[22], fourthLine[22];		// 128 / 6 = 21

Coordinate merchant, consumer, charge;

// debug information
uint8_t jy62_DMA_ErrorCount, jy62_IT_SuccessCount, xb_DMA_SW_ErrorCount, xb_DMA_HW_ErrorCount,
		xb_IT_SuccessCount = 0;
uint8_t jy62_uart_normal, xb_uart_normal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void HUAN_MOTOR1_Init(void);
static void HUAN_MOTOR2_Init(void);
static void HUAN_MOTOR3_Init(void);
static void HUAN_MOTOR4_Init(void);
static void HUAN_IMU_Init(void);
static void HUAN_ZIGBEE_Init(void);

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
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	//Motor initialization part
	cmotor_lf.encoderInverted = 1;
	cmotor_lb.encoderInverted = 1;
	HUAN_MOTOR1_Init();
	HUAN_MOTOR2_Init();
	HUAN_MOTOR3_Init();
	HUAN_MOTOR4_Init();
	HUAN_IMU_Init();
	HUAN_ZIGBEE_Init();
	ssd1306_Init();
	huansic_order_init();
	order_list_init();
	exitpoints_init();
	// tick per motor rev = 1080 (measured)
	// tick per rotor rev = 54 (calculated)
	// reduction ratio = 20 (given)

	//Set PID timer after data stables
	HAL_Delay(20);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	jy62_DMA_ErrorCount = 0;
	jy62_IT_SuccessCount = 0;
	xb_DMA_SW_ErrorCount = 0;
	xb_DMA_HW_ErrorCount = 0;
	xb_IT_SuccessCount = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	sprintf(firstLine, "    ERR   SUC");
	sprintf(secondLine, "XB");
	sprintf(thirdLine, "JY");
	ssd1306_WriteString(firstLine, Font_6x8, White);
	ssd1306_SetCursor(0, 8);
	ssd1306_WriteString(secondLine, Font_6x8, White);
	ssd1306_SetCursor(0, 16);
	ssd1306_WriteString(thirdLine, Font_6x8, White);
	ssd1306_UpdateScreen();

	CoordinateUpdate = 0;
	int8_t merchant_index = -1;

	while (1) {
		// test code to ensure the motor can work
//    	HAL_Delay(3000);
//    	cmotor_rb.goalSpeed = 2000;
//    	while(1){
//
//    	}
//    	huansic_xb_requestGameInfo(&hxb);
//    	cmotor_lb.goalSpeed = 1000;
//		HAL_Delay(1000);
//		cmotor_lb.goalSpeed = -1000;
//		HAL_Delay(1000);
//		chao_move_angle(135, 1000);
//		HAL_Delay(1000);
//		chao_move_angle(90, 2000);
//		HAL_Delay(1000);
//		chao_move_angle(180, 2000);
//		HAL_Delay(1000);
//		chao_move_angle(270, 2000);

		if (gameStatus == 0) {		// if the game is not running
//	    	LED1_ON;
//	    	HAL_Delay(1000);
//	    	LED1_OFF;
		}
		else
		{
			while (gameStage == 0) {		// pre-match
				chao_move_angle(0, 0);
				// find angle offset
				//initangleZ = -himu.theta[2];
				// do some initialization
				Cal_Battery_Coord();
				// get obstacle list
				huansic_xb_requestGameInfo(&hxb);
				task_mode = 0;
			}

			while (gameStage == 1) {			// first-half
				if (task_mode == 0) {
					//setChargingPile
					set_Beacons();
					while (orders.length == 0)
					{
						chao_move_angle(0, 0);
					}
					task_mode = 4;
				}
				else if(hxb.lastUpdated + 1000 < HAL_GetTick()) {
					chao_move_angle(0, 0);
				}
				else {
					if (task_mode == 1) {
						for (uint8_t i = merchant_index + 1; i < orders.length; i++)
								{
							orders.buffer[i - 1] = orders.buffer[i];
						}
						orders.length -= 1;
						Get_packet(merchant);
						task_mode = 4;
						HAL_Delay(100);
					} else if (task_mode == 2) {
						Send_packet(consumer);
						task_mode = 4;
					}
					else if (task_mode == 4)			// if task_mode == 4
					{
						merchant_index = Get_nearest_order();
						if (merchant_index == -1)
							merchant = myCoord;
						else
							merchant = orders.buffer[merchant_index];
						consumer = Get_nearest_consumer();

						if (delivering_num > 4) {
							task_mode = 2;
						}
						else if (merchant_index == -1)
						{
							if (delivering_num == 0) {
								move_random();
							}
							else {
								task_mode = 2;
							}
						}
						else if (delivering_num == 0) {
							task_mode = 1;
						}
						else if (overtime == 1) {
							task_mode = 2;
							overtime = 0;
						}
						else if (gameStageTimeLeft < 7000 && delivering_num > 0) {
							task_mode = 2;
						}
						else if ((abs(merchant.x - myCoord.x) + abs(merchant.y - myCoord.y))
								< (abs(consumer.x - myCoord.x) + abs(consumer.y - myCoord.y))) {
							task_mode = 1;
						} else {
							task_mode = 2;
						}
					}
				}

			}

			while (gameStage == 2) {			// second-half
				while (myCharge < 500)
				{
//					huansic_xb_requestGameInfo(&hxb);
					charge = Get_nearest_Beacon();
					GotoDestination(charge, 0);
				}
				if(hxb.lastUpdated + 2000 < HAL_GetTick()) {
					chao_move_angle(0, 0);
				}
				else if (task_mode == 1) {
					for (uint8_t i = merchant_index + 1; i < orders.length; i++)
							{
						orders.buffer[i - 1] = orders.buffer[i];
					}
					orders.length -= 1;
					Get_packet(merchant);
					task_mode = 4;
					HAL_Delay(100);
				} else if (task_mode == 2) {
					Send_packet(consumer);
					task_mode = 4;
				}
				else if (task_mode == 4)			// if task_mode == 4
						{
					merchant_index = Get_nearest_order();
					if (merchant_index == -1)
						merchant = myCoord;
					else
						merchant = orders.buffer[merchant_index];
					consumer = Get_nearest_consumer();

					if (delivering_num > 4) {
						task_mode = 2;
					}
					else if (merchant_index == -1)
							{
						if (delivering_num == 0) {
							charge = Get_nearest_Beacon();
//							if(abs(myCoord.x - charge.x)+abs(myCoord.y-charge.y) < 5)
							GotoDestination(charge, 0);
						}
						else {
							task_mode = 2;
						}
					}
					else if (delivering_num == 0) {
						task_mode = 1;
					}
					else if (overtime == 1) {
						task_mode = 2;
						overtime = 0;
					}
					else if (gameStageTimeLeft < 10000 && delivering_num > 0) {
						task_mode = 2;
					}
					else if ((abs(merchant.x - myCoord.x) + abs(merchant.y - myCoord.y))
							< (abs(consumer.x - myCoord.x) + abs(consumer.y - myCoord.y))) {
						task_mode = 1;
					} else {
						task_mode = 2;
					}
				}
				else
					task_mode = 4;
			}
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
			{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

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
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
			{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
			{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
			{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
			{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
			{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
			{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
			{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
			{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 65535;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
			{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 72 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 50000;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
			{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 7200 - 1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 10000 - 1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
			{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 65535;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
			{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
			{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
			{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
			{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
			{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
			{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
			{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
			{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
			{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void HUAN_MOTOR1_Init(void) {
	cmotor_lf.counter = &htim2;
	cmotor_lf.dt = 0.05;
	cmotor_lf.posTimer = &htim1;
	cmotor_lf.pos_channel = TIM_CHANNEL_4;
//	cmotor_lf.pos_channel = TIM_CHANNEL_3;
	cmotor_lf.negTimer = &htim1;
	cmotor_lf.neg_channel = TIM_CHANNEL_3;
//	cmotor_lf.neg_channel = TIM_CHANNEL_4;
	cmotor_lf.encoderInverted = 1;
	cmotor_lf.kp = 0.0005;
	cmotor_lf.ki = 0.000015;
	cmotor_lf.kd = 0.00003;
	huansic_motor_init(&cmotor_lf);
}

static void HUAN_MOTOR2_Init(void) {
	cmotor_rf.counter = &htim4;
	cmotor_rf.dt = 0.05;
	cmotor_rf.posTimer = &htim1;
	cmotor_rf.pos_channel = TIM_CHANNEL_1;
	cmotor_rf.negTimer = &htim1;
	cmotor_rf.neg_channel = TIM_CHANNEL_2;
	cmotor_rf.encoderInverted = 0;
	cmotor_rf.kp = 0.0005;
	cmotor_rf.ki = 0.000015;
	cmotor_rf.kd = 0.00003;
	huansic_motor_init(&cmotor_rf);
}

static void HUAN_MOTOR3_Init(void) {
	cmotor_lb.counter = &htim3;
	cmotor_lb.dt = 0.05;
	cmotor_lb.posTimer = &htim8;
	cmotor_lb.pos_channel = TIM_CHANNEL_4;
//	cmotor_lb.pos_channel = TIM_CHANNEL_3;
	cmotor_lb.negTimer = &htim8;
	cmotor_lb.neg_channel = TIM_CHANNEL_3;
//	cmotor_lb.neg_channel = TIM_CHANNEL_4;
	cmotor_lb.encoderInverted = 1;
	cmotor_lb.kp = 0.0005;
	cmotor_lb.ki = 0.000015;
	cmotor_lb.kd = 0.00003;
	huansic_motor_init(&cmotor_lb);
}

static void HUAN_MOTOR4_Init(void) {
	cmotor_rb.counter = &htim5;
	cmotor_rb.dt = 0.05;
	cmotor_rb.posTimer = &htim8;
	cmotor_rb.pos_channel = TIM_CHANNEL_1;
	cmotor_rb.negTimer = &htim8;
	cmotor_rb.neg_channel = TIM_CHANNEL_2;
	cmotor_rb.encoderInverted = 0;
	cmotor_rb.kp = 0.0005;
	cmotor_rb.ki = 0.000015;
	cmotor_rb.kd = 0.00003;
	huansic_motor_init(&cmotor_rb);
}

static void HUAN_IMU_Init(void) {
	himu.huart = &huart3;
	himu.hdma = &hdma_usart3_rx;
	huansic_jy62_init(&himu);
}

static void HUAN_ZIGBEE_Init(void) {
	hxb.huart = &huart2;
	hxb.hdma = &hdma_usart2_rx;
	huansic_xb_init(&hxb);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (himu.huart == huart) {
		jy62_uart_normal = 1;
		if (himu.pending_alignment) {
			if (!huansic_jy62_isr(&himu))
				jy62_IT_SuccessCount++;
		} else {
			if (huansic_jy62_dma_isr(&himu))
				jy62_DMA_ErrorCount++;
		}
	} else if (hxb.huart == huart) {
		xb_uart_normal = 1;
		if (hxb.pending_alignment) {
			if (!huansic_xb_isr(&hxb))
				xb_IT_SuccessCount++;
		} else {
			if (huansic_xb_dma_isr(&hxb))
				xb_DMA_SW_ErrorCount++;
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (himu.huart == huart) {
		jy62_uart_normal = 1;
		if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
			__HAL_DMA_CLEAR_FLAG(himu.hdma, DMA_FLAG_TE2 | DMA_FLAG_HT2 | DMA_FLAG_TC2 | DMA_FLAG_GL2);
			huansic_jy62_dma_error(&himu);
			jy62_DMA_ErrorCount++;
		} else {
			if (himu.huart->ErrorCode & HAL_UART_ERROR_ORE) {
				__HAL_UART_CLEAR_OREFLAG(himu.huart);
			}
			huansic_jy62_it_error(&himu);
		}
	} else if (hxb.huart == huart) {
		xb_uart_normal = 1;
		if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
//			HAL_UART_DeInit(&huart2);
//			MX_USART2_UART_Init();
			__HAL_DMA_CLEAR_FLAG(hxb.hdma, DMA_FLAG_TE6 | DMA_FLAG_HT6 | DMA_FLAG_TC6 | DMA_FLAG_GL6);
			huansic_xb_dma_error(&hxb);
			xb_DMA_HW_ErrorCount++;
		} else {
			if (hxb.huart->ErrorCode & HAL_UART_ERROR_ORE) {
				__HAL_UART_CLEAR_OREFLAG(hxb.huart);
			}
//			HAL_UART_DeInit(&huart2);
//			MX_USART2_UART_Init()
			huansic_xb_it_error(&hxb);
		}
	}
	huart->ErrorCode = HAL_UART_ERROR_NONE;
}

void HUAN_PeriodicInt1000ms_ISR(void) {
	sprintf(firstLine, "    ERR   SUC");
	sprintf(secondLine, "XB  %02X    %02X", xb_DMA_HW_ErrorCount, xb_IT_SuccessCount);
	sprintf(thirdLine, "JY  %02X    %02X", jy62_DMA_ErrorCount, jy62_IT_SuccessCount);
	ssd1306_WriteString(firstLine, Font_6x8, White);
	ssd1306_SetCursor(0, 8);
	ssd1306_WriteString(secondLine, Font_6x8, White);
	ssd1306_SetCursor(0, 16);
	ssd1306_WriteString(thirdLine, Font_6x8, White);
	ssd1306_UpdateScreen();

	// check status of UART
	if (!jy62_uart_normal) {
		himu.huart->Instance->DR;		// just read
		huansic_jy62_it_error(&himu);
	}
	if (!xb_uart_normal) {
		hxb.huart->Instance->DR;		// just read
		huansic_xb_it_error(&hxb);
	}
	jy62_uart_normal = 0;
	xb_uart_normal = 0;
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
	while (1) {
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
