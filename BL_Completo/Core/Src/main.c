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
#include <stdlib.h>
#include <stdbool.h>
#include "math.h"

#include "Configuration.h"
#include "bno055_stm32.h"
#include "servo_motor.h"
#include "BL_motor.h"
#include "PID.h"
#include "Raspberry_UART.h"
#include "manovre.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct SerialData {
	int flag;
	/*
	 * 1: Lanekeeping
	 * 2: Segue una curva
	 * 3: Sorpasso
	 * 4: Parcheggio
	 * 5: Cambiamento velcità
	 * 6: spostamento lineare di n metri
	 */

	float curvature_radius_ref_m; // [m]
	float linear_speed_ref_m_s; //[m/s]

	//Manovra
	float dy; //[m]
	float dx; //[m]

	//Stop
	float distanza_frenata; //[m]
	float old_linear_speed_ref_m_s; //[m/s]
} serialData;

typedef struct VehicleData {
	//Trazione
	int counts;
	int ref_count;
	int delta_count;
	float delta_angle_deg; // [°]
	float motor_speed_deg_sec; // [°/s]
	float motor_speed_RPM; // RPM
	float linear_speed_m_s; //[m/s]
	float motor_speed_ref_RPM; // RPM

	//Sterzo
	double yaw_rate_rad_sec; // [rad/s]
	double yaw_rate_deg_sec; // [°/s]
	double yaw_rate_ref_rad_sec; //[rad/s]
	double x_acceleration;
} vehicleData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_VALUES 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

serialData data;
vehicleData vehicleState;

//PID
PID pid_traction, pid_traction_RWD, pid_traction_DESC, pid_steering, pid_steering_STR;
float u_trazione = 0;
float u_sterzo = 0;

//Event control
int max_flag_button; //Definisce quanti click diversi del pulsante posso fare
int flag_button = 0; //Gestisce le premute del pulsante
int flag_10ms = 0; //Temporizzazione per l'encoder e la trazione
int flag_serial_read; //Serve per fermare la lettura della seriale

//Parcheggio
float tempo_salita_pid;
float cnt_manovre = 0; //Conta il tempo che passa durante la manovra, usato nella formula
int flag_ingresso_parcheggio = 0;
	/* flag_ingresso_parcheggio
	 * 0: Muoviti avanti
	 * 1: Parcheggia
	 * 2: Sistemati dentro il parcheggio
	 */
int flag_uscita_parcheggio = 0;
	/* flag_uscita_parcheggio
	 * 0: Sistemati dentro il parcheggio
	 * 1: Esci dal parchegigo
	 */

//Accelerazione e decellerazione
float current_linear_speed_ref_m_s;
int cambiamento_velocita = 0;

//Rampa
int flag_discesa = 0;

//Procedura per eseguire percorso
int flag_procedura = 0;
int cnt_procedura = 0;

//User button press
uint32_t buttonPressStartTime = 0;
uint32_t buttonPressEndTime = 0;
uint32_t pressDuration = 0;
uint32_t cnt_10ms_button = 0; //Aumenta indefinitamente ogni 10ms

//Variabili per calibrazione ESC
int counter_cal_ESC = 0;
float duty;

//Serial input var
uint8_t msg[45] = { "\0" };
float floatArray[MAX_VALUES];
int flag_controllo = 0;
int flag_msg_ok = 1;

//Comunicazione interrupt
uint8_t app[2] = {'\0'};
int stato_rx = 0;
int msg_len = 0;
int rx_complete = 0;
int pt_rx = 0;

//Conversione velocità
float RPM_2_m_s = ((2 * M_PI / 60) * WHEEL_RADIUS / MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION) / DIFFERENTIAL_GEARBOX_RATIO;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
//void CheckPressDuration();
void ProceduraCalibrazione();
void lettura_vel_encoder();
void controllo_trazione(float, float);
void lane_keeping();
void ingresso_parcheggio();
void uscita_parcheggio();
void parcheggio();
void get_inclination();
void get_yaw_rate();
float valore_assoluto(float);
void cambio_velocita(float);
void muoviti_n_metri_dritto(float, float);
void sistemazione_parcheggio(float, float);
void cambio_corsia();
void PrintState();
void lettura_seriale();
void procedura();
void next_procedura(float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6) {
		// Abilita la ricezione di un altro carattere
		HAL_UART_Receive_IT(&huart6, msg, sizeof(msg));
		printf("Ciao\r\n");
		//PrintState();
	}
}
*/
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	//PWM Servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//PWM DC motor
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	//ENCODER TIMER
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	//10ms TIMER
	HAL_TIM_Base_Start_IT(&htim11);

	//PID traction
	init_PID(&pid_traction, TRACTION_SAMPLING_TIME, MAX_U_TRACTION, MIN_U_TRACTION);
	tune_PID(&pid_traction, KP_TRACTION, KI_TRACTION, 0);

	init_PID(&pid_traction_RWD, TRACTION_SAMPLING_TIME, MAX_U_TRACTION, MIN_U_TRACTION);
	tune_PID(&pid_traction_RWD, KP_TRACTION_RWD, KI_TRACTION_RWD, 0);

	init_PID(&pid_traction_DESC, TRACTION_SAMPLING_TIME, MAX_U_TRACTION, MIN_U_TRACTION);
	tune_PID(&pid_traction_DESC, KP_TRACTION_DESC, KI_TRACTION_DESC, 0);

	//PID steering per le curve
	init_PID(&pid_steering, STEERING_SAMPLING_TIME, MAX_U_STEERING, MIN_U_STEERING);
	tune_PID(&pid_steering, KP_STEERING, KI_STEERING, 0);

	//PID steering per le curve
	//init_PID(&pid_steering_STR, STEERING_SAMPLING_TIME, MAX_U_STEERING, MIN_U_STEERING);
	//tune_PID(&pid_steering_STR, KP_STEERING_STR, KI_STEERING_STR, 0);

	// IMU BNO055 config
	HAL_I2C_IsDeviceReady(&hi2c1, BNO055_I2C_ADDR << 1, 5, 1000);
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	servo_motor(0);
	data.flag = 6;
	flag_serial_read = 1;

	printf("Initialization Completed!\r\n");

	//HAL_UART_Receive_IT(&huart6, (uint8_t*)(msg + msg_index), 1);
	HAL_UART_Receive_IT(&huart6, app, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//printf("Ciclo while\r\n");
		//-------------------------------------------------------------


		//-------------------------------------------------------------
		//GESTIONE PULSANTE
		max_flag_button = 3;

		switch(flag_button){
		//Calibrazione
		case -1:
			data.flag = -1;
			ProceduraCalibrazione();
			break;
			//Idle
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			data.flag = 4;
			data.linear_speed_ref_m_s = -0.20; //[m/s]
			data.curvature_radius_ref_m = 1.05; // [m]
			data.dy = 0.35; //[m]
			data.dx = 1.00; //[m]
			data.distanza_frenata = 0; //[m]
			data.old_linear_speed_ref_m_s = 0; //[m]
			//data.flag= 6;
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

			data.flag = 1;
			data.linear_speed_ref_m_s = 0.00; //[m/s]
			data.curvature_radius_ref_m = 10; // [m]
			data.dy = 0.35; //[m]
			data.dx = 1.00; //[m]
			data.distanza_frenata = 0; //[m]
			data.old_linear_speed_ref_m_s = 0; //[m]

			if(cnt_manovre >= 100){
				cnt_manovre = 0;
				flag_button = 3;
			}
			break;
		case 2:
			if(flag_discesa){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			} else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			}

			//lettura_seriale();
			//PrintState();
			break;
		case 3:
			if(flag_discesa){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			} else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			}

			data.flag = 4;
			data.linear_speed_ref_m_s = -0.20; //[m/s]
			data.curvature_radius_ref_m = 1.05; // [m]
			data.dy = 0.35; //[m]
			data.dx = 1.00; //[m]
			data.distanza_frenata = 0; //[m]
			data.old_linear_speed_ref_m_s = 0; //[m]
			break;
		}

		//-------------------------------------------------------------
		//PERCORSO AUTOMATICO
		if(flag_button == 3){
			procedura();
		}

		//-------------------------------------------------------------
		//CONTROLLO
		if (data.flag > 0 && data.flag < 6) {
			if (flag_10ms == 1) { //Car enabled from user
				flag_10ms = 0;

				//Measure speed with encoder
				lettura_vel_encoder();

				//TRACTION control
				switch (data.flag){
				case 1:
				case 2:
				case 3:
					controllo_trazione(vehicleState.motor_speed_RPM, data.linear_speed_ref_m_s / RPM_2_m_s);
					break;
				case 5:
					cambio_velocita(data.distanza_frenata);
					break;
				}

				//-------------------------------------------------------------

				get_yaw_rate();
				get_inclination();

				//STEERING control
				switch (data.flag){
				case 3:
					cambio_corsia();
					break;
				case 4:
					parcheggio();
					break;
				case 1:
				case 2:
				case 5:
					lane_keeping();
					break;
				}
			//printf("%f;%f\r\n", u_trazione, vehicleState.linear_speed_m_s);
		}
	} else if (data.flag == 6)
	{
		BL_set_PWM(NEUTRAL_PWM);
		servo_motor(0);
	}

	//-------------------------------------------------------------

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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_11;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1681-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1001-1;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 840-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 2000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 120-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 7000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart6.Init.Mode = UART_MODE_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//-------------------------------------------------------------
//Timer11 for temporization (10ms)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		flag_10ms = 1;

		//Encoder
		vehicleState.counts = TIM2->CNT;
		TIM2->CNT = TIM2->ARR / 2;

		//Variabile per la calibrazione
		cnt_10ms_button++;

		//Variabile per la manovra
		if(data.flag == 3 || data.flag == 4 || flag_button == 1){
			cnt_manovre++;
		}

		//Temporizzazione della calibrazione
		if(flag_button == -1){
			counter_cal_ESC++;
		}

		if(flag_button == 3){
			cnt_procedura++;
		}
	}
}

//-------------------------------------------------------------
//MISURAZIONI
void lettura_vel_encoder(){
	vehicleState.ref_count = TIM2->ARR / 2;
	vehicleState.delta_count = vehicleState.counts - vehicleState.ref_count;

	vehicleState.delta_angle_deg = (vehicleState.delta_count * 360) / ((double) (ENCODER_PPR * ENCODER_COUNTING_MODE * GEARBOX_RATIO));
	vehicleState.motor_speed_deg_sec = vehicleState.delta_angle_deg / ENCODER_SAMPLING_TIME;
	vehicleState.motor_speed_RPM = BL_DegreeSec2RPM(vehicleState.motor_speed_deg_sec);
	vehicleState.linear_speed_m_s = vehicleState.motor_speed_RPM * RPM_2_m_s;

	//Speed reference for motor
	vehicleState.motor_speed_ref_RPM = data.linear_speed_ref_m_s / RPM_2_m_s;
}

void get_yaw_rate()
{
	bno055_vector_t v = bno055_getVectorGyroscope();
	vehicleState.yaw_rate_deg_sec = v.z;
	vehicleState.yaw_rate_rad_sec = (vehicleState.yaw_rate_deg_sec * M_PI) / 180;
}

void get_inclination()
{
	bno055_vector_t u = bno055_getVectorGravity();
	vehicleState.x_acceleration = u.x;

	if(vehicleState.x_acceleration < -0.6){
		flag_discesa = 1;
	} else{
		flag_discesa = 0;
	}
	//printf("%d, %f\r\n", flag_discesa, vehicleState.x_acceleration);
}

void controllo_trazione(float v, float v_ref){
	//Fa la scelta tra il PID che va avanti e quello che va indietro
	if(v_ref >= 0){
		if(flag_discesa){
			u_trazione = PID_controller(&pid_traction_DESC, v, v_ref, NEUTRAL_PWM);
		} else {
			u_trazione = PID_controller(&pid_traction, v, v_ref, NEUTRAL_PWM);
		}
	} else{
		u_trazione = PID_controller(&pid_traction_RWD, v, v_ref, NEUTRAL_PWM);
	}

	BL_set_PWM(u_trazione);
}

//-------------------------------------------------------------
//LANE KEEPING
void lane_keeping()
{
/*
	if (valore_assoluto(data.curvature_radius_ref_m) >= MAX_CURVATURE_RADIUS_FOR_STRAIGHT)
	{
		//vehicleState.yaw_rate_ref_rad_sec = data.linear_speed_ref_m_s / data.curvature_radius_ref_m;
		//vehicleState.yaw_rate_ref_rad_sec = vehicleState.linear_speed_m_s / data.curvature_radius_ref_m;
		vehicleState.yaw_rate_ref_rad_sec = 0;
		u_sterzo = PID_controller(&pid_steering,vehicleState.yaw_rate_rad_sec, vehicleState.yaw_rate_ref_rad_sec, 0);
		servo_motor(-u_sterzo); //minus because yawrate and steering are opposite
		printf("DRITTO\r\n");
	} else
	{
		vehicleState.yaw_rate_ref_rad_sec = data.linear_speed_ref_m_s / data.curvature_radius_ref_m;
		//vehicleState.yaw_rate_ref_rad_sec = vehicleState.linear_speed_m_s / data.curvature_radius_ref_m;

		float yaw_rate_ref_rad_sec_abs = valore_assoluto(vehicleState.yaw_rate_ref_rad_sec);
		float yaw_rate_rad_sec_abs = valore_assoluto(vehicleState.yaw_rate_rad_sec);

		u_sterzo = PID_controller(&pid_steering, yaw_rate_rad_sec_abs, yaw_rate_ref_rad_sec_abs, 0);

		//minus because yawrate and steering are opposite
		if (data.curvature_radius_ref_m >= 0 && u_sterzo > 0)
			u_sterzo *= -1.0;
		if (data.curvature_radius_ref_m < 0 && u_sterzo < 0)
			u_sterzo *= -1.0;

		servo_motor(u_sterzo);
		printf("CURVA\r\n");
	}
*/

	vehicleState.yaw_rate_ref_rad_sec = data.linear_speed_ref_m_s / data.curvature_radius_ref_m;
	//vehicleState.yaw_rate_ref_rad_sec = vehicleState.linear_speed_m_s / data.curvature_radius_ref_m;

	float yaw_rate_ref_rad_sec_abs = valore_assoluto(vehicleState.yaw_rate_ref_rad_sec);
	float yaw_rate_rad_sec_abs = valore_assoluto(vehicleState.yaw_rate_rad_sec);

	u_sterzo = PID_controller(&pid_steering, yaw_rate_rad_sec_abs, yaw_rate_ref_rad_sec_abs, 0);

	//minus because yawrate and steering are opposite
	if (data.curvature_radius_ref_m >= 0 && u_sterzo > 0)
		u_sterzo *= -1.0;
	if (data.curvature_radius_ref_m < 0 && u_sterzo < 0)
		u_sterzo *= -1.0;

	servo_motor(u_sterzo);
	//printf("%f\r\n", vehicleState.yaw_rate_rad_sec);
	//printf("%f\r\n", u_sterzo);

}

//-------------------------------------------------------------
//PARCHEGGIO
void parcheggio()
{
	if (data.linear_speed_ref_m_s < 0)
	{ //Entro nel parcheggio
		switch (flag_ingresso_parcheggio){
		case 0:
			muoviti_n_metri_dritto(1.70, 0.20);
			break;
		case 1:
			ingresso_parcheggio();
			break;
		case 2:
			sistemazione_parcheggio(0.15, 0.20);
			break;
		}
	}
	else if (data.linear_speed_ref_m_s > 0) //Esco dal parcheggio
	{
		switch (flag_uscita_parcheggio){
		case 0:
			sistemazione_parcheggio(0.25, -0.20);
			break;
		case 1:
			uscita_parcheggio();
			break;
		}
	}
}

void ingresso_parcheggio(){

	vehicleState.yaw_rate_ref_rad_sec = calcolo_yaw_rate(data.dy, data.dx, data.linear_speed_ref_m_s, cnt_manovre/100);
	u_sterzo = PID_controller(&pid_steering, vehicleState.yaw_rate_rad_sec, vehicleState.yaw_rate_ref_rad_sec, 0);
	controllo_trazione(vehicleState.motor_speed_RPM, data.linear_speed_ref_m_s / RPM_2_m_s);
	//printf("%f; %f \r\n", vehicleState.yaw_rate_rad_sec, vehicleState.yaw_rate_ref_rad_sec);

	if(data.linear_speed_ref_m_s > 0)
	{
		servo_motor((int) -u_sterzo);
	}
	else
	{
		servo_motor((int) u_sterzo);
	}

	//Manovra finita, resetto tutto
	if(cnt_manovre/100 >= (CORREZIONE_LAMBDA*sqrt(data.dx*data.dx + data.dy*data.dy)/valore_assoluto(data.linear_speed_ref_m_s))+TEMPO_SALITA_PID_TRAZIONE_RWD)
	{
		cnt_manovre = 0;
		flag_ingresso_parcheggio++;
	}
}

void uscita_parcheggio(){

	vehicleState.yaw_rate_ref_rad_sec = calcolo_yaw_rate(data.dy, data.dx, data.linear_speed_ref_m_s, cnt_manovre/100);
	u_sterzo = PID_controller(&pid_steering, vehicleState.yaw_rate_rad_sec, vehicleState.yaw_rate_ref_rad_sec, 0);
	controllo_trazione(vehicleState.motor_speed_RPM, data.linear_speed_ref_m_s / RPM_2_m_s);
	//printf("%f;%f \r\n", vehicleState.yaw_rate_ref_rad_sec, vehicleState.yaw_rate_rad_sec);

	if(data.linear_speed_ref_m_s > 0)
	{
		servo_motor((int) -u_sterzo);
	}
	else
	{
		servo_motor((int) u_sterzo);
	}

	//Manovra finita, resetto tutto
	if(cnt_manovre/100 >= (CORREZIONE_LAMBDA*sqrt(data.dx*data.dx + data.dy*data.dy)/valore_assoluto(data.linear_speed_ref_m_s))+TEMPO_SALITA_PID_TRAZIONE)
	{
		cnt_manovre = 0;
		flag_uscita_parcheggio = 0;
		flag_serial_read = 1;
		data.flag = 1;
		flag_button = 2;
	}
}

//-------------------------------------------------------------
//MOVIMENTI CALCOLATI DRITTO
void muoviti_n_metri_dritto(float spostamento, float v){
	float t = spostamento/v;
	servo_motor(0);
	controllo_trazione(vehicleState.motor_speed_RPM, v/RPM_2_m_s);

	//Manovra finita, resetto tutto
	if(cnt_manovre/100 >= t){
		cnt_manovre = 0;
		//flag_serial_read = 0;
		flag_ingresso_parcheggio++;
		//data.flag = 4;
	}
}

void sistemazione_parcheggio(float spostamento, float v){
	float t = spostamento/v;
	servo_motor(0);
	controllo_trazione(vehicleState.motor_speed_RPM, v/RPM_2_m_s);
	//In base al verso di rotazione seleziono il tempo di salita del motore
	if(v > 0)
	{
		tempo_salita_pid = TEMPO_SALITA_PID_TRAZIONE;
	}
	else
	{
		tempo_salita_pid = TEMPO_SALITA_PID_TRAZIONE_RWD;
	}

	//Manovra finita, resetto tutto
	if(cnt_manovre/100 >= valore_assoluto(t+tempo_salita_pid)){
		if (v > 0){
			data.flag = 6;
			cnt_manovre = 0;
			flag_ingresso_parcheggio = 0;
			flag_serial_read = 1;
			flag_button = 0;
			//flag_ingresso_parcheggio = 0;
		}
		else if(v < 0){
			cnt_manovre = 0;
			flag_uscita_parcheggio++;
		}
	}
}

//-------------------------------------------------------------
//CAMBIO CORSIA
void cambio_corsia(){
	vehicleState.yaw_rate_ref_rad_sec = calcolo_yaw_rate(data.dy, data.dx, data.linear_speed_ref_m_s, cnt_manovre/100) + (data.linear_speed_ref_m_s/(data.curvature_radius_ref_m+valore_assoluto(data.dy)));
	u_sterzo = PID_controller(&pid_steering, vehicleState.yaw_rate_rad_sec, vehicleState.yaw_rate_ref_rad_sec, 0);

	if(data.linear_speed_ref_m_s > 0)
	{
		servo_motor((int) -u_sterzo);
	}
	else
	{
		servo_motor((int) u_sterzo);
	}

	//Manovra finita, resetto tutto
	if(cnt_manovre/100 >= (CORREZIONE_LAMBDA*sqrt(data.dx*data.dx + data.dy*data.dy)/valore_assoluto(data.linear_speed_ref_m_s)))
	{
		cnt_manovre = 0;
		flag_serial_read = 1;
		data.linear_speed_ref_m_s = -0.20;
	}
}

//-------------------------------------------------------------
//RALLENTAMENTO
// cambia il valore della velocita da un valore iniziale a uno finale in uno spazio stop_space definito
void cambio_velocita(float stop_space) {
	float acceleration = (data.linear_speed_ref_m_s*data.linear_speed_ref_m_s - data.old_linear_speed_ref_m_s*data.old_linear_speed_ref_m_s) / (2 * stop_space); // Accelerazione costante per fermarsi uniformemente

	if(flag_serial_read == 1)
	{
		current_linear_speed_ref_m_s = data.old_linear_speed_ref_m_s;
		flag_serial_read = 0;
	}
	else
	{
		// Calcola la nuova velocità e la nuova distanza percorsa
		current_linear_speed_ref_m_s += acceleration*TRACTION_SAMPLING_TIME;

		// Per smettere di decelerare
		if(data.linear_speed_ref_m_s - data.old_linear_speed_ref_m_s < 0)
		{
			if (current_linear_speed_ref_m_s <= data.linear_speed_ref_m_s)
			{
				//data.flag = 0; //data.flag = 1;
				cambiamento_velocita = 0;
			}
		}
		else
		// Per smettere di accelerare
		{
			if(current_linear_speed_ref_m_s >= data.linear_speed_ref_m_s)
			{
				//data.flag = 1; forse, dipende dal contesto
				flag_serial_read = 1;
			}

		}

		controllo_trazione(vehicleState.motor_speed_RPM, current_linear_speed_ref_m_s/RPM_2_m_s);
	}
}

//-------------------------------------------------------------
//PROCEDURA AUTOMATICA PER IL PERCORSO
void procedura(){
	switch(flag_procedura){
	case 0:
		data.flag = 4;
		data.dy = 0.35;
		data.dx = 0.90;
		//next_procedura()
		break;
	case 1:
		break;
	}
}

void next_procedura(float s){
	if(cnt_procedura == s){
		cnt_procedura = 0;
		flag_procedura++;
	}
}

//-------------------------------------------------------------
//BLUE user button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) { // Button pressed
			buttonPressStartTime = cnt_10ms_button;

		} else { // Button released
			buttonPressEndTime = cnt_10ms_button;

			//Verifico quantotemp ho tenuto premuto il tasto
			pressDuration = buttonPressEndTime - buttonPressStartTime;
			if (pressDuration < SHORT_PRESS_THRESHOLD)
			{
				if(flag_button >= 0 && flag_button < max_flag_button){
					flag_button++;
				}
				else
				{
					flag_button = 0;
				}
			} else if (pressDuration >= LONG_PRESS_THRESHOLD)
			{
				flag_button = -1;
				counter_cal_ESC = 0;
			}
		}
	}
	if(GPIO_Pin == GPIO_PIN_10){
		printf("letto interrupt \r\n");
	}
}

float valore_assoluto(float x)
{
	if(x < 0)
	{
		return -x;
	}
	else
	{
		return x;
	}
}

//-------------------------------------------------------------
//STAMPE
//USART2 -> ST_Link UART for DEBUG with USB (e.g. PUTTY)
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void PrintState(){
	//printf("%s\r\n", msg);
	//printf("%d, %f, %f, %f, %f, %f \r\n", data.flag, data.linear_speed_ref_m_s, data.curvature_radius_ref_m, data.dy, data.dx, data.distanza_frenata);
}

//-------------------------------------------------------------
//LETTURA SERIALE
/*
void lettura_seriale(){
	if(flag_serial_read)
	{

		while(mainSerialRead(msg, sizeof(msg)) == 0);
		printf("%s\r\n", msg);

		parseCSV(msg, floatArray);
		data.flag = floatArray[0];
		data.linear_speed_ref_m_s = floatArray[1]; //[m/s]
		data.curvature_radius_ref_m = floatArray[2]; //m]
		data.dy = floatArray[3]; //[m]
		data.dx = floatArray[4]; //[m]
		data.distanza_frenata = floatArray[5]; //[m]
		printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f \r\n", data.flag, data.linear_speed_ref_m_s, data.curvature_radius_ref_m, data.dy, data.dx, data.distanza_frenata);


		//Leggo messaggio in entrata
		while(mainSerialRead(msg, sizeof(msg)) == 0);

		//Interpreto il messaggio
		float floatArray[MAX_VALUES];
		parseCSV(msg, floatArray);

		//Assegnazione dei dati trasmessi
		flag_controllo = floatArray[0];

		if ((flag_controllo >= 1 && flag_controllo <= 6))
		{

			data.flag = flag_controllo;

			//Salvo la vecchia velocita per il rallentamento
			data.old_linear_speed_ref_m_s = data.linear_speed_ref_m_s; //[m/s]

			data.linear_speed_ref_m_s = floatArray[1]; //[m/s]

			if(floatArray[2] == 0){
				data.curvature_radius_ref_m = 10; // [m]
			} else {
				data.curvature_radius_ref_m = floatArray[2];
			}


			//Manovra
			data.dy = floatArray[3]; //[m]
			data.dx = floatArray[4]; //[m]

			//Rallentamento
			data.distanza_frenata = floatArray[5]; //[m]
		}
		//PrintState();

		//Ferma la lettura della seriale per non bloccare il parcheggio
		if(data.flag == 3 || data.flag == 4){
			flag_serial_read = 0;
		}
	}
}
*/
//-------------------------------------------------------------
//COMUNICAZIONE SERIALE IN INTERRUPT
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(app[0] == 's'){
		stato_rx = 1;
		msg_len = 0;
		pt_rx = 0;


		for(int j = 0; j < sizeof(msg); j++){
			msg[j] = '\0';
		}
	} else if (app[0] == '\n'){
		stato_rx = 0;
		rx_complete = 1; //Ho ricevuto tutto il messaggio
	} else {
		msg[pt_rx] = app[0];
		msg_len = pt_rx;
		pt_rx++;
	}

  HAL_UART_Receive_IT(&huart6, app, 1); //You need to toggle a breakpoint on this line!
  printf("%s", app);
}

//-------------------------------------------------------------
//CALIBRAZIONE
void ProceduraCalibrazione(){
	if(counter_cal_ESC < 5){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	if(counter_cal_ESC <= 300){
		if(!(counter_cal_ESC % 15)){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
	}
	else if(counter_cal_ESC <= 600){
		duty = NEUTRAL_PWM;
		BL_set_PWM(duty);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else if(counter_cal_ESC <= 900){
		duty = MAX_PWM;
		BL_set_PWM(duty);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	else if(counter_cal_ESC <= 1100){
		duty = MIN_PWM;
		BL_set_PWM(duty);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else if(counter_cal_ESC <= 1200){
		if(!(counter_cal_ESC % 15)){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
	}
	else if (counter_cal_ESC <= 1300){
		data.flag = 6;
		flag_button = 0;
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
