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
#include <string.h>
#include "math.h"

#include "Configuration.h"
#include "bno055_stm32.h"
#include "servo_motor.h"
#include "PID.h"
#include "BL_motor.h"
#include "RX_UART.h"
#include "DC_motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct SerialDataRX {
	float curvature_radius_ref_m; // [m]
	float linear_speed_ref_m_s; //[m/s]
	int enable; //Start Flag [bool]
	float offset;
} serialDataRX;

typedef struct SerialDataTX {
	float current_speed_rpm; // [m]
	float current_servo_angle_deg; // [m/s]
	float current_yaw_rate_deg_sec; // [°/s]
	float quaternion_x;
	float quaternion_y;
	float quaternion_z;
	float quaternion_w;
	float magne_x;
	float magne_y;
	float magne_z;
	float accel_x;
	float accel_y;
	float accel_z;
	float angle_x;
	float angle_y;
	float angle_z;
} serialDataTX;

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
	uint8_t motor_direction_ref;

	//Sterzo
	double yaw_rate_rad_sec; // [rad/s]
	double yaw_rate_deg_sec; // [°/s]
	double yaw_rate_ref_rad_sec; //[rad/s]
} vehicleData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RxBuf_SIZE 50
#define msg_SIZE 45
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
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

serialDataRX dataRX;
serialDataTX dataTX;
vehicleData vehicleState;

//PID
PID pid_traction, pid_traction_RWD, pid_traction_DESC, pid_steering;
double u_trazione = 0;
double u_sterzo = 0;

//Event control
int Flag_10ms = 0;
int HardwareEnable = 0;
int flag_button = 0;

//User button press
uint32_t buttonPressStartTime = 0;
uint32_t buttonPressEndTime = 0;
uint32_t pressDuration = 0;
uint32_t cnt_10ms_button = 0; //Aumenta ogni 10ms
int max_flag_button;

//Variabili per calibrazione ESC
int counter_cal_ESC = 0;
float duty;
int flag_cal = 0;

//Inclinazione per il PID in discesa
float x_acceleration = 0;

//Filtro per la velocità
#define MAX_RPM_VALUES 10
float tempRPM = 0;
int PtrRPM = 0;
float MeanRPM = 0;
float ArrayRPM[MAX_RPM_VALUES];

//Serial input buffer
uint8_t RxBuf[RxBuf_SIZE];
uint8_t msg[45] = { "\0" };
uint8_t char_in = '\0';
int msg_len = 0;
int MAX_VALUES = 3;
float last_read;
int cnt_DMA;

//Test sterzo massimo
int cnt_sterzo = 0;
float angolo_sterzo = 0;

float RPM_2_m_s = ((2 * M_PI / 60) * WHEEL_RADIUS / MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION)*0.787;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void TransmitTelemetry();
void ProceduraCalibrazione();
void resetBNO055();
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
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	//IMU BNO055 Configuration
	resetBNO055();	//IMU BNO055 Configuration

	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	//attivo DMA per ricezione dati seriale
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
	//PWM Servo
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//PWM DC motor
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	//ENCODER TIMER
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	//10ms TIMER
	HAL_TIM_Base_Start_IT(&htim11);

	//PID traction
	init_PID(&pid_traction, TRACTION_SAMPLING_TIME, MAX_U_TRACTION, MIN_U_TRACTION, 0, 1);
	tune_PID(&pid_traction, KP_TRACTION, KI_TRACTION, 0);

	//PID steering
	init_PID(&pid_steering, STEERING_SAMPLING_TIME, MAX_U_STEERING, MIN_U_STEERING, 0, 12);
	tune_PID(&pid_steering, KP_STEERING, KI_STEERING, 0);



	printf("Initialization Completed!\r\n");

	//L MOTOR ENABLE
	set_PWM_and_dir(0, vehicleState.motor_direction_ref);
	//Servo Neutral Position
	servo_motor(0);

	//Begin Interrupt Serial listening
	//HAL_UART_Receive_IT(&huart6, char_in, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		max_flag_button = 1;
		switch(flag_button){
		//Calibrazione
		case -1:
			if(dataRX.enable == 0)
				//printf("%f\r\n", duty);
				break;
			//Idle
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HardwareEnable = 0;
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HardwareEnable = 1;
			//dataRX.enable = 1;
			//dataRX.linear_speed_ref_m_s = 0.20;
			//dataRX.curvature_radius_ref_m = 0.73;
			//angolo_sterzo = -46;
			//servo_motor(angolo_sterzo);
			//printf("%f\r\n", angolo_sterzo);
			break;
		}

		//CALIBRAZIONE CON PULSANTE
		/*
		if(flag_button != -1)
			flag_cal = 0;
		 */

		//bno055_vector_t magnet = bno055_getVectorMagnetometer();
		//printf("magnet.x:%f, magnet.y:%f\r\n", magnet.x, magnet.y);
		//printf("%f;%f\r\n", magnet.x, magnet.y);

		//-------------------------------------------------------------
		//Controllo
		if(bno055_getSystemStatus() != 5)
			HAL_NVIC_SystemReset();

		if (HardwareEnable == 1 && dataRX.enable == 1) {
			if (Flag_10ms == 1) {
				Flag_10ms = 0;

				//-------------------------------------------------------------

				//TRACTION control

				//Measure speed with encoder
				vehicleState.ref_count = TIM2->ARR / 2;
				vehicleState.delta_count = vehicleState.counts - vehicleState.ref_count;

				vehicleState.delta_angle_deg = (vehicleState.delta_count * 360) / ((double) (ENCODER_PPR * ENCODER_COUNTING_MODE * GEARBOX_RATIO));
				vehicleState.motor_speed_deg_sec = vehicleState.delta_angle_deg / ENCODER_SAMPLING_TIME;
				tempRPM = DegreeSec2RPM(vehicleState.motor_speed_deg_sec);

				//Filtraggio della velocità
				ArrayRPM[PtrRPM] = tempRPM;
				MeanRPM = 0;
				for(int i = 0; i < MAX_RPM_VALUES; i++){
					MeanRPM += ArrayRPM[i];
				}
				MeanRPM /= MAX_RPM_VALUES;

				if(PtrRPM == MAX_RPM_VALUES-1)
					PtrRPM = 0;
				else
					PtrRPM++;
				vehicleState.motor_speed_RPM = MeanRPM;

				//Speed reference for motor
				vehicleState.motor_speed_ref_RPM = dataRX.linear_speed_ref_m_s / RPM_2_m_s;

				//Verifica l'inclinazine della macchina per la rampa
				bno055_vector_t u = bno055_getVectorGravity();
				x_acceleration = u.x;
				//printf("%f;%f\r\n", x_acceleration, 0);

				u_trazione = PID_controller(&pid_traction, abs(vehicleState.motor_speed_RPM), abs(vehicleState.motor_speed_ref_RPM));

				if(vehicleState.motor_speed_ref_RPM>0)
					vehicleState.motor_direction_ref = -1;
				else
					vehicleState.motor_direction_ref = 1;

				//Assegno il duty al motore
				if (vehicleState.motor_speed_ref_RPM == 0)
					set_PWM_and_dir(0, vehicleState.motor_direction_ref);
				else
					set_PWM_and_dir((uint32_t) Voltage2Duty(u_trazione), vehicleState.motor_direction_ref);

				//-------------------------------------------------------------

				//STEERING control

				//Get yawrate from IMU
				bno055_vector_t v = bno055_getVectorGyroscope();
				vehicleState.yaw_rate_deg_sec = v.z;
				vehicleState.yaw_rate_rad_sec = (vehicleState.yaw_rate_deg_sec * M_PI) / 180;
				last_read = dataRX.curvature_radius_ref_m;

				if (dataRX.curvature_radius_ref_m >= MAX_CURVATURE_RADIUS_FOR_STRAIGHT) {

					vehicleState.yaw_rate_ref_rad_sec = 0;

					u_sterzo = PID_controller(&pid_steering, vehicleState.yaw_rate_rad_sec, vehicleState.yaw_rate_ref_rad_sec);

					servo_motor(-u_sterzo); //Minus because yawrate and steering are opposite
				} else {

					vehicleState.linear_speed_m_s = vehicleState.motor_speed_RPM * RPM_2_m_s;
					if (dataRX.curvature_radius_ref_m == 0)
						vehicleState.yaw_rate_ref_rad_sec = 0;
					else
						vehicleState.yaw_rate_ref_rad_sec = vehicleState.linear_speed_m_s / dataRX.curvature_radius_ref_m;

					float yaw_rate_ref_rad_sec_abs = vehicleState.yaw_rate_ref_rad_sec;
					float yaw_rate_rad_sec_abs = vehicleState.yaw_rate_rad_sec;
					if (vehicleState.yaw_rate_ref_rad_sec < 0)
						yaw_rate_ref_rad_sec_abs = -vehicleState.yaw_rate_ref_rad_sec;
					if (vehicleState.yaw_rate_rad_sec < 0)
						yaw_rate_rad_sec_abs = -vehicleState.yaw_rate_rad_sec;

					u_sterzo = PID_controller(&pid_steering, yaw_rate_rad_sec_abs, yaw_rate_ref_rad_sec_abs);

					//Minus because yawrate and steering are opposite
					if (dataRX.curvature_radius_ref_m >= 0 && u_sterzo > 0)
						u_sterzo *= -1.0;
					if (dataRX.curvature_radius_ref_m < 0 && u_sterzo < 0)
						u_sterzo *= -1.0;

					servo_motor(u_sterzo);
				}
				dataTX.current_servo_angle_deg = u_sterzo;
			}
		} else {
			if(flag_button != -1){
				set_PWM_and_dir(0, vehicleState.motor_direction_ref);
				servo_motor(0);

				// Reset dei pid
				resetPID(&pid_steering);
				resetPID(&pid_traction);
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
	hi2c1.Init.ClockSpeed = 400000;
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
	huart6.Init.BaudRate = 115200;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(IMU_RESET_GPIO_Port, IMU_RESET_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_Pin IMU_RESET_Pin */
	GPIO_InitStruct.Pin = DIR_Pin|IMU_RESET_Pin;
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

	/*Configure GPIO pin : External_button_Pin */
	GPIO_InitStruct.Pin = External_button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(External_button_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Timer11 for temporization
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		Flag_10ms = 1;
		TransmitTelemetry();

		if (dataRX.curvature_radius_ref_m = last_read){
			cnt_DMA++;
			if(cnt_DMA >= 5){
				HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuf, RxBuf_SIZE);
				cnt_DMA = 0;
			}
		}
		//Encoder
		vehicleState.counts = TIM2->CNT;
		TIM2->CNT = TIM2->ARR / 2;

		cnt_10ms_button++;

		//Variabile per la calibrazione
		if(flag_button == -1){
			counter_cal_ESC++;
		}

		if(flag_button == 1){
			cnt_sterzo++;
		}

		// Prova per aumentare lo sterzo gradualmente
		if(cnt_sterzo == 200){
			cnt_sterzo = 0;
			angolo_sterzo -= 5;
		} else if(angolo_sterzo < -45){
			angolo_sterzo = -30;
		}

	}
}

//USART2 -> ST_Link UART for DEBUG with USB (e.g. PUTTY)
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); //putty
	HAL_UART_Transmit(&huart6, (uint8_t*) &ch, 1, 0xFFFF); //rpi
	return ch;
}

//-------------------------------------------------------------
//BLUE user button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13 ||GPIO_Pin == GPIO_PIN_2) {
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
}

//-------------------------------------------------------------
// COMUNICAZIONE (RICEZIONE DATI)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART6){
		memcpy(msg, RxBuf, Size);
		float floatArray[MAX_VALUES];
		parseCSV(msg, floatArray);

		if(floatArray[0] == 0 || floatArray[0] == 1){
			dataRX.enable = floatArray[0];
			dataRX.linear_speed_ref_m_s = floatArray[1];
			dataRX.curvature_radius_ref_m = floatArray[2];
		}

		if(floatArray[0] == 3){
			// Reset dei pid
			resetPID(&pid_steering);
			resetPID(&pid_traction);
			resetPID(&pid_traction_RWD);
			resetPID(&pid_traction_DESC);
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuf, RxBuf_SIZE);
	}
}

// COMUNICAZIONE (TRASMISSIONE DATI)
void TransmitTelemetry(){
	dataTX.current_speed_rpm = vehicleState.motor_speed_RPM;
	dataTX.current_yaw_rate_deg_sec = vehicleState.yaw_rate_deg_sec;

	bno055_vector_t accel = bno055_getVectorAccelerometer();
	bno055_vector_t angle = bno055_getVectorGyroscope();
	bno055_vector_t magne = bno055_getVectorMagnetometer();
	bno055_vector_t quat = bno055_getVectorQuaternion();
	dataTX.accel_x = accel.x;
	dataTX.accel_y = accel.y;
	dataTX.accel_z = accel.z;
	dataTX.angle_x = angle.x;
	dataTX.angle_y = angle.y;
	dataTX.angle_z = angle.z;
	dataTX.magne_x = magne.x;
	dataTX.magne_y = magne.y;
	dataTX.magne_z = magne.z;
	dataTX.quaternion_x = quat.x;
	dataTX.quaternion_y = quat.y;
	dataTX.quaternion_z = quat.z;
	dataTX.quaternion_w = quat.w;
	printf("%2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f\r\n", dataTX.accel_x, dataTX.accel_y, dataTX.accel_z, dataTX.angle_x, dataTX.angle_y, dataTX.angle_z, dataTX.quaternion_x, dataTX.quaternion_y, dataTX.quaternion_z, dataTX.quaternion_w);
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
