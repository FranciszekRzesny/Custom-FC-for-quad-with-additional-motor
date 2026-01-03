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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_mpu6500.h"
#include "driver_mpu6500_basic.h"
#include "driver_mpu6500_fifo.h"

#include "ibus.h"

#include "math.h"

#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	float roll; 	// x axis
	float pitch; 	// y axis
	float yaw; 		// z axis
} Orientation;

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//MPU6500 variables
uint8_t res;
uint32_t i;
uint32_t times;
float g[3];
float dps[3] = {0.0f, 0.0f, 0.0f}; // degrees per second
float degrees;
mpu6500_address_t addr;
uint8_t MPU_Flag = 0;

Orientation orientation = {0.0f, 0.0f, 0.0f};
float gyro_bias[3] = {0};

float dt = 0;

uint16_t ibus_data[IBUS_USER_CHANNELS];

PID_t pid_roll  = { 15.0f, 0.02f, 1.8f };
PID_t pid_pitch = { 15.0f, 0.02f, 1.8f };
PID_t pid_yaw   = { 15.0f, 0.02f, 0.0f };

uint16_t m1, m2, m3, m4; //motor outputs

uint8_t uart_rx_buffer[32];
volatile uint8_t ibus_data_ready = 0;
volatile uint8_t sync_found = 0;
uint8_t ibus_frame[32];
//static uint8_t sync_buffer[64];
//static uint8_t sync_index = 0;

uint8_t arming = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float pid_compute(PID_t *pid, float error, float dt)
{
    pid->integral += error * dt;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

void compute_motor_pwm(float roll, float pitch, float yaw, float dt,
                       uint16_t throttle,
                       uint16_t *m1, uint16_t *m2,
                       uint16_t *m3, uint16_t *m4)
{
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;

    // Calculate errors
    float roll_error  = target_roll  - roll;
    float pitch_error = target_pitch - pitch;
    float yaw_error   = target_yaw   - yaw;

    // PID output
    float roll_correction  = pid_compute(&pid_roll,  roll_error,  dt);
    float pitch_correction = pid_compute(&pid_pitch, pitch_error, dt);
    float yaw_correction   = pid_compute(&pid_yaw,   yaw_error,   dt);

    // Summarise signals
    float m1_f = throttle + pitch_correction + roll_correction - yaw_correction;  	// Front Left
    float m2_f = throttle + pitch_correction - roll_correction + yaw_correction;  	// Front Right
    float m3_f = throttle - pitch_correction - roll_correction - yaw_correction;  	// Rear Right
    float m4_f = throttle - pitch_correction + roll_correction + yaw_correction;  	// Rear Left

    // PWM LIMITATION
    if (m1_f > 2000) m1_f = 2000;
    if (m1_f < 1000) m1_f = 1000;
    if (m2_f > 2000) m2_f = 2000;
    if (m2_f < 1000) m2_f = 1000;
    if (m3_f > 2000) m3_f = 2000;
    if (m3_f < 1000) m3_f = 1000;
    if (m4_f > 2000) m4_f = 2000;
    if (m4_f < 1000) m4_f = 1000;

    *m1 = (uint16_t)m1_f;
    *m2 = (uint16_t)m2_f;
    *m3 = (uint16_t)m3_f;
    *m4 = (uint16_t)m4_f;
}

static inline void ESC_SetPulse_us(uint16_t us, uint8_t motor_index)
{

//	if (us < 1000) us = 1000; new limits are 500 1000
//	if (us > 2000) us = 2000;

	if(motor_index == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, us);
	} else if(motor_index == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, us);
	} else if(motor_index == 3)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
	} else if(motor_index == 4)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, us);
	} else if(motor_index == 5)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, us);
	} else if(motor_index == 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, us);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, us);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, us);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, us);
	}

}

static inline void Servo_SetPulse_us(uint16_t us, uint8_t servo_index)
{
//	if (us < 490) us = 490; new limits are 500 1000
//	if (us > 2550) us = 2550;

	if(servo_index == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, us);
	} else if(servo_index == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
	} else if(servo_index == 0)
	{
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, us);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
	}
}

float get_delta_time(void)
{
	static uint64_t last = 0;
	uint64_t now = HAL_GetTick();
	float dt = (now - last) / 1000.0f;
	last = now;
	return dt;
}

void update_orientation(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	orientation.roll 	+= (gx - gyro_bias[0]) * dt;
	orientation.pitch 	+= (gy - gyro_bias[1]) * dt;
	orientation.yaw 	+= (gz - gyro_bias[2]) * dt;

	float acc_roll = atan2f(ay, az) * 180.0f / M_PI;
	float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

	const float alpha = 0.88f;
	orientation.roll  = alpha * orientation.roll  + (1.0f - alpha) * acc_roll;
	orientation.pitch = alpha * orientation.pitch + (1.0f - alpha) * acc_pitch;

	if (orientation.roll > 180)   orientation.roll -= 360;
	if (orientation.roll < -180)  orientation.roll += 360;

	if (orientation.pitch > 180)  orientation.pitch -= 360;
	if (orientation.pitch < -180) orientation.pitch += 360;

	if (orientation.yaw > 180)    orientation.yaw -= 360;
	if (orientation.yaw < -180)   orientation.yaw += 360;
}

void calibrate_gyro(int samples)
{
	float sum[3] = {0};

	for (int i = 0; i < samples; i++)
	{
		float g_calib[3];
		mpu6500_basic_read(g_calib, dps);
	    sum[0] += g_calib[0];
	    sum[1] += g_calib[1];
	    sum[2] += g_calib[2];
	    HAL_Delay(10);
	 }

	 gyro_bias[0] = sum[0] / samples;
	 gyro_bias[1] = sum[1] / samples;
	 gyro_bias[2] = sum[2] / samples;
}

void arm_esc()
{
	if(arming == 0 && ibus_data[9] == 2000)
	{
		ESC_SetPulse_us(500, 1);
		ESC_SetPulse_us(500, 2);
		ESC_SetPulse_us(500, 3);
		ESC_SetPulse_us(500, 4);
		ESC_SetPulse_us(500, 5);
		arming = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		MPU_Flag = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == IBUS_UART)
		ibus_reset_failsafe();
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim1);
  //HAL_UART_Receive_DMA(&huart4, uart_rx_buffer, 32); possible trash
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

  res = mpu6500_basic_init(MPU6500_INTERFACE_SPI, MPU6500_ADDRESS_AD0_LOW);

  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
  calibrate_gyro(500);

  ibus_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(ibus_read(ibus_data))
	  {
		  arm_esc();

//		  compute_motor_pwm(
//		  	      orientation.roll,
//		  	      orientation.pitch,
//		  	      orientation.yaw,
//		  	      dt,
//		  	      ibus_data[2],     // throttle z aparatury
//		  	      &m1, &m2, &m3, &m4
//		  	  );

		  Servo_SetPulse_us(ibus_data[0]/2, 2);
		  Servo_SetPulse_us(ibus_data[1]/2, 1);


		  if(ibus_data[9] == 2000 && arming == 1)
		  {
			  ESC_SetPulse_us(ibus_data[2]/2, 1);
			  ESC_SetPulse_us(ibus_data[2]/2, 2);
			  ESC_SetPulse_us(ibus_data[2]/2, 3);
			  ESC_SetPulse_us(ibus_data[2]/2, 4);
			  ESC_SetPulse_us(ibus_data[4]/2, 5);
//			  ESC_SetPulse_us(m1, 1);
//			  ESC_SetPulse_us(m2, 2);
//			  ESC_SetPulse_us(m3, 3);
//			  ESC_SetPulse_us(m4, 4);
//			  ESC_SetPulse_us(ibus_data[4], 5);

		  } else
		  {
			  ESC_SetPulse_us(500, 1);
			  ESC_SetPulse_us(500, 2);
			  ESC_SetPulse_us(500, 3);
			  ESC_SetPulse_us(500, 4);
			  ESC_SetPulse_us(500, 5);
		  }


		  if(MPU_Flag == 1)
		  {
		  	if (mpu6500_basic_read(g, dps) != 0)
		  	{
		  		(void)mpu6500_basic_deinit();
		  	}


		  	if (mpu6500_basic_read_temperature(&degrees) != 0)
		  	{
		  		(void)mpu6500_basic_deinit();
		  	}

		  	dt = get_delta_time();
		  	update_orientation(dps[0], dps[1], dps[2], g[0], g[1], g[2], dt);

		  	MPU_Flag = 0;
		  }
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

		  //ibus_soft_failsafe(ibus_data, 10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
