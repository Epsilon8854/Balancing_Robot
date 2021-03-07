/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "math.h"
#include "MPU6050.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG 57.29578
#define DEG_TO_RAD 0.017453
//#define M_PI 3.141592//65358979323846

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char uart3_rx[100],uart1_rx[100];
int16_t ax,ay,az,gx,gy,gz,tmpr;
float rate_gyroX=0, rate_gyroZ=0, rate_gyroY=0;
float raw_accX=0, raw_accY=0, raw_accZ=0;
float rate_accX=0, rate_accY=0, rate_accZ=0;
float gyroAngleX=0, gyroAngleY=0, gyroAngleZ=0;
float time_diff=0.01;

float accAngX=0,accAngX1=0;
float CFangleX=0, CFangleX1=0;

char std[200];

float K = 0.99;

float PIDspeed=0;
const float default_setpoint = -0.0405;
float move_setpoint=0;

PID first;

float P_constant = 2000.0;	//2000.0;
float I_constant = 80.0;	//75.0;
float D_constant = 5000.0;	//5000.0;

int TURN_flag = 0; //
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setPoint(PID *target) {
	target->set_point = default_setpoint + move_setpoint;
}

float update(PID *target,float current_Val) {
	target->error = target->set_point - current_Val;

	float P_Val = (target->Kp)* target->error;
	float D_Val = target->Kd * (target->error - target->Derivator);
	target->Derivator = target->error;

	if (target->Integrator > target->Integrator_max)
		target->Integrator = target->Integrator_max;
	else if (target->Integrator < target->Integrator_min)
		target->Integrator = target->Integrator_min;

	float I_Val = target->Integrator * target->Ki;

	//float PID_Val = P_Val + I_Val + D_Val;

	target->Integrator = target->Integrator + (target->error);

	//sprintf(std,"P_Val: %.2f,	I_Val: %.2f,	D_Val: %.2f,	error:%.2f \r\n",P_Val,I_Val,D_Val,target->error);
	//HAL_UART_Transmit(&huart1, std, strlen(std), 20);
	return P_Val + I_Val + D_Val;
}

void initPID(PID *target,float P, float I, float D) {
	target->Kp = P;
	target->Ki = I;
	target->Kd = D;

	target->Integrator = 0;
	target->Derivator = 0;
	target->Integrator_max = 100;
	target->Integrator_min = -100;

	target->set_point = default_setpoint;
	target->error = 0;
}


void MotorInit(){
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
}
void MotorStop(){

	GPIOB->BRR = GPIO_PIN_0;
	GPIOB->BRR = GPIO_PIN_1;

	GPIOC->BRR = GPIO_PIN_0;
	GPIOC->BRR = GPIO_PIN_1;
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
}
void MotorBackward(float speed){
	if(speed>=1000)
		speed =1000;

	if(TURN_flag==-1){
		TIM3->CCR1 = (int)speed;
		TIM3->CCR2 = (int)speed*0.7;
	}
	else if(TURN_flag==1){
		TIM3->CCR1 = (int)speed*0.7;
		TIM3->CCR2 = (int)speed;
	}
	else{
		TIM3->CCR1 = (int)speed;
		TIM3->CCR2 = (int)speed;
	}

	GPIOB->BRR = GPIO_PIN_0;
	GPIOB->BSRR = GPIO_PIN_1;

	GPIOC->BSRR = GPIO_PIN_1;
	GPIOC->BRR = GPIO_PIN_0;
}
void MotorForward(float speed){
	if(speed>=1000)
		speed =1000;

	if(TURN_flag==-1){
		TIM3->CCR1 = (int)speed*0.7;
		TIM3->CCR2 = (int)speed;
	}
	else if(TURN_flag==1){
		TIM3->CCR1 = (int)speed;
		TIM3->CCR2 = (int)speed*0.7;
	}
	else{
		TIM3->CCR1 = (int)speed;
		TIM3->CCR2 = (int)speed;
	}

	GPIOB->BSRR = GPIO_PIN_0;
	GPIOB->BRR = GPIO_PIN_1;

	GPIOC->BRR = GPIO_PIN_1;
	GPIOC->BSRR = GPIO_PIN_0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//TURN_flag 瑜� 醫뚰쉶�쟾 �늻瑜대㈃ -1�씠�릺怨� �븣硫� 0�릺硫� �슦�쉶�쟾 �늻瑜대㈃ 1�릺怨� �븣硫� �떎�떆 0�맖.
	//�쟾吏꾪썑吏� ��吏곸씠硫� 理쒕� +-0.3 �쑝濡� move_setpoint 媛믪씠 蹂��빐�빞�븿. �빐二쇨퀬 setPoint(&first); �떎�뻾 �빐以섏빞�뙋.
	if(huart->Instance == USART1)
	{
		//HAL_TIM_Base_Stop_IT(&htim2);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_8);
		//HAL_TIM_Base_Start_IT(&htim2);
	}

	if(huart->Instance == USART3)
	{
		//F,B,R,L,S
		switch(uart3_rx[0]){
		case 'F':
			TURN_flag = 0;
			move_setpoint = -0.04;
			setPoint(&first);
			break;
		case 'B':
			TURN_flag = 0;
			move_setpoint = 0.04;
			setPoint(&first);
			break;
		case 'R':
			TURN_flag = -1;
			move_setpoint = -0.03;
			setPoint(&first);
			break;
		case 'L':
			TURN_flag = 1;
			move_setpoint = -0.03;
			setPoint(&first);
			break;
		case 'N':
			TURN_flag = 0;
			move_setpoint=0.0;
			PIDspeed=0;
			CFangleX1=0;
			accAngX1=0;
			first.Integrator=0;
			first.Derivator=0;
			first.error=0;
			setPoint(&first);
			break;
		default://'S'
			TURN_flag = 0;
			move_setpoint=0.0;
			setPoint(&first);
			break;
		}

		uart3_rx[0]=0;

		HAL_UART_Receive_IT(&huart3, (uint8_t*)uart3_rx, 1);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim ->Instance == TIM2)//10ms
	{
		GPIOB->ODR ^= GPIO_PIN_9; //�넗湲�

		MPU6050_GetData(&ax,&ay,&az,&gx,&gy,&gz,&tmpr);
		//			sprintf(std,"MPU6050: ax:%d , ay:%d , az:%d , gx:%d , gy:%d , gz:%d , tmpr:%d \r\n",ax,ay,az,gx,gy,gz,tmpr);
		//sprintf(std,"tim is running\n\r");
		//			sprintf(std,"MPU6050: rate_gyroX:%.2f , rate_gyroY:%.2f , rate_gyroZ:%.2f , rate_accX:%.2f , rate_accY:%.2f , rate_accZ:%.2f \r\n",rate_gyroX,rate_gyroY,rate_gyroZ,rate_accX,rate_accY,rate_accZ);
		//			HAL_UART_Transmit(&huart1, std, strlen(std), 200);

		rate_gyroX = (float)gx/ 131.0;
		rate_gyroY = (float)gy/ 131.0;
		rate_gyroZ = (float)gz/ 131.0;
		gyroAngleX += rate_gyroX * time_diff ;
		gyroAngleY += rate_gyroY * time_diff ;
		gyroAngleZ += rate_gyroZ * time_diff ;

		rate_accX = -(float)ax/16384.0;
		rate_accY = -(float)ay/16384.0;
		rate_accZ = (float)az/16384.0;

		accAngX1 = atan2(rate_accX, rate_accZ)* RAD_TO_DEG;
		CFangleX1 =  K * ( CFangleX1 + rate_gyroY*time_diff) + (1 - K) * accAngX1 ;
		//sprintf(std,"MPU6050: rate_gyroX:%.2f , rate_gyroY:%.2f , rate_gyroZ:%.2f , rate_accX:%.2f , rate_accY:%.2f , rate_accZ:%.2f \r\n",rate_gyroX,rate_gyroY,rate_gyroZ,rate_accX,rate_accY,rate_accZ);
		//HAL_UART_Transmit(&huart1, std, strlen(std), 200);
		//sprintf(std,"current Val: %.2f \r\n",CFangleX1);
		//sprintf(std,"accAngX1: %.2f, CFangleX1=%.2f \r\n",accAngX1,CFangleX1);
		//sprintf(std,"accAngX1: %.2f, gyroAngleY=%.2f \r\n",accAngX1,gyroAngleY);
		//HAL_UART_Transmit(&huart1, std, strlen(std), 20);
		PIDspeed = update(&first,CFangleX1*DEG_TO_RAD);

		if(PIDspeed<0){
			//HAL_UART_Transmit(&huart1, std, strlen(std), 20);
			MotorBackward(-PIDspeed);
			//sprintf(std,"PIDspeed: %f \r\n Forward \r\n",PIDspeed);
			//HAL_UART_Transmit(&huart1, std, strlen(std), 20);
		}
		else if(PIDspeed>0){
			//HAL_UART_Transmit(&huart1, std, strlen(std), 20);
			MotorForward(PIDspeed);
			//sprintf(std,"PIDspeed: %f \r\n backward \r\n",PIDspeed);
			//HAL_UART_Transmit(&huart1, std, strlen(std), 20);
		}
		else{
			//HAL_UART_Transmit(&huart1, std, strlen(std), 20);
			MotorStop();
		}
		//		}
	}

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
	MX_I2C2_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	MPU6050_Init(0x03);//LPF 42Hz
	MotorInit();
	//MotorStop();

	//HAL_UART_Receive_IT(&huart3, (uint8_t*)uart3_rx, 1);
	initPID(&first,P_constant,I_constant,D_constant);
	setPoint(&first);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart3, (uint8_t*)uart3_rx, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//MPU6050_GetData(&ax,&ay,&az,&gx,&gy,&gz,&tmpr);
		//		sprintf(std,"MPU6050: rate_gyroX:%.2f , rate_gyroY:%.2f , rate_gyroZ:%.2f , rate_accX:%.2f , rate_accY:%.2f , rate_accZ:%.2f \r\n",rate_gyroX,rate_gyroY,rate_gyroZ,rate_accX,rate_accY,rate_accZ);
		//		HAL_UART_Transmit(&huart1, std, strlen(std), 100);
		//sprintf(std,"MPU6050: gx:%d , gy:%d , gz:%d , ax:%d , ay:%d , az:%d , tmpr:%d \r\n",gx,gy,gz,ax,ay,az,tmpr);
		//HAL_UART_Transmit(&huart1, std, strlen(std), 10);

		//HAL_Delay(100);
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

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
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

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
