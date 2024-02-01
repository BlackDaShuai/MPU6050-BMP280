/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "mpu6050.h"
#include "oled.h"
#include "stdlib.h"
#include "math.h"
#include "bmp280.h"

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

/* USER CODE BEGIN PV */
int Flag_LED = 1;

//MPU6050
float pitch =0.0,roll=0.0,yaw=0.0;
short x,y,z;
double speed = 0.0;
char speedToShow[6];
char tempToShow[5];
//float tempMPU = 0.0;
int32_t T = 0;
uint64_t P = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 /*
  *power by WeAct Studio
  *The board with `WeAct` Logo && `version number` is our board, quality guarantee. 
  *For more information please visit: https://github.com/WeActTC/BluePill-Plus.git
  *更多信息请访问：https://gitee.com/WeAct-TC/BluePill-Plus
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	//OLED初始化
	printf("OLED begin to work!\r\n");
	OLED_Init();
	OLED_Clear();

	OLED_ShowString(40,2,(uint8_t*)"System",16);
	OLED_ShowString(8,4,(uint8_t*)"Initialization",16);
	//MPU6050初始化
	printf("MPU6050 begin to work!\r\n");
	while(mpu_dmp_init())
	{
		
		HAL_Delay(200);
		
	}
	printf("MPU-6050 Init Successfully\r\n");//

	float tempMPU = MPU_Get_Temperature();
	printf("temp : %f\r\n",tempMPU / 100 );

	
	if(!MPU_Get_Gyroscope(&x, &y,&z))
	{
		printf("initial num is x : %d   y:%d    z%d\r\n",x,y,z);
	}
	
	//BMP280初始化
//	bmp280_init();
	BMP280_InitALL();


	OLED_Clear();
	OLED_ShowString(0,0,(uint8_t*)"Speed:",16);
	OLED_ShowString(0,2,(uint8_t*)"Temp:",16);
	OLED_ShowString(0,4,(uint8_t*)"AtomPre:",16);
//	OLED_ShowString(0,2,(uint8_t*)"Y:",16);
//	OLED_ShowString(0,4,(uint8_t*)"Z:",16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Flag_LED)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			HAL_Delay(500);
		}
		
//		tempMPU = MPU_Get_Temperature();
//		printf("temp : %f\r\n",tempMPU / 100 );
//		if(!MPU_Get_Accelerometer(&x, &y,&z))//角加速度
//		{
//			printf("x : %f   y:%f    z%f\r\n",x/16384.0,y/16384.0,z/16384.0);
//		}
		
		if(!MPU_Get_Gyroscope(&x, &y,&z))//陀螺仪
		{
//			if((abs(x)<5)&(abs(y)<5)&(abs(z)<5)){x=0;y=0;z=0;}
//			printf("initial num is x : %d   y:%d    z%d\r\n",x,y,z);
//			printf("initial num is x : %d   y:%d    z%d\r\n",x,y,z);
			speed = pow(x*x+y*y+z*z,1.0/3);
			printf("speed=%f\r\n",speed);
//			sprintf(speedToShow,"%.1f",speed);
//			OLED_ShowString(16,6,speedToShow,16);
			OLED_ShowNum(48,0,speed/10,4,16);
			
		}
		
		T = BMP280_GetTemperature();
		P = BMP280_GetPressure();
		
//		BMP280_calc_values();
//		HAL_Delay(300);
		sprintf(tempToShow,"%.2f",T/100.0);
//		printf("P=%llu\r\n",P/256);
		OLED_ShowString(40,2,(uint8_t*)tempToShow,16);
//		OLED_ShowNum(40,2,speed/10,4,16);
		OLED_ShowNum(64,4,P/256,6,16);
		
		
//		OLED_ShowAngle();
		
		
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
