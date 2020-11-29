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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "led_7seg_74hc595.h"


//#include "led_7seg_74hc595.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLK(x) HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,x)
#define ST(x)  HAL_GPIO_WritePin(RT_GPIO_Port,RT_Pin,x)
#define DATA(x)  HAL_GPIO_WritePin(DO_GPIO_Port,DO_Pin,x)
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t dem;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void xuat_1byte(unsigned char x);
void xuat_led7(unsigned char vitri,unsigned char gt);
void xuat_full(unsigned char gt1,unsigned char gt2,unsigned char gt3,unsigned char gt4,
							 unsigned char gt5,unsigned char gt6,unsigned char gt7,unsigned char gt8);/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void xuat_1byte(unsigned char x)
{
unsigned char i, y,giatri;

for(i=0;i<8;i++)
{	
				y=x;
				giatri=(y&0x80);
				//giatri= giatri!=0?1:0;
        DATA(giatri);
        CLK(1);
				CLK(0);
				x=x<<1;
}

}
void xuat_led7(unsigned char vitri,unsigned char gt)//,unsigned char byte2,unsigned char byte3,unsigned char byte4,
	//						 unsigned char byte5,unsigned char byte6,unsigned char byte7,unsigned char byte8)
 {
	 xuat_1byte(vitri);
	 xuat_1byte(gt);
	 ST(1);
	 ST(0);
 }
 
void xuat_full(unsigned char gt1,unsigned char gt2,unsigned char gt3,unsigned char gt4,
							 unsigned char gt5,unsigned char gt6,unsigned char gt7,unsigned char gt8)
 {
		unsigned char vitri1[]={0x00,0x01,0x02,0x4,0x8,0x10,0x20,0x40,0x80};
		uint16_t i,z;
	 for(i=0;i<200;i++)
		{
			xuat_led7(vitri1[0],gt1);//HAL_Delay(1);
			xuat_led7(vitri1[1],gt1);
			xuat_led7(vitri1[0],gt2);//HAL_Delay(1);
			xuat_led7(vitri1[2],gt2);			
			xuat_led7(vitri1[0],gt3);//HAL_Delay(1);
			xuat_led7(vitri1[3],gt3);
			xuat_led7(vitri1[0],gt4);//HAL_Delay(1);	
			xuat_led7(vitri1[4],gt4);
			xuat_led7(vitri1[0],gt5);//HAL_Delay(1);
			xuat_led7(vitri1[5],gt5);
			xuat_led7(vitri1[0],gt6);//HAL_Delay(1);
			xuat_led7(vitri1[6],gt6);
			xuat_led7(vitri1[0],gt7);//HAL_Delay(1);
			xuat_led7(vitri1[7],gt7);
			xuat_led7(vitri1[0],gt8);//HAL_Delay(1);
			xuat_led7(vitri1[8],gt8);
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
unsigned char led7[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		dem++;
		if(dem<10)
		{
		xuat_full(0xff,0xff,0xff,0xff,0xff,0xff,0xff,led7[dem%10]);
		}
		else if(dem<100)
		{
		xuat_full(0xff,0xff,0xff,0xff,0xff,0xff,led7[dem/10],led7[dem%10]);
		}
		else if (dem<1000)
		{
		xuat_full(0xff,0xff,0xff,0xff,0xff,led7[dem/100],led7[dem%100/10],led7[dem%100%10]);	
		}
		else if (dem<10000)
		{
			
		}
		else if (dem<100000)
		{
			
		}
		else if (dem<1000000)
		{
			
		}
		else if (dem<10000000)
		{
			
		}
		else
		{
			
		}
		
		
		//HAL_Delay(1000);
		
  }
}
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Pin|RT_Pin|DO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin RT_Pin DO_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|RT_Pin|DO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
