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
#include "rc522.h"
#include "FLASH_SECTOR_F4.h"
#include <string.h>
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define   ID_card_true    	1 ;
#define   ID_card_false    	0 ;

#define   Lcd_open_door  	1;
#define   Lcd_card_false  	2;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
uint8_t ExTi_1;
uint8_t *pExTi_1;

uint8_t u8_SPI1_RxBuff[20];
uint8_t	str[MFRC522_MAX_LEN]; // MFRC522_MAX_LEN = 16
uint8_t control_Door; //open = 1, close = 0 ;

uint8_t Rx_Data[48];
uint8_t Data[48];
uint8_t remaining_updates;
uint8_t state = 0; //display lcd

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

//uid card comparison
uint8_t uid_compare(uint8_t uid_card_flash[], uint8_t str[], uint8_t compare)
	{
		uint8_t flag = ID_card_true;
		
			if(compare == 0)
			{
				for(int i = 1; i < 5; i++) //Ignore byte start of str therefor begin i = 1
					{
						if(str[i] != uid_card_flash[i]) {flag = ID_card_false ;break;} 
					}
			}
			if(compare == 1)
			{
				for(int i = 1; i < 5; i++)
					{
						if(str[i] != uid_card_flash[i + 16]) {flag = ID_card_false ;break;} ; // card 2 begin at index 17
					}
			}
			if(compare == 2)
			{
				for(int i = 1; i < 5; i++)
					{
						if(str[i] != uid_card_flash[i + 32]) {flag = ID_card_false ;break;} ; // card 3 begin at index 33
					}
			}
			return flag;
	}

	//count_element_duplicate
uint8_t count_element_duplicate(uint8_t* array, uint8_t size, uint8_t x)
	{
    uint8_t count = 0;
    for(uint8_t i=0; i<size; i++){
      if(array[i]==x) 
        count ++;
    }
    return count;
	}
	
	
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
	if(GPIO_Pin == GPIO_PIN_0)
	{
			ExTi_1 = 1 ;
	}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}


void Handle_EXTI1()
{
		// ----------begin write data into flash, max 3---------------
		
		Flash_Read_Data(0x08061000 , Rx_Data, 48);
	 
		// coppy Rx_Data into Data
		memcpy((void *)Data, (void *)Rx_Data, sizeof(Rx_Data));
	
	if(remaining_updates == 0)
		{
		 //Data = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
			memcpy(&Data[0], (void *)str, sizeof(str)); //card 1
			Data[16] = 0; //card 2
			Data[32] = 0 ; //card 3
		}
	else
		{
		if(Data[0] == 0)
		{
			memcpy(&Data[0], (void *)str, sizeof(str));
		}
		else
		{
			if(Data[16] == 0)
			{
			memcpy(&Data[16], (void *)str, sizeof(str));
			}
			else
			{
				if(Data[32] == 0){memcpy(&Data[32], (void *)str, sizeof(str));}
			}
		}
		}
		Flash_Write_Data(0x08061000 , (uint8_t *)Data, 48);
		
		//count the number of updates remaining
		Flash_Read_Data(0x08061000 , Rx_Data, 48);
		 switch (count_element_duplicate(Rx_Data, 48, 0)) 
		{
			case 35: //35 number 0 of Rx_Data <=> remaining 2 slot can be update
				{
					remaining_updates = 2;
					break;
				}
			case 34: 
				{
					remaining_updates = 1;
					break;
				}
			default: remaining_updates = 0; //33 number 0 when add full 3 slot
				
		}
		// ----------end write data into flash, max 3---------------
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		ExTi_1 = 0;
}

void Display_Lcd(uint8_t state)
{
		switch(state)
		{
			case 0: //start
			{
				lcd_put_cur(0,0);
				lcd_send_string("This is Project");
				HAL_Delay(10);
				lcd_put_cur(1,4); //row 1, index 4
				lcd_send_string("Lock Door");
				break;
			}
			case 1: //open door
			{
				lcd_put_cur(0,2);
				lcd_send_string("Well Come To");
				HAL_Delay(10);
				lcd_put_cur(1,4); //row 1, index 4
				lcd_send_string("My House");
				break;
			}
			case 2: //card false
			{
				lcd_put_cur(0,0);
				lcd_send_string("Can't Open Door ");
				HAL_Delay(10);
				lcd_put_cur(1,0); //row 1, index 4
				lcd_send_string("Press UpdateCard");
				lcd_put_cur(1,14);
				//lcd_send_data(remaining_updates + 48);
				break;
			}
		}
}

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
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	MFRC522_Init();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	
	lcd_init();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		lcd_clear();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Display Lcd
		Display_Lcd(state);
	
		//RFID
		if (!MFRC522_Request(PICC_REQIDL, str)) { 
			
			if (!MFRC522_Anticoll(str)) {
				
		// ----------begin READ data form flash, then compare with data---------------
						Flash_Read_Data(0x08061000 , Rx_Data, 48);
						
						uint8_t compare = 0;
						while(compare < 3)
						{
							if(uid_compare(Rx_Data, str, compare))
							{
								control_Door = 1;
								lcd_clear();
								HAL_Delay(400);
								state = Lcd_open_door;
								break;
							}
							else
							{
								compare++;
							}
							
							lcd_clear();
							HAL_Delay(100);
							state = Lcd_card_false;
						}
		
		// ----------end READ data form flash, then compare with data---------------
				
			}
		}
		//EXTI1
		if(ExTi_1 == 1)
		{
			HAL_Delay(20); // Ignore the noise time of button
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
			{
					Handle_EXTI1();
				while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1); // wait for button release
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
