/**
  ******************************************************************************************** 
  *@file   : STM32F407_I2C_LCD16x02_Driver.c
  *@author : Sharath N
  *@brief  : LCD driver source file for interfacing 16x02 LCD on STM32Fxx MCU using I2C Serial 
	         Interface Module
  ********************************************************************************************
*/	

#include "stm32f4xx_hal.h"
#include "STM32F407_I2C_LCD16x02_Driver.h"



/**
 * @brief  Initializes LCD
 * @retval None
 */
void LCD_Init(void)
{
	
  
	/* Wait for 15ms */
	HAL_Delay(15);
	
	/*Function Set - As per HD44780U*/
	LCD_Send_Cmd(LCD_FUNCTION_SET1);
	
	/*Function Set -As per HD44780U*/
	LCD_Send_Cmd(LCD_FUNCTION_SET2);
	
	/*Set 4bit mode and 2 lines */
	LCD_Send_Cmd(LCD_4BIT_2LINE_MODE);
	
	/*Display on, cursor off*/
	LCD_Send_Cmd(0x0C);
	
	/* SET Row1 and Col1 (1st Line) */
	LCD_Send_Cmd(0x80);
	
	/*Clear Display*/
	LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
	
}



/**
 * @brief Send Strings to LCD
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Send_String(char *str)
{
	while (*str)  
	{
		LCD_Send_Data(*str++);
	}
}


/**
 * @brief Clears screen first, then displays the given string
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Clear_Then_Display(char *str)
{
	LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
	LCD_Send_String(str);
}



/**
 * @brief Display the strings on Line1
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Send_String_On_Line1(char *str)
{
	LCD_Send_Cmd(LCD_SET_ROW1_COL1);
	LCD_Send_String(str);
}


/**
 * @brief Display the strings on Line2
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Send_String_On_Line2(char *str)
{
	LCD_Send_Cmd(LCD_SET_ROW2_COL1);
	LCD_Send_String(str);
}


/**
 * @brief Display long messages of any size on LCD
 * @param str: pointer to strings
 * @retval None
 */
void LCD_Display_Long_Message(char *string)
{
	int i =0, count =1, j=0;
	/*Clear display and Set position to Line1 start*/
	LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
	LCD_Send_Cmd(LCD_SET_ROW1_COL1);
	
	while(string[i] != '\0')
	{
		LCD_Send_Data(string[i]);
		
		/*If we reach 1st Line end, then goto 2nd line start*/
		if(j>=15 && (count%2 == 1))
		{
			count++;
			LCD_Send_Cmd(LCD_SET_ROW2_COL1);
		}
		
		/*If we reach second line end, clear display start from line1 again*/
		if(j>=31 && (count%2 == 0))
		{
			count++;
			j=0;
			LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
			LCD_Send_Cmd(LCD_SET_ROW1_COL1);
		}
		
		HAL_Delay(100);
		i++;
		j++;
	}
}







