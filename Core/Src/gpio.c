/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PF5 PF8 PF9 PF10
                           PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG3 PG4 PG5 PG6
                           PG7 PG8 PG9 PG10
                           PG11 PG12 PG13 PG14
                           PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 PD4
                           PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

uint8_t GPIO_INPUT(uint8_t num)
{
	uint8_t status;
	switch (num)
	{
		case 1 : status = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_15);break;
		case 2 : status = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_14);break;
		case 3 : status = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_13);break;
		case 4 : status = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_12);break;
		case 5 : status = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_11);break;
		case 6 : status = HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_10);break;
		case 7 : status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9);break;
		case 8 : status = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);break;
		case 9 : status = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);break;
		case 10: status = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);break;
		case 11: status = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);break;
		case 12: status = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);break;
		case 13: status = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);break;
		case 14: status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0);break;
		case 15: status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8);break;
		case 16: status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7);break;
		case 17: status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);break;
		case 18: status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5);break;
		case 19: status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4);break;
		case 20: status = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3);break;
	}
	return status;
}

void GPIO_OUTPUT_OPEN(uint8_t num)
{
	switch (num)
	{
		case 1 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14, GPIO_PIN_RESET);break;
		case 2 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_RESET);break;
		case 3 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12, GPIO_PIN_RESET);break;
		case 4 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);break;
		case 5 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_RESET);break;
		case 6 : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);break;
		case 7 : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);break;
		case 8 : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);break;
		case 9 : HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);break;
		case 10: HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);break;
		case 11: HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);break;
		case 12: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_RESET);break;
		case 13: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_RESET);break;
		case 14: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_RESET);break;
		case 15: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_RESET);break;
		case 16: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11, GPIO_PIN_RESET);break;
		case 17: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_RESET);break;
		case 18: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);break;
		case 19: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);break;
		case 20: HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_RESET);break;
	}
}

void GPIO_OUTPUT_CLOSE(uint8_t num)
{
	switch (num)
	{
		case 1 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14, GPIO_PIN_SET);break;
		case 2 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);break;
		case 3 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12, GPIO_PIN_SET);break;
		case 4 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);break;
		case 5 : HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_SET);break;
		case 6 : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);break;
		case 7 : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);break;
		case 8 : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);break;
		case 9 : HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);break;
		case 10: HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);break;
		case 11: HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);break;
		case 12: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_SET);break;
		case 13: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);break;
		case 14: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_SET);break;
		case 15: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_SET);break;
		case 16: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11, GPIO_PIN_SET);break;
		case 17: HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_SET);break;
		case 18: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);break;
		case 19: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);break;
		case 20: HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_SET);break;
	}
}

void GPIO_OUTPUT_CLOSEALL(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_SET);
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
