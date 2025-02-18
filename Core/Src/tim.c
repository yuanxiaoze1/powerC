/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "user_can.h"
#define GPIO_TEST GPIOE
#include "INA266.h"
#include "PowerConrtol.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16800 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM1_MspInit 0 */

    /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    /* USER CODE BEGIN TIM1_MspInit 1 */

    /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM1_MspDeInit 0 */

    /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    /* USER CODE BEGIN TIM1_MspDeInit 1 */

    /* USER CODE END TIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
extern int16_t setV[MOTOR_NUM];
int8_t cntTim2 = 0;
int8_t flagtime2 = -1;
extern Robermaster robermaster;
extern CONTROL_MODE control_mode;
void control_real_compete(void)
{
  cntTim2++;

  if (cntTim2 == 1)
  {
    setV[0] = 0;
    setV[1] = 0;
  }
  if (cntTim2 == 2)
  {
    setV[0] = 3000;
    setV[1] = 3000;
  }
  if (cntTim2 == 5)
  {
    setV[0] = 0;
    setV[1] = 0;
  }
  if (cntTim2 == 6)
  {
    setV[0] = -3000;
    setV[1] = -3000;
  }
  if (cntTim2 == 9)
  {
    setV[0] = 0;
    setV[1] = 0;
  }
  if (cntTim2 > 9)
  {
    cntTim2 = 0;
  }
}

void control_acclerate(void)
{
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    setV[i] += flagtime2 * 100;
    if (setV[i] > 7500)
      flagtime2 = -1;
    if (setV[i] < -7500)
      flagtime2 = 1;
  }
}

void control_position(void)
{
  ;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    if (robermaster == RobermasterA)
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
    else if (robermaster == RobermasterC)
      HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
    if (control_mode == CONTROL_REAL_COMPETE)
      control_real_compete();
    if (control_mode == CONTROL_ACCLERATE)
      control_acclerate();
    if (control_mode == CONTROL_POSITION)
      control_position();
  }
}

/* USER CODE END 1 */
