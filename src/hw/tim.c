/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "cli.h"
/* USER CODE BEGIN 0 */
typedef struct
{
  bool is_start;

  void (*func_cb)(void);

  TIM_HandleTypeDef *h_tim;

} tim_t;
/* USER CODE END 0 */

tim_t tim_tbl[HW_TIM_MAX_CH];

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

#ifdef _USE_HW_CLI
static void cliTimer(cli_args_t *args);
#endif


void tim_Init(void)
{
  for (int i=0; i<HW_TIM_MAX_CH; i++)
  {
    tim_tbl[i].is_start = false;
    tim_tbl[i].func_cb  = NULL;
  }
}

bool timBegin(uint8_t ch)
{
  bool ret = false;
  tim_t *p_tim = &tim_tbl[ch];

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  switch(ch)
  {
    case _DEF_TIM1:
      p_tim->h_tim = &htim5;
      p_tim->func_cb = NULL;

      htim5.Instance = TIM5;
      htim5.Init.Prescaler = 83;
      htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim5.Init.Period = 4294967295;
      htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
      if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
      {
        Error_Handler();
      }
      sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
      if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
      {
        Error_Handler();
      }
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_TIM_Base_Start(&htim5) == HAL_OK)
      {
        p_tim->is_start = true;
        ret = true;
      }
      logPrintf("[%s] tim5_Init()\r\n", ret ? "OK":"NG");
      break;

    case _DEF_TIM2:
      p_tim->h_tim = &htim6;
      p_tim->func_cb = NULL;

      htim6.Instance = TIM6;
      htim6.Init.Prescaler = 83;
      htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim6.Init.Period = 49;
      htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
      if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
      {
        Error_Handler();
      }
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_TIM_Base_Start_IT(&htim6) == HAL_OK)
      {
        p_tim->is_start = true;
        ret = true;
      }
      logPrintf("[%s] tim6_Init()\r\n", ret ? "OK":"NG");
      break;
  }
  #ifdef _USE_HW_CLI
    cliAdd("timer", cliTimer);
  #endif

  return ret;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  tim_t *p_tim;
  for(uint8_t i = 0; i<HW_TIM_MAX_CH; i++)
  {
      if (htim->Instance == tim_tbl[i].h_tim->Instance)
      {
        p_tim = &tim_tbl[i];
        if (p_tim->func_cb != NULL)
        {
          (*p_tim->func_cb)();
        }
      }
  }
}

void timAttachInterrupt(uint8_t ch, void (*func)())
{
  tim_t *p_tim = &tim_tbl[ch];

  if (p_tim->is_start == false)     return;

  p_tim->func_cb = func;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* TIM5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#ifdef _USE_HW_CLI
void cliTimer(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "test") == true)
  {
    uint32_t curr_tick, last_tick;
    last_tick = micros();
    while(cliKeepLoop())
    {
      curr_tick = micros();
      cliPrintf("%d\r\n", curr_tick - last_tick);
      last_tick = curr_tick;
      delay(100);
    }
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("timer test\r\n");
  }
}
#endif
/* USER CODE END 1 */
