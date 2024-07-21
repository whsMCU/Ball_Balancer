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

#include "main.h"

#include "TouchScreen.h"
#include "InverseKinematics.h"

Machine machine(2, 3.125, 1.75, 3.669291339);     //(d, e, f, g) object to define the lengths of the machine
TouchScreen ts = TouchScreen(ADC_Touch_xm, ADC_TouchY_ym, ADC_Touch_xp, ADC_Touch_yp, 0);  //touch screen pins (XGND, YGND, X5V, Y5V)

AccelStepper stepper_A;

void lv_draw_chart(lv_chart_series_t * ser1, lv_chart_series_t * ser2)
{
  static uint32_t last_tick = 0;
  uint32_t curr_tick = millis();

  if((curr_tick - last_tick) >= (500)) {
      last_tick = curr_tick;

      lv_chart_set_next_value(ui_SpeedStepChart, ser1, stepper_A._targetPos);
      lv_chart_set_next_value(ui_SpeedStepChart, ser2, stepper_A._currentPos);

      //lv_chart_refresh(ui_SpeedStepChart); /*Required after direct set*/
  }
}

void SystemClock_Config(void);

int main(void)
{
	HAL_Init();

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  hwInit();

  lv_init();

  lv_port_disp_init();
  lv_port_indev_init();

  //LCD_Draw_Logo();
  //HAL_Delay(2000);

  ui_init();

  lv_chart_series_t * ui_SpeedStepChart_series_Target = lv_chart_add_series(ui_SpeedStepChart, lv_color_hex(0xFF0000),
                                                                           LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_series_t * ui_SpeedStepChart_series_Step = lv_chart_add_series(ui_SpeedStepChart, lv_color_hex(0x0005FF),
                                                                           LV_CHART_AXIS_SECONDARY_Y);

  stepper_Init(&stepper_A, StepA_EN, StepA_DIR, StepA_STEP);
  setMaxSpeed(&stepper_A, 2000);
  setAcceleration(&stepper_A, 500);
  moveTo(&stepper_A, 1600);


  while (1)
  {
	  cliMain();
	  lv_draw_chart(ui_SpeedStepChart_series_Target, ui_SpeedStepChart_series_Step);

	  lv_timer_handler();
	  HAL_Delay(5);
  }

}

void hwInit(void)
{
  #ifdef _USE_HW_RTC
    rtcInit();
  #endif
  usbInit();
  uartInit();
  cliInit();
  logInit();
  logOpen(HW_LOG_CH, 115200);
  logPrintf("\r\n[ Firmware Begin... ]\r\n");

  tim_Init();
  timBegin(_DEF_TIM1);
  timBegin(_DEF_TIM2);
  gpioInit();
  adcInit();
  buttonInit();
  flashInit();
  MX_DMA_Init();

  spiInit();

//  if (sdInit() == true)
//  {
//    fatfsInit();
//  }

  cliOpen(_DEF_USB, 115200);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
