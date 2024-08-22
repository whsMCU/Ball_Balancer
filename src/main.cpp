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
#include "AccelStepper.h"
#include "MultiStepper.h"

Machine machine(2, 3.125, 1.75, 3.669291339);     //(d, e, f, g) object to define the lengths of the machine
TouchScreen ts = TouchScreen(Touch_xm, ADC_Touch_ym, ADC_Touch_xp, Touch_yp, 0);  //touch screen pins (XGND, YGND, X5V, Y5V)

//stepper motors
AccelStepper stepperA(1, StepA_STEP, StepA_DIR);  //(driver type, STEP, DIR) Driver A
AccelStepper stepperB(1, StepB_STEP, StepB_DIR);  //(driver type, STEP, DIR) Driver B
AccelStepper stepperC(1, StepC_STEP, StepC_DIR);  //(driver type, STEP, DIR) Driver C
MultiStepper steppers;           // Create instance of MultiStepper

//stepper motor variables
long pos[3] = {0, 0, 0};                            // An array to store the target positions for each stepper motor
double angOrig = 206.662752199;                        //original angle that each leg starts at
double speed[3] = { 0, 0, 0 }, speedPrev[3], ks = 20;  //the speed of the stepper motor and the speed amplifying constant

//touch screen variables
double Xoffset = 500;  //X offset for the center position of the touchpad
double Yoffset = 500;  //Y offset for the center position of the touchpad

//PID variables
double kp = 4E-4, ki = 2E-6, kd = 7E-3;                                                       //PID constants
double error[2] = { 0, 0 }, errorPrev[2], integr[2] = { 0, 0 }, deriv[2] = { 0, 0 }, out[2];  //PID terms for X and Y directions
long timeI;                                                                           //variables to capture initial times

//Other Variables
double angToStep = 3200 / 360;  //angle to step conversion factor (steps per degree) for 16 microsteps or 3200 steps/rev
bool detected = 0;              //this value is 1 when the ball is detected and the value is 0 when the ball in not detected

void moveTo(double hz, double nx, double ny);
void PID(double setpointX, double setpointY);

//void lv_draw_chart(lv_chart_series_t * ser1, lv_chart_series_t * ser2)
//{
//  static uint32_t last_tick = 0;
//  uint32_t curr_tick = millis();
//
//  if((curr_tick - last_tick) >= (500)) {
//      last_tick = curr_tick;
//
//      //lv_chart_set_next_value(ui_SpeedStepChart, ser1, stepper_A._targetPos);
//      //lv_chart_set_next_value(ui_SpeedStepChart, ser2, stepper_A._currentPos);
//
//      //lv_chart_refresh(ui_SpeedStepChart); /*Required after direct set*/
//  }
//}

void SystemClock_Config(void);

int main(void)
{
	HAL_Init();

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  hwInit();

  //lv_init();

  //lv_port_disp_init();
  //lv_port_indev_init();

  //LCD_Draw_Logo();
  //HAL_Delay(2000);

  //ui_init();

  //lv_chart_series_t * ui_SpeedStepChart_series_Target = lv_chart_add_series(ui_SpeedStepChart, lv_color_hex(0xFF0000),
  //                                                                         LV_CHART_AXIS_PRIMARY_Y);
  //lv_chart_series_t * ui_SpeedStepChart_series_Step = lv_chart_add_series(ui_SpeedStepChart, lv_color_hex(0x0005FF),
  //                                                                         LV_CHART_AXIS_SECONDARY_Y);

  while (1)
  {
    PID(0, 0);  //(X setpoint, Y setpoint) -- must be looped

    //cliMain();
	  //lv_draw_chart(ui_SpeedStepChart_series_Target, ui_SpeedStepChart_series_Step);

	  //lv_timer_handler();
	  //HAL_Delay(5);
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

  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);
  //Enable pin
  gpioPinMode(StepA_EN, _DEF_OUTPUT);    //define enable pin as output
  gpioPinMode(StepB_EN, _DEF_OUTPUT);    //define enable pin as output
  gpioPinMode(StepC_EN, _DEF_OUTPUT);    //define enable pin as output
  gpioPinWrite(StepA_EN, _DEF_HIGH);  //sets the drivers on initially
  gpioPinWrite(StepB_EN, _DEF_HIGH);  //sets the drivers on initially
  gpioPinWrite(StepC_EN, _DEF_HIGH);  //sets the drivers on initially

  delay(1000);             //small delay to allow the user to reset the platform
  gpioPinWrite(StepA_EN, _DEF_LOW);  //sets the drivers on initially
  gpioPinWrite(StepB_EN, _DEF_LOW);  //sets the drivers on initially
  gpioPinWrite(StepC_EN, _DEF_LOW);  //sets the drivers on initially

  moveTo(4.25, 0, 0);             //moves the platform to the home position
  steppers.runSpeedToPosition();  //blocks until the platform is at the home position
}

//moves/positions the platform with the given parameters
void moveTo(double hz, double nx, double ny) {
  //if the ball has been detected
  if (detected) {
    //calculates stepper motor positon
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep);
    }
    //sets calculated speed
    stepperA.setMaxSpeed(speed[A]);
    stepperB.setMaxSpeed(speed[B]);
    stepperC.setMaxSpeed(speed[C]);
    //sets acceleration to be proportional to speed
    stepperA.setAcceleration(speed[A] * 30);
    stepperB.setAcceleration(speed[B] * 30);
    stepperC.setAcceleration(speed[C] * 30);
    //sets target positions
    stepperA.moveTo(pos[A]);
    stepperB.moveTo(pos[B]);
    stepperC.moveTo(pos[C]);
    //runs stepper to target position (increments at most 1 step per call)
    stepperA.run();
    stepperB.run();
    stepperC.run();
  }
  //if the hasn't been detected
  else {
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angOrig - machine.theta(i, hz, 0, 0)) * angToStep);
    }
    //sets max speed
    stepperA.setMaxSpeed(800);
    stepperB.setMaxSpeed(800);
    stepperC.setMaxSpeed(800);
    //moves the stepper motors
    steppers.moveTo(pos);
    steppers.run();  //runs stepper to target position (increments at most 1 step per call)
  }
}

//takes in an X and Y setpoint/position and moves the ball to that position
void PID(double setpointX, double setpointY) {
  TSPoint p = ts.getPoint();  //measure X and Y positions
  //if the ball is detected (the x position will not be 0)
  if (p.x != 0) {
    detected = 1;
    //calculates PID values
    for (int i = 0; i < 2; i++) {
      errorPrev[i] = error[i];                                                                     //sets previous error
      error[i] = (i == 0) * (Xoffset - p.x - setpointX) + (i == 1) * (Yoffset - p.y - setpointY);  //sets error aka X or Y ball position
      integr[i] += error[i] + errorPrev[i];                                                        //calculates the integral of the error (proportional but not equal to the true integral of the error)
      deriv[i] = error[i] - errorPrev[i];                                                          //calcuates the derivative of the error (proportional but not equal to the true derivative of the error)
      deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                                //checks if the derivative is a real number or infinite
      out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i];                                     //sets output
      out[i] = constrainf(out[i], -0.25, 0.25);                                                     //contrains output to have a magnitude of 0.25
    }
    //calculates stepper motor speeds
    for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];                                                                                                           //sets previous speed
      speed[i] = (i == A) * stepperA.currentPosition() + (i == B) * stepperB.currentPosition() + (i == C) * stepperC.currentPosition();  //sets current position
      speed[i] = abs(speed[i] - pos[i]) * ks;                                                                                            //calculates the error in the current position and target position
      speed[i] = constrainf(speed[i], speedPrev[i] - 200, speedPrev[i] + 200);                                                            //filters speed by preventing it from beign over 100 away from last speed
      speed[i] = constrainf(speed[i], 0, 1000);                                                                                           //constrains sped from 0 to 1000
    }
    //Serial.println((String) "X OUT = " + out[0] + "   Y OUT = " + out[1] + "   Speed A: " + speed[A]);  //print X and Y outputs
  }
  //if the ball is not detected (the x value will be 0)
  else {
    //double check that there is no ball
    delay(10);                  //10 millis delay before another reading
    TSPoint p = ts.getPoint();  //measure X and Y positions again to confirm no ball
    if (p.x == 0) {             //if the ball is still not detected
      //Serial.println("BALL NOT DETECTED");
      detected = 0;
    }
  }
  //continues moving platforma and waits until 20 millis has elapsed
  timeI = millis();
  while (millis() - timeI < 20) {
    moveTo(4.25, -out[0], -out[1]);  //moves the platfrom
  }
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
