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
TSPoint p;

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
//double kp = 4E-4, ki = 2E-6, kd = 7E-3;                                                       //PID constants
double error[2] = { 0, 0 }, errorPrev[2], integr[2] = { 0, 0 }, deriv[2] = { 0, 0 }, out[2];  //PID terms for X and Y directions
long timeI, timeII;                                                                           //variables to capture initial times

//Other Variables
double angToStep = 3200 / 360;  //angle to step conversion factor (steps per degree) for 16 microsteps or 3200 steps/rev
bool detected = 0;              //this value is 1 when the ball is detected and the value is 0 when the ball in not detected

void moveTo(double hz, double nx, double ny);
void PID(double setpointX, double setpointY);

void moveToPID(int X, int Y, int wait);
void linePattern(double rx, int ry, int wait, int num);
void trianglePattern(int num);
void squarePattern(int num);
void pinBallPattern(int Y, int wait);
void ellipsePattern(double rx, int ry, double start, int wait, int num);
void sinusoidalPattern(double ampli, double freq, int wait);
void figure8Pattern(double r, double start, int wait, int num);
void DEMO(void);

void SystemClock_Config(void);

uint32_t pre_time = 0;

int main(void)
{
	HAL_Init();

  SystemClock_Config();

  hwInit();

  pre_time = micros();

  while (1)
  {
    //moveToPID(0, 0, 5000); //moves the ball to a position and keeps it there (X, Y, wait)

    switch(mode)
    {
      case PID_pattern:
        PID(0, 0);  //(X setpoint, Y setpoint) -- must be looped
        break;

      case line_pattern:
        linePattern(100, 0, 800, 2);  //moves the ball in a line (rx, ry, wait, num)
        break;

      case triangle_pattern:
        trianglePattern(3); //moves the ball in a triangle (num)
        break;

      case square_pattern:
        squarePattern(3);  //moves the ball in a square (num)
        break;

      case pinBall_pattern:
        pinBallPattern(200, 600);  //moves the ball in a pinball pattern (Y, wait)
        break;

      case ellipse_pattern:
        ellipsePattern(100, 100, 0, 20, 5);  //moves the ball in an elipse (rx, ry, start, wait, num)
        break;

      case sinusoidal_pattern:
        sinusoidalPattern(50, 30, 20);  //moves ball in a sinusoidal pattern (A, B, wait)
        break;

      case figure8_pattern:
        figure8Pattern(200, 0, 10, 5);  //moves the ball in an elipse (r, start, wait, num)
        break;

      case DEMO_pattern:
        DEMO(); //does all of the patterns sequentially;
        break;
    }

    if(micros() - pre_time >= 50000)
    {
        pre_time = micros();
        Encode_Msg_Status(&telemetry_tx_buf[0]);
        uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
        msg_paser();
    }

    cliMain();
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
  uartOpen(_DEF_UART3, 115200);

  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  stepperA.setPinsInverted(1, 0, 0);
  stepperB.setPinsInverted(1, 0, 0);
  stepperC.setPinsInverted(1, 0, 0);
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
  p = ts.getPoint();  //measure X and Y positions
  memcpy(&ts_point, &p, sizeof(TSPoint));
//  ts_point.x = p.x;
//  ts_point.y = p.y;
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

//moves the ball to a location with
void moveToPID(int X, int Y, int wait) {
  //X = x setpoint
  //Y = y setpoint
  //wait = elapsed time
  timeII = millis();
  while (millis() - (uint32_t)timeII < (uint32_t)wait) {
    PID(X, Y);
  }
}
//moves the ball in a line
void linePattern(double rx, int ry, int wait, int num) {
  //rx = half the traversal length in the X direction (0 - 200)
  //ry = half the traversal length in the Y direction (0 - 200)
  //wait = time delay between movements in ms (0 - 1000)
  //num = number of interations
  for (int i = 0; i < num; i++) {
    timeII = millis();
    while (millis() - (uint32_t)timeII < (uint32_t)wait) {
      PID(rx, ry);  //(X setpoint, Y setpoint)
    }
    timeII = millis();
    while (millis() - (uint32_t)timeII < (uint32_t)wait) {
      PID(-rx, -ry);  //(X setpoint, Y setpoint)
    }
  }
}
//moves the ball in a triangle
void trianglePattern(int num) {
  //num = number of interations
  double s = 400;  //side length
  for (int i = 0; i < num; i++) {
    for (int j = 0; j < 3; j++) {
      timeII = millis();
      while (millis() - timeII < 800) {
        PID((1 - j) * (s / 2), j == 1 ? s * (sqrt(3) / 4) : -s * (sqrt(3) / 4));  //(X setpoint, Y setpoint)
      }
    }
  }
}
//moves the ball in a square
void squarePattern(int num) {
  //num = number of interations
  int s = 400;  //side length
  for (int i = 0; i < num; i++) {
    for (int j = 0; j < 4; j++) {
      timeII = millis();
      while (millis() - timeII < 700) {
        PID(j < 2 ? s / 2 : -s / 2, j == 1 || j == 2 ? s / 2 : -s / 2);  //(X setpoint, Y setpoint)
      }
    }
  }
}
//moves the ball in a pinBall pattern
void pinBallPattern(int Y, int wait) {
  //Y = amplitude - (100-200)
  //wait = time delay between movements in ms (0 - 1000)
  Y *= -1;
  for (int X = 200; X >= -300; X -= 100) {
    timeII = millis();
    while (millis() - (uint32_t)timeII < (uint32_t)wait) {
      PID(X, Y);  //(X setpoint, Y setpoint)
    }
    Y *= -1;
  }
  for (int X = -200; X <= 300; X += 100) {
    timeII = millis();
    while (millis() - (uint32_t)timeII < (uint32_t)wait) {
      PID(X, Y);  //(X setpoint, Y setpoint)
    }
    Y *= -1;
  }
}
//moves the ball in an elipse
void ellipsePattern(double rx, int ry, double start, int wait, int num) {
  //rx = x axis radius (0 - 150)
  //ry = y axis radius (0 - 150)
  //start = 0 or 2*PI
  //wait = time delay between movements in ms (0 - 20)
  //num = number of times to traverse the elipse
  double theta;
  for (int i = 0; i < num; i++) {
    theta = start;
    for (double j = 0; j <= 2 * M_PI; j += 0.1) {
      timeII = millis();
      while (millis() - (uint32_t)timeII < (uint32_t)wait) {        //moves the ball
        PID(rx * cos(theta), ry * sin(theta));  //(X setpoint, Y setpoint)
      }
      theta += start == 0 ? 0.1 : (start == 2 * M_PI ? -0.1 : 0);
    }
  }
}
//moves ball in a sinusoidal pattern
void sinusoidalPattern(double ampli, double freq, int wait) {
  //ampli = amplitude
  //freq = frequency
  //wait = time delay between movements in ms (0 - 20)
  for (double X = 300; X >= -300; X -= 5) {
    timeII = millis();
    while (millis() - (uint32_t)timeII < (uint32_t)wait) {  //moves the ball
      PID(X, ampli * sin(X / freq));    //(X setpoint, Y setpoint)
    }
  }
  for (double X = -300; X <= 300; X += 5) {
    timeII = millis();
    while (millis() - (uint32_t)timeII < (uint32_t)wait) {  //moves the ball
      PID(X, ampli * sin(X / freq));    //(X setpoint, Y setpoint)
    }
  }
}
//moves the ball in figure 8
void figure8Pattern(double r, double start, int wait, int num) {
  //r = x and Y axis radius (0 - 150)
  //start = 0 or -2*PI
  //wait = time delay between movements in ms (0 - 20)
  //num = number of times to traverse the elipse
  double theta;
  double scale;
  for (int i = 0; i < num; i++) {
    theta = start;
    for (double j = 0; j < 2 * M_PI; j += 0.05) {
      timeII = millis();
      scale = r * (2 / (3 - cos(2 * theta)));  //moves the ball
      while (millis() - (uint32_t)timeII < (uint32_t)wait) {
        PID(scale * cos(theta), scale * sin(2 * theta) / 1.5);  //(X setpoint, Y setpoint)
      }
      theta += start == 0 ? -0.05 : (start == -2 * M_PI ? 0.05 : 0);
    }
  }
}
//demo
void DEMO() {
  moveToPID(0, 0, 8000);
  linePattern(200, 0, 1000, 1);  //moves the ball in a line (rx, ry, wait, num)
  linePattern(200, 0, 600, 2);  //moves the ball in a line (rx, ry, wait, num)
  //triangle demo
  trianglePattern(2);  //moves the ball in a triangle (num)
  //square demo
  squarePattern(2);  //moves the ball in a square (num)
  //pinBall demo
  pinBallPattern(175, 500);  //moves the ball in a pinball pattern (wait)
  pinBallPattern(100, 300);  //moves the ball in a pinball pattern (wait)
  //circle demo
  ellipsePattern(50, 50, 0, 1, 2);     //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(100, 100, 0, 10, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(150, 150, 0, 20, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(50, 150, 0, 20, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(150, 50, 0, 20, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  //sinusoidal demo
  sinusoidalPattern(50, 30, 10);   //moves ball in a sinusoidal pattern (ampli, freq, wait)
  sinusoidalPattern(100, 50, 10);  //moves ball in a sinusoidal pattern (ampli, freq, wait)
  sinusoidalPattern(150, 80, 10);  //moves ball in a sinusoidal pattern (ampli, freq, wait)
  //figure 8 demo
  figure8Pattern(200, 0, 20, 3);  //moves the ball in an elipse (r, start, wait, num)
  //end
  moveToPID(0, 0, 2000);
  for (int i = 0; i < 3; i++) {
    pos[i] = round((angOrig - machine.theta(i, 4.25, 0, 0.25)) * angToStep);
  }
  stepperA.setMaxSpeed(2000);
  stepperB.setMaxSpeed(2000);
  stepperC.setMaxSpeed(2000);
  steppers.moveTo(pos);
  steppers.runSpeedToPosition();  //blocks until the platform is at the home position
  delay(1000);
  detected = 0;
  moveTo(4.25, 0, 0);             //moves the platform to the home position
  steppers.runSpeedToPosition();  //blocks until the platform is at the home position
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 3; j++) {
      pos[j] = round((angOrig - machine.theta(j, 5 - 1.25 * (i % 2 != 0), 0, 0)) * angToStep);
    }
    //sets max speed
    stepperA.setMaxSpeed(2000);
    stepperB.setMaxSpeed(2000);
    stepperC.setMaxSpeed(2000);
    //moves the stepper motors
    steppers.moveTo(pos);
    steppers.runSpeedToPosition();  //runs stepper to target position (increments at most 1 step per call)
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
