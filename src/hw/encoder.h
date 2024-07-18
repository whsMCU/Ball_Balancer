/*
 * encoder.h
 *
 *  Created on: Jan 22, 2024
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_ENCODER_H_
#define SRC_COMMON_HW_INCLUDE_ENCODER_H_

#include "hw_def.h"
#include "hw.h"

#define ENCODER_PHASE_0 0
#define ENCODER_PHASE_1 2
#define ENCODER_PHASE_2 3
#define ENCODER_PHASE_3 1


#define ENCODER_10X_STEPS_PER_SEC   30  // (steps/s) Encoder rate for 10x speed
#define ENCODER_100X_STEPS_PER_SEC  80  // (steps/s) Encoder rate for 100x speed


/*********************** Encoder Set ***********************/

typedef struct {
  bool enabled;
  int encoderMoveValue;
  millis_t lastEncoderTime;
} ENCODER_Rate;

extern ENCODER_Rate EncoderRate;

typedef enum {
  ENCODER_DIFF_NO    = 0,  // no state
  ENCODER_DIFF_CW    = 1,  // clockwise rotation
  ENCODER_DIFF_CCW   = 2,  // counterclockwise rotation
  ENCODER_DIFF_ENTER = 3   // click
} EncoderState;

// Encoder initialization
void Encoder_Configuration();

// Analyze encoder value and return state
EncoderState Encoder_ReceiveAnalyze();


#endif /* SRC_COMMON_HW_INCLUDE_BUTTON_H_ */
