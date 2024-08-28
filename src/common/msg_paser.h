/*
 * msg_paser.h
 *
 *  Created on: Aug 26, 2024
 *      Author: WANG
 */

#ifndef SRC_COMMON_MSG_PASER_H_
#define SRC_COMMON_MSG_PASER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"

typedef struct tspoint_s {
    int16_t x;
    int16_t y;
    int16_t z;
} tspoint_t;

extern double kp, ki, kd;
extern tspoint_t ts_point;
extern uint8_t telemetry_tx_buf[40];

void msg_paser(void);

void Encode_Msg_Status(unsigned char* telemetry_tx_buf);

#ifdef __cplusplus
}
#endif
#endif /* SRC_COMMON_MSG_PASER_H_ */
