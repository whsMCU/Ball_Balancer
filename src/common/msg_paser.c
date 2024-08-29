/*
 * msg_paser.c
 *
 *  Created on: Aug 26, 2024
 *      Author: WANG
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "msg_paser.h"

double kp = 4E-4, ki = 2E-6, kd = 7E-3;

tspoint_t ts_point;
uint8_t mode = 0;

uint8_t telemetry_tx_buf[40];


static void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d)
{
    telemetry_tx_buf[0] = 0x46;
    telemetry_tx_buf[1] = 0x43;

    telemetry_tx_buf[2] = id;

//    memcpy(&telemetry_tx_buf[3], &p, 4);
//    memcpy(&telemetry_tx_buf[7], &i, 4);
//    memcpy(&telemetry_tx_buf[11], &d, 4);

    *(float*)&telemetry_tx_buf[3] = kp;
    *(float*)&telemetry_tx_buf[7] = ki;
    *(float*)&telemetry_tx_buf[11] = kd;

    telemetry_tx_buf[15] = 0x00;
    telemetry_tx_buf[16] = 0x00;
    telemetry_tx_buf[17] = 0x00;
    telemetry_tx_buf[18] = 0x00;

    telemetry_tx_buf[19] = 0xff;

    for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
}

static void Encode_Msg_Mode(unsigned char* telemetry_tx_buf, unsigned char id, uint8_t mode)
{
    telemetry_tx_buf[0] = 0x46;
    telemetry_tx_buf[1] = 0x43;

    telemetry_tx_buf[2] = id;
    telemetry_tx_buf[3] = mode;

    for(int i=4; i<=18; i++)
    {
      telemetry_tx_buf[i] = 0x00;
    }

    telemetry_tx_buf[19] = 0xff;

    for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
}

void Encode_Msg_Status(unsigned char* telemetry_tx_buf)
{
  telemetry_tx_buf[0] = 0x46;
  telemetry_tx_buf[1] = 0x43;

  telemetry_tx_buf[2] = 0x10;

  telemetry_tx_buf[3] = (short)(ts_point.x);
  telemetry_tx_buf[4] = ((short)(ts_point.x))>>8;

  telemetry_tx_buf[5] = (short)(ts_point.y);
  telemetry_tx_buf[6] = ((short)(ts_point.y))>>8;

  telemetry_tx_buf[7] = 0x00;
  telemetry_tx_buf[8] = 0x00;

  telemetry_tx_buf[9] = 0x00;
  telemetry_tx_buf[10] = 0x00;

  telemetry_tx_buf[11] = 0x00;
  telemetry_tx_buf[12] = 0x00;

  telemetry_tx_buf[13] = 0x00;
  telemetry_tx_buf[14] = 0x00;

  telemetry_tx_buf[15] = 0x00;
  telemetry_tx_buf[16] = 0x00;

  telemetry_tx_buf[17] = 0x00;
  telemetry_tx_buf[18] = 0x00;

  telemetry_tx_buf[19] = 0xff;

  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
}

void msg_paser(void)
{
  if(telemetry_rx_cplt_flag == 1)
  {
    telemetry_rx_cplt_flag = 0;

      unsigned char chksum = 0xff;
      for(int i=0;i<19;i++) chksum = chksum - telemetry_rx_buf[i];

      if(chksum == telemetry_rx_buf[19])
      {

        switch(telemetry_rx_buf[2])
        {
        case 0x00:
          kp = *(float*)&telemetry_rx_buf[3];
          ki = *(float*)&telemetry_rx_buf[7];
          kd = *(float*)&telemetry_rx_buf[11];
          //writeSDCard(PID_Roll_in);
          Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0x20, kp, ki, kd);
          uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
          break;
        case 0x10:
          Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0x20, kp, ki, kd);
          uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
          break;
        case 0x20:
          kp = *(float*)&telemetry_rx_buf[3];
          ki = *(float*)&telemetry_rx_buf[7];
          kd = *(float*)&telemetry_rx_buf[11];
          //writeSDCard(PID_Roll_in);
          Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], kp, ki, kd);
          uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
          break;
        case 0x30:
          switch(telemetry_rx_buf[3])
          {
          case PID_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case line_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case triangle_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case square_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case pinBall_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case ellipse_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case sinusoidal_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case figure8_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;

          case DEMO_pattern:
            mode = *(uint8_t*)&telemetry_rx_buf[3];
            Encode_Msg_Mode(&telemetry_tx_buf[0], telemetry_rx_buf[2], mode);
            uartWriteIT(_DEF_UART3, &telemetry_tx_buf[0], 20);
            break;
          }
          break;

        }
      }
  }
}

#ifdef __cplusplus
}
#endif

