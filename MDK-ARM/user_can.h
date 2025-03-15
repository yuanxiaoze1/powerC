#ifndef _USER_CAN_H_
#define _USER_CAN_H_

#include "main.h"
#include "can.h"

typedef enum
{
  CONTROL_ACCLERATE,
  CONTROL_REAL_COMPETE,
  CONTROL_POSITION,
  CONTROL_CONSTANT_SPEED
} CONTROL_MODE;
typedef struct _MOTOR
{
  int16_t speed, torque;
  int8_t temp;
  uint16_t angle;
  uint16_t lastAngle; //??????????
  int32_t totalAngle; //??????????

  float nowAngle;
  float targetAngle;
} SingleMotor;

void CAN_Init(void);
void USER_CAN_SetMotorCurrent(CAN_HandleTypeDef *hcans, int16_t StdId, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void Motor_Update(SingleMotor *motor, uint16_t angle, int16_t speed, int16_t torque, int8_t temp);
void CAN_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcans);
void CAN_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
void USER_CAN_SetMotorAngle(CAN_HandleTypeDef *hcan, int16_t StdId, int32_t angle, int16_t speedLimit);

#endif
