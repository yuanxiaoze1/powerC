#include "user_can.h"
extern SingleMotor motors[1];
extern SingleMotor motorM4005;

uint32_t CAN_TX_BOX0 = CAN_TX_MAILBOX0;
extern int state;
#define ABS(x) ((x) > 0 ? (x) : -(x))
CONTROL_MODE control_mode = CONTROL_CONSTANT_SPEED;
void CAN_Init()
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 0;                     // filter 0
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK; // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;                 // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);                         // init can filter
  HAL_CAN_Start(&hcan1);                                             // start can1
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can1 rx interrupt
  //   HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
}
void USER_CAN_SetMotorCurrent(CAN_HandleTypeDef *hcan, int16_t StdId, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CAN_TxHeaderTypeDef tx_header;

  uint8_t tx_data[8];
  tx_header.ExtId = 0;
  tx_header.StdId = StdId;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  tx_data[0] = (iq1 >> 8) & 0xff;
  tx_data[1] = (iq1) & 0xff;
  tx_data[2] = (iq2 >> 8) & 0xff;
  tx_data[3] = (iq2) & 0xff;
  tx_data[4] = (iq3 >> 8) & 0xff;
  tx_data[5] = (iq3) & 0xff;
  tx_data[6] = (iq4 >> 8) & 0xff;
  tx_data[7] = (iq4) & 0xff;

  HAL_StatusTypeDef state = HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}
// 设置电机角度
void USER_CAN_SetMotorAngle(CAN_HandleTypeDef *hcan, int16_t StdId, int32_t angle, int16_t speedLimit)
{
  // 定义CAN发送头
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  tx_header.ExtId = 0;
  tx_header.StdId = StdId;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_data[0] = 0xA4;
  tx_data[1] = 0x00;
  tx_data[2] = (speedLimit) & 0xff;
  tx_data[3] = (speedLimit >> 8) & 0xff;
  tx_data[4] = (angle) & 0xff;
  tx_data[5] = (angle >> 8) & 0xff;
  tx_data[6] = (angle >> 16) & 0xff;
  tx_data[7] = (angle >> 24) & 0xff;
  // tx_data[2] = 0x00;
  // tx_data[3] = 0x00;
  // tx_data[4] = 0x00;
  // tx_data[5] = 0x00;
  // tx_data[6] = 0x00;
  // tx_data[7] = 0x00;
  HAL_StatusTypeDef state = HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}
void Motor_Update(SingleMotor *motor, uint16_t angle, int16_t speed, int16_t torque, int8_t temp)
{
  motor->lastAngle = motor->angle;
  motor->angle = angle;
  motor->speed = speed;
  motor->torque = torque;
  motor->temp = temp;

  motor->totalAngle += speed / 5.0;
  motor->totalAngle = motor->totalAngle % (129144);
  if (motor->totalAngle < 0)
  {
    motor->totalAngle += 129144;
  }
}
void M4005_Update(SingleMotor *motor, uint16_t angle, int16_t speed, int16_t torque, int8_t temp)
{
  motor->lastAngle = motor->angle;
  motor->angle = angle;
  motor->speed = speed;
  motor->torque = torque;
  motor->temp = temp;
}

void CAN_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcans)
{

  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcans, CAN_RX_FIFO0, &rx_header, rx_data);

  CAN_Rx0Callback(&rx_header, rx_data);
}

void CAN_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rxdata)
{
  if (rx_header->StdId == 0x201)
  {
    Motor_Update(&motors[0], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
                 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
  }
  else if (rx_header->StdId == 0x203)
  {
    Motor_Update(&motors[1], (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]),
                 (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
  }
  else if (rx_header->StdId == 0x144)
  {
    M4005_Update(&motorM4005, (rxdata[6] | rxdata[7] << 8), (rxdata[4] | rxdata[5] << 8), (rxdata[2] | rxdata[3] << 8), rxdata[1]);
  }
}