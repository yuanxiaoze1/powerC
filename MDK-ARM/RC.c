#include "RC.h"
uint8_t rc_data[36];
RC_TypeDef rcInfo = {0};
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart->Instance == USART3)
    {
        rcInfo.ch1 = (rc_data[0] | rc_data[1] << 8) & 0x07FF;
        rcInfo.ch1 -= 1024;
        rcInfo.ch2 = (rc_data[1] >> 3 | rc_data[2] << 5) & 0x07FF;
        rcInfo.ch2 -= 1024;
        rcInfo.ch3 = (rc_data[2] >> 6 | rc_data[3] << 2 | rc_data[4] << 10) & 0x07FF;
        rcInfo.ch3 -= 1024;
        rcInfo.ch4 = (rc_data[4] >> 1 | rc_data[5] << 7) & 0x07FF;
        rcInfo.ch4 -= 1024;

        /* prevent remote control zero deviation */
        if (rcInfo.ch1 <= 5 && rcInfo.ch1 >= -5)
            rcInfo.ch1 = 0;
        if (rcInfo.ch2 <= 5 && rcInfo.ch2 >= -5)
            rcInfo.ch2 = 0;
        if (rcInfo.ch3 <= 5 && rcInfo.ch3 >= -5)
            rcInfo.ch3 = 0;
        if (rcInfo.ch4 <= 5 && rcInfo.ch4 >= -5)
            rcInfo.ch4 = 0;

        rcInfo.left = ((rc_data[5] >> 4) & 0x000C) >> 2; // sw1   中间是3，上边是1，下边是2
        rcInfo.right = (rc_data[5] >> 4) & 0x0003;       // sw2
        /*
         if ((abs(rc->ch1) > 660) || \
             (abs(rc->ch2) > 660) || \
             (abs(rc->ch3) > 660) || \
             (abs(rc->ch4) > 660))
         {
           memset(rc, 0, sizeof(struct rc));
           return ;
         }
       */
        rcInfo.mouse.x = rc_data[6] | (rc_data[7] << 8); // x axis
        rcInfo.mouse.y = rc_data[8] | (rc_data[9] << 8);
        rcInfo.mouse.z = rc_data[10] | (rc_data[11] << 8);

        rcInfo.mouse.l = rc_data[12];
        rcInfo.mouse.r = rc_data[13];

        rcInfo.kb.key_code = rc_data[14] | rc_data[15] << 8; // key borad code
        rcInfo.wheel = (rc_data[16] | rc_data[17] << 8) - 1024;
    }
}