#ifndef __powerconrtol_h__
#define __powerconrtol_h__

#define MOTOR_NUM 2
#define MAXBUFFER 10
#include "PID.h"
#include "user_can.h"
#define SCALE_POWER 0
#define SCALE_CURRENT 1

typedef struct
{
    int16_t measurePower;
    int16_t predictPower;
    uint8_t limitPower;
    float setBuffer;
    PID bufferPid;
    uint8_t scalePower_MODEL;
    // 电机参数
    float a;
    float k2;
    float constant;
    float toque_coefficient;

} PowerController;
typedef struct
{
    float realBuffer;

} Buffer_Enegy;

void PowerControl_Init(void);
void Chassis_PowerCtrl(SingleMotor motors[MOTOR_NUM], float setCurrent[MOTOR_NUM], int16_t setV[MOTOR_NUM]);
void Power_detect(void);

#endif
