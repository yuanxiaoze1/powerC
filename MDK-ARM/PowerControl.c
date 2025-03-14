#include "PowerConrtol.h"
#include "main.h"
#include "pid.h"
#include "user_can.h"
#include "math.h"
#include "struct_typedef.h"

#define HEAL_RATE 1
#define Buffer_CON 10.0f
#define SacaleTorque

/*************************新版功率控制**************************
使用转矩电流代替缓冲能量控制
*********************************************************/
PowerController powerController;
Buffer_Enegy bufferSimulation;

void Power_detect()
{
    float error = powerController.measurePower - powerController.limitPower;
    if (error > 0)
    {
        powerController.limitPower -= error * 0.001f;
    }
    else
    {
        powerController.limitPower += 0.01f;
    }
}
extern float paramVector[3][1];
// (20/16384)*(0.3)*(187/3591)/9.55

void Chassis_PowerCtrl(SingleMotor motors[MOTOR_NUM], float setCurrent[MOTOR_NUM], int16_t setV[MOTOR_NUM]) // 新版功率控制 只需要在发送电流前加入即可
{
    powerController.a = fmaxf(paramVector[1][0], 1e-8);
    powerController.k2 = fmaxf(paramVector[0][0], 1e-8); // k2
    powerController.constant = paramVector[2][0];

    PID_Init(&powerController.bufferPid, Buffer_CON * powerController.limitPower / powerController.setBuffer, 0, 0, 0, 20);
    PID_SingleCalc(&powerController.bufferPid, powerController.setBuffer, bufferSimulation.realBuffer);
    float inputPower = fmaxf(powerController.limitPower - powerController.bufferPid.output, 5.0f); // Input power floating at maximum power
    float initial_give_power[MOTOR_NUM];
    float initial_total_power = 0;
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        initial_give_power[i] = powerController.toque_coefficient * motors[i].speed * motors[i].torque +
                                powerController.k2 * motors[i].speed * motors[i].speed +
                                powerController.a * setCurrent[i] * setCurrent[i] +
                                powerController.constant;
        initial_total_power += initial_give_power[i];
    }
    if (initial_total_power > inputPower)
    {
        if (powerController.scalePower_MODEL == SCALE_CURRENT)
        {
            float scaleSetcurrent = 0;
            float a0 = 0;
            float b0 = 0;
            float c0 = 0;
            for (uint8_t i = 0; i < 4; i++)
            {
                if (initial_give_power[i] < 0)
                    continue;
                a0 += powerController.a * setCurrent[i] * setCurrent[i];
                b0 += powerController.toque_coefficient * setCurrent[i] * motors[i].speed;
                c0 += powerController.k2 * motors[i].speed * motors[i].speed + powerController.constant;
            }
            c0 -= inputPower;
            float delta = b0 * b0 - 4 * a0 * c0;

            if (delta < 0)
                scaleSetcurrent = 0;
            else
            {
                float temp1 = (-b0 + sqrt(delta)) / (2 * a0);
                float temp2 = (-b0 - sqrt(delta)) / (2 * a0);
                if (temp1 > 0 && temp1 < 1)
                    scaleSetcurrent = temp1;

                if (temp2 > 0 && temp2 < 1)
                {
                    scaleSetcurrent = fmaxf(scaleSetcurrent, temp2);
                }
            }
            if (scaleSetcurrent > 0.0f && scaleSetcurrent < 1.0f)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    if (initial_give_power[i] < 0)
                        continue;
                    setCurrent[i] *= scaleSetcurrent;
                    LIMIT(setCurrent[i], -16000, +16000);
                }
                return;
            }
        }
        if (powerController.scalePower_MODEL == SCALE_POWER)
        {

            float power_scale = inputPower / initial_total_power;
            float scaled_give_power[MOTOR_NUM];
            int32_t alltargetspeed = 0;

            for (uint8_t i = 0; i < MOTOR_NUM; i++)
                alltargetspeed += ABS(setV[i]);

            for (uint8_t i = 0; i < MOTOR_NUM; i++)
            {
                scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
                scaled_give_power[i] *= 1.0f * ABS(setV[i]) * MOTOR_NUM / alltargetspeed;
                if (scaled_give_power[i] <= 0)
                {
                    setCurrent[i] = 0;
                    continue;
                }
                float a = powerController.a;

                float b = powerController.toque_coefficient * motors[i].speed;
                float c = powerController.k2 * motors[i].speed * motors[i].speed - scaled_give_power[i] + powerController.constant;
                float mo = b * b - 4 * a * c;

                if (mo < 0)
                {
                    setCurrent[i] = 0;
                    return;
                }

                if (setCurrent[i] > 0) // Selection of the calculation formula according to the direction of the original moment
                {
                    float temp = (-b + sqrt(mo)) / (2 * a);
                    if (temp > 16000)
                    {
                        setCurrent[i] = 16000;
                    }
                    else
                        setCurrent[i] = temp;
                }

                else
                {
                    float temp = (-b - sqrt(mo)) / (2 * a);
                    if (temp < -16000)
                    {
                        setCurrent[i] = -16000;
                    }
                    else
                        setCurrent[i] = temp;
                }
            }
        }
    }
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        LIMIT(setCurrent[i], -16000, 16000);
    }
}

void PowerControl_Init(void)
{
    powerController.a = 1.74e-07;
    powerController.k2 = 1.54e-07;
    powerController.constant = 1.0f;
    powerController.toque_coefficient = 2.4324e-06;
    powerController.scalePower_MODEL = SCALE_POWER;
    powerController.setBuffer = 5.0f;
}