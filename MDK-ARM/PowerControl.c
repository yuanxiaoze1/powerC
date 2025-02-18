#include "PowerConrtol.h"
#include "main.h"
#include "pid.h"
#include "user_can.h"
#include "math.h"
#include "struct_typedef.h"

#define HEAL_RATE 1
#define Buffer_CON 10.0f
/*************************新版功率控制**************************
使用转矩电流代替缓冲能量控制
*********************************************************/
fp32 templ;
fp32 b;
fp32 c;
fp32 a = 1.74e-07;
fp32 scaled_give_power[MOTOR_NUM];
fp32 power_scale;
float mo;

float load[4];
float input_power = 0; // input power from battery (referee system)
extern PID BufferPid;
extern PID pid_V[MOTOR_NUM];
extern SingleMotor motors[MOTOR_NUM];
uint16_t MotorPowerMax = 10;
uint16_t PowerLimit = 10;
extern float power;
extern int16_t setCurrent[MOTOR_NUM];
uint8_t powerCnt;
float initial_total_power = 0;

fp32 expectBuffer = 5.0f;
fp32 RealBuffer = 3.0f;

extern uint16_t detaTime;

extern int16_t setV[MOTOR_NUM];
void Power_detect()
{
    if (power > MotorPowerMax)
    {
        RealBuffer -= (power - MotorPowerMax) * 0.001 * detaTime;
        powerCnt = 0;
    }
    else if (powerCnt++ > MAXBUFFER)
    {
        powerCnt = 0;
        RealBuffer += 0.01f * HEAL_RATE * detaTime;
        if (RealBuffer > MAXBUFFER)
        {
            RealBuffer = MAXBUFFER;
        }
    }
}
extern float paramVector[3][1];
fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55

void Chassis_PowerCtrl() // 新版功率控制 只需要在发送电流前加入即可
{

    initial_total_power = 0;
    float initial_give_power[MOTOR_NUM]; // initial power from PID calculation
    float befor_inital[MOTOR_NUM] = {0};
    uint16_t max_power_limit = 60;

    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    a = paramVector[1][1];
    // k1
    fp32 k2 = paramVector[0][1];       // k2
    fp32 constant = paramVector[2][1]; // constant
    max_power_limit = MotorPowerMax;
    chassis_power_buffer = RealBuffer;
    PID_Init(&BufferPid, Buffer_CON * max_power_limit / expectBuffer, 0, 0, 0, 0);
    PID_SingleCalc(&BufferPid, expectBuffer, chassis_power_buffer);
    input_power = max_power_limit - BufferPid.output; // Input power floating at maximum power
    if (input_power <= 5)
    {
        input_power = 5;
    }
    for (uint8_t i = 0; i < MOTOR_NUM; i++) // first get all the initial motor power and total motor power
    {
        befor_inital[i] = setCurrent[i];
        initial_give_power[i] = setCurrent[i] * toque_coefficient * motors[i].speed +
                                k2 * motors[i].speed * motors[i].speed +
                                a * setCurrent[i] * setCurrent[i] + constant;

        if (initial_give_power[i] < 0) // negative power not included (transitory)
            continue;

        initial_total_power += initial_give_power[i];
    }

    if (initial_total_power > MotorPowerMax) // determine if larger than max power
    {
        power_scale = input_power / initial_total_power;
        int32_t alltargetspeed = 0;

        for (uint8_t i = 0; i < MOTOR_NUM; i++)
            alltargetspeed += ABS(setV[i]);

        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
            scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
            scaled_give_power[i] *= ABS(setV[i]) * MOTOR_NUM / alltargetspeed;
            load[i] = scaled_give_power[i] / ABS(motors[i].speed);

            if (scaled_give_power[i] <= 0)
            {
                setCurrent[i] = 0;
                continue;
            }

            b = toque_coefficient * motors[i].speed;
            c = k2 * motors[i].speed * motors[i].speed - scaled_give_power[i] + constant;
            mo = b * b - 4 * a * c;

            if (mo <= 0)
            {
                setCurrent[i] = 0;
                return;
            }

            if (setCurrent[i] > 0) // Selection of the calculation formula according to the direction of the original moment
            {
                templ = (-b + sqrt(mo)) / (2 * a);
                if (templ > 16000)
                {
                    setCurrent[i] = 16000;
                }
                else
                    setCurrent[i] = templ;
            }

            else
            {
                templ = (-b - sqrt(mo)) / (2 * a);
                if (templ < -16000)
                {
                    setCurrent[i] = -16000;
                }
                else
                    setCurrent[i] = templ;
            }
        }
    }

    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        LIMIT(pid_V[i].output, -16000, 16000);
        // setV[i] = pid_V[i].output;
    }
    if (RealBuffer <= 0)
    {
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
            setCurrent[i] = 0;
        }
    }
}