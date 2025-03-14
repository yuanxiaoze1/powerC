#include "ExPowerControl.h"
#include "main.h"
#include "PID.h"
#include "user_can.h"
#include "math.h"
#include "RLS.h"
#include "arm_math.h"

typedef enum
{
    ScalePower = 0,
    ScaleCurrent,
} ScaleModel;
class ExPowerControl
{
private:
    float measurePower;
    int16_t predictPower;
    uint8_t limitPower;
    float setBuffer;
    PID bufferPid;
    ScaleModel scaleModel;
    // 电机参数(3508)
    float a;
    float k2;
    float constant;
    float toque_coefficient;

    float a_M4005;
    float k2_M4005;
    float constant_M4005;
    float toque_coefficient_M4005;

public:
    ExPowerControl(/* args */);
    ~ExPowerControl();
    float getMeasurePower();
    void setMeasurePower(float power);
    void ParameterUpdate();
    void chassisPowerControl(uint16_t realBuffer, float setCurrent[], SingleMotor *motor, uint8_t limitPowerMax);
};

/////////////////////////////////////////////////////

ExPowerControl exPowerControl;
extern float paramVector[3][1];
extern uint8_t RLS_EABLE;

////////////////////////////////////////////////////

ExPowerControl::ExPowerControl(/* args */)
    : measurePower(0),
      predictPower(0),
      limitPower(0),
      setBuffer(50),
      scaleModel(ScaleCurrent),
      a(1.74e-07),
      k2(1.54e-07),
      constant(1.0),
      toque_coefficient(2.4324e-06), // 3508  2.4324e-06
      a_M4005(2.5e-05),
      k2_M4005(2.61e-08),
      constant_M4005(1.0),
      toque_coefficient_M4005(1.0124e-05) // 4005  2.4324e-06
{
    PID_Init(&bufferPid, 8, 0, 0, 0, 100);
    PowerControl_AutoUpdateParamInit();
}

// 析构函数，用于释放ExPowerControl对象占用的资源
ExPowerControl::~ExPowerControl()
{
    // 释放资源
}
float ExPowerControl::getMeasurePower()
{
    return measurePower;
}
void ExPowerControl::setMeasurePower(float power)
{
    this->measurePower = power;
}
void ExPowerControl::ParameterUpdate()
{
    if (RLS_EABLE == 0)
        ExPowerControl(); // 异常重启

    a = fmaxf(paramVector[1][0], 1e-8);
    k2 = fmaxf(paramVector[0][0], 1e-8); // k2
    constant = paramVector[2][0];
}
void ExPowerControl::chassisPowerControl(uint16_t realBuffer,
                                         float setCurrent[4],
                                         SingleMotor motor[4],
                                         uint8_t limitPowerMax)
{
    ParameterUpdate();
    PID_SingleCalc(&bufferPid, setBuffer, realBuffer);
    float inputPower = fmaxf(limitPowerMax - bufferPid.output, 5); // 输入功率
    this->limitPower = inputPower;
    float initPower[4] = {0};
    float totoalPower = 0;

    for (int i = 0; i < 4; i++)
    {
        initPower[i] = this->toque_coefficient * setCurrent[i] * motor[i].speed +
                       this->a * setCurrent[i] * setCurrent[i] +
                       this->k2 * motor[i].speed * motor[i].speed +
                       this->constant;
        totoalPower += initPower[i];
    }

    if (totoalPower >= inputPower)
    {
        if (scaleModel == ScaleCurrent)
        {
            float scaleSetcurrent = 0;
            float a0 = 0;
            float b0 = 0;
            float c0 = 0;
            for (uint8_t i = 0; i < 1; i++)
            {
                if (initPower[i] < 0)
                    continue;
                a0 += a * setCurrent[i] * setCurrent[i];
                b0 += toque_coefficient * setCurrent[i] * motor[i].speed;
                c0 += k2 * motor[i].speed * motor[i].speed + constant;
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
            if (scaleSetcurrent >= 0.0f && scaleSetcurrent < 1.0f)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    if (initPower[i] < 0)
                    {
                        LIMIT(setCurrent[i], -16000, +16000);
                        continue;
                    }
                    setCurrent[i] *= scaleSetcurrent;
                    LIMIT(setCurrent[i], -16000, +16000);
                }
                return;
            }
        }
    }
}
extern "C" float getMeasurePower(void)
{
    return exPowerControl.getMeasurePower();
}
extern "C" void ChassisPowerControl(uint16_t realBuffer,
                                    float setCurrent[4],
                                    SingleMotor motor[4],
                                    uint8_t limitPowerMax)
{
    exPowerControl.chassisPowerControl(realBuffer, setCurrent, motor, limitPowerMax);
}
extern "C" void setMeasurePower(float power)
{
    exPowerControl.setMeasurePower(power);
}