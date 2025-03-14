#ifndef __ExPowerControl__
#define __ExPowerControl__
#include "main.h"
#include "user_can.h"
#ifdef __cplusplus
extern "C"
{
#endif
    float getMeasurePower(void);
    void ChassisPowerControl(uint16_t realBuffer,
                             float setCurrent[4],
                             SingleMotor motor[4],
                             uint8_t limitPowerMax);
    void setMeasurePower(float power);
#ifdef __cplusplus
}
#endif
#endif // DEBUG