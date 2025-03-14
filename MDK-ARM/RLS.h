#ifndef __RLS__
#define __RLS__
#include "arm_math.h"
#ifdef __cplusplus
extern "C"{
#endif // __cplusplus
    void PowerControl_AutoUpdateParamInit();
    float PowerControl_AutoUpdateParam(float sumPowRPM, float sumPowTorque, float x3, float detaP);
#ifdef __cplusplus
}
#endif // __cplusplus


#endif