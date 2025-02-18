#ifndef _USER_PID_H_
#define _USER_PID_H_

#include "stdint.h"

#define LIMIT(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#ifndef ABS
#define ABS(x) ((x) >= 0 ? (x) : -(x))
#endif

typedef struct _PID
{
	float kp, ki, kd;
	float error, lastError;		 // ???????
	float integral, maxIntegral; // ???????
	float output, maxOutput;	 // ???????
	float deadzone;				 // ??
} PID;
typedef struct IPID
{
	float kp, ki, kd;
	float error, lastError, lastError2; // ???????
	float integral, maxIntegral;		// ???????
	float output, maxOutput;			// ???????
	float deadzone;						// ??
} IPID;

/*?????????*/
typedef struct _DEPID
{
	float kp;		   // ????
	float ki;		   // ????
	float kd;		   // ????
	float lasterror;   // ?????
	float error;	   // ??error
	float output;	   // ???
	float integral;	   // ???
	float derivative;  // ???
	float lastPv;	   // ???????
	float gama;		   // ????????
	float maxOutput;   // ????
	float maxIntegral; // ????
} DEPID;

typedef struct _CascadePID
{
	PID inner;	   // ??
	PID outer;	   // ??
	DEPID deOuter; // ??????
	float output;  // ????,??inner.output
} CascadePID;

void PID_Init(PID *pid, float p, float i, float d, float maxSum, float maxOut);
void PID_SingleCalc(PID *pid, float reference, float feedback);
void IncrementalPID_CAlC(IPID *pid, float reference, float feedback);
void PID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb);
void PID_Clear(PID *pid);
void DEPID_Clear(DEPID *pid);
void PID_SetMaxOutput(PID *pid, float maxOut);
void PID_SetDeadzone(PID *pid, float deadzone);
void DEPID_Init(DEPID *pid, float p, float i, float d, float maxI, float maxOut, float gama);
void PIDRegulation(DEPID *vPID, float reference, float feedback, float differentiation);
void DEPID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb);
void PID_CascadeCalc_totalAngle(CascadePID *pid, float angleRef, float angleFdb, float speedFdb);

void PID_SingleCalc_totalangle(PID *pid, float reference, float feedback);
#endif
