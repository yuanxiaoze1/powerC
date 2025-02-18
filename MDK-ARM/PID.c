/****************PID??****************/

#include "PID.h"

// ???pid??
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxIntegral = maxI;
	pid->maxOutput = maxOut;
	pid->deadzone = 0;
	pid->output = 0;
}

// ???????pid??
void DEPID_Init(DEPID *pid, float p, float i, float d, float maxI, float maxOut, float gama)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxIntegral = maxI;
	pid->maxOutput = maxOut;
	pid->gama = gama;
}
// ??????pid??
void PIDRegulation(DEPID *vPID, float reference, float feedback, float differentiation)
{
	// ????
	vPID->lasterror = vPID->error;

	vPID->error = reference - feedback;
	// ????
	differentiation = vPID->gama * differentiation + (1 - vPID->gama) * vPID->lastPv;
	// ????
	vPID->output = differentiation * vPID->kd;
	// ????
	vPID->output += vPID->error * vPID->kp;
	// ????
	vPID->integral += vPID->error * vPID->ki;

	LIMIT(vPID->integral, -vPID->maxIntegral, vPID->maxIntegral); // ????

	vPID->output += vPID->integral;
	// ????
	LIMIT(vPID->output, -vPID->maxOutput, vPID->maxOutput);
	// ??????
	vPID->lastPv = differentiation;
}
void IncrementalPID_CAlC(IPID *pid, float reference, float feedback)
{
	// ????
	pid->error = reference - feedback;

	// ????
	float increment = pid->kp * (pid->error - pid->lastError) + pid->ki * pid->error + pid->kd * (pid->error - 2 * pid->lastError + pid->lastError2);
	// ????
	pid->output += increment;

	// ????
	if (pid->output > pid->maxOutput)
	{
		pid->output = pid->maxOutput;
	}
	else if (pid->output < -pid->maxOutput)
	{
		pid->output = -pid->maxOutput;
	}

	// ??????
	pid->lastError2 = pid->lastError;
	pid->lastError = pid->error;
}
// ??pid??
void PID_SingleCalc(PID *pid, float reference, float feedback)
{
	// ????
	pid->lastError = pid->error;
	if (ABS(reference - feedback) < pid->deadzone) // ????????error???0
		pid->error = 0;
	else
		pid->error = reference - feedback;
	// ????
	pid->output = (pid->error - pid->lastError) * pid->kd;
	// ????
	pid->output += pid->error * pid->kp;
	// ????
	pid->integral += pid->error * pid->ki;

	LIMIT(pid->integral, -pid->maxIntegral, pid->maxIntegral); // ????

	pid->output += pid->integral;
	// ????
	LIMIT(pid->output, -pid->maxOutput, pid->maxOutput);
}

// ??pid??
void PID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb)
{
	PID_SingleCalc(&(pid->outer), angleRef, angleFdb);			// ????(???)
	PID_SingleCalc(&(pid->inner), pid->outer.output, speedFdb); // ????(???)
	pid->output = pid->inner.output;
}
void PID_CascadeCalc_totalAngle(CascadePID *pid, float angleRef, float angleFdb, float speedFdb)
{
	PID_SingleCalc_totalangle(&(pid->outer), angleRef - 129144 / 2, angleFdb - 129144 / 2); // ????(???)
	PID_SingleCalc(&(pid->inner), pid->outer.output, speedFdb);								// ????(???)
	pid->output = pid->inner.output;
}
void PID_SingleCalc_totalangle(PID *pid, float reference, float feedback)
{
	// ????
	pid->lastError = pid->error;
	if (ABS(reference - feedback) < pid->deadzone) // ????????error???0
		pid->error = 0;
	else
		pid->error = reference - feedback;
	if (pid->error > 129144 / 2)
		pid->error -= 129144;
	else if (pid->error < -129144 / 2)
		pid->error += 129144;
	// ????
	pid->output = (pid->error - pid->lastError) * pid->kd;
	// ????
	pid->output += pid->error * pid->kp;
	// ????
	pid->integral += pid->error * pid->ki;

	LIMIT(pid->integral, -pid->maxIntegral, pid->maxIntegral); // ????

	pid->output += pid->integral;
	// ????
	LIMIT(pid->output, -pid->maxOutput, pid->maxOutput);
}
// ??????pid??		????? TODO???????
void DEPID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb)
{
	PIDRegulation(&(pid->deOuter), angleRef, angleFdb, -speedFdb); // ????????(???)
	PID_SingleCalc(&(pid->inner), pid->deOuter.output, speedFdb);  // ????(???)
	pid->output = pid->inner.output;
}

// ????pid?????
void PID_Clear(PID *pid)
{
	pid->error = 0;
	pid->lastError = 0;
	pid->integral = 0;
	pid->output = 0;
}

void DEPID_Clear(DEPID *pid)
{
	pid->error = 0;
	pid->lasterror = 0;
	pid->integral = 0;
	pid->output = 0;
	pid->lastPv = 0;
}

// ????pid????
void PID_SetMaxOutput(PID *pid, float maxOut)
{
	pid->maxOutput = maxOut;
}

// ??PID??
void PID_SetDeadzone(PID *pid, float deadzone)
{
	pid->deadzone = deadzone;
}
