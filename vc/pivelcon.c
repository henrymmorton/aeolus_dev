#include "picontrol.h"

void PIVelControl_Init(PIVelController *pid) {

    /*Reset controller memory*/
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->conOut = 0.0f;

}

float PIVelControl_Update(PIVelController *pid, float target, float measuredval) {

    /*Error*/
    float error = target - measuredval;

    /*Proportional Term*/
    float proportional = pid->Kp *error;

    /*Integral Term*/
    float integrator = (pid->Ki * pid->T * 0.5f) * (error + pid->prevError) + integrator;

    /* Dynamic integrator clamping limits*/
    float dClampMin, dClampMax;

    if (pid->conMax > proportional) {
        dClampMax = pid->conMax - proportional;
    } else {
        dClampMax = 0.0f;
    }

    if (pid->conMin < proportional) {
        dClampMin = pid->conMin - proportional;
    } else {
        dClampMin = 0.0f;
    }

    /*Dynamic integrator clamping*/
    if (pid->integrator > dClampMax) {
        pid->integrator = dClampMax;
    } else if (pid->integrator < dClampMin) {
        pid->integrator = dClampMin;
    }

    /*Compute output control signal*/
    pid->conOut = proportional + pid->integrator;

    /*Apply plant limits*/
    if(pid->conOut > pid->conMax) {
        pid->conOut = pid->conMax;
    } else if (pid->conOut < pid->conMin) {
        pid->conOut = pid->conMin;
    }

    /*Store current error*/
    pid->prevError = error;

    return pid->conOut;
}

