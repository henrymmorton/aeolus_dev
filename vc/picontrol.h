#ifndef PI_VEL_CONTROL
#define PI_VEL_CONTROL

typedef struct  {
    /*Controller gains*/
    float Kp = 1;
    float Ki = 1;

    /*Motor control signal limits*/
    float conMin = 180;
    float conMax = 0;

    /*Controller memory*/
    float integrator;
    float prevError;

    /* TODO:Add timestep into main arduino function
    //float T; 

    /*Controller output*/
    float conOut;

} PIVelController;

void PIVelControl_Init(PIVelController *pid);
float PIVelControl_Update(PIVelController *pid, float target, float measuredval);

#endif