#ifndef sensors_h
#define sensors_h

#include "Arduino.h"
#include "ICM_20948.h" //Inlcude accelerometer library
#include "HCSR04.h" //include the ultrasonic sensor library

#define IMUI2C 1      // The value of the last bit of the I2C address. On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0

/*IMU variable declarations*/
extern ICM_20948_I2C imu;  //Declare the the imu object but do not define it. It is defined in imu.cpp
const float mg2mps = 0.00981; //The constant ratio between milli-gs and meters per second

/*IMU functions*/
void init_imu();

/*IMU data class*/
class IMU {
    private:
    ICM_20948_I2C imu;

    public:
    float x_acc;
    float y_acc;
    float z_acc;
    float yaw_rate;
    float pitch_rate;
    float roll_rate;
    double yaw;
    double pitch;
    double roll;

    IMU() {
        x_acc = 0;
        y_acc = 0;
        z_acc = 0;
        yaw_rate = 0;
        pitch_rate = 0;
        roll_rate = 0;
        yaw = 0;
        pitch = 0;
        roll = 0;
    }

    void init_imu();

    void get_imu_data();

    void serial_print_IMUdata();

};

class Tachometer {
    public:
    const int hall_pin = 14;
    int rev_thrsh = 25;
    float rpm;
    float velocity;
    bool new_dat;

    Tachometer() {
        rpm = 0;
        velocity = 0;
    }

    void init_tachometer();

    void simple_read();

    void update_tachometer();

    void print_RPM();
    
};

class UltraDistSensor {
    private:
    
    
    public:
    double ob_dist;  //The distance detected by the ultrasonic sensor
    bool detected; 

    UltraDistSensor() {
        ob_dist = 400;
        detected = false;
    }

    void get_ultra_distance();

    void print_ultra_data();
};

#endif



