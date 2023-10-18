#ifndef tstate_h
#define tstate_h

#include "math.h"

class TeensyState {
    private:
    const float gear_ratio;
    const float wheel_radius;

    public:
    double time;

    /* IMU variables */
    float x_acc; 
    float y_acc;
    float z_acc;
    float yaw_rate;
    float pitch_rate;
    float roll_rate;
    double yaw;
    double pitch;
    double roll;
    bool new_agdat;
    bool new_dmpdat;

    /* Ultrasonic Variables*/
    double obstc_dist;
    bool obstc_detected;

    /*Tachometer Variables*/
    double rpm;
    bool new_tachdat;

    /*Calulated Variables*/
    float velocity_mag;
    float vel_gx; //The velocity in the global x direction
    float vel_gy; //The velocity in the global y direction
    float acc_mag;

    TeensyState() {
        time = micros();

        x_acc = 0;
        y_acc = 0;
        z_acc = 0;
        yaw_rate = 0;
        pitch_rate = 0;
        roll_rate = 0;
        yaw = 0;
        pitch = 0;
        roll = 0;
        new_agdat = false;
        new_tachdat = false;

        obstc_dist = 0;
        obstc_detected = false;

        rpm = 0;
        new_tachdat = false;

        velocity_mag = 0;
        vel_gx = 0;
        vel_gy = 0;
        acc_mag = 0;
    }

    /**
     * Updates the Teensy state based on the latest data from the IMU, Tachometer,
     * and Ultrasonic Distance Sensor.
     * @param IMU_data The latest IMU data
     * @param tach_data The latest tachometer data
     * @param ultra_data The latest ultrasonic distance sensor data
     */
    void updateTState(IMU IMU_data, Tachometer tach_data, UltraDistSensor ultra_data);

    /**
     * Calculates the velocity magnitude and global components from the rpm and yaw.
     */
    void calcVelocity();

    /** 
     * Calculates the acceleration magnitude from the acceleration components
     */
    void calcAccelerationMagnitude();

    /**
     * Prints the formatted teensy state data to the serial monitor
     */
    void serialPrintTState();
};

#endif