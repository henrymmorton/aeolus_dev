#include "tstate.h"

void TeensyState::updateTState(IMU IMU_data, Tachometer tach_data, UltraDistSensor ultra_data) {

    time = micros();

    if (IMU_data.new_agdat) {
        x_acc = IMU_data.x_acc;
        y_acc = IMU_data.y_acc;
        z_acc = IMU_data.z_acc;
        yaw_rate = IMU_data.yaw_rate;
        pitch_rate = IMU_data.pitch_rate;
        roll_rate = IMU_data.roll_rate;
        new_agdat = true;
    } else {
        new_agdat = false;
    }
    
    if (IMU_data.new_dmpdat) {
        yaw = IMU_data.yaw;
        pitch = IMU_data.pitch;
        roll = IMU_data.roll;
        new_dmpdat = true;
    } else {
        new_dmpdat = false;
    }

    if (tach_data.new_tachdat) {
        rpm = tach_dat.rpm;
        new_tachdat = true;
    } else {
        new_tachdat = false;
    }

    obstc_dist = ultra_data.obstc_dist;
    obstc_detected = ultra_data.detected;

    calcVelocity();
    calcAcceleartionM
}

void TeensyState::calcVelocity() {
    velocity_mag = (((rpm / gear_ratio) * (2 * pi * wheel_radius)) / 60)
    vel_gx = sin(yaw) * velocity_mag;
    vel_gy = cos(yaw) * velocity_mag;
}

void TeensyState::calcAccelerationMagnitude() {
    acc_mag = sqrt(pow(x_acc, 2) + pow(y_acc, 2) + pow(z_acc, 2))
}

void TeensyState::serialPrintTState() {
    Serial.print("Time Since Initaition: ");
    Serial.println(time/pow(10, 6));

    if (new_agdat) {
        Serial.print(F("X Accel: "));
        Serial.print(x_acc, 3);
        Serial.print(F(" | Y Accel: "));
        Serial.print(y_acc, 3);
        Serial.print(F(" | Z Accel: "));
        Serial.print(z_acc, 3);
        Serial.print(" | Accel Magnitude: ");
        Serial.println(acc_mag, 3);

        Serial.print(F("Yaw Rate: "));
        Serial.print(yaw_rate, 3);
        Serial.print(F(" | Pitch Rate: "));
        Serial.print(pitch_rate,3);
        Serial.print(F(" | Roll Rate: "));
        Serial.println(roll_rate, 3);
    } else {
        Serial.println("No new acceleromter and gyro data");
    }

    if (new_dmpdat) {
        Serial.print(F("Yaw: "));
        Serial.print(yaw, 3);
        Serial.print(F(" | Pitch: "));
        Serial.print(pitch, 3);
        Serial.print(F(" | Roll: "));
        Serial.println(roll, 3);
    } else {
        Serial.println("No new DMP data");
    }
    
    Serial.print("Velocity Magnitude: ");
    Serial.print(velocity_mag, 3);
    Serial.print(" | Velocity Global X Component: ");
    Serial.print(vel_gx, 3);
    Serial.print(" | Velocity Global Y Component: ");
    Serial.println(vel_gy, 3);

    Serial.print("Obstacle Detected?: ");
    if (obstc_detected) {
        Serial.print("Yes ");
        Serial.print("| Obstacle Distance: ");
        Serial.println(obstc_dist, 3);
    } else {
        Serial.println("No")
    }
}