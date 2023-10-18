#include "sensors.h"  //include sensor header

UltraSonicDistanceSensor ultraSens(5,6);

void UltraDistSensor::get_ultra_distance() {
    double cdist = ultraSens.measureDistanceCm();
    ob_dist = cdist;
    if (cdist > 200 || cdist < 0) {
        detected = false;
    } else {
        detected = true;
    }
}

void UltraDistSensor::print_ultra_data() {
    Serial.print("Object Detected?: ");
    if (detected) {
        Serial.print("Yes");
    } else {
        Serial.print("No");
    }
    Serial.print(" | Distance: ");
    Serial.println(ob_dist);
}