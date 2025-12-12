#include "Robot/distance_calibration.h"
#include "vex.h"
#include "Robot/utility_functions.h"

#include <iostream>

// Forward sensor's distance from robot's origin
double Forward_Sensor_Offset_X = 0.0;
double Forward_Sensor_Offset_Y = 5.0;

// Left sensor's distance from robot's origin
double Left_Sensor_Offset_X = 4.0;
double Left_Sensor_Offset_Y = 0.0;

// Right sensor's distance from robot's origin
double Right_Sensor_Offset_X = -4.0;
double Right_Sensor_Offset_Y = 0.0;

// Accounts for any error that the sensor has compared to real life
double Forward_Sensor_Calibration_Bias = 5.0/8.0;
double Left_Sensor_Calibration_Bias = 0.0;
double Right_Sensor_Calibration_Bias = 0.0;

// Offset from robot's forward side
double Forward_Sensor_Angle_Offset = 0.0;
double Left_Sensor_Angle_Offset = -90.0;
double Right_Sensor_Angle_Offset = 90.0;

std::vector<double> FieldPositionFromDistance() {
    // Variable to save estimated field position
    std::vector<double> estimatedFieldPosition;

    double estimatedXDistance = 0.0;
    double estimatedYDistance = 0.0;

    // Get current sensor data
    double distance_forwards = forward_distance_sensor.objectDistance(distanceUnits::in);
    double distance_sideways = right_distance_sensor.objectDistance(distanceUnits::in);

    estimatedFieldPosition.push_back(72 - estimatedXDistance);
    estimatedFieldPosition.push_back(72 - estimatedYDistance);

    return estimatedFieldPosition;
}

std::vector<double> EstimatePositionWithDistance(Wall closestWall) {
    return EstimatePositionWithDistance(closestWall, Right);
}

std::vector<double> EstimatePositionWithDistance(Wall closestWall, SensorSide sensorSide) {
    std::vector<double> estimatedFieldPosition;

    double estimatedXPos = 0.0;
    double estimatedYPos = 0.0;

    // Get current sensor data
    double distance_forwards = 0;
    double distance_right = 0;

    int LeftRightMultiplier = (sensorSide == Left) ? -1 : 1;

    // Get average to reduce noise

    for (int i = 0; i < 10; i++) {
        distance_forwards += forward_distance_sensor.objectDistance(distanceUnits::in) - Forward_Sensor_Calibration_Bias;
        distance_right += right_distance_sensor.objectDistance(distanceUnits::in) - Right_Sensor_Calibration_Bias;
    }

    distance_forwards /= 10.0;
    distance_right /= 10.0;

    // Calculate angle offsets

    double forwardHeadingInRadians = DegToRad(inertial_sensor.heading());
    double sidewaysHeadingInRadians = DegToRad(inertial_sensor.heading() + 90);

    switch (closestWall) {
        case X_Pos:
            estimatedXPos = 70.5 - (distance_forwards + Forward_Sensor_Offset_Y) * sin(forwardHeadingInRadians);
            estimatedYPos = -70.5 - (distance_right + Right_Sensor_Offset_X) * cos(sidewaysHeadingInRadians);
            break;
        case X_Neg:
            estimatedXPos = -70.5 - (distance_forwards + Forward_Sensor_Offset_Y) * sin(forwardHeadingInRadians);
            estimatedYPos = 70.5 - (distance_right + Right_Sensor_Offset_X) * cos(sidewaysHeadingInRadians);
            break;
        case Y_Pos:
            estimatedXPos = 70.5 - (distance_right + Right_Sensor_Offset_X) * sin(sidewaysHeadingInRadians);
            estimatedYPos = 70.5 - (distance_forwards + Forward_Sensor_Offset_Y) * cos(forwardHeadingInRadians);
            break;
        case Y_Neg:
            estimatedXPos = -70.5 - (distance_right + Right_Sensor_Offset_X) * sin(sidewaysHeadingInRadians);
            estimatedYPos = -70.5 - (distance_forwards + Forward_Sensor_Offset_Y) * cos(forwardHeadingInRadians);
            break;
    }

    estimatedFieldPosition.push_back(estimatedXPos);
    estimatedFieldPosition.push_back(estimatedYPos);

    return estimatedFieldPosition;
}