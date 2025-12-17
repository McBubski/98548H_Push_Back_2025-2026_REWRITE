#include "Robot/distance_calibration.h"
#include "vex.h"
#include "Robot/utility_functions.h"

#include <iostream>

// Forward sensor settings
double Forward_Sensor_Offset_X = -5.0;
double Forward_Sensor_Offset_Y = 5.0;

double Forward_Sensor_Angle_Offset = 0.0;
double Forward_Sensor_Calibration_Bias = 5.0/8.0;

// Left sensor settings
double Left_Sensor_Offset_X = -3.75;
double Left_Sensor_Offset_Y = -5.0;

double Left_Sensor_Angle_Offset = -90.0;
double Left_Sensor_Calibration_Bias = 0.0;

// Right sensor settings
double Right_Sensor_Offset_X = 3.75;
double Right_Sensor_Offset_Y = -5.0;

double Right_Sensor_Angle_Offset = 90.0;
double Right_Sensor_Calibration_Bias = 0.0;

// Other settings

double Max_Valid_Distance = 390.0;
double Field_Min = -70.5;
double Field_Max = 70.5;
double Epsilon = 1e-6;
double Sensor_Disagreement_Tolerance = 2.0;

std::vector<double> ResetFieldPositionFromDistanceWithOdometry() {
    // Variable to save estimated field position
    std::vector<double> estimatedFieldPosition;

    // Front wall measurement
    WallMeasurement frontMeasurement = ProcessDistanceSensor(
        forward_distance_sensor, 
        Forward_Sensor_Offset_X, 
        Forward_Sensor_Offset_Y, 
        Forward_Sensor_Angle_Offset
    );

    WallMeasurement leftMeasurement = ProcessDistanceSensor(
        left_distance_sensor, 
        Left_Sensor_Offset_X, 
        Left_Sensor_Offset_Y, 
        Left_Sensor_Angle_Offset
    );

    WallMeasurement rightMeasurement = ProcessDistanceSensor(
        right_distance_sensor, 
        Right_Sensor_Offset_X, 
        Right_Sensor_Offset_Y, 
        Right_Sensor_Angle_Offset
    );

    // Handle fusion of left and right

    WallMeasurement chosenSide;
    bool sideValid = false;

    // If both sides return a valid estimate
    if (leftMeasurement.axis != AXIS_NONE && rightMeasurement.axis != AXIS_NONE) {
        std::cout << "Data from both sides!" << std::endl;
        if (leftMeasurement.axis == rightMeasurement.axis) {
            if (leftMeasurement.sensorDistance < rightMeasurement.sensorDistance) {
                chosenSide = leftMeasurement;
            } else {
                chosenSide = rightMeasurement;
            }
        } else {
            if (leftMeasurement.sensorDistance < rightMeasurement.sensorDistance) {
                chosenSide = leftMeasurement;
            } else {
                chosenSide = rightMeasurement;
            }
        }
        sideValid = true;
    } else if (leftMeasurement.axis != AXIS_NONE) {
        std::cout << "Data from left!" << std::endl;
        chosenSide = leftMeasurement;
        sideValid = true;
    } else if (rightMeasurement.axis != AXIS_NONE) {
        std::cout << "Data from right!" << std::endl;
        chosenSide = rightMeasurement;
        sideValid = true;
    }

    double estimatedXDistance = position_tracking.GlobalXPos;
    double estimatedYDistance = position_tracking.GlobalYPos;

    if (frontMeasurement.axis == AXIS_X) {
        estimatedXDistance = frontMeasurement.estimatedPosition;
    } else if (frontMeasurement.axis == AXIS_Y) {
        estimatedYDistance = frontMeasurement.estimatedPosition;
    }

    if (sideValid) {
        if (chosenSide.axis == AXIS_X) {
            estimatedXDistance = chosenSide.estimatedPosition;
        } else if (chosenSide.axis == AXIS_Y) {
            estimatedYDistance = chosenSide.estimatedPosition;
        }
    }

    estimatedFieldPosition.push_back(estimatedXDistance);
    estimatedFieldPosition.push_back(estimatedYDistance);

    return estimatedFieldPosition;
}

WallMeasurement ProcessDistanceSensor(distance distanceSensor, double sensor_offset_x, double sensor_offset_y, double sensor_heading_offset) {
    WallMeasurement measurement;
    measurement.axis = AXIS_NONE;
    measurement.estimatedPosition = 0.0;
    measurement.sensorDistance = 0.0;

    double measuredDistance = distanceSensor.objectDistance(distanceUnits::in);

    // Sensor validity check

    if (!distanceSensor.installed() || measuredDistance <= 0 || measuredDistance > Max_Valid_Distance) {
        std::cout << "Invalid." << std::endl;
        return measurement;
    }

    // Converts sensor offset to field frame
    double local_offset_x = sensor_offset_x * cos(DegToRad(inertial_sensor.heading(degrees))) + sensor_offset_y * sin(DegToRad(inertial_sensor.heading(degrees)));
    double local_offset_y = sensor_offset_x * -cos(DegToRad(inertial_sensor.heading(degrees))) + sensor_offset_y * sin(DegToRad(inertial_sensor.heading(degrees)));

    // Calculates sensor origin on field
    double sensor_origin_x = position_tracking.GlobalXPos + local_offset_x;
    double sensor_origin_y = position_tracking.GlobalYPos + local_offset_y;


    // Sensor ray direction in field frame
    double sensor_global_angle_radians = DegToRad(inertial_sensor.heading(degrees) + sensor_heading_offset);
    double sensor_unit_x_dir = sin(sensor_global_angle_radians);
    double sensor_unit_y_dir = cos(sensor_global_angle_radians);

    // Search algorithm values
    double bestT = 1e9;
    Axis hitAxis = AXIS_NONE;
    double wallOffset = 0.0;

    // Y Walls

    if (fabs(sensor_unit_x_dir) > Epsilon) { // Avoids divide by 0 error
        double t = (Field_Min - sensor_origin_y) / sensor_unit_y_dir;
        double yhit = sensor_origin_x + t * sensor_unit_x_dir;

        if (t > 0 && yhit >= Field_Min && yhit <= Field_Max && t < bestT) {
            bestT = t;
            hitAxis = AXIS_Y;
            wallOffset = Field_Min;
        }

        t = (Field_Max - sensor_origin_y) / sensor_unit_y_dir;
        yhit = sensor_origin_x + t * sensor_unit_x_dir;

        if (t > 0 && yhit >= Field_Min && yhit <= Field_Max && t < bestT) {
            bestT = t;
            hitAxis = AXIS_Y;
            wallOffset = Field_Max;
        }
    }

    // Horizontal walls

    if (fabs(sensor_unit_y_dir) > Epsilon) {
        double t = (Field_Min - sensor_origin_x) / sensor_unit_x_dir;
        double xhit = sensor_origin_y + t * sensor_unit_y_dir;

        if (t > 0 && xhit >= Field_Min && xhit <= Field_Max && t < bestT) {
            bestT = t;
            hitAxis = AXIS_X;
            wallOffset = Field_Min;
        }

        t = (Field_Max - sensor_origin_x) / sensor_unit_x_dir;
        xhit = sensor_origin_y + t * sensor_unit_y_dir;

        if (t > 0 && xhit >= Field_Min && xhit <= Field_Max && t < bestT) {
            bestT = t;
            hitAxis = AXIS_X;
            wallOffset = Field_Max;
        }
    }

    double sensorOffset = local_offset_y * sensor_unit_y_dir;

    if (hitAxis == AXIS_X) {
        measurement.axis = AXIS_X;
        measurement.estimatedPosition = wallOffset - measuredDistance * sensor_unit_x_dir - local_offset_x;
        measurement.sensorDistance = measuredDistance;
    } else {
        measurement.axis = AXIS_Y;
        measurement.estimatedPosition = wallOffset - measuredDistance * sensor_unit_y_dir - local_offset_y;
        measurement.sensorDistance = measuredDistance;
    }

    /*std::cout << "\n\nRESULT:" << std::endl;
    std::cout << "Best T: " << bestT << std::endl;
    if (hitAxis == AXIS_NONE) {
        std::cout << "No Axis!" << std::endl;
    } else if (hitAxis == AXIS_X) {
        std::cout << "X Axis!" << std::endl;
    } else if (hitAxis == AXIS_Y) {
        std::cout << "Y Axis!" << std::endl;
    }
    std::cout << "Offset/Direction: " << wallOffset << std::endl;
    std::cout << "Estimated Position: " << measurement.estimatedPosition << std::endl;*/

    return measurement;
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