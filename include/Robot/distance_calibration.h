#pragma once

#include <vector>
#include "vex.h"

enum Wall {
    X_Pos,
    X_Neg,
    Y_Pos,
    Y_Neg
};

enum SensorSide {
    Left,
    Right
};

std::vector<double> EstimatePositionWithDistance(Wall closestWall);
std::vector<double> EstimatePositionWithDistance(Wall closestWall, SensorSide sensorSide);

// New Data

enum Axis {
    AXIS_NONE,
    AXIS_X,
    AXIS_Y
};

struct WallMeasurement {
    Axis axis;
    double estimatedPosition;
    double sensorDistance;
};

std::vector<double> ResetFieldPositionFromDistanceWithOdometry();
WallMeasurement ProcessDistanceSensor(distance distanceSensor, double sensor_offset_x, double sensor_offset_y, double sensor_heading_offset);