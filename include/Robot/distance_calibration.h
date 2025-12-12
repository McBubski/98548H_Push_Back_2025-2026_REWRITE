#pragma once

#include <vector>

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

std::vector<double> FieldPositionFromDistance();
std::vector<double> EstimatePositionWithDistance(Wall closestWall);
std::vector<double> EstimatePositionWithDistance(Wall closestWall, SensorSide sensorSide);