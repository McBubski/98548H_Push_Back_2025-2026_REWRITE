#pragma once

#include <vector>

enum Wall {
    X_Pos,
    X_Neg,
    Y_Pos,
    Y_Neg
};

std::vector<double> FieldPositionFromDistance();
std::vector<double> EstimatePositionWithDistance(Wall closestWall);