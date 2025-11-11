#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "vex.h"

#include <cmath>

void driveTo(double x, double y, double speed, vex::directionType dir){
    pointAt(x, y, 100, dir);

    if (dir == forward) {
        double dist = GetDistance(position_tracking.GlobalXPos, position_tracking.GlobalYPos, x, y);
        driveFor(dist, speed);
    }

    if (dir == reverse) {
        double dist = GetDistance(position_tracking.GlobalXPos, position_tracking.GlobalYPos, x, y);
        driveFor(-dist, speed);
    }
}