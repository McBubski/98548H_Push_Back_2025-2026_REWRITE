#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "vex.h"

#include <cmath>

void pointAt(double x, double y, double turnSpeed, vex::directionType dir) {
    float targetOrientation = atan2(x - position_tracking.GlobalXPos, y - position_tracking.GlobalYPos);

    int angleOffset = 0;

    if (dir == reverse) {
        angleOffset = 180;
    }

    if ((std::abs(WrapAngle(targetOrientation * (180/M_PI) - inertial_sensor.heading())) + angleOffset) <= 5) {
        return;
    }

    if (dir == forward) {
        turnToHeading(targetOrientation * (180/M_PI), turnSpeed);
    } else if (dir == reverse) {
        turnToHeading(targetOrientation * (180/M_PI) + 180, turnSpeed);
    }
}