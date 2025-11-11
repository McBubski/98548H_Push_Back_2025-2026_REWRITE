#include "Robot/utility_functions.h"

#include <cmath>

// Wraps an angle in degrees to the range [-180, 180)

double WrapAngle(double angle) {
    angle = fmod(angle + 180.0, 360.0);
    return (angle >= 0) ? angle - 180.0 : angle + 180.0;
}

// Get distance between two points (x1, y1) and (x2, y2)

double GetDistance(double x1, double y1, double x2, double y2) {
    double a = (x1 - x2) * (x1 - x2);
    double b = (y1 - y2) * (y1 - y2);
    double dist = sqrt(a + b);

    return dist;
}

// Get angle in degrees from point (x1, y1) to point (x2, y2)

double GetAngle(double x1, double y1, double x2, double y2) {
    return RadToDeg(atan2(y2 - y1, x2 - x1));
}

// Convert degrees to radians

double DegToRad(double degrees) {
    return degrees * (M_PI / 180.0);
}

// Convert radians to degrees

double RadToDeg(double radians) {
    return radians * (180.0 / M_PI);
}

// Get the sign of a value

double Sign(double value) {
    if (value > 0) {
        return 1.0;
    } else if (value < 0) {
        return -1.0;
    } else {
        return 0.0;
    }
}