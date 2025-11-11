#pragma once

class Odometry
{
private:
    // Travel of tracking wheels (INCHES)

    double forwardPos;
    double sidewaysPos;

    double previousForwardPos;
    double previousSidewaysPos;

    // Distance travelled per loop (INCHES)

    double deltaForward;
    double deltaSideways;

    // Distance summations (INCHES)

    double totalDeltaForward;
    double totalDeltaSideways;

    // Heading (RADIANS)
    
    double previousHeadingInRadians;

    // Change in Heading (RADIANS)

    double headingInRadians;
    double deltaHeadingInRadians;
    double averageHeadingForArc;

    // Local Position Change (INCHES)

    double deltaXLocal;
    double deltaYLocal;

    // Global Position Change (INCHES)

    double deltaXGlobal;
    double deltaYGlobal;
    
public:
    // User Defined Values, Inches unless otherwise noted.

    double TrackingWheelRadius;

    double ForwardTrackingDistance;
    double SidewaysTrackingDistance;

    // Global Robot Values

    double Heading;
    double GlobalXPos;
    double GlobalYPos;

    Odometry(double tracking_wheel_radius, double forward_tracking_distance, double sideways_tracking_distance);

    void Update(void);
    
    void SetPosition(double new_x, double new_y, double new_heading);
    void SetPositionToCurrentAuton(void);
};