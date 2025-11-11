#include "Robot/odometry.h"
#include "Autonomous/autonomous_definitions.h"
#include "Robot/utility_functions.h"
#include "vex.h"

// Initialize Odometry Class

Odometry::Odometry(double tracking_radius, double forward_tracking_distance, double sideways_tracking_distance) {
    // User Defined Values
    
    TrackingWheelRadius = tracking_radius;
    ForwardTrackingDistance = forward_tracking_distance;
    SidewaysTrackingDistance = sideways_tracking_distance;

    GlobalXPos = 0;
    GlobalYPos = 0;
    Heading = 0;

    headingInRadians = 0;
    previousHeadingInRadians = 0;

    // Travel of tracking wheels (INCHES)

    forwardPos = 0;
    sidewaysPos = 0;

    previousForwardPos = 0;
    previousSidewaysPos = 0;

    // Distance travelled per loop (INCHES)

    deltaForward = 0;
    deltaSideways = 0;

    // Distance summations (INCHES)

    totalDeltaForward = 0;
    totalDeltaSideways = 0;

     // Change in Heading (RADIANS)

    deltaHeadingInRadians = 0;
    averageHeadingForArc = 0;

    // Local Position Change (INCHES)

    deltaXLocal = 0;
    deltaYLocal = 0;

    // Global Position Change (INCHES)

    deltaXGlobal = 0;
    deltaYGlobal = 0;
}

// Update Odometry

void Odometry::Update(void) {
    if (inertial_sensor.isCalibrating()) {
        return;
    }

    // Rotation Sensor Values (DEGREES)
    forwardPos = forward_tracking_wheel.position(rotationUnits::deg);
    sidewaysPos = sideways_tracking_wheel.position(rotationUnits::deg);

    // Converts degrees to RADIANS, then calculates distance travelled (INCHES)
    deltaForward = (DegToRad(forwardPos - previousForwardPos)) * TrackingWheelRadius;
    deltaSideways = (DegToRad(sidewaysPos - previousSidewaysPos)) * TrackingWheelRadius;

    // Updates previous sensor values (DEGREES)
    previousForwardPos = forwardPos;
    previousSidewaysPos = sidewaysPos;

    // Total change in encoders (INCHES)
    totalDeltaForward += deltaForward;
    totalDeltaSideways += deltaSideways;

    // Saves robot heading (DEGREES)
    Heading = inertial_sensor.heading(rotationUnits::deg);

    // Saves robot heading (RADIANS)
    headingInRadians = DegToRad(Heading);

    // Calculates change in angle in RADIANS 
    deltaHeadingInRadians = headingInRadians - previousHeadingInRadians;

    // Updates previous orientation in RADIANS
    previousHeadingInRadians = headingInRadians;

    if (deltaHeadingInRadians == 0) { // If no rotation, it was just a translation
        deltaXLocal = deltaSideways;
        deltaYLocal = deltaForward;
    } else { // Else, do some boring math
        deltaXLocal = 2 * sin(deltaHeadingInRadians / 2.0) * ((deltaSideways / deltaHeadingInRadians) + SidewaysTrackingDistance);
        deltaYLocal = 2 * sin(deltaHeadingInRadians / 2.0) * ((deltaForward / deltaHeadingInRadians) - ForwardTrackingDistance); 
    }

    // Average heading for this arc (RADIANS)
    averageHeadingForArc = headingInRadians - (deltaHeadingInRadians / 2);

    // Gets change in X and Y position (INCHES)
    deltaXGlobal = (deltaYLocal * sin(averageHeadingForArc)) + (deltaXLocal * cos(averageHeadingForArc));
    deltaYGlobal = (deltaYLocal * cos(averageHeadingForArc)) - (deltaXLocal * sin(averageHeadingForArc));

    GlobalXPos += deltaXGlobal;
    GlobalYPos += deltaYGlobal;
}

// Sets odometry to specific position and heading

void Odometry::SetPosition(double new_x, double new_y, double new_heading) {
    headingInRadians = DegToRad(new_heading);
    previousHeadingInRadians = headingInRadians;

    inertial_sensor.setHeading(new_heading, degrees);

    GlobalXPos = new_x;
    GlobalYPos = new_y;
}

void Odometry::SetPositionToCurrentAuton() {
    Auton* current_auton = autons[auton_path];
    SetPosition(current_auton->startX, current_auton->startY, current_auton->startHeading);
}