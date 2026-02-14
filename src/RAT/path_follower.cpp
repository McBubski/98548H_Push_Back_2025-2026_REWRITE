#include "RAT/path_follower.h"

#include "Robot/utility_functions.h"
#include <cmath>
#include <iostream>

// Follows a path, with a lookahead distance

bool RAT_Interrupt = false;

void FollowPath(Path& path, vex::directionType direction, double lookaheadDistance) {
    bool driving = true;
    
    // Point Index Tracking

    int lastClosestIndex = 0;
    int lastLookaheadIndex = 0;
    int lastTriggeredIndex = 0;

    double maxRate = path.maxAcceleration; // Motor percent/second
    double previousTime = Brain.Timer.system();

    double previous_left_target_velocity = path.waypoints[0].targetVelocity;
    double previous_right_target_velocity = path.waypoints[0].targetVelocity;
    double previous_target_velocity = path.waypoints[0].targetVelocity;

    double abs_velocity_target = 0.0;

    double pathLength = path.waypoints[path.size() - 1].distanceAlongPath;
    double timeout = std::max(1000.0, (std::floor(pathLength) * 90.0));
    double startTime = Brain.Timer.system();

    double kA = 0.05;//0.02;
    double kV = 1.0;//1.0;
    double kP = 0.4;//0.02;

    while (driving) {
        // Calculate deltaTime
        double currentTime = Brain.Timer.system();
        double dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        // Step One - Find closest point
        int closestIndex = FindClosestPoint(path, lastClosestIndex);
        lastClosestIndex = closestIndex;

        // Step Two - Find the lookahead point
        auto[lookaheadPoint, fractionalIndex] = FindLookaheadPoint(path, lastLookaheadIndex, lookaheadDistance);
        lastLookaheadIndex = fractionalIndex;

        // Step Three - Compute curvature to lookahead point
        double kappa = ComputeCurvature(lookaheadPoint, lookaheadDistance);

        // Step Four - Rate limit target velocity
        double target_velocity = path.waypoints[lastClosestIndex].targetVelocity;
        double maxChangeUp = dt * maxRate;
        double maxChangeDown = dt * maxRate * 2; // Faster decceleration
        double deltaVel = target_velocity - previous_target_velocity;

        if (deltaVel > 0) { // Accelerating?
            if (deltaVel > maxChangeUp) deltaVel = maxChangeUp;
        } else { // Decelerating
            if (deltaVel < -maxChangeDown) deltaVel = -maxChangeDown;
        }

        target_velocity = previous_target_velocity + deltaVel;
        previous_target_velocity = target_velocity;

        // Step Five - Calculate target wheel velocities
        double track_width = 11.0;
        double left_target_velocity = target_velocity * (2.0 + kappa * track_width) / 2.0;
        double right_target_velocity = target_velocity * (2.0 - kappa * track_width) / 2.0;

        double left_target_accel = (left_target_velocity - previous_left_target_velocity) / dt;
        double right_target_accel = (right_target_velocity - previous_right_target_velocity) / dt;

        // Step Five - Feedback Controller
        double ff_left = kV * left_target_velocity + kA * left_target_accel;
        double fb_left = kP * (left_target_velocity - left_drive.velocity(percent));

        double ff_right = kV * right_target_velocity + kA * right_target_accel;
        double fb_right= kP * (right_target_velocity - right_drive.velocity(percent));

        double dir = 1.0;

        if (direction == forward) {
            dir = 1.0;
        } else if (direction == reverse) {
            dir = -1.0;
        }

        double left_output = (ff_left + fb_left) * dir;
        double right_output = (ff_right + fb_right) * dir;

        if (left_output < -100.0) {left_output = -100.0;} else if (left_output > 100.0) {left_output = 100.0;};
        if (right_output < -100.0) {right_output = -100.0;} else if (right_output > 100.0) {right_output = 100.0;};

        left_drive.spin(forward, left_output, percent);
        right_drive.spin(forward, right_output , percent);

        // Checks end condition

        Waypoint lastPoint = path.waypoints[path.size() - 1];
        double distanceToEnd = GetDistance(position_tracking.GlobalXPos, position_tracking.GlobalYPos, lastPoint.x, lastPoint.y);

        // I changed this on Feb 14. Please undo if broken.

        if (closestIndex >= (path.size() - 2) && distanceToEnd <= 4.0) {
            left_drive.stop(coast);
            right_drive.stop(coast);

            driving = false;
        };

        if (Brain.Timer.system() - startTime > timeout) {
            left_drive.stop(coast);
            right_drive.stop(coast);

            driving = false;
        }

        if (RAT_Interrupt) {
            driving = false;
            RAT_Interrupt = false;
        }

        //std::cout << "Path size: " << path.size() << ", Lookahead Index: " << closestIndex << ", L.A. Point: (" << lookaheadPoint.x << ", " << lookaheadPoint.y << ")" << std::endl;

        //std::cout << target_velocity << ", " << (LeftDrive.velocity(percent) + RightDrive.velocity(percent)) / 2.0 << std::endl;
        //std::cout << distanceToEnd << std::endl;

        //std::cout << distanceToEnd << ", " << closestIndex << "/" << path.size() << std::endl;

        // Checks Function Calls
        if (closestIndex >= lastTriggeredIndex + 1 && path.waypoints[closestIndex].onReach) {
            path.waypoints[closestIndex].onReach();
            lastTriggeredIndex = closestIndex;
        }

        //std::cout << closestIndex << std::endl;

        float pixelsPerInch = (200.0 / 144.0);
        float XOnBrainScreen = 337 + (pixelsPerInch * lookaheadPoint.x);
        float YOnbrainScreen = 110 + (-pixelsPerInch * lookaheadPoint.y);


        Brain.Screen.setFillColor(purple);
        Brain.Screen.setPenColor(black);
        Brain.Screen.drawCircle(XOnBrainScreen, YOnbrainScreen, 6);

        previous_left_target_velocity = left_target_velocity;
        previous_right_target_velocity = right_target_velocity;

       // std::cout << "Driving" << std::endl;

        wait(20, msec);
    }

    Controller.rumble(".");
}

int FindClosestPoint(Path& path, int lastClosestIndex) {
    double shortestDistance = 99999.0;
    int closestIndex = lastClosestIndex;

    for (size_t i = lastClosestIndex; i < path.size() - 1; i++) {
        // Calculate distance

        double dx = position_tracking.GlobalXPos - path.waypoints[i].x;
        double dy = position_tracking.GlobalYPos - path.waypoints[i].y;
        double segmentLength = std::sqrt(dx * dx + dy * dy);

        // Save shortest distance and index

        if (segmentLength < shortestDistance) {
            shortestDistance = segmentLength;
            closestIndex = i;
        }
    }

    return closestIndex;
}

std::pair<Waypoint, double> FindLookaheadPoint(const Path& path, int lastLookaheadIndex, double lookaheadDistance) {
    Waypoint lookaheadPoint = path.waypoints[lastLookaheadIndex];
    double fractionalIndex = lastLookaheadIndex;

    bool intersectionFound = false;

    for (size_t i = lastLookaheadIndex; i < path.size() - 1; i++) {
        const Waypoint& start = path.waypoints[i];
        const Waypoint& end = path.waypoints[i + 1];

        std::vector<double> t_vals = CircleLineIntersection(lookaheadDistance, start, end);

        for (size_t j = 0; j < t_vals.size(); j++) {
            double t = t_vals[j];
            double candidateIndex = i + t;

            if (candidateIndex > fractionalIndex) {
                lookaheadPoint = InterpolateSegment(start, end, t);
                fractionalIndex = candidateIndex;
                intersectionFound = true;
                
            }
        }
    }

    if (!intersectionFound) {
        Waypoint lastPoint = path.waypoints[path.size() - 1];
        double distToEnd = GetDistance(position_tracking.GlobalXPos, position_tracking.GlobalYPos, lastPoint.x, lastPoint.y);

        if (lookaheadDistance > distToEnd) {
            return std::make_pair(lastPoint, (double)(path.size() - 1));
        }
    }

    return std::make_pair(lookaheadPoint, fractionalIndex);
}

double ComputeCurvature(const Waypoint& lookaheadPoint, double lookaheadDistance) {
    // Robot position and heading in radians (+Y = 0°)
    double Rx = position_tracking.GlobalXPos;
    double Ry = position_tracking.GlobalYPos;
    double headingRad = (inertial_sensor.heading(degrees)) * M_PI / 180.0;

    // Vector from robot to lookahead point
    double dx = lookaheadPoint.x - Rx;
    double dy = lookaheadPoint.y - Ry;

    // Rotate into robot frame (X = forward, Y = left)
    double x_rel =  sin(headingRad) * dx + cos(headingRad) * dy;  // forward
    double y_rel = -cos(headingRad) * dx + sin(headingRad) * dy;  // lateral

    if (lookaheadDistance == 0) return 0;

    // Curvature formula: kappa = 2 * lateral offset / lookahead distance squared
    return -2.0 * y_rel / (lookaheadDistance * lookaheadDistance);
}

std::vector<double> CircleLineIntersection(double r, const Waypoint& E, const Waypoint& L) {
    const Waypoint& C = Waypoint(position_tracking.GlobalXPos, position_tracking.GlobalYPos);
    std::vector <double> t_values;

    double dx = L.x - E.x;
    double dy = L.y - E.y;
    double fx = E.x - C.x;
    double fy = E.y - C.y;

    double a = dx*dx + dy*dy;
    double b = 2*(fx*dx + fy*dy);
    double c = fx*fx + fy*fy - r*r;

    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) return t_values;

    discriminant = std::sqrt(discriminant);
    double t1 = (-b - discriminant) / (2 * a);
    double t2 = (-b + discriminant) / (2 * a);

    if (t1 >= 0 && t1 <= 1) t_values.push_back(t1);
    if (t2 >= 0 && t2 <= 1) t_values.push_back(t2);

    return t_values;

}

Waypoint InterpolateSegment(const Waypoint& start, const Waypoint& end, double t) {
    return Waypoint(
        start.x + t * (end.x - start.x),
        start.y + t * (end.y - start.y)
    );
}

