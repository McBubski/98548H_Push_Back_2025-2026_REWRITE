#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "Robot/PID.h"
#include "vex.h"

#include <cmath>

void driveFor(double distance, double speed) {
    double targetHeading = inertial_sensor.heading();
    double encoderStart = forward_tracking_wheel.position(turns);

    bool driving = true;

    int timeout = (std::abs(distance) / 12) * 750 + 500;

    PID drivePID = PID(5.3, 0, 1.2, 0.1, 10, speed, timeout, 100);
    
    //double p = 1.15 * 0.5;
    //double i = 0;
    //double d = 2.6;

    double p = 1.0 * 0.5;
    double i = 0;//0.0025;//0.005;
    double d = 1.6;//2.82;//2.85;

    PID turnPID = PID(p, i, d, 100, 5, 100, 99999999, 0);//0.48, 0.0001, 2.75,

    double driveError = distance;
    double turnError = WrapAngle(targetHeading - inertial_sensor.heading());

    double drive_margin = 0.25;
    //double linear_velocity_margin = 1;

    double previousTime = Brain.Timer.system();

    while (driving) {
        double encoderChange = forward_tracking_wheel.position(turns) - encoderStart;
        double inchesMoved = encoderChange * 2 * M_PI; // Circumference of Wheels

        driveError = distance - inchesMoved;
        turnError = WrapAngle(targetHeading - inertial_sensor.heading());

        double dt = (Brain.Timer.system() - previousTime);

        double driveOutput = drivePID.Update(driveError, dt);
        double turnOutput = turnPID.Update(turnError, dt);

        // Checks if error is small enough and robot is slow enough
        if (std::abs(driveError) <= drive_margin) {
            driving = false;
        }

        // *** ADD SETTLE TIME ***

        if (drivePID.Time > drivePID.Timeout) {
            driving = false;
        }

        //if (std::abs(driveOutput) <= 5) {
        //    driveOutput = 5 * GetSign(speed);
        //}

        left_drive.spin(forward, (driveOutput + turnOutput), percent);
        right_drive.spin(forward, (driveOutput - turnOutput), percent);

        previousTime = Brain.Timer.system();

        wait(10, msec);

        //std::cout << "Error: " << driveError << std::endl;
    }

    left_drive.stop(brake);
    right_drive.stop(brake);
}