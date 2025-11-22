#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "Robot/PID.h"
#include "vex.h"

#include <cmath>
#include <iostream>

void driveFor(double distance, double speed) {
    double targetHeading = inertial_sensor.heading();
    double encoderStart = forward_tracking_wheel.position(turns);

    bool driving = true;

    int timeout = (std::abs(distance) / 12) * 750 + 500;

    double turn_kp = 0.375;
    double turn_ki = 0.01;
    double turn_kd = 0.7;

    double drive_kp = 2.75;
    double drive_ki = 0.075;
    double drive_kd = 5.0;

    double acceleration = 0.1;

    PID drivePID = PID(drive_kp, 0, drive_kd, acceleration, 10, speed, timeout, 100);
    PID turnPID = PID(turn_kp, turn_ki, turn_kd, 100, 5, 100, 99999999, 0);//0.48, 0.0001, 2.75,

    double driveError = distance;
    double turnError = WrapAngle(targetHeading - inertial_sensor.heading());

    double drive_margin = 0.25;
    //double linear_velocity_margin = 1;

    double previousTime = Brain.Timer.system();

    while (driving) {
        double encoderChange = forward_tracking_wheel.position(turns) - encoderStart;
        double inchesMoved = encoderChange * 2.75 * M_PI; // Circumference of Wheels

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

        if (std::abs(driveError) <= 4) {
            drivePID.I = drive_ki;
        }
        
        // Check stalling, as long as it's at least tried to move for a bit
         if ((left_drive.current(percent) >= 98.0 || right_drive.current(percent) >= 98.0) && drivePID.Time >= drivePID.Timeout / 2) {
            driving = false;
            std::cout << "Stall!" << std::endl;
        }

        left_drive.spin(forward, (driveOutput + turnOutput), percent);
        right_drive.spin(forward, (driveOutput - turnOutput), percent);

        previousTime = Brain.Timer.system();

        wait(10, msec);

        //std::cout << "Error: " << driveError << std::endl;
    }

    left_drive.stop(brake);
    right_drive.stop(brake);
}

void setDrivetrainSpeed(double speed) {
    left_drive.spin(forward, speed, percent);
    right_drive.spin(forward, speed, percent);
}