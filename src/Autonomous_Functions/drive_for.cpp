#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "Robot/PID.h"
#include "vex.h"

#include <cmath>
#include <iostream>

void driveFor(double distance, double speed) {
    std::cout << "Driving for: " << distance << " inches." << std::endl;
    
    // Error info
    double targetHeading = inertial_sensor.heading();
    double encoderStart = forward_tracking_wheel.position(turns);
    double driveError = distance;
    double turnError = WrapAngle(targetHeading - inertial_sensor.heading());

    double previousInchesMoved = 0;
    double previousSpeed = 0;

    // Time info
    double startTime = Brain.Timer.system();
    double previousTime = startTime;

    // Timeout time and time spent "finished"
    double timeout = (std::abs(distance) / 12) * 750 + 500;
    double settleReachedTime = 0;

    // Driving bool
    bool driving = true;

    // PID Constants
    double turn_kp = 0.315;
    double turn_ki = 0;
    double turn_kd = 0.3;

    double drive_kp = 3.05;//2.65;
    double drive_ki = 0;//0.075;
    double drive_kd = 1.05;//3.85;

    // Ramp up speed
    double acceleration = 0.25;

    // Margins to determine drive end
    double drive_margin = 0.25;
    double linear_velocity_margin = 0.05; 

    if (std::abs(driveError) <= drive_margin) {
        return;
    }

    PID drivePID = PID(drive_kp, 0, drive_kd, acceleration, 5, speed, timeout, 100);
    PID turnPID = PID(turn_kp, turn_ki, turn_kd, 100, 5, 100, 99999999, 0);//0.48, 0.0001, 2.75,

    while (driving) {
        double currentTime = Brain.Timer.system();
        double dt = currentTime - previousTime;
        previousTime = currentTime;

        double encoderChange = forward_tracking_wheel.position(turns) - encoderStart;
        double inchesMoved = encoderChange * 2.75 * M_PI;
        double speedInInches = std::abs(inchesMoved - previousInchesMoved) / (dt / 1000.0); // I think this is wrong, but whatever.

        previousSpeed = speedInInches;
        previousInchesMoved = inchesMoved;

        driveError = distance - inchesMoved;
        turnError = WrapAngle(targetHeading - inertial_sensor.heading());

        double driveSpeed = drivePID.Update(driveError, dt);
        double turnSpeed = turnPID.Update(turnError, dt);

        double minSpeed = 3;

        if (std::abs(driveError) > drive_margin) {
            if (std::abs(driveSpeed) < minSpeed) {
                driveSpeed = minSpeed * Sign(driveError);
            }
        }

        bool atTarget = (std::abs(driveError) <= drive_margin);
        bool isSlowed = (std::abs(inertial_sensor.acceleration(yaxis)) <= linear_velocity_margin);

        if (atTarget && isSlowed) {
            settleReachedTime += dt;
        } else {
            settleReachedTime = 0;
        }

        if (settleReachedTime > drivePID.SettleTime) {
            std::cout << "\nSettled down." << std::endl;
            driving = false;
        }

        if (drivePID.Time > drivePID.Timeout) {
            std::cout << "\nTimed out." << std::endl;
            driving = false;
        }

        if ((left_drive.current(percent) >= 98.0 || right_drive.current(percent) >= 98.0) && drivePID.Time >= drivePID.Timeout / 2) {
            driving = false;
            std::cout << "Stall!" << std::endl;
        }


        left_drive.spin(forward, driveSpeed + turnSpeed, percent);
        right_drive.spin(forward, driveSpeed - turnSpeed, percent);

        previousTime = Brain.Timer.system();

        wait(10, msec);
        /*double encoderChange = forward_tracking_wheel.position(turns) - encoderStart;
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

        //std::cout << "Error: " << driveError << std::endl;*/

        //std::cout << turnSpeed << std::endl;
        //std::cout << "Error: " << driveError << ", " << atTarget << ", " << isSlowed << std::endl;
        //std::cout << inertial_sensor.acceleration(yaxis) << std::endl;
    }

    std::cout << "Ended driving. (" << Brain.Timer.system() - startTime << "ms)" << std::endl;
    std::cout << "Final Error: " << driveError << std::endl;

    left_drive.stop(brake);
    right_drive.stop(brake);
}

void setDrivetrainSpeed(double speed) {
    left_drive.spin(forward, speed, percent);
    right_drive.spin(forward, speed, percent);
}