#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "Robot/PID.h"
#include "vex.h"

#include <cmath>
#include <iostream>

void turnToHeading(double heading, double turnSpeed) {
    std::cout << "Started turning." << std::endl;

    double error = WrapAngle(heading - inertial_sensor.heading());
    double startError = error;
    double previousTime = Brain.Timer.system();

    double timeout = 750 + (std::abs(startError) / 360.0) * 1500; // Base timeout plus extra time for larger turns

    std::cout << timeout << std::endl;

    bool notDone = true;

    double p = 0.38;
    double i = 0.01;
    double d = 0.6;

    double acceleration = 1.0;

    double error_margin = 1;
    double angular_velocity_margin = 3;

    PID turnPid = PID(p, 0, d, acceleration, 5, turnSpeed, timeout, 100);//0.48, 0.0001, 2.75,


    while (notDone) {
        // Update Error

        error = WrapAngle(heading - inertial_sensor.heading());

        // Update PID

        double speed = turnPid.Update(error, (Brain.Timer.system() - previousTime));

        // Check End Condition

        // Checks if error is small enough and robot is slow enough
        if (std::abs(error) <= error_margin && std::abs(inertial_sensor.gyroRate(zaxis, dps)) <= angular_velocity_margin && turnPid.RanOnce == true && turnPid.HasReachedEnd == false) {
            notDone = false;
            turnPid.HasReachedEnd = true;
            turnPid.TimeReachedEnd = turnPid.Time;

            std::cout << "Hit error and velocity target." << std::endl;
        }

        if(turnPid.Time - turnPid.TimeReachedEnd > turnPid.SettleTime && turnPid.HasReachedEnd) {
            notDone = false;
            turnPid.TimeReachedEnd = turnPid.Time;

            std::cout << "Settled down." << std::endl;
        }

        if (turnPid.Time > turnPid.Timeout) {
            notDone = false;
            turnPid.TimeReachedEnd = turnPid.Time;

            std::cout << "Timed out." << std::endl;
        }

        if (std::abs(error) <= 25) {
            turnPid.I = i;
        }

        //std::cout << error << std::endl;

        // Spin Wheels

        left_drive.spin(forward, speed, percent);
        right_drive.spin(reverse, speed, percent);

        previousTime = Brain.Timer.system();

        wait(10, msec);

        //std::cout << "Error: " << error << std::endl;
    }

    //std::cout << "Ended turning." << std::endl;

    //Controller.rumble("..");

    left_drive.stop(brake);
    right_drive.stop(brake);
}
