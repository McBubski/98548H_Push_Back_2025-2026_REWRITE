#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "Robot/PID.h"
#include "vex.h"

#include <cmath>
#include <iostream>

void turnToHeading(double heading, double turnSpeed) {
    std::cout << "Started turning from " << inertial_sensor.heading() << "deg to " << heading << "deg (" << WrapAngle(heading - inertial_sensor.heading()) << "deg)" << std::endl;

    // Error info
    double error = WrapAngle(heading - inertial_sensor.heading());
    double startError = error;
    double previousTime = Brain.Timer.system();

    // Timeout time and time spent "finished"
    double timeout = 750 + (std::abs(startError) / 360.0) * 1600; // Base timeout plus extra time for larger turns
    double settleReachedTime = 0;

    std::cout << timeout << std::endl;

    // Bool if PID is running
    bool notDone = true;

    // PID Constants
    double p = 0.35;
    double i = 0.01;
    double d = 0.70;

    // Ramp up
    double acceleration = 1.0;

    // Margins to decide when the turn is done
    double error_margin = 1;
    double angular_velocity_margin = 3;

    PID turnPid = PID(p, 0, d, acceleration, 5, turnSpeed, timeout, 100);//0.48, 0.0001, 2.75,


    while (notDone) {
        double currentTime = Brain.Timer.system();
        double dt = currentTime - previousTime;
        previousTime = currentTime;

        error = WrapAngle(heading - inertial_sensor.heading());

        if (std::abs(error) <= 5) {
            turnPid.I = i;
        } 

        double speed = turnPid.Update(error, dt);

        bool atTarget = (std::abs(error) <= error_margin);
        bool isSlowed = (std::abs(inertial_sensor.gyroRate(zaxis, dps)) <= angular_velocity_margin);

        if (atTarget && isSlowed) {
            settleReachedTime += dt;
        } else {
            settleReachedTime = 0;
        }

        if (settleReachedTime > turnPid.SettleTime) {
            std::cout << "Settled down." << std::endl;
            notDone = false;
        }

        if (turnPid.Time > turnPid.Timeout) {
            std::cout << "Timed out." << std::endl;
            notDone = false;
        }

        left_drive.spin(forward, speed, percent);
        right_drive.spin(reverse, speed, percent);

        wait(10, msec);
        /*// Update Error

        error = WrapAngle(heading - inertial_sensor.heading());

        // Update PID

        double speed = turnPid.Update(error, (Brain.Timer.system() - previousTime));

        // Check End Condition

        // Checks if error is small enough and robot is slow enough
        if (std::abs(error) <= error_margin && std::abs(inertial_sensor.gyroRate(zaxis, dps)) <= angular_velocity_margin && turnPid.RanOnce == true && turnPid.HasReachedEnd == false) {
            //notDone = false;
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

        // Check this maybeSo

        if (std::abs(error) <= 5) {
            turnPid.I = i;
        }

        //std::cout << error << std::endl;

        // Spin Wheels

        left_drive.spin(forward, speed, percent);
        right_drive.spin(reverse, speed, percent);

        previousTime = Brain.Timer.system();

        wait(10, msec);

        //std::cout << "Error: " << error << std::endl;**/
    }

    //std::cout << "Ended turning." << std::endl;

    //Controller.rumble("..");

    left_drive.stop(brake);
    right_drive.stop(brake);
}
