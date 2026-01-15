#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "Robot/PID.h"
#include "vex.h"

#include <cmath>
#include <iostream>

void turnToHeading(double heading, double turnSpeed) {
    std::cout << "\nStarted turning from " << inertial_sensor.heading() << " deg to " << heading << " deg (" << WrapAngle(heading - inertial_sensor.heading()) << " deg)" << std::endl;

    // Error info
    double error = WrapAngle(heading - inertial_sensor.heading());
    double startError = error;

    // Time info
    double startTime = Brain.Timer.system();
    double previousTime = startTime;

    // Timeout time and time spent "finished"
    double timeout = 750 + (std::abs(startError) / 360.0) * 1500; // Base timeout plus extra time for larger turns
    double settleReachedTime = 0;

    // Bool if PID is running
    bool notDone = true;

    // PID Constants
    double p = 0.315;//0.35;
    double i = 0;//0.01;
    double d = 0.28;//0.3;

    // Ramp up
    double acceleration = 1.0;

    // Margins to decide when the turn is done
    double error_margin = 1.5;
    double angular_velocity_margin = 10;

    if (std::abs(startError) <= error_margin) {
        return; // Why waste the time?
    }

    PID turnPid = PID(p, 0, d, acceleration, 5, turnSpeed, timeout, 100);//0.48, 0.0001, 2.75,

    while (notDone) {
        double currentTime = Brain.Timer.system();
        double dt = currentTime - previousTime;
        previousTime = currentTime;

        error = WrapAngle(heading - inertial_sensor.heading());

        if (std::abs(error) <= 3) {
            turnPid.I = i;
        } 

        double speed = turnPid.Update(error, dt);

        double minSpeed = 5; // minimum motor spin

        if (std::abs(error) > error_margin) {
            if (std::abs(speed) < minSpeed) {
                speed = minSpeed * Sign(error);
            }
        }

        bool atTarget = (std::abs(error) <= error_margin);
        bool isSlowed = (std::abs(inertial_sensor.gyroRate(zaxis, dps)) <= angular_velocity_margin);

        if (atTarget && isSlowed) {
            settleReachedTime += dt;
        } else {
            settleReachedTime = 0;
        }

        if (settleReachedTime > turnPid.SettleTime) {
            std::cout << "\nSettled down." << std::endl;
            notDone = false;
        }

        if (turnPid.Time > turnPid.Timeout) {
            std::cout << "\nTimed out." << std::endl;
            notDone = false;
        }

        // Spin Wheels

        left_drive.spin(forward, speed, percent);
        right_drive.spin(reverse, speed, percent);

        previousTime = Brain.Timer.system();

        wait(10, msec);

        //std::cout << "Error: " << error << std::endl;**/
    }

    std::cout << "Ended turning. (" << Brain.Timer.system() - startTime << "ms)" << std::endl;
    std::cout << "Final Error: " << error << std::endl;

    left_drive.stop(brake);
    right_drive.stop(brake);
}
