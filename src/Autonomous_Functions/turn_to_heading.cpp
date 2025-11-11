#include "Autonomous_Functions/auton_functions.h"
#include "Robot/utility_functions.h"
#include "Robot/PID.h"
#include "vex.h"

#include <cmath>

void turnToHeading(double heading, double turnSpeed) {
    double error = WrapAngle(heading - inertial_sensor.heading());
    double startError = error;
    double previousTime = Brain.Timer.system();

    double timeout = 1000;

    bool notDone = true;

    double p = 1.0 * 0.5;
    double i = 0;//0.0025;//0.005;
    double d = 1.6;//2.82;//2.85;

    double error_margin = 1;
    double angular_velocity_margin = 3;

    //int minMotorPower = 2;

    PID turnPid = PID(p, i, d, 1, 5, 100, timeout, 0);//0.48, 0.0001, 2.75,

    //double minimumSpeed = 1.5;

    while (notDone) {
        // Update Error

        error = WrapAngle(heading - inertial_sensor.heading());

        // Update PID

        double speed = turnPid.Update(error, (Brain.Timer.system() - previousTime));

        // Check End Condition

        // Checks if error is small enough and robot is slow enough
        if (std::abs(error) <= error_margin && std::abs(inertial_sensor.gyroRate(zaxis, dps)) <= angular_velocity_margin && turnPid.RanOnce == true && turnPid.HasReachedEnd == false) {
            turnPid.HasReachedEnd = true;
            turnPid.TimeReachedEnd = turnPid.Time;
        }

        if(turnPid.Time - turnPid.TimeReachedEnd > turnPid.SettleTime && turnPid.HasReachedEnd) {
            notDone = false;
        }

        if (std::abs(error) <= std::abs(startError) * 0.1) {
            //turnPid.I = 0.01;
        }

        if (turnPid.Time > turnPid.Timeout) {
            notDone = false;
        }

        //if (std::abs(speed) <= minimumSpeed) {
        //    speed = minimumSpeed * GetSign(speed);
        //}

        // Spin Wheels

        left_drive.spin(forward, speed, percent);
        right_drive.spin(reverse, speed, percent);

        previousTime = Brain.Timer.system();

        wait(10, msec);

        //std::cout << "Error: " << error << std::endl;
    }

    left_drive.stop(brake);
    right_drive.stop(brake);
}
