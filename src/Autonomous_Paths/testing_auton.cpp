#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "Robot/distance_calibration.h"
#include "Robot/color_sorting.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/testing_auton.h"

void TestingAuton(void);

Auton testingAuton = {
    "Testing Auton",
    "Testing stuff for Trey fr",
    0, 0, 0,
    TestingAuton
};

bool oscillating = false;
int wiggleOutput = 0;

void oscillate (void) {
    int startTime = Brain.Timer.system();
    while (Brain.Timer.system() - startTime < 500) {
        wiggleOutput = sin(Brain.Timer.system() / 100.0) * 20;
        left_drive.spin(forward, wiggleOutput, percent);
        right_drive.spin(forward, -wiggleOutput, percent);
    }
}

void TestingAuton(void) {
    intake.spin(forward, 80, percent);

    tracking_wheel_piston.set(true);
    setDrivetrainSpeed(50);
    wait(750, msec);
    turnToHeading(115, 30);
    setDrivetrainSpeed(-40);
    wait(550, msec);
    hood.set(true);
    //turnToHeading(195, 60);
    setDrivetrainSpeed(0);
    /*driveFor(2, 80);
    oscillate();
    driveFor(-3, 100);
    driveFor(6, 70);
    oscillate();
    driveFor(-6, 100);
    driveFor(6, 100);
    oscillate();
    driveFor(-24, 100);*/
}