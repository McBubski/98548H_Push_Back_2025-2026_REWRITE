#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"

#include "Autonomous_Paths/left_auton.h"

void LeftAuton(void);

Auton leftAuton = {
    "Left Auton",
    "Gets lots of points!",
    50.0, -16.0, 180.0,
    LeftAuton
};

void LeftAuton(void) {
    // Score in middle goal

    matchloader.set(true);
    //right_trapdoor.set(true);
    intake.spin(forward, 100, percent);

    driveFor(30, 100);
    turnToHeading(90, 100);
    driveFor(14, 28);

    wait(100, msec);
    driveFor(-20, 100);

    pointAt(33, -44.5, 100, reverse);
    hood.set(true);

    indexer.spin(reverse, 100, percent);
    left_drive.spin(reverse, 100, percent);
    right_drive.spin(reverse, 100, percent);

    wait(1100, msec);

    matchloader.set(false);
    indexer.stop();
    hood.set(false);

    driveFor(8, 100);
    left_drive.spin(reverse, 100, percent);
    right_drive.spin(reverse, 100, percent);

    wait(400, msec);


    driveFor(20, 100);

    wing.set(false);
   
    intake.stop();
    intake.spin(forward, 80, percent);
    //FollowPath(middle_goal_path, forward, 36.0);
    driveTo(21, -19.5, 45, forward);
    driveTo(9.5, -9.5, 85, reverse);

    intake.spin(forward, 80, percent);
    indexer.spin(forward, 60, percent);
    hood.set(true);

    pointAt(-1, 0, 100, reverse);

    driveFor(0.5, 20);
}
