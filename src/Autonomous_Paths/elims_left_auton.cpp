#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/elims_left_auton.h"
#include <iostream>

void ElimsLeftAuton(void);

Auton elimsLeftAuton = {
    "Elims Left",
    "Gets lots of points!",
    50.0, -14.5, 252.0,
    ElimsLeftAuton
};

int drivetrainCurrentCheck(void) {
    bool driving = true;
    while (driving) {
        //std::cout << left_drive.current(percent) << ", " << right_drive.current(percent);


        wait(20, msec);
    }

    return 1;
}

void ElimsLeftAuton(void) {
    wing.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path matchloader_super_ball_epic_path_supreme = PathGenerator::GeneratePath(
    	{{46.0, -16.0},
    	 {21.5, -26.5},
    	 //{40.0, -30.0},
    	 {42.0, -47.5},
    	 {71.0, -47.5}
    	},
    	65.0,
    	25.0,
    	4.0,
    	0.35,
    	1.75
    );

    matchloader_super_ball_epic_path_supreme.waypoints[1].onReach = []() {
        matchloader.set(true);
    };

    FollowPath(matchloader_super_ball_epic_path_supreme, forward, 10.0);
    driveFor(-0.5, 100);

    wait(800, msec);

    //task checkCurrent = task(drivetrainCurrentCheck);
    driveFor(-32, 60);
    setDrivetrainSpeed(-5);

    indexer.spin(forward, 100, percent);
    hood.set(true);

    wait(1000, msec);
    intake.spin(reverse, 100, percent);
    wait(75, msec);
    intake.spin(forward, 100, percent);
    wait(1000, msec);

    driveFor(12, 100);
}