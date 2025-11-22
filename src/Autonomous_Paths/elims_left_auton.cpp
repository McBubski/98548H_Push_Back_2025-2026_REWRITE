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

    Path three_super_ball_epic_path_supreme = PathGenerator::GeneratePath(
    	{{46.0, -16.0},
    	 {21.5, -26.5},
    	 //{40.0, -30.0},
    	 //{42.0, -48},
    	 //{32.0, -36}
    	},
    	75.0,
    	25.0,
    	4.0,
    	0.35,
    	0.025
    );

    three_super_ball_epic_path_supreme.waypoints[1].onReach = []() {
        matchloader.set(true);
    };

    FollowPath(three_super_ball_epic_path_supreme, forward, 12.0);
    //pointAt(72, -48, 100, forward);

    Path swag_coolio_matchloader_awesome_sauce_path = PathGenerator::GeneratePath(
    	{{36.0, -32.0},
    	 {40.0, -48.5},
         {60, -46.0},
    	 {72.5, -47.0}
    	},
    	40.0,
    	25.0,
    	4.0,
    	0.5,
    	1.8
    );

    FollowPath(swag_coolio_matchloader_awesome_sauce_path, forward, 12);

    //driveFor(1, 100);
//
    wait(400, msec);

    driveFor(-32, 60);
    setDrivetrainSpeed(-5);
    hood.set(true);

    intake.spin(forward, 100, percent);
    wait(50, msec);
    intake.spin(reverse, 100, percent);
    wait(250, msec);
    indexer.spin(forward, 100, percent);
    intake.spin(forward, 100, percent);
    wait(1900, msec);

    matchloader.set(false);
    driveFor(18, 100);
    hood.set(false);

    Path epic_wing_awesome = PathGenerator::GeneratePath(
    	{{46, -48.00},
    	 {39.0, -41},
    	 {8.0, -46}
    	},
    	40.0,
    	25.0,
    	4.0,
    	0.5,
    	1.5
    );

    wing.set(false);
    FollowPath(epic_wing_awesome, reverse, 16);

    pointAt(24, -48, 100, forward);

    left_drive.stop(hold);
    right_drive.stop(hold);
    intake.stop();
    indexer.stop();
}