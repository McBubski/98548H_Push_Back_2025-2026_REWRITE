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
    	 {40.0, -47.5},
         {50, -48.5},
    	 {60.0, -49.0}
    	},
    	35.0,
    	25.0,
    	4.0,
    	0.8,
    	2.0
    );

    FollowPath(swag_coolio_matchloader_awesome_sauce_path, forward, 12);

    //driveTo(48, -48, 100, forward);
    //turnToHeading(90, 100);
    pointAt(70, -48, 100, forward);
    driveFor(12, 100);

    //driveFor(1, 100);
//
    //wait(400, msec);
    pointAt(24,-48, 100, reverse);

    driveFor(-32, 60);
    setDrivetrainSpeed(-5);
    hood.set(true);

    intake.spin(forward, 100, percent);
    wait(50, msec);
    intake.spin(reverse, 100, percent);
    wait(250, msec);
    position_tracking.SetPosition(30.5, -48.0, 90);
    indexer.spin(forward, 100, percent);
    intake.spin(forward, 100, percent);
    wait(1900, msec);

    //hood.set(true);
    matchloader.set(false);
    indexer.stop();
    intake.stop();
    driveFor(18, 100);
    //hood.set(false);

    Path epic_wing_awesome = PathGenerator::GeneratePath(
    	{{46, -37.0},
    	 {45.0, -35.0},
         {30.0, -36.0},
    	 {9.0, -46}
    	},
    	40.0,
    	25.0,
    	4.0,
    	0.45,
    	1.5
    );

    wing.set(false);
    FollowPath(epic_wing_awesome, reverse, 16);
    driveFor(-6, 100);

    pointAt(24, -48, 100, forward);

    left_drive.stop(hold);
    right_drive.stop(hold);
    intake.stop();
    indexer.stop();
}

