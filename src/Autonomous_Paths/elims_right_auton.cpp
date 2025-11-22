#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/elims_right_auton.h"

void ElimsRightAuton(void);

Auton elimsRightAuton = {
    "Elim Right",
    "Eat that trash teams",
    50.0, 14.5, 288.0,
    ElimsRightAuton
};

void ElimsRightAuton(void) {
    wing.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path three_super_ball_epic_path_supreme = PathGenerator::GeneratePath(
    	{{46.0, 16.0},
    	 {21.5, 26.5},
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
    	{{36.0, 32.0},
    	 {40.0, 48.5},
         {60, 45.5},
    	 {71.5, 44.0}
    	},
    	40.0,
    	25.0,
    	4.0,
    	0.5,
    	2.5
    );

    FollowPath(swag_coolio_matchloader_awesome_sauce_path, forward, 14);

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
    	{{48, 48.00},
    	 {44.0, 59.0},
    	 {9.0, 52.0}
    	},
    	40.0,
    	25.0,
    	4.0,
    	0.5,
    	1.5
    );

    wing.set(false);
    FollowPath(epic_wing_awesome, reverse, 16);

    pointAt(24, 48, 100, forward);

    left_drive.stop(hold);
    right_drive.stop(hold);
    intake.stop();
    indexer.stop();
}