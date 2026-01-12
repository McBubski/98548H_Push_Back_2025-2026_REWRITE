#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/distance_calibration.h"
#include "Robot/color_sorting.h"

#include "Autonomous_Paths/elims_right_auton.h"
#include <iostream>

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

    ResetFieldPositionFromDistanceWithOdometry();

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
         {60, 46},
    	 {71.5, 47.0}
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
    wait(100, msec);
Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 47.5},
    	 {34.0, 50.0},
    	},
    	50.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    // Reverse intake a bit to prevent jams
    goal_path.waypoints[3].onReach = []() {
        std::cout << "Reversing!" << std::endl;
        intake.spin(reverse, 100, percent);
        wait(100, msec);
        intake.stop();
    };

    FollowPath(goal_path, reverse, 18.0);
    hood.set(true);

    intake.spin(forward, 100, percent);
    //wait(50, msec);
    //intake.spin(reverse, 100, percent);
    //wait(250, msec);
    //indexer.spin(forward, 100, percent);
    //intake.spin(forward, 100, percent);
    wait(1750, msec);

    matchloader.set(false);
    driveFor(18, 100);
    hood.set(false);

    Path epic_wing_awesome = PathGenerator::GeneratePath(
    	{{48, 48.00},
    	 {44.0, 60.0},
         {38.0, 58},
    	 {12.0, 56.0}
    	},
    	40.0,
    	25.0,
    	4.0,
    	0.5,
    	1.5
    );

    wing.set(false);
    FollowPath(epic_wing_awesome, reverse, 16);

    //pointAt(24, 48, 100, forward);

    left_drive.stop(hold);
    right_drive.stop(hold);
    intake.stop();
    indexer.stop();
}