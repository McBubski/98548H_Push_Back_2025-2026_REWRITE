#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Autonomous_Paths/solo_awp.h"

void SoloAWPAuton(void);

Auton soloAWPAuton = {
    "Solo AWP",
    "Eat that trash teams",
    63.25, 15.0, 270.0,
    SoloAWPAuton
};


void SoloAWPAuton(void) {
    // Score in middle goal

    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.00, 17.5},
    	 {40.0, 33.5},
    	 {48.00, 54},
    	 {72.0, 41}
    	},
    	50.0,
    	25.0,
    	6.0,
    	0.8,
    	2.5
    );

    FollowPath(matchload_path, forward, 16.0);
    setDrivetrainSpeed(5);

    wait(900, msec);

    driveFor(-28, 100);

    hood.set(true);
    indexer.spin(forward, 100, percent);
    setDrivetrainSpeed(-100);

    wait(200, msec);

    driveFor(1, 100);

    wait(600, msec);

    Path middle_goal_ball_path = PathGenerator::GeneratePath(
    	{{37.76, 48.59},
    	 {48.29, 47.71},
    	 {48.88, 33.95},
    	 {31.34, 32.20},
    	 {26.00, 24.00},
    	 {27.00, -26.00}
    	},
    	60.0,
    	70.0,
    	6.0,
    	0.6019,
    	3.0
    );

    FollowPath(middle_goal_ball_path, forward, 16.0);

    //driveTo(-13, -13, 100, reverse);

    wait(300, msec);
    pointAt(0, 0, 100, reverse);
    driveFor(-19, 100);
    indexer.spin(forward, 55, percent);
    hood.set(true);
    driveFor(2, 30);

    wait(1000, msec);

    indexer.stop();
    hood.set(false);
    matchloader.set(true);

    // Get other matchloader

    Path matchload_path_1 = PathGenerator::GeneratePath(
    	{{48.00, -17.5},
    	 {40.0, -33.5},
    	 {48.00, -55.5},
    	 {72.0, -45}
    	},
    	50.0,
    	25.0,
    	6.0,
    	0.8,
    	2.5
    );

    FollowPath(matchload_path_1, forward, 16.0);
    setDrivetrainSpeed(5);

    wait(900, msec);

    driveFor(-28, 100);

    left_drive.spin(reverse, 100, percent);
    right_drive.spin(reverse, 100, percent);

    hood.set(true);
    indexer.spin(forward, 100, percent);
    setDrivetrainSpeed(-100);

    wait(100, msec);

    driveFor(1, 100);

    wait(600, msec);
}