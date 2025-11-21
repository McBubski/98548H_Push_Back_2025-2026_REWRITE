#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/right_auton.h"

void RightAuton(void);

Auton rightAuton = {
    "Right Auton",
    "Gets lots of points!",
    63.25, 15.0, 270.0,
    RightAuton
};


void RightAuton(void) {
    // Drives into matchloader and gets three balls

    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.00, 17.5},
    	 {40.0, 33.5},
    	 {48.00, 54.5},
    	 {72.0, 39.0}
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

    driveFor(-6, 50);
    pointAt(24, 47, 100, reverse);
    driveFor(-24, 49);


    // Drives into long goal

    hood.set(true);
    indexer.spin(forward, 100, percent);
    setDrivetrainSpeed(-100);

    wait(400, msec);

    driveFor(0.5, 100);

    wait(800, msec);

    driveFor(6, 100);
    hood.set(false);
    setDrivetrainSpeed(-100);

    wait(600, msec);

    matchloader.set(false);
    intake.stop();
    intake_low.spin(forward, 100, percent);

    // Gets three middle balls

    Path middle_balls_path = PathGenerator::GeneratePath(
    	{{32.0, 44},
    	 {50.0, 44.0},
    	 {32.0, 24.0},
    	 {8.0, 7.0}
    	},
    	35.0,
    	25.0,
    	6.0,
    	0.6,
    	3.0
    );

    FollowPath(middle_balls_path, forward, 16.0);
    intake_low.spin(reverse, 100, percent);
}