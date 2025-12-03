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
    	 {47.0, 33.5},
    	 {48.00, 54.5},
    	 {72.25, 39.0}
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

    intake.spin(reverse, 100, percent);
    wait(200, msec);
    intake.spin(forward, 100, percent);
    wait(200, msec);

    driveFor(0.5, 100);

    wait(500, msec);

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
    	 {28.0, 24.0},
    	 {16, 12.0}
    	},
    	35.0,
    	25.0,
    	6.0,
    	0.6,
    	3.0
    );

    middle_balls_path.waypoints[7].onReach = []() {
        matchloader.set(true);
    };

    FollowPath(middle_balls_path, forward, 16.0);
    driveFor(6, 30);
    pointAt(10.25, 6, 100, forward);
    matchloader.set(false);
    driveFor(7.5, 30);
    intake.spin(reverse, 100, percent);
}