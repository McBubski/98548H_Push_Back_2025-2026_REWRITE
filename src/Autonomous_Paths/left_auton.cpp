#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/left_auton.h"

void LeftAuton(void);

Auton leftAuton = {
    "Left Auton",
    "Gets lots of points!",
    63.25, -15.0, 270.0,
    LeftAuton
};

void LeftAuton(void) {
    // Drives into matchloader and gets three balls

    matchloader.set(true);
    intake.spin(forward, 100, percent);

    Path matchload_path = PathGenerator::GeneratePath(
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

    FollowPath(matchload_path, forward, 16.0);
    setDrivetrainSpeed(5);

    wait(900, msec);

    driveFor(-28, 100);

    // Drives into long goal

    hood.set(true);
    indexer.spin(forward, 100, percent);
    setDrivetrainSpeed(-100);

    wait(400, msec);

    driveFor(1, 100);

    wait(600, msec);

    driveFor(6, 100);
    hood.set(false);
    setDrivetrainSpeed(-100);

    wait(600, msec);

    matchloader.set(false);

    // Gets three middle balls

    Path middle_balls_path = PathGenerator::GeneratePath(
    	{{32.0, -48},
    	 {52.0, -48.0},
    	 {52.0, -24.5},
    	 {16.0, -26.0}
    	},
    	40.0,
    	25.0,
    	6.0,
    	0.6,
    	4.0
    );

    middle_balls_path.waypoints[middle_balls_path.size() - 2].onReach = []() {
        matchloader.set(true);
    };

    FollowPath(middle_balls_path, forward, 16.0);

    //pointAt(6, -6, 100, reverse);
    driveTo(12, -14, 40, reverse);
}

/*
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
*/
