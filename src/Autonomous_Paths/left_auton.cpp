#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/color_sorting.h"

#include "Autonomous_Paths/left_auton.h"

void LeftAuton(void);

Auton leftAuton = {
    "Left Auton",
    "Gets lots of points!",
    48.25, -14.5, 180.0,
    LeftAuton
};

void LeftAuton(void) {
    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, -25.0},
    	 {48.0, -47.0},
    	 {62.0, -41.0},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, -44.5},
    	 {33.0, -44.5},
    	},
    	50.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    // Reverse intake a bit to prevent jams
    goal_path.waypoints[3].onReach = []() {
        intake.spin(reverse, 100, percent);
        wait(300, msec);
        intake.stop();
    };

    FollowPath(goal_path, reverse, 18.0);

    matchloader.set(false);
    setDrivetrainSpeed(-10);
    indexer.spin(forward, 100, percent);
    intake.spin(forward, 100, percent);
    hood.set(true);

    // Color Sort!

    color_sort_mode allianceColor = colorSortMode;
    color otherColor;

    if (allianceColor == RED) {
        otherColor = color::blue;
    } else {
        otherColor = color::red;
    }

    // Wait until we see blue, or theres a timeout
    int startScoreTime = Brain.Timer.system();
    waitUntil((color_sensor.isNearObject() && color_sensor.color() == otherColor) || (Brain.Timer.system() - startScoreTime) > 1750);

    // Stop scoring
    indexer.stop();
    intake.spin(reverse, 100, percent);
    driveFor(6, 100);

    // Bump
    hood.set(false);
    setDrivetrainSpeed(-50);
    wait(500, msec);

    /*Path three_balls_path = PathGenerator::GeneratePath(
    	{{34.0, -48.0},
    	 {44.0, -48.0},
    	 {44.0, -20.0},
         {20, -20}
    	},
    	55.0,
    	20.0,
    	3.0,
    	0.4,
    	3.5
    );*/

    Path three_balls_path = PathGenerator::GeneratePath(
    	{{36.0, -45.5},
    	 {48.0, -45.5},
         {19.5, -17.0}
    	},
    	35.0,
    	10.0,
    	3,
    	0.7,
    	2.0
    );

    intake.spin(forward, 100, percent);
    FollowPath(three_balls_path, forward, 12.0);
    matchloader.set(true);

    pointAt(6, -6, 100, reverse);

    driveFor(-16.0, 100);
    indexer.spin(reverse, 60, percent);
    wait(1500, msec);
    driveFor(24, 100);

    // Drives into matchloader and gets three balls

    //matchloader.set(true);
    //intake.spin(forward, 100, percent);
    //indexer.spin(forward, 100, percent);
    //task indexerTask = task(CheckMotorStallTask);
//
    //Path matchload_path = PathGenerator::GeneratePath(
    //	{{50.00, -17.5},
    //	 {46.0, -36.5},
    //	 {46.00, -60.5},
    //	 {69.0, -39.0}
    //	},
    //	50.0,
    //	25.0,
    //	6.0,
    //	0.8,
    //	2.5
    //);
//
    //FollowPath(matchload_path, forward, 16.0);
    //setDrivetrainSpeed(5);
//
    //wait(900, msec);
//
    //driveFor(-28, 100);
    ////driveFor(-6, 100);
    ////driveTo(28, -50, 100, reverse);
//
    //// Drives into long goal
//
    //hood.set(true);
    //indexer.spin(forward, 100, percent);
    //setDrivetrainSpeed(-100);
//
    //intake.spin(reverse, 100, percent);
    //wait(200, msec);
    //intake.spin(forward, 100, percent);
    //wait(200, msec);
//
    //driveFor(0.5, 100);
//
    //wait(500, msec);
//
    //driveFor(6, 100);
    //hood.set(false);
    //setDrivetrainSpeed(-100);
//
    //wait(600, msec);
//
    //matchloader.set(false);
//
    //// Gets three middle balls
//
    //Path middle_balls_path = PathGenerator::GeneratePath(
    //	{{32.0, -48},
    //	 {52.0, -48.0},
    //	 {52.0, -24.5},
    //	 {16.0, -26.0}
    //	},
    //	40.0,
    //	25.0,
    //	4.0,
    //	0.6,
    //	4.0
    //);
//
    //middle_balls_path.waypoints[middle_balls_path.size() - 3].onReach = []() {
    //    //wait(100, msec);
    //    matchloader.set(true);
    //};
//
    //FollowPath(middle_balls_path, forward, 16.0);
//
    ////pointAt(6, -6, 100, reverse);
    //driveTo(11.5, -15.5, 40, reverse);
    //indexer.spin(reverse, 100, percent);
    //pointAt(0, 0, 100, reverse);
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
