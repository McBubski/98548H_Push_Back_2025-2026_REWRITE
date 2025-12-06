#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/distance_calibration.h"
#include "Robot/color_sorting.h"

#include "Autonomous_Paths/right_auton.h"
#include <iostream>

void RightAuton(void);

Auton rightAuton = {
    "Right Auton",
    "Gets lots of points!",
    48.25, 14.5, 0.0,
    RightAuton
};


void RightAuton(void) {
    // Drives into matchloader and gets three balls

    std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {48.0, 46.0},
    	 {64.5, 39.5},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);
    setDrivetrainSpeed(10);
    wait(1250, msec);

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 43.5},
    	 {30.0, 43.5},
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
    
    indexer.spin(reverse, 100, percent);
    intake.spin(reverse, 100, percent);
    driveFor(6, 100);

    // Bump
    hood.set(false);
    setDrivetrainSpeed(-50);
    wait(500, msec);

    Path middle_ball_path = PathGenerator::GeneratePath(
    	{{36.0, 47.5},
    	 {48.0, 46.5},
         {11.5, 7.0}
    	},
    	30.0,
    	10.0,
    	3,
    	0.7,
    	2.0
    );

    middle_ball_path.waypoints[2].onReach = []() {
        indexer.spin(reverse, 100, percent);
    };

    middle_ball_path.waypoints[4].onReach = []() {
        indexer.stop();
    };

    intake.stop();
    intake_low.spin(forward, 100, percent);
    FollowPath(middle_ball_path, forward, 14.0);
    intake.spin(reverse, 100, percent);
    driveFor(3, 100);

    wait(2000, msec);
    intake.stop();

    //setDrivetrainSpeed(5);
//
    //wait(900, msec);
//
    //driveFor(-6, 50);
    //pointAt(24, 47, 100, reverse);
    //driveFor(-24, 49);
//
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
    //intake.stop();
    //intake_low.spin(forward, 100, percent);
//
    //// Gets three middle balls
//
    //Path middle_balls_path = PathGenerator::GeneratePath(
    //	{{32.0, 44},
    //	 {50.0, 44.0},
    //	 {28.0, 24.0},
    //	 {16, 12.0}
    //	},
    //	35.0,
    //	25.0,
    //	6.0,
    //	0.6,
    //	3.0
    //);
//
    //middle_balls_path.waypoints[7].onReach = []() {
    //    matchloader.set(true);
    //};
//
    //FollowPath(middle_balls_path, forward, 16.0);
    //driveFor(6, 30);
    //pointAt(10.25, 6, 100, forward);
    //matchloader.set(false);
    //driveFor(7.5, 30);
    //intake.spin(reverse, 100, percent);
}