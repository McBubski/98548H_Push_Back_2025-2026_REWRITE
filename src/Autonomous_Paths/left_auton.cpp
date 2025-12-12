#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/color_sorting.h"

#include "Autonomous_Paths/left_auton.h"
#include <iostream>

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
    	 {48.0, -48.0},
    	 {63.5, -38.0},
    	},
    	50.0,
    	20.0,
    	2.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 14.0);
    setDrivetrainSpeed(10);
    wait(1000, msec);

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, -40.5},
    	 {33.0, -39.5},
    	},
    	50.0,
    	10.0,
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
    bool scoring = true;

    color_sensor.objectDetectThreshold(1);

    while (scoring) {
        if (color_sensor.color() == red) {
            std::cout << "Seeing: Red"  << std::endl;
        } else if (color_sensor.color() == blue) {
            std::cout << "Seeing: Blue"  << std::endl;
        } else if (color_sensor.color() == cyan) {
            std::cout << "Seeing: Cyan"  << std::endl;
        } else {
            std::cout << "Seeing: None"  << std::endl;
        }

        if (color_sensor.isNearObject())  {
            if ((Brain.Timer.system() - startScoreTime) > 1750) {
                scoring = false;
            }

            if (otherColor == blue) {
                if (color_sensor.color() == blue || color_sensor.color() == cyan) {
                    scoring = false;
                }
            } else if (otherColor == red) {
                if (color_sensor.color() == red) {
                    scoring = false;
                }
            }
        } else {
            scoring = false;
        }
        wait(20, msec);
    }

    std::cout << "sorted" << std::endl;

    // Stop scoring
    indexer_piston.set(false);
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

    three_balls_path.waypoints[1].onReach = []() {
        std::cout << "Stop indexer (1)" << std::endl;
        indexer.stop();
    };

    three_balls_path.waypoints[2].onReach = []() {
        std::cout << "Stop indexer (2)" << std::endl;
        indexer.stop();
    };

    intake.spin(forward, 100, percent);
    FollowPath(three_balls_path, forward, 12.0);
    matchloader.set(true);

    pointAt(6.5, -6, 100, reverse);

    driveFor(-17.5, 100);
    indexer_piston.set(true);
    wait(1500, msec);
    
    Path final_matchloader_path = PathGenerator::GeneratePath(
    	{{24.0, -24.0},
    	 {42.0, -45.0},
         {64.0, -46.0}
    	},
    	60.0,
    	25.0,
    	6.0,
    	0.5,
    	2.0
    );

    FollowPath(final_matchloader_path, forward, 16.0);
}
