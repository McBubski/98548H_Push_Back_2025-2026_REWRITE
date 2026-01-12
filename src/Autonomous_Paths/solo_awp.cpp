#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/distance_calibration.h"
#include "Robot/color_sorting.h"

#include "Autonomous_Paths/solo_awp.h"
#include <iostream>
void SoloAWPAuton(void);

Auton soloAWPAuton = {
    "Solo AWP",
    "Eat that trash teams",
    48.25, 14.5, 0.0,
    SoloAWPAuton
};


void SoloAWPAuton(void) {
    // Score in middle goal

    std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos, Right);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    matchloader.set(true);
    intake.spin(forward, 100, percent);

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
    wait(450, msec);

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 44.5},
    	 {28.5, 44.0},
    	},
    	50.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    // Reverse intake a bit to prevent jams
   /*
    goal_path.waypoints[3].onReach = []() {
        std::cout << "Reversing!" << std::endl;
        intake.spin(reverse, 100, percent);
        wait(300, msec);
        intake.stop();
    };
    */
    FollowPath(goal_path, reverse, 18.0);

    matchloader.set(false);
    //setDrivetrainSpeed(-10);
    //indexer_piston.set(true);
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


    while (scoring) {
        if ((Brain.Timer.system() - startScoreTime) > 3000) {
            scoring = false;
        }
        if (otherColor == blue) {
            if (color_sensor.color() == blue) {
                scoring = false;
            }
        } else if (otherColor == red) {
            if (color_sensor.color() == red) {
                scoring = false;
            }
        }
        wait(5, msec);
    }

    std::cout << "sorted" << std::endl;

    // Stop scoring
/*  indexer_piston.set(false);
    intake.spin(forward, 100, percent);
    driveFor(6, 100);

    // Bump
    hood.set(false);
    setDrivetrainSpeed(-50);
    wait(500, msec);
*/
    hood.set(false);
    indexer_piston.set(true);
    Path middle_goal_ball_path = PathGenerator::GeneratePath(
    	{{35.76, 48.59},
    	 {46.29, 47.71},
    	 {44.88, 33.95},
    	 {27.34, 32.20},
    	 {20.50, 24.00},
    	 {24.00, -30.00}
    	},
    	60.0,
    	70.0,
    	6.0,
    	0.6019,
    	3.0
    );
    
    middle_goal_ball_path.waypoints[3].onReach = []() {
        indexer_piston.set(false);
    };
    middle_goal_ball_path.waypoints[4].onReach = []() {
        indexer_piston.set(false);
    };
    middle_goal_ball_path.waypoints[5].onReach = []() {
        indexer_piston.set(false);
    };


    FollowPath(middle_goal_ball_path, forward, 16.0);

    //driveTo(-13, -13, 100, reverse);

    //wait(300, msec);
    pointAt(3.0, -3.5, 100, reverse);

    driveFor(-17.5, 100);
    indexer_piston.set(true);
    wait(600, msec);

    matchloader.set(true);
    indexer_piston.set(false);
    // Get other matchloader

    Path matchload_path_1 = PathGenerator::GeneratePath(
    	{{35.0, -33.5},
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
    //setDrivetrainSpeed(5);

    wait(450, msec);

    Path goal_path1 = PathGenerator::GeneratePath(
    	{{56.0, -44.5},
    	 {32.5, -51.0},
    	},
    	50.0,
    	10.0,
    	3,
    	0.6,
    	3.0
    );
    
    FollowPath(goal_path1, reverse, 18.0);

    matchloader.set(false);
    //setDrivetrainSpeed(-10);
    intake.spin(forward, 100, percent);
    hood.set(true);

    /* Color Sort!

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


    //while (scoring) {
    //    if ((Brain.Timer.system() - startScoreTime) > 1750) {
    //        scoring = false;
    //    }
    //    if (otherColor == blue) {
    //        if (color_sensor.color() == blue) {
    //            scoring = false;
    //        }
    //    } else if (otherColor == red) {
    //        if (color_sensor.color() == red) {
    //            scoring = false;
    //        }
    //    }
    //    wait(20, msec);
    //}
    */
    
    wait(1250, msec);

}