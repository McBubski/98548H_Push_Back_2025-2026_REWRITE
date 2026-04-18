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
    50.5, 17.75, 0.0,
    SoloAWPAuton
};

int matchloader_hack (void) {
    std::cout << "Start" << std::endl;
    wait(1800, msec);
    std::cout << "Driving forward" << std::endl;
    Controller.rumble("---");
    RAT_Interrupt = true;
    setDrivetrainSpeed(50);
    wait(400, msec);
    std::cout << "Stop!" << std::endl;

    return 1;
}

int far_matchloader_hack(void) {
    std::cout << "Start" << std::endl;
    wait(1500, msec);
    std::cout << "Stop!" << std::endl;
    RAT_Interrupt = true;

    return 1;    
}

int timerEst(void) {
    wait(15, seconds);
    std::cout << ("DING DING DING") << std::endl;
    Controller.rumble(".");

    return 1;
}

void SoloAWPAuton(void) {
    task t = task(timerEst);

    if (color_sensor.isNearObject()) {
        color allianceColor = color_sensor.color();
        if (allianceColor == red) {
            colorSortMode = RED;
        } else if (allianceColor == blue) {
            colorSortMode = BLUE;
        }
    }

    // Score in middle goal

    //std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos, Right);
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    matchloader.set(true);
    hood.set(true);

    intake.spin(forward, 100, percent);

    // Bad solution to fix matchloader overdriving.
    task matchloader_hacky_fix = task(matchloader_hack);

    /*Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {48.0, 45.0},
    	 {65.5, 39.0},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );                       Might be important*/

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {50.0, 41.0},
    	 {73.0 /*almost nice*/, 48.5},
    	},
    	55.0,
    	20.0,
    	3.0,
    	0.55,
    	0.4
    );

    std::cout << "Finished path" << std::endl;
    FollowPath(matchload_path, forward, 12.0);
    setDrivetrainSpeed(70);
    wait(50, msec);
    std::cout << "Finished driving" << std::endl;

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 50.0},
    	 {29.5, 48.0},
    	},
    	55.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    FollowPath(goal_path, reverse, 18.0);

    intake.spin(forward, 100, percent);
    indexer_piston.set(true);
    setDrivetrainSpeed(-50);
    wait(100, msec);
    setDrivetrainSpeed(0);

    // Color Sort!

    color_sort_mode allianceColor = colorSortMode;
    color otherColor;

    if (allianceColor == RED) {
        otherColor = color::blue;
    } else {
        otherColor = color::red;
    }

    int startScoreTime = Brain.Timer.system();
    bool scoring = true;

    while (scoring) {
        if ((Brain.Timer.system() - startScoreTime) > 1100) {
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

    indexer_piston.set(true);
    wing.set(true);
    matchloader.set(false);

    Path middle_goal_ball_path = PathGenerator::GeneratePath(
	    {{34.0, 49.0},
	     {30.0, 34.0},
	     {29.0, 28.5},
	     {24.0, -28.0}
	    },
	    80.0,
	    25.0,
	    6.0,
	    0.40,
	    3.5
    );

    middle_goal_ball_path.waypoints[7].onReach = []() {
        indexer_piston.set(false);
        hood.set(false);
    };

    middle_goal_ball_path.waypoints[14].onReach = []() {
        matchloader.set(true);
    };

    FollowPath(middle_goal_ball_path, forward, 16.0);

    //driveTo(-13, -13, 100, reverse);

    //wait(300, msec);

    // Middle goal
    pointAt(2.5, 1.0, 80, reverse);
    //pointAt(0.0, -5.5, 100, reverse);

    intake.stop();
    //driveFor(-15.0, 100);
    setDrivetrainSpeed(-30);
    indexer_piston.set(true);
    wait(750, msec);
    
    intake.spin(forward, 70, percent);
    setDrivetrainSpeed(0);
    wait(600, msec);

    matchloader.set(true);
    indexer_piston.set(false);
    hood.set(true);
    // Get other matchloader

    Path matchload_path_2 = PathGenerator::GeneratePath(
    	{{16, -12},
         {35.0, -31.5},
    	 {48.0, -48.5},
    	 {75.5, -43.5}
    	},
    	80.0,
    	20.0,
    	6.0,
    	0.8,
    	2.5
    );

    matchload_path_2.waypoints[1].onReach = []() {
        task hack_2 = task(far_matchloader_hack);
    };

    FollowPath(matchload_path_2, forward, 20.0);
    //setDrivetrainSpeed(5);

    //wait(450, msec);

    Path goal_path1 = PathGenerator::GeneratePath(
    	{{56.0, -47.0},
    	 {27.0, -46.25},
    	},
    	50.0,
    	10.0,
    	3,
    	0.6,
    	3.0
    );
    
    FollowPath(goal_path1, reverse, 18.0);

    matchloader.set(false);
    indexer_piston.set(true);
    setDrivetrainSpeed(-5);
    wait(150, msec);
    intake.spin(forward, 100, percent);

    wait(1250, msec);
}