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
    50.5, 18.0, 0.0,
    RightAuton
};


void RightAuton(void) {
    // Drives into matchloader and gets three balls

    //std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos);
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    
    // Auto detect color
    if (color_sensor.isNearObject()) {
        color allianceColor = color_sensor.color();
        if (allianceColor == red) {
            colorSortMode = RED;
        } else if (allianceColor == blue) {
            colorSortMode = BLUE;
        }
    }

    // Start auton

    matchloader.set(true);
    hood.set(true);

    intake.spin(forward, 100, percent);

    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    //std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos, Right);
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    // Drive to matchloader
    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {48.0, 46.0},
    	 {73.0 /*almost nice*/, 44.5},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);

    // Drive to goal
    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 47.5},
    	 {30.0, 48.5},
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
    wait(300, msec);
    setDrivetrainSpeed(0);

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
        if ((Brain.Timer.system() - startScoreTime) > 1000) {
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
    //driveFor(6, 100);
    //setDrivetrainSpeed(-100);
    //wait(400, msec);
    //indexer_piston.set(false);

    // Drive to middle
    Path middle_ball_path = PathGenerator::GeneratePath(
    	{{36.0, 47.5},
    	 {48.0, 46.5},
         {11.0, 13.0}
    	},
    	55.0,
    	15.0,
    	3,
    	0.7,
    	2.0
    );

    middle_ball_path.waypoints[16].onReach = []() {
        std::cout << "reached" << std::endl;
        intake.stop();
        intake_low.spin(forward, 100, percent);

        //indexer_piston.set(false);
    };

    matchloader.set(false);
    FollowPath(middle_ball_path, forward, 14.0);
    intake.spin(reverse, 55, percent);
    low_goal_BS.set(true);
    turnToHeading(225, 100);
    driveFor(4.0, 100);
    //driveFor(-4.0, 100);
    //driveFor(2, 100);   


    // Wing

    Path reverse_path = PathGenerator::GeneratePath(
	    {{15.5, 19.0},
	     {29.0, 28.0},
	     {34.0, 38.0},
	     {42.0, 47.0}
	    },
	    50.0,
	    10.0,
	    3.0,
	    0.2,
	    4.0
    );

    FollowPath(reverse_path, reverse, 14.0);

    low_goal_BS.set(false);

    //driveFor(-34, 100);
    //turnToHeading(270, 100);

    Path wing_path = PathGenerator::GeneratePath(
	    {{37.5, 47.5},
         {26.0, 45.0},
         {16.0, 44.0},
         {10, 41.0}
	    },
	    40.0,
	    5.0,
	    3.0,
	    0.4,
	    2.0
    );

    FollowPath(wing_path, forward, 18.0);
    turnToHeading(250, 100); // 250
    //turnToHeading(265, 100);
}