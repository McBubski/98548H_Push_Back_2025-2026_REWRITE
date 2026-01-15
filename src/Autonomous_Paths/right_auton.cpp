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

    //std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos);
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    
    if (color_sensor.isNearObject()) {
        color allianceColor = color_sensor.color();
        if (allianceColor == red) {
            colorSortMode = RED;
        } else if (allianceColor == blue) {
            colorSortMode = BLUE;
        }
    }

    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos, Right);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {48.0, 46.0},
    	 {66.0, 40.5},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);
    //setDrivetrainSpeed(10);
    //wait(500, msec);

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 44.5},
    	 {30.0, 43.0},
    	},
    	50.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    // Reverse intake a bit to prevent jams
    /*goal_path.waypoints[3].onReach = []() {
        std::cout << "Reversing!" << std::endl;
        intake.spin(reverse, 100, percent);
        wait(300, msec);
        intake.stop();
    };*/

    FollowPath(goal_path, reverse, 18.0);

    intake.spin(forward, 100, percent);
    hood.set(true);
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
    //driveFor(6, 100);
    //setDrivetrainSpeed(-100);
    hood.set(false);
    //wait(400, msec);
    indexer_piston.set(true);

    Path middle_ball_path = PathGenerator::GeneratePath(
    	{{36.0, 47.5},
    	 {48.0, 46.5},
         {9.0, 6.5}
    	},
    	45.0,
    	15.0,
    	3,
    	0.7,
    	2.0
    );

    middle_ball_path.waypoints[16].onReach = []() {
        std::cout << "reached" << std::endl;
        intake.stop();
        intake_low.spin(forward, 100, percent);

        indexer_piston.set(false);
    };

    matchloader.set(false);
    FollowPath(middle_ball_path, forward, 14.0);
    intake.spin(reverse, 35, percent);
    driveFor(3.0, 100);
    wait(500, msec);

    // Wing

    Path reverse_path = PathGenerator::GeneratePath(
	    {{15.5, 15.0},
	     {29.0, 25.0},
	     {34.0, 32.0},
	     {42.0, 35.0}
	    },
	    40.0,
	    10.0,
	    3.0,
	    0.2,
	    4.0
    );

    FollowPath(reverse_path, reverse, 14.0);

    //driveFor(-34, 100);
    //turnToHeading(270, 100);

    Path wing_path = PathGenerator::GeneratePath(
	    {{37.5, 42.5},
         {26.0, 39.0},
         {16.0, 38.0},
         {3, 33.0}
	    },
	    40.0,
	    5.0,
	    3.0,
	    0.4,
	    2.0
    );

    FollowPath(wing_path, forward, 18.0);
    turnToHeading(270, 100);
}