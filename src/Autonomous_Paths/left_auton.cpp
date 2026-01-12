#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/color_sorting.h"
#include "Robot/distance_calibration.h"

#include "Autonomous_Paths/left_auton.h"
#include <iostream>

void LeftAuton(void);

Auton leftAuton = {
    "Left Auton",
    "Gets lots of points!",
    48.25, -14.5, 180.0,
    LeftAuton
};

int forceStop() {
    int startTime = Brain.Timer.system();
    while (true) {
        if (Brain.Timer.system() - startTime >= 15000) {
            Controller.rumble("-");
            //std::cout << Brain.Timer.system() - startTime << std::endl;
        }

        wait(20, msec);
    };
    return 1;
}

void LeftAuton(void) {
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
    //task rumble = task(forceStop);

    //std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Neg, Left);
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    ResetFieldPositionFromDistanceWithOdometry();

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, -25.0},
    	 {48.0, -48.0},
    	 {63.5, -40.0},
    	},
    	50.0,
    	20.0,
    	2.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 14.0);
    setDrivetrainSpeed(10);
    //wait(1000, msec);

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, -44.5},
    	 {33.0, -44.5},
    	},
    	50.0,
    	10.0,
    	3,
    	0.6,
    	3.0
    );
    
    FollowPath(goal_path, reverse, 18.0);

    matchloader.set(false);
    setDrivetrainSpeed(-10);
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

    // Stop scoring
    hood.set(false);
    indexer_piston.set(true);

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
    	45.0,
    	10.0,
    	3,
    	0.7,
    	2.0
    );

    intake.spin(forward, 100, percent);
    FollowPath(three_balls_path, forward, 12.0);
    matchloader.set(true);
    indexer_piston.set(false);

    pointAt(8.5, -6, 100, reverse);

    driveFor(-17.5, 100);
    indexer_piston.set(true);
    wait(600, msec);
    
    left_drive.setStopping(hold);
    right_drive.setStopping(hold);

    driveTo(35, -37, 100, forward);
    driveTo(11, -37.5, 70, reverse);
}
