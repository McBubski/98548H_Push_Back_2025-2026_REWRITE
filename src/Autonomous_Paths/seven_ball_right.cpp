#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/color_sorting.h"
#include "Robot/distance_calibration.h"

#include "Autonomous_Paths/seven_ball_right.h"
#include <iostream>

void SevenBallRightAuton(void);

Auton sevenBallRightAuton = {
    "7-Ball Right",
    "Gets lots of points!",
    50.0, 14.5, 288.5,
    SevenBallRightAuton
};

void SevenBallRightAuton(void) {
    Path three_balls_path = PathGenerator::GeneratePath(
    	{{40.0, 17.50},
    	 {19.0, 21.0},
    	},
    	50.0,
    	15.0,
    	4.0,
    	0.8,
    	3.0
    );

    three_balls_path.waypoints[3].onReach = []() {
        matchloader.set(true);
    };

    intake.spin(forward, 100, percent);

    FollowPath(three_balls_path, forward, 20);

    turnToHeading(45, 100);

    Path matchloader_path = PathGenerator::GeneratePath(
    	{{34.0, 35.0},
    	 {43.0, 42.5},
    	 {67.0, 44.5}
    	},
    	55.0,
    	15.0,
    	3.0,
    	0.55,
    	0.75
    );

    FollowPath(matchloader_path, forward, 18);

    setDrivetrainSpeed(5);
    wait(600, msec);

    //std::vector<double> positionEstimate = EstimatePositionWithDistance(X_Pos, Right);
    //std::cout << "Estimate: " << positionEstimate[0] << ", " << positionEstimate[1] << std::endl;
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 47.5},
    	 {33.5, 46.5},
    	},
    	50.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    FollowPath(goal_path, reverse, 18.0);

    setDrivetrainSpeed(-2);
    hood.set(true);
    wait(200, msec);
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
        if ((Brain.Timer.system() - startScoreTime) > 1500) {
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

    intake.spin(reverse, 100, percent);
    driveFor(2, 100);
    hood.set(false);
    setDrivetrainSpeed(-20);
    wait(1000, msec);

    left_drive.stop(hold);
    right_drive.stop(hold);
}