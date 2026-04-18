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

int matchloader_hack_seven_ball_right (void) {
    std::cout << "Start" << std::endl;
    wait(1800, msec);
    std::cout << "Stop!" << std::endl;
    RAT_Interrupt = true;

    return 1;
}

void SevenBallRightAuton(void) {
    Path three_balls_path = PathGenerator::GeneratePath(
    	{{40.0, 17.50},
    	 {24.0, 24.0},
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
    	 {43.0, 45.5},
    	 {67.0, 47.5}
    	},
    	55.0,
    	15.0,
    	3.0,
    	0.55,
    	0.75
    );

    task hacky_sacky = task(matchloader_hack_seven_ball_right);
    FollowPath(matchloader_path, forward, 18);

    //std::vector<double> positionEstimate = EstimatePositionWithDistance(X_Pos, Right);
    //std::cout << "Estimate: " << positionEstimate[0] << ", " << positionEstimate[1] << std::endl;
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, 47.0},
    	 {28.5, 47.0},
    	},
    	50.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    FollowPath(goal_path, reverse, 18.0);

    setDrivetrainSpeed(-2);
    indexer_piston.set(true);
    hood.set(true);
    wait(200, msec);
    setDrivetrainSpeed(0);

    // Color Sort!

    left_drive.stop(hold);
    right_drive.stop(hold);

    wait(1500, msec);
    indexer_piston.set(false);
}