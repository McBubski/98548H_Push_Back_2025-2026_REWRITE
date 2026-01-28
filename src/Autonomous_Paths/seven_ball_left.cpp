#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/color_sorting.h"
#include "Robot/distance_calibration.h"

#include "Autonomous_Paths/seven_ball_left.h"
#include <iostream>

void SevenBallLeftAuton(void);

Auton sevenBallLeftAuton = {
    "7-Ball Left",
    "Gets lots of points!",
    50.0, -14.5, 252.0,
    SevenBallLeftAuton
};

void SevenBallLeftAuton(void) {
    Path three_balls_path = PathGenerator::GeneratePath(
    	{{40.0, -17.50},
    	 {22.0, -24.0},
    	},
    	50.0,
    	15.0,
    	4.0,
    	0.8,
    	3.0
    );

    three_balls_path.waypoints[2].onReach = []() {
        matchloader.set(true);
    };

    intake.spin(forward, 100, percent);

    FollowPath(three_balls_path, forward, 20);

    turnToHeading(135, 100);

    Path matchloader_path = PathGenerator::GeneratePath(
    	{{34.0, -35.0},
    	 {43.0, -44.5},
    	 {67.0, -47.0}
    	},
    	50.0,
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
    	{{56.0, -47.5},
    	 {34.0, -48.5},
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

    wait(1500, msec);
    driveFor(2, 100);
    hood.set(false);
    setDrivetrainSpeed(-20);
    wait(1000, msec);

    left_drive.stop(hold);
    right_drive.stop(hold);
}