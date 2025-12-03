#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "Robot/distance_calibration.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/testing_auton.h"

void TestingAuton(void);

Auton testingAuton = {
    "Testing Auton",
    "Testing stuff for Trey fr",
    -30.5, 48.0, 180,
    TestingAuton
};


void TestingAuton(void) {
    Path first_long_goal_path = PathGenerator::GeneratePath(
    	{{0.0, 0.0},
         {-30.5, 48.0}
    	},
    	40.0,
    	15.0,
    	6.0,
    	0.35,
    	2.5
    );

    FollowPath(first_long_goal_path, forward, 20.0);
    //std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Neg);
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
}