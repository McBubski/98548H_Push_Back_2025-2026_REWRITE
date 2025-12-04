#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "Robot/distance_calibration.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/testing_auton.h"

void TestingAuton(void);

Auton testingAuton = {
    "Testing Auton",
    "Testing stuff for Trey fr",
    -30.5, 48.0, 0,
    TestingAuton
};


void TestingAuton(void) {
    std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    Path path_1 = PathGenerator::GeneratePath(
	    {{36.0, 0},
	    {36.0, 26.0},
	    {-36.0, 26.0},
	    {-36.0, -36.0},
	    {36.0, -18.0}
	    },
	    50.0,
	    35.0,
	    6.0,
	    0.6,
	    2.75
    );

    FollowPath(path_1, forward, 16.0);
}