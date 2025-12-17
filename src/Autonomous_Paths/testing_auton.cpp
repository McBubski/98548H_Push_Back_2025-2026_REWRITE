#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "Robot/distance_calibration.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/testing_auton.h"

void TestingAuton(void);

Auton testingAuton = {
    "Testing Auton",
    "Testing stuff for Trey fr",
    0, 0, 270,
    TestingAuton
};


void TestingAuton(void) {
	std::vector<double> positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
}