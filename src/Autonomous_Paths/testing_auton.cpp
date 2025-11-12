#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/testing_auton.h"

void TestingAuton(void);

Auton testingAuton = {
    "Testing Auton",
    "Testing stuff for Trey fr",
    -36, -36, 0,
    TestingAuton
};


void TestingAuton(void) {
Path path_1 = PathGenerator::GeneratePath(
	{{-36.59, -18.15},
	 {-36.88, 12.00},
	 {-48.88, 36.00},
	 {-48.00, 57.37},
	 {-24.59, 60.00},
	 {-0.29, 59.71},
	 {35.41, 59.12},
	 {47.32, 34.29}
	},
	60.0,
	20.0,
	6.0,
	0.65,
	3.0
);

FollowPath(path_1, forward, 24.0);

pointAt(28, 31, 100, forward);

intake.spin(forward, 100, percent);
wing.set(true);

Path path_2 = PathGenerator::GeneratePath(
	{{28.68, 31.02},
	 {-4.10, 31.90},
	 {-57.37, 35.71},
	 {-67.02, 31.02},
	 {-65.27, -16.80}
	},
	55.0,
	30.0,
	6.0,
	0.65,
	3.0
);

FollowPath(path_2, forward, 24.0);
}