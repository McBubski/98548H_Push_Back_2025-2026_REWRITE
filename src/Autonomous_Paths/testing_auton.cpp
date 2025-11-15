#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/testing_auton.h"

void TestingAuton(void);

Auton testingAuton = {
    "Testing Auton",
    "Testing stuff for Trey fr",
    36, 36, 0,
    TestingAuton
};


void TestingAuton(void) {
    hood.set(true);
	intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
}