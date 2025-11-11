#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"

#include "Autonomous_Paths/elims_left_auton.h"

void ElimsLeftAuton(void);

Auton elimsLeftAuton = {
    "Elims Left",
    "Gets lots of points!",
    50.0, -16.0, 180.0,
    ElimsLeftAuton
};

void ElimsLeftAuton(void) {

}