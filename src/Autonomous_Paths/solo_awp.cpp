#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"

#include "Autonomous_Paths/solo_awp.h"

void SoloAWPAuton(void);

Auton soloAWPAuton = {
    "Solo AWP",
    "Eat that trash teams",
    50.0, 16.0, 0.0,
    SoloAWPAuton
};


void SoloAWPAuton(void) {

}