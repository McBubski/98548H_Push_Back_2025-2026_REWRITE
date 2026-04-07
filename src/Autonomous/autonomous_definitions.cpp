#include "Autonomous/autonomous_definitions.h"

// Each individual auton path

#include "Autonomous_Paths/left_auton.h"
#include "Autonomous_Paths/right_auton.h"
#include "Autonomous_Paths/elims_left_auton.h"
#include "Autonomous_Paths/elims_right_auton.h"
#include "Autonomous_Paths/seven_ball_left.h"
#include "Autonomous_Paths/seven_ball_right.h"
#include "Autonomous_Paths/skills.h"
#include "Autonomous_Paths/solo_awp.h"
#include "Autonomous_Paths/testing_auton.h"

// Index and array of selected auton path

int auton_path = 8;

Auton* autons[] = {
    &leftAuton,             // 0
    &rightAuton,            // 1
    &elimsLeftAuton,        // 2
    &elimsRightAuton,       // 3
    &sevenBallLeftAuton,    // 4
    &sevenBallRightAuton,   // 5
    &skillsAuton,           // 6
    &soloAWPAuton,          // 7
    &testingAuton           // 8
};

int num_autons = sizeof(autons) / sizeof(autons[0]);