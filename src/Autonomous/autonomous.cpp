#include "Autonomous/autonomous.h"
#include "Autonomous/autonomous_definitions.h"
#include "vex.h"

#include <iostream>

void autonomous() {
    // Start timer
    float startTime = Brain.Timer.system();
    std::cout << "----- Auton Start ------" << std::endl;

    // Run selected auton path
    autons[auton_path]->function();

    // Print run time
    float runTime = (Brain.Timer.system() - startTime) / 1000;
    std::cout << "Auton Time: " << runTime << " seconds" << std::endl;
}