#include "Autonomous/autonomous.h"
#include "Autonomous/autonomous_definitions.h"
#include "Robot/color_sorting.h"
#include "vex.h"

#include <iostream>

void autonomous() {
    // Get color sort color (only if the preload is visible)

    if (color_sensor.isNearObject()) {
        color allianceColor = color_sensor.color();
        if (allianceColor == red) {
            colorSortMode = RED;
        } else if (allianceColor == blue) {
            colorSortMode = BLUE;
        }
    }

    // Start timer
    float startTime = Brain.Timer.system();
    std::cout << "----- Auton Start ------" << std::endl;

    // Run selected auton path
    autons[auton_path]->function();

    // Print run time
    float runTime = (Brain.Timer.system() - startTime) / 1000;
    std::cout << "Auton Time: " << runTime << " seconds" << std::endl;
}