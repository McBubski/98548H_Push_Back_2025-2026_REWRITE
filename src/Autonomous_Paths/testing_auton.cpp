#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "Robot/distance_calibration.h"
#include "Robot/color_sorting.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/testing_auton.h"

void TestingAuton(void);

Auton testingAuton = {
    "Testing Auton",
    "Testing stuff for Trey fr",
    0, 0, 0,
    TestingAuton
};


void TestingAuton(void) {
    turnToHeading(90, 100);
	/*intake.spin(forward, 100, percent);
    hood.set(true);

    color_sort_mode allianceColor = colorSortMode;
    color otherColor;

    if (allianceColor == RED) {
        otherColor = color::blue;
    } else {
        otherColor = color::red;
    }

    // Wait until we see blue, or theres a timeout
    int startScoreTime = Brain.Timer.system();
    bool scoring = true;


    while (scoring) {
        if ((Brain.Timer.system() - startScoreTime) > 3000) {
            scoring = false;
        }
        if (otherColor == blue) {
            if (color_sensor.color() == blue) {
                scoring = false;
            }
        } else if (otherColor == red) {
            if (color_sensor.color() == red) {
                scoring = false;
            }
        }
        wait(5, msec);
    }

    intake.spin(reverse, 100, percent);
    hood.set(false);*/
}