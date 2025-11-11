#include "DriverControl/driver_control.h"
#include "DriverControl/driver_control_functions.h"
#include "vex.h"

void drivercontrol(void) {
    // Connect buttons to pneumatics
    
    Controller.ButtonB.pressed(toggleMatchload);
    Controller.ButtonY.pressed(toggleHood);
    Controller.ButtonRight.pressed(toggleWing);

    tracking_wheel_piston.set(true);

    while (1) {
        // Drive

        float leftStickPosition = Controller.Axis3.position();
        float rightStickPosition = Controller.Axis2.position();

        left_drive.spin(forward, leftStickPosition, percent);
        right_drive.spin(forward, rightStickPosition, percent);

        // Intake

        if (Controller.ButtonR1.pressing()) {
            intake.spin(forward, 100, percent);
        } else if (Controller.ButtonR2.pressing()) {
            intake.spin(reverse, 100, percent);
        } else {
            if (!Controller.ButtonL1.pressing() || !Controller.ButtonR2.pressing()) {
                intake.stop(coast);
            }
        }

        // Indexer

        if (Controller.ButtonL1.pressing()) {
            indexer.spin(forward, 100, percent);
            intake.spin(forward, 100, percent);
        } else if (Controller.ButtonL2.pressing()) {
            indexer.spin(reverse, 100, percent);
            intake.spin(forward, 100, percent);
        } else {
            indexer.stop(coast);
        }

        wait(20, msec);
    }
}