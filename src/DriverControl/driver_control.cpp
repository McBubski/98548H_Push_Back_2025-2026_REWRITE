#include "DriverControl/driver_control.h"
#include "DriverControl/driver_control_functions.h"
#include "Autonomous/autonomous_definitions.h"
#include "Robot/color_sorting.h"
#include "Robot/distance_calibration.h"
#include "vex.h"

#include <iostream>
#include <vector>

// The goal here is to stop the indexer if it's stalling on a ball
bool hasTorqueStalled = false;

// This tracks if the indexer stall timer is running (to make sure we've been stalled for long enough, and it's not one time thing)
bool stall_timer_running = false;

// This is the start time of the internal timer to check how long it's been since stall started
int stall_timer_start = 0;

void drivercontrol(void) {
    // Connect buttons to pneumatics
    
    Controller.ButtonB.pressed(toggleMatchload);
    Controller.ButtonY.pressed(toggleHood);
    Controller.ButtonRight.pressed(toggleWing);

    left_drive.setStopping(coast);
    right_drive.setStopping(coast);

    // No need for odometry during driver, so we raise the wheel to protect it

    tracking_wheel_piston.set(true);
    wing.set(true);

    while (1) {
        // Maps motor speed to stick position %

        float leftStickPosition = Controller.Axis3.position();
        float rightStickPosition = Controller.Axis2.position();

        left_drive.spin(forward, leftStickPosition, percent);
        right_drive.spin(forward, rightStickPosition, percent);

        // Indexer

        if (Controller.ButtonL2.pressing()) {
            indexer_piston.set(true);
        } else {
            indexer_piston.set(false);
        }

        if (Controller.ButtonL1.pressing()) {       // To score long goal
            intake.spin(forward, 100, percent);

            // If the current maxes out, the motor is stalled
            if (indexer.current(percent) == 100.0) {
                // If the stall timer isn't already running, start it and update the timer start
                if (stall_timer_running == false) {
                    stall_timer_running = true;
                    stall_timer_start = Brain.Timer.system();
                }
            } else {
                // If the current is less than max, but still really high, don't reset the timer
                // If the current is really low though, we've stopped stalling
                if (indexer.current(percent) < 95.0) {
                    stall_timer_running = false;
                    stall_timer_start = Brain.Timer.system();
                }
            }

            // If the time since we started stalling is more than a second, set hasTorqueStalled to true
            if ((Brain.Timer.system() - stall_timer_start) >= 1000.0) {
                hasTorqueStalled = true;
            }

            // If we haven't stalled, spin the indexer. If we have, stop it.
            if (hasTorqueStalled == false) {
                if (!colorSortingIndexerOverride) {
                    indexer.spin(forward, 100, percent);
                }
            } else {
                std::cout << "Stall!" << std::endl;
                if (!colorSortingIndexerOverride) {
                    indexer.stop(coast);
                }
            }
        } else if (Controller.ButtonL2.pressing()) {    // To score middle high goal
            // Resets torque stall variables

            hasTorqueStalled = false;
            stall_timer_running = false;
            stall_timer_start = Brain.Timer.system();

            if (!colorSortingIndexerOverride) {
                indexer.spin(reverse, 100, percent);
            }

            if (auton_path == 6) {
                intake.spin(forward, 50, percent);
            } else {
                intake.spin(forward, 100, percent);
            }
            
        } else {
            // Resets torque stall variables

            hasTorqueStalled = false;
            stall_timer_running = false;
            stall_timer_start = Brain.Timer.system();

            if (!colorSortingIndexerOverride) {
                indexer.stop(coast);
            }
        }

        // Intake

        if (Controller.ButtonR1.pressing()) {   // Normal intaking
            intake.spin(forward, 100, percent);
            if (!colorSortingIndexerOverride) {
                indexer.spin(forward, 100, percent);
            }
        } else if (Controller.ButtonR2.pressing()) {    // Outaking
            intake.spin(reverse, 100, percent);
            if (!colorSortingIndexerOverride) {
                indexer.spin(reverse, 100, percent);
            }
        } else {
            // Don't stop intake if the indexer is doing stuff
            if (!(Controller.ButtonL1.pressing() || Controller.ButtonL2.pressing())) {
                intake.stop(coast);
            }
        }

        wait(20, msec);
    }
}