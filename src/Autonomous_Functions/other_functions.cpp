#include "Autonomous_Functions/auton_functions.h"
#include "vex.h"

#include <iostream>

int CheckMotorStallTask(void) {
    std::cout << "Stall task started" << std::endl;
    // The goal here is to stop the indexer if it's stalling on a ball
    bool hasTorqueStalled = false;

    // This tracks if the indexer stall timer is running (to make sure we've been stalled for long enough, and it's not one time thing)
    bool stall_timer_running = false;

    // This is the start time of the internal timer to check how long it's been since stall started
    int stall_timer_start = 0;

    while (hasTorqueStalled == false) {
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
    }

    std::cout << "Stall!" << std::endl;
    indexer.stop(coast);

    return 1;
}