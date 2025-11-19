#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/skills.h"
#include <iostream>

void Skills(void);

Auton skillsAuton = {
    "Skills",
    "Epic Skills Oh Yeah",
    63.25, 15.0, 270.0,
    Skills
};

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


void Skills(void) {
    // Curve to the first matchloader

    Path first_matchloader_path = PathGenerator::GeneratePath(
    	{{62, 15.0},
         {50.0, 15.0},
    	 {50.0, 46.0},
         {58, 46}
    	 //{44.0, 45.0},
    	 //{73.5, 43.0}
    	},
    	40.0,
    	25.0,
    	6.0,
    	0.45,
    	1.5
    );

    // Spin the intake to score out of matchloader

    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);
    FollowPath(first_matchloader_path, forward, 12.0);
    driveTo(63.5, 45, 60, forward);

    // Keep pressure on matchloader to grab all balls

    setDrivetrainSpeed(6);
    wait(350, msec);
    setDrivetrainSpeed(0.5);
    wait(1400, msec);
    //driveFor(-6, 100);

    // Drives to other side of matchloader

    Path first_long_goal_path = PathGenerator::GeneratePath(
    	{{57.5, 48.0},
    	 {50.0, 48.0},
    	 {38.5, 56.0},
    	 {-26, 60},
         {-38, 38}
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.5
    );

    first_long_goal_path.waypoints[4].onReach = []() {
        matchloader.set(false);
    };
    
    wing.set(true);
    FollowPath(first_long_goal_path, reverse, 20.0);

    /*Path curve_into_long_goal = PathGenerator::GeneratePath(
    	{{-26.5, 56.0},
    	 {-34.5, 56.0},
    	 {-38.0, 54.0},
    	 {-36.5, 47.0},
    	 {-18.0, 46.0}
    	},
    	25.0,
    	25.0,
    	4.0,
    	0.5,
    	8.0
    );*/

    //FollowPath(curve_into_long_goal, reverse, 10.0);

    // Drive into matchloader

    driveTo(-24, 47.5, 100, reverse);

    setDrivetrainSpeed(-40);
    intake.spin(reverse, 100, percent);
    wait(250, msec);

    // Get rid of our current blocks

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(800, msec);
    intake.spin(reverse, 100, percent);
    wait(150, msec);
    intake.spin(forward, 100, percent);
    wait(1200, msec);

    // Drive into farside matchloader

    matchloader.set(true);
    driveFor(6, 100);
    hood.set(false);
    driveTo(-51.5, 46.5, 50, forward);

    wait(800, msec);
    setDrivetrainSpeed(10);
    wait(100, msec);

    driveTo(-24, 45, 100, reverse);

    // Score in long goal again

    setDrivetrainSpeed(-40);
    intake.spin(reverse, 100, percent);
    wait(250, msec);

    // Jam fix attempt

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(800, msec);
    intake.spin(reverse, 100, percent);
    wait(150, msec);
    intake.spin(forward, 100, percent);
    wait(1200, msec);

    // Perhaps get control?

    driveFor(4, 100);
    hood.set(false);
    driveFor(-8, 50);

    // Drive to other long goal/matchloader

    Path third_matchloader_path = PathGenerator::GeneratePath(
    	{{-36.0, 48.0},
    	 {-34.0, 36.0},
    	 {-26.0, 24.0},
    	 {-26.0, 0.0},
    	 {-36.0, -52.5},
    	},
    	40.0,
    	25.0,
    	6.0,
    	0.35,
    	1.5
    );

    FollowPath(third_matchloader_path, forward, 18.0);
    driveTo(-48.5, -46.5, 50, forward);

    setDrivetrainSpeed(10);
    wait(100, msec);
    setDrivetrainSpeed(0);
    wait(1200, msec);

    // Drive back to the close side

    Path return_to_our_side_matchloader_path = PathGenerator::GeneratePath(
    	{{-57.5, -48.0},
    	 {-50.0, -48.0},
    	 {-38.5, -56.0},
    	 {50, -60},
         {56, -36}
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.5
    );

    return_to_our_side_matchloader_path.waypoints[4].onReach = []() {
        matchloader.set(false);
    };
    
    wing.set(true);
    FollowPath(return_to_our_side_matchloader_path, reverse, 20.0);

    driveTo(24, -46, 20, reverse);

    // Score in long goal again

    setDrivetrainSpeed(-40);
    intake.spin(reverse, 100, percent);
    wait(250, msec);

    // Jam fix attempt

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(800, msec);
    intake.spin(reverse, 100, percent);
    wait(150, msec);
    intake.spin(forward, 100, percent);
    wait(1200, msec);
}