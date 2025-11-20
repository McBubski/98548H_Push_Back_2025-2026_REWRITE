#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/skills.h"
#include <iostream>

void Skills(void);

Auton skillsAuton = {
    "Skills",
    "Epic Skills Oh Yeah",
    61.25, 15.25, 270.0,
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
         {46.0, 15.0},
    	 {46.0, 47.0},
         {62.5, 47.0}
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
    driveTo(60.75, 45.5, 60, forward);

    // Keep pressure on matchloader to grab all balls

    setDrivetrainSpeed(6);
    wait(350, msec);
    setDrivetrainSpeed(0.5);
    wait(1400, msec);
    //driveFor(-6, 100);

    // Drives to other side of matchloader

    Path first_long_goal_path = PathGenerator::GeneratePath(
    	{{57.5, 48.0},
    	 {50.0, 54.0},
    	 {38.5, 58.0},
    	 {-26, 60.0},
         {-44, 38.5}
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

    //pointAt(-23, 47.5, 100, reverse);
    driveTo(-23, 49.5, 70, reverse);

    setDrivetrainSpeed(-30);
    intake.spin(reverse, 100, percent);
    wait(250, msec);

    // Get rid of our current blocks

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(800, msec);
    // Reset position rq
    std::cout << "Reset Position" << std::endl;
    position_tracking.SetPosition(-30.5, 48.0, inertial_sensor.heading(degrees));
    intake.spin(reverse, 100, percent);
    wait(150, msec);
    intake.spin(forward, 100, percent);
    wait(1600, msec);

    // Drive into farside matchloader

    matchloader.set(true);
    driveTo(-58.5, 48.0, 50, forward);
    hood.set(false);

    wait(800, msec);
    setDrivetrainSpeed(10);
    wait(100, msec);

    driveTo(-28, 47.5, 100, reverse);

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

    position_tracking.SetPosition(-30.5, 48.0, inertial_sensor.heading(degrees));

    indexer.spin(forward, 100, percent);
    task tensionTask = task(CheckMotorStallTask);

    // Drive to other long goal/matchloader

    Path third_matchloader_path = PathGenerator::GeneratePath(
    	{{-40.0, 48.0},
    	 {-40.0, 36.0},
    	 {-34.0, 24.0},
    	 {-34.0, 0.0},
    	 {-44.0, -59},
    	},
    	50.0,
    	25.0,
    	4.0,
    	0.35,
    	1.75
    );

    FollowPath(third_matchloader_path, forward, 18.0);
    matchloader.set(true);
    intake.spin(forward, 100, percent);
    driveTo(-55.0, -48.25, 40, forward);

    wait(1400, msec);
    setDrivetrainSpeed(10);
    wait(100, msec);

    // Drive back to the close side

    Path return_to_our_side_matchloader_path = PathGenerator::GeneratePath(
    	{{-57.5, -50.0},
    	 {-50.0, -56.0},
    	 {-38.5, -60.0},
    	 {42, -60},
         {44, -40}
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.0
    );

    return_to_our_side_matchloader_path.waypoints[4].onReach = []() {
        matchloader.set(false);
    };
    
    wing.set(true);
    FollowPath(return_to_our_side_matchloader_path, reverse, 20.0);

    driveTo(24, -51, 40, reverse);

    // Score in long goal again

    setDrivetrainSpeed(-30);
    intake.spin(reverse, 100, percent);
    wait(300, msec);

    // Jam fix attempt

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(800, msec);
    position_tracking.SetPosition(30.5, -48.0, inertial_sensor.heading(degrees));
    intake.spin(reverse, 100, percent);
    wait(150, msec);
    intake.spin(forward, 100, percent);
    wait(1200, msec);

    // Drive into second close matchloader

    matchloader.set(true);
    driveTo(58.5, -48.0, 50, forward);
    hood.set(false);

    wait(700, msec);
    setDrivetrainSpeed(10);
    wait(100, msec);

    driveTo(28, -47.5, 100, reverse);

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
    wait(1600, msec);

    driveFor(4, 100);
    hood.set(false);
    driveFor(-8, 50);

    matchloader.set(false);

    // Park!

    Path park_path = PathGenerator::GeneratePath(
	    {{30.0, -48.0},
	     {46.0, -44.0},
	     {52, -42},
	     {64, -30},
	     {66, -4},
	    },
	    50.0,
	    25.0,
	    6.0,
	    0.8,
	    2.0
    );

    FollowPath(park_path, forward, 16.0);
    driveFor(-1, 100);
    turnToHeading(5, 100);

    matchloader.set(true);

    wait(500, msec);

    intake.spin(reverse, 100, percent);
    driveFor(20, 35);
    matchloader.set(false);
}