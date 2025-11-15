#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"

#include "Autonomous_Paths/skills.h"

void Skills(void);

Auton skillsAuton = {
    "Skills",
    "Epic Skills Oh Yeah",
    63.25, 15.0, 270.0,
    Skills
};


void Skills(void) {
    // Drive into matchloader

    matchloader.set(true);
    intake.spin(forward, 100, percent);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.00, 17.5},
    	 {40.0, 33.5},
    	 {48.00, 55},
    	 {72.0, 39.5}
    	},
    	50.0,
    	25.0,
    	6.0,
    	0.8,
    	2.5
    );

    FollowPath(matchload_path, forward, 16.0);
    setDrivetrainSpeed(5);

    wait(1000, msec); 
    
    // Drive into first long goal

    driveFor(-6, 50);
    pointAt(24, 45, 100, reverse);
    driveFor(-24, 50);

    // Score in first long goal

    indexer.spin(reverse, 100, percent);
    intake.spin(reverse, 100, percent);
    setDrivetrainSpeed(-100);
    hood.set(true);

    wait(2500, msec);

    // First ram

    driveFor(4, 100);
    setDrivetrainSpeed(-100);

    wait(600, msec);
    indexer.stop();
    hood.set(false);

    // Drive to second matchloader

    Path second_matchload_path = PathGenerator::GeneratePath(
	    {{32.0, 48.00},
	     {44, 36.0},
	     {44, -32},
	     {44, -48.0},
	     {50.0, -56.5},
         {60.0, -48.5},
	     {72.0, -48.5}
	    },
	    50.0,
	    25.0,
	    6.0,
	    0.8,
	    2.0
    );

    FollowPath(matchload_path, forward, 16.0);
    setDrivetrainSpeed(5);

    wait(1000, msec); 
    
    // Drive into second long goal

    driveFor(-6, 50);
    pointAt(24, -45, 100, reverse);
    driveFor(-24, 50);

    // Score in second long goal

    indexer.spin(reverse, 100, percent);
    intake.spin(reverse, 100, percent);
    setDrivetrainSpeed(-100);
    hood.set(true);

    wait(2500, msec);

    // First ram

    driveFor(4, 100);
    setDrivetrainSpeed(-100);

    wait(600, msec);
    indexer.stop();
    hood.set(false);

    // Park?

    intake.spin(reverse, 100, percent);

    Path park_path = PathGenerator::GeneratePath(
	    {{30.0, -48.0},
	     {48.0, -48.0},
	     {60, -42},
	     {67, -30}, // SIX SEVEN AHHAHAAHAH
	     {67, 0},
	    },
	    80.0,
	    25.0,
	    6.0,
	    0.8,
	    2.0
    );

    FollowPath(park_path, forward, 16.0);
}