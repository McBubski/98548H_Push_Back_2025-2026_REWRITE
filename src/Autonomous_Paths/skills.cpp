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
    indexer.spin(forward, 100, percent);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.00, 17.5},
    	 {40.0, 33.5},
    	 {48.00, 53.5},
    	 {72.0, 39.0}
    	},
    	50.0,
    	25.0,
    	6.0,
    	0.8,
    	2.5
    );

    FollowPath(matchload_path, forward, 16.0);
    setDrivetrainSpeed(1);

    wait(900, msec);
    intake.stop();
    wait(200, msec);
    intake.spin(forward, 100, percent);
    wait(600, msec);
    setDrivetrainSpeed(0);
    intake.stop();
    wait(200, msec);
    intake.spin(forward, 100, percent);
    wait(600, msec);
    
    driveFor(-8, 100);
    wait(1000, msec);
    driveFor(10, 100);
    wait(1200, msec);
    
    // Drive into first long goal

    driveFor(-6, 50);
    pointAt(24, 47, 100, reverse);
    driveFor(-24, 49);

    // Score in first long goal

    indexer.spin(forward, 100, percent);
    intake.spin(forward, 100, percent);
    setDrivetrainSpeed(-100);
    hood.set(true);
    driveFor(1, 100);

    driveFor(-2, 100);

    driveFor(1, 100);

    wait(600, msec);
    intake.spin(reverse, 100, percent);
    wait(200, msec);
    intake.spin(forward, 100, percent);

    // First ram

    driveFor(4, 100);
    hood.set(false);
    setDrivetrainSpeed(-70);

    wait(600, msec);

    // Drive to second matchloader

    Path second_matchload_path = PathGenerator::GeneratePath(
	    {{32.0, 48.00},
	     {44, 36.0},
	     {40, -32},
	     {40, -48.0},
         {54.0, -54.0},
    	 //{66.0, -39.0}
	    },
	    50.0,
	    30.0,
	    6.0,
	    0.7,
	    2.0
    );

    FollowPath(second_matchload_path, forward, 24.0);
    driveTo(62, -49, 100, forward);

    setDrivetrainSpeed(1);

    wait(900, msec);
    intake.stop();
    wait(200, msec);
    intake.spin(forward, 100, percent);
    wait(600, msec);
    setDrivetrainSpeed(0);
    intake.stop();
    wait(200, msec);
    intake.spin(forward, 100, percent);
    wait(600, msec);

    driveFor(-8, 100);
    wait(1000, msec);
    driveFor(10, 100);
    wait(1200, msec);
    
    
    //driveFor(-4, 100);
    //setDrivetrainSpeed(20);
    
    // Drive into second long goal

    driveFor(-6, 50);
    pointAt(24, -49, 100, reverse);
    driveFor(-24, 50);

    // Score in second long goal

    indexer.spin(forward, 100, percent);
    intake.spin(forward, 100, percent);
    setDrivetrainSpeed(-100);
    hood.set(true);
    driveFor(1, 100);

    driveFor(-2, 100);

    driveFor(1, 100);

    wait(1200, msec);

    // First ram

    driveFor(4, 100);
    hood.set(false);
    setDrivetrainSpeed(-100);

    wait(600, msec);
    indexer.stop();
    hood.set(false);
    matchloader.set(false);

    // Park?

    indexer.spin(reverse, 100, percent);

    Path park_path = PathGenerator::GeneratePath(
	    {{30.0, -48.0},
	     {48.0, -48.0},
	     {60, -42},
	     {71, -30}, // SIX SEVEN AHHAHAAHAH
	     {72, -9},
	    },
	    40.0,
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
    driveFor(22, 30);
    matchloader.set(false);
}