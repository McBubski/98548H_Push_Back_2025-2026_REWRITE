#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/color_sorting.h"

#include "Autonomous_Paths/left_auton.h"

void LeftAuton(void);

Auton leftAuton = {
    "Left Auton",
    "Gets lots of points!",
    48.25, -14.5, 180.0,
    LeftAuton
};

void LeftAuton(void) {
    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, -25.0},
    	 {48.0, -47.0},
    	 {62.0, -41.0},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);

    Path goal_path = PathGenerator::GeneratePath(
    	{{56.0, -44.5},
    	 {33.0, -44.5},
    	},
    	50.0,
    	20.0,
    	3,
    	0.6,
    	3.0
    );
    
    // Reverse intake a bit to prevent jams
    goal_path.waypoints[3].onReach = []() {
        intake.spin(reverse, 100, percent);
        wait(300, msec);
        intake.stop();
    };

    FollowPath(goal_path, reverse, 18.0);

    matchloader.set(false);
    setDrivetrainSpeed(-10);
    indexer.spin(forward, 100, percent);
    intake.spin(forward, 100, percent);
    hood.set(true);

    // Color Sort!

    color_sort_mode allianceColor = colorSortMode;
    color otherColor;

    if (allianceColor == RED) {
        otherColor = color::blue;
    } else {
        otherColor = color::red;
    }

    // Wait until we see blue, or theres a timeout
    int startScoreTime = Brain.Timer.system();
    waitUntil((color_sensor.isNearObject() && color_sensor.color() == otherColor) || (Brain.Timer.system() - startScoreTime) > 1750);

    // Stop scoring
    indexer.stop();
    intake.spin(reverse, 100, percent);
    driveFor(6, 100);

    // Bump
    hood.set(false);
    setDrivetrainSpeed(-50);
    wait(500, msec);

    /*Path three_balls_path = PathGenerator::GeneratePath(
    	{{34.0, -48.0},
    	 {44.0, -48.0},
    	 {44.0, -20.0},
         {20, -20}
    	},
    	55.0,
    	20.0,
    	3.0,
    	0.4,
    	3.5
    );*/

    Path three_balls_path = PathGenerator::GeneratePath(
    	{{36.0, -45.5},
    	 {48.0, -45.5},
         {19.5, -17.0}
    	},
    	35.0,
    	10.0,
    	3,
    	0.7,
    	2.0
    );

    intake.spin(forward, 100, percent);
    FollowPath(three_balls_path, forward, 12.0);
    matchloader.set(true);

    pointAt(6, -6, 100, reverse);

    driveFor(-16.0, 100);
    indexer.spin(reverse, 60, percent);
    wait(1500, msec);
    
    Path final_matchloader_path = PathGenerator::GeneratePath(
    	{{24.0, -24.0},
    	 {42.0, -48.0},
         {60.0, -48.0}
    	},
    	60.0,
    	25.0,
    	6.0,
    	0.5,
    	2.0
    );

    FollowPath(final_matchloader_path, forward, 16.0);
}
