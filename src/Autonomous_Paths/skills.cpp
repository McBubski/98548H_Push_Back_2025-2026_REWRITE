#include "Autonomous/autonomous_definitions.h"
#include "Autonomous_Functions/auton_functions.h"
#include "RAT/path_follower.h"
#include "Robot/distance_calibration.h"

#include "Autonomous_Paths/skills.h"
#include <iostream>

void Skills(void);

Auton skillsAuton = {
    "Skills",
    "Epic Skills Oh Yeah",
    48.25, 14.5, 0.0,
    Skills
};


void Skills(void) {
    // Curve to the first matchloader

    std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {48.0, 46.0},
    	 {62.0, 40.0},
    	},
    	50.0,
    	20.0,
    	2.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);

    

    Path first_long_goal_path = PathGenerator::GeneratePath(
    	{{57.5, 48.0},
    	 {50.0, 56.0},
    	 {38.5, 56.0},
    	 {-20, 56.0},
         {-40, 38.5}
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

    first_long_goal_path.waypoints[10].onReach = []() {
        std::cout << "Reversing!" << std::endl;
        intake.spin(reverse, 100, percent);
        wait(150, msec);
        intake.stop();
    };
    
    wing.set(true);
    FollowPath(first_long_goal_path, reverse, 20.0);

    
    //pointAt(-23, 47.5, 100, reverse);
    driveTo(-23, 44, 70, reverse);


    setDrivetrainSpeed(-20);
    //intake.spin(reverse, 100, percent);
    //wait(250, msec);

    // Get rid of our current blocks

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(300, msec);
    // Reset position rq
    positionEstimate = EstimatePositionWithDistance(X_Neg);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    // Reverse to fix sticking
    intake.spin(reverse, 100, percent);
    wait(150, msec);
    intake.spin(forward, 100, percent);
    wait(2000, msec);

    // Drive into farside matchloader

    Path second_matchloader_path = PathGenerator::GeneratePath(
	    {{-36.0, 46.0},
	     {-62.0, 46.0}
	    },
	    50.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    second_matchloader_path.waypoints[1].onReach = []() {
        hood.set(false);
    };

    matchloader.set(true);
    FollowPath(second_matchloader_path, forward, 18.0);
    setDrivetrainSpeed(10);

    wait(1000, msec);

    Path first_long_goal_second_time_path = PathGenerator::GeneratePath(
	    {{-58.0, 47.5},
	     {-27.0, 47.5}
	    },
	    50.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    FollowPath(first_long_goal_second_time_path, reverse, 18.0);
    setDrivetrainSpeed(-20);

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(300, msec);
    // Reset position rq
    positionEstimate = EstimatePositionWithDistance(X_Neg);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    // Reverse to fix sticking
    intake.spin(reverse, 100, percent);
    wait(150, msec);
    intake.spin(forward, 100, percent);
    wait(1750, msec);

    // Push in deeper
    driveFor(3, 100);
    hood.set(false);
    setDrivetrainSpeed(-10);
    wait(750, msec);
    driveFor(6, 100);

    

    // Drive to clear first park zone

    Path clear_first_park_zone_path = PathGenerator::GeneratePath(
    	{{-38.0, 46.0},
    	 {-57.0, 26.0},
    	 {-65.0, 18.0}
    	},
    	50.0,
    	25.0,
    	3.0,
    	0.7,
    	3.0
    );

    matchloader.set(false);
    FollowPath(clear_first_park_zone_path, forward, 20.0);
    turnToHeading(180, 100);
    driveFor(3, 100);

    // Clear zone
    matchloader.set(true);
    tracking_wheel_piston.set(true);
    wait(200, msec);
    setDrivetrainSpeed(35);

    int startTime = Brain.Timer.system();
    waitUntil((forward_distance_sensor.objectDistance(inches) <= 50.0 && Brain.Timer.system() - startTime >= 2000));  // Until far enough and at least trying to move

    setDrivetrainSpeed(0);
    tracking_wheel_piston.set(false);
    wait(250, msec);
    matchloader.set(false);

    positionEstimate = EstimatePositionWithDistance(Y_Neg);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    wait(50, msec);
    pointAt(-56, -24, 100, forward);

    Path drive_to_third_matchloader_path = PathGenerator::GeneratePath(
    	{{-56.0, -24.0},
    	 {-44.5, -30.0},
    	 {-46.0, -46.0}
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.5
    );

    FollowPath(drive_to_third_matchloader_path, forward, 18.0);

    pointAt(-64, -47.0, 100, forward);
    matchloader.set(true);

    Path third_matchloader_path = PathGenerator::GeneratePath(
	    {{-36.0, -47.0},
	     {-62.0, -47.0}
	    },
	    50.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    FollowPath(third_matchloader_path, forward, 18.0);

    setDrivetrainSpeed(10);

    wait(1250, msec);

    Path first_second_long_goal_path = PathGenerator::GeneratePath(
    	{{-57.5, -48.0},
    	 {-50.0, -56.0},
    	 {-38.5, -56.0},
    	 {20, -56.0},
         {40, -38.5}
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.5
    );

    first_second_long_goal_path.waypoints[4].onReach = []() {
        matchloader.set(false);
    };

    first_second_long_goal_path.waypoints[10].onReach = []() {
        std::cout << "Reversing!" << std::endl;
        intake.spin(reverse, 100, percent);
        wait(150, msec);
        intake.stop();
    };
    
    wing.set(true);
    FollowPath(first_second_long_goal_path, reverse, 20.0);

    
    //pointAt(-23, 47.5, 100, reverse);
    driveTo(23, -44, 70, reverse);

    /*
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
}*/
}