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

int matchloader_hack_skills (void) {
    std::cout << "Start" << std::endl;
    wait(1800, msec);
    std::cout << "Stop!" << std::endl;
    RAT_Interrupt = true;

    return 1;
}

void Skills(void) {
    // Curve to the first matchloader

    std::vector<double> positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    matchloader.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    task indexerTask = task(CheckMotorStallTask);
    //task matchloader_hacky_fix = task(matchloader_hack_skills);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {48.0, 45.0},
    	 {65.0, 39.75},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);
    setDrivetrainSpeed(5);
    wait(1400, msec);

    // DONT TOUCH

    Path first_long_goal_path = PathGenerator::GeneratePath(
    	{{57.5, 48.0},
    	 {50.0, 56.0},
    	 {38.5, 56.0},
    	 {-30, 56.0},
         {-40, 36.5}
    	},
    	45.0,
    	30.0,
    	4.0,
    	0.3,
    	2.5
    );

    first_long_goal_path.waypoints[4].onReach = []() {
        matchloader.set(false);
    };

    first_long_goal_path.waypoints[10].onReach = []() {
        std::cout << "Reversing!" << std::endl;
        intake.spin(reverse, 100, percent);
        wait(250, msec);
        intake.stop();
    };
    
    wing.set(true);
    FollowPath(first_long_goal_path, reverse, 18.0);

    
    pointAt(-23, 41.0, 100, reverse);
    driveTo(-23, 39.0, 60, reverse);


    //setDrivetrainSpeed(-20);
    //intake.spin(reverse, 100, percent);
    //wait(250, msec);

    // Get rid of our current blocks

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(400, msec);
    // Reset position rq
    positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    // Reverse to fix sticking
    wait(2000, msec);

    // Drive into farside matchloader

    Path second_matchloader_path = PathGenerator::GeneratePath(
	    {{-36.0, 44.5},
	     {-62.0, 45.0}
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

    wait(2000, msec);

    Path first_long_goal_second_time_path = PathGenerator::GeneratePath(
	    {{-58.0, 44.5},
	     {-25.5, 46.5}
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
    setDrivetrainSpeed(0);
    // Reset position rq
    positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    // Reverse to fix sticking
    wait(2500, msec);

    // Push in deeper
    driveFor(3, 100);
    hood.set(false);
    setDrivetrainSpeed(-10);
    wait(750, msec);
    driveFor(6, 100);

    // Drive to clear first park zone

    Path clear_first_park_zone_path = PathGenerator::GeneratePath(
    	{{-36.0, 46.0},
    	 {-54.0, 30.0},
    	 {-62.0, 12.5}
    	},
    	50.0,
    	25.0,
    	3.0,
    	0.7,
    	3.0
    );

    matchloader.set(false);
    FollowPath(clear_first_park_zone_path, forward, 16.0);
    turnToHeading(183, 100);
    //driveFor(3, 100);

    // Clear zone
    matchloader.set(true);
    tracking_wheel_piston.set(true);
    wait(200, msec);
    setDrivetrainSpeed(35);

    int startTime = Brain.Timer.system();
    bool clearing = true;
    bool hasSlowed = false;

    while (clearing) {
        std::cout << forward_distance_sensor.objectDistance(inches) << std::endl;

        std::cout << forward_distance_sensor.objectDistance(inches) << std::endl;

        if (forward_distance_sensor.objectDistance(inches) <= 74.0 && !hasSlowed && Brain.Timer.system() - startTime >= 500) {
            hasSlowed = true;

            intake.spin(reverse, 100, percent);
            matchloader.set(false);
            setDrivetrainSpeed(30);
            wait(150, msec);
            intake.spin(forward, 100, percent);
            std::cout << "Slowing down!" << std::endl;
        } 
        
        if (forward_distance_sensor.objectDistance(inches) <= 50.0 && Brain.Timer.system() - startTime >= 2000) {
            clearing = false;
            std::cout << "Cleared!" << std::endl;
        }

        wait(20, msec);
    }

    setDrivetrainSpeed(0);
    tracking_wheel_piston.set(false);
    wait(250, msec);
    matchloader.set(false);

    //turnToHeading(180, 100);
    positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    wait(50, msec);
    pointAt(-48.0, -52.5, 100, forward);

    Path drive_to_third_matchloader_path = PathGenerator::GeneratePath(
    	{{-56.0, -24.0},
    	 {-52, -30.0},
    	 {-46.0, -51.5} // -51.5
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.5
    );

    FollowPath(drive_to_third_matchloader_path, forward, 18.0);


    //pointAt(-62, -43.0, 100, forward);
    //driveFor(-4, 100);
    matchloader.set(true);

    /*Path third_matchloader_path = PathGenerator::GeneratePath(
	    {{-50.0, -46.0},
	     {-62.0, -45.0}
	    },
	    40.0,
	    10.0,
	    3.0,
	    0.6,
	    3.0
    );

    FollowPath(third_matchloader_path, forward, 18.0);*/

    driveTo(-62.0, -44.0, 80, forward);

    setDrivetrainSpeed(10);

    wait(1800, msec);

    Path first_second_long_goal_path = PathGenerator::GeneratePath(
    	{{-57.5, -48.0},
    	 {-50.0, -57.0},
    	 {-38.5, -57.0},
    	 {24, -57.0},
         {44, -35.0} // -34.5
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

    // Goal close to you left side
    driveTo(23, -40.5, 70, reverse);

    driveFor(-2, 100);
    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(300, msec);
    // Reset position rq
    positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    // Reverse to fix sticking
    intake.spin(reverse, 100, percent);
    wait(200, msec);
    intake.spin(forward, 100, percent);
    wait(2500, msec);

    Path fourth_matchloader_path = PathGenerator::GeneratePath(
	    {{36.0, -44.0},
	     {62.0, -43.5}
	    },
	    50.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    fourth_matchloader_path.waypoints[1].onReach = []() {
        hood.set(false);
    };

    matchloader.set(true);
    FollowPath(fourth_matchloader_path, forward, 18.0);
    setDrivetrainSpeed(10);

    wait(2000, msec);

    Path second_long_goal_second_time_path = PathGenerator::GeneratePath(
	    {{58.0, -46.0},
	     {26.0, -45.0}
	    },
	    50.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    FollowPath(second_long_goal_second_time_path, reverse, 18.0);
    setDrivetrainSpeed(-20);

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(300, msec);
    setDrivetrainSpeed(0);
    // Reset position rq
    positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    // Reverse to fix sticking
    wait(1900, msec);

    // Push in deeper
    driveFor(3, 100);
    hood.set(false);
    setDrivetrainSpeed(-10);
    wait(750, msec);
    driveFor(6, 100);

    // Park

    Path park_path = PathGenerator::GeneratePath(
    	{{38.0, -46.0},
    	 {54.0, -26.0},
    	 {61.5 , -13.5}
    	},
    	50.0,
    	25.0,
    	3.0,
    	0.7,
    	3.0
    );

    matchloader.set(false);
    FollowPath(park_path, forward, 20.0);
    turnToHeading(3, 100);
    //driveFor(3, 100);

    // Clear zone
    matchloader.set(true);
    intake.spin(reverse, 100, percent);
    tracking_wheel_piston.set(true);
    wait(200, msec);
    setDrivetrainSpeed(50);

    startTime = Brain.Timer.system();

    bool parking = true;

    while (parking) {
        double error = forward_distance_sensor.objectDistance(inches) - 65.0;

        double driveOutput = error * 3;

        if (driveOutput > 40) {
            driveOutput = 40;
        } else if (driveOutput < 40) {
            driveOutput = -40;
        }

        if (error <= 12) {
            matchloader.set(false);
        }

        left_drive.spin(forward, error * 3, percent);
        right_drive.spin(forward, error * 3, percent);

        if (error <= 1.0) {
            parking = false;
            std::cout << "Parked!" << std::endl;

            matchloader.set(false);

            left_drive.stop(brake);
            right_drive.stop(brake);
        }
    }
}