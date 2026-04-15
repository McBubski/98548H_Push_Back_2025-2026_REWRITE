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
    47.0, 13.5, 0.0,//47.0, 13.5
    Skills
};

int matchloader_hack_skills (void) {
    std::cout << "Start" << std::endl;
    wait(1800, msec);
    std::cout << "Stop!" << std::endl;
    RAT_Interrupt = true;

    return 1;
}

int matchload_clear_task(void) {
   bool drivingIntoZone = true;

   float wiggle_I = 0;
   float wiggle_Amp = 60;
   float wiggle_Baseline = 90;

    while(drivingIntoZone) {
        if (forward_distance_sensor.objectDistance(mm) <= 70) {
            wiggle_Baseline = 60;
            wiggle_I += 0.4;
        }
        if (forward_distance_sensor.objectDistance(mm) <= 50) {
            drivingIntoZone = false;
            std::cout << "Fully into zone. (" << forward_distance_sensor.objectDistance(mm) << " mm)" << std::endl;
        }

        float output = (sin(wiggle_I) * wiggle_Amp);

        std::cout << output << std::endl;

        left_drive.spin(forward, wiggle_Baseline + output, percent);
        right_drive.spin(forward, wiggle_Baseline - output, percent);

        wait(20, msec);
    }

    left_drive.stop(brake);
    right_drive.stop(brake);

    return 1;
}

int first_loader_lineup(void) {
    bool reachedPoint = false;
    while (position_tracking.GlobalYPos >= 46.0 && !reachedPoint) {
        wait(20, msec);
    }
    reachedPoint = true;
    RAT_Interrupt = true;
    std::cout << position_tracking.GlobalYPos << std::endl;
    std::cout << "Lined up." << std::endl;
    return 1;
}

void Skills(void) {

    // Curve to the first matchloader

    std::vector<double> positionEstimate = EstimatePositionWithDistance(Y_Pos);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());

    std::cout << positionEstimate[0] << ", " << positionEstimate[1] << std::endl;

    matchloader.set(true);
    intake.spin(forward, 100, percent);

    Path matchload_path = PathGenerator::GeneratePath(
    	{{48.0, 24.0},
    	 {49.0, 44.0},
    	 {73.0 /*almost nice*/, 41.0},
    	},
    	50.0,
    	20.0,
    	3.0,
    	0.55,
    	0.20
    );

    FollowPath(matchload_path, forward, 12.0);
    setDrivetrainSpeed(5);
    wait(200, msec);

    // DONT TOUCH

    Path first_long_goal_path = PathGenerator::GeneratePath(
    	{{57.5, 52.0},
    	 {50.0, 58.0},
    	 {36.5, 60.0},
    	 {-39, 59.0},
         {-41, 41.0}
    	},
    	45.0,
    	25.0,
    	3.0,
    	0.3,
    	2.0
    );

    first_long_goal_path.waypoints[10].onReach = []() { //4
        matchloader.set(false);
    };

    first_long_goal_path.waypoints[12].onReach = []() {
        //task lineup = task(first_loader_lineup);
    };
    
    wing.set(true);
    FollowPath(first_long_goal_path, reverse, 20.0);

    std::cout << "First long goal!" << std::endl;

    Path second_matchloader_path = PathGenerator::GeneratePath(
	    {{-56.0, 49.0},
	     {-64.0, 52.0}
	    },
	    40.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    matchloader.set(true);
    //pointAt(-70.0, 49.0, 100, forward);
    driveTo(-64, 46.5, 60, forward);
    //FollowPath(second_matchloader_path, forward, 20.0);
    setDrivetrainSpeed(10);

    wait(1750, msec);

    Path first_long_goal_second_time_path = PathGenerator::GeneratePath(
	    {{-58.0, 48.0},
	     {-25.5, 45.5}
	    },
	    50.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    hood.set(true);
    FollowPath(first_long_goal_second_time_path, reverse, 18.0);
    setDrivetrainSpeed(-30);

    //hood.set(true);
    intake.spin(reverse, 100, percent);
    indexer_piston.set(true);
    wait(300, msec);
    setDrivetrainSpeed(0);
    intake.spin(forward, 100, percent);
    // Reset position rq
    positionEstimate = EstimatePositionWithDistance(X_Neg);
    //position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    // Reverse to fix sticking
    wait(3500, msec);

    // Push in deeper
    driveFor(6, 100);

    // Drive to clear first park zone

    Path clear_first_park_zone_path = PathGenerator::GeneratePath(
    	{{-36.0, 46.0},
    	 {-54.0, 30.0},
    	 {-65.0, 15.25}
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
    tracking_wheel_piston.set(true);
    indexer_piston.set(false);
    wait(200, msec);
    setDrivetrainSpeed(45);
    matchloader.set(true);

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
        
        if (forward_distance_sensor.objectDistance(inches) <= 48.0 && Brain.Timer.system() - startTime >= 2000) {
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
    positionEstimate = EstimatePositionWithDistance(Y_Neg);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    wait(50, msec);
    turnToHeading(135, 100);

    Path drive_to_third_matchloader_path = PathGenerator::GeneratePath(
    	{{-48.0, -22.0},
    	 {-44.0, -38.0},
    	 {-40.0, -50.5} // -51.5
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.5
    );

    pointAt(-48.0, -22.0, 100, forward);
    FollowPath(drive_to_third_matchloader_path, forward, 18.0);

    std::cout << position_tracking.GlobalXPos << ", " << position_tracking.GlobalYPos << std::endl;

    //turnToHeading(140, 100);
    pointAt(-60.0, -44.0, 100, forward);
    //driveFor(-4, 100);
    matchloader.set(true);

    Path third_matchloader_path = PathGenerator::GeneratePath(
	    {{-60.0, -46.0},
	     {-64.0, -46.0}
	    },
	    40.0,
	    10.0,
	    3.0,
	    0.6,
	    3.0
    );

    FollowPath(third_matchloader_path, forward, 18.0);

    //driveTo(-62.0, -48.0, 80, forward);

    setDrivetrainSpeed(10);

    wait(1800, msec);

    Path second_long_goal_path = PathGenerator::GeneratePath(
    	{{-57.0, -48.0},
    	 {-50.0, -60.0},
    	 {-38.5, -60.0},
    	 {24, -58.0},
         {44, -40.5} // -34.5
    	},
    	45.0,
    	25.0,
    	6.0,
    	0.35,
    	2.5
    );

    second_long_goal_path.waypoints[4].onReach = []() {
        matchloader.set(false);
    };

    wing.set(true);
    FollowPath(second_long_goal_path, reverse, 20.0);

    /*driveFor(-2, 100);
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
	    {{58.0, -45.0},
	     {27.0, -45.5}
	    },
	    50.0,
	    20.0,
	    3.0,
	    0.6,
	    3.0
    );

    FollowPath(second_long_goal_second_time_path, reverse, 18.0);
    setDrivetrainSpeed(-30);

    hood.set(true);
    intake.spin(forward, 100, percent);
    indexer.spin(forward, 100, percent);
    wait(300, msec);
    setDrivetrainSpeed(0);
    // Reset position rq
    positionEstimate = ResetFieldPositionFromDistanceWithOdometry();
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    intake.spin(reverse, 100, percent);
    wait(200, msec);
    intake.spin(forward, 100, percent);
    // Reverse to fix sticking
    wait(2000, msec);

    // Push in deeper
    driveFor(3, 100);
    hood.set(false);
    setDrivetrainSpeed(-10);
    wait(750, msec);
    driveFor(6, 100);

    // Park

    Path park_path = PathGenerator::GeneratePath(
    	{{38.0, -46.0},
    	 {47.0, -24.0},
    	 {61.5 , -13.75}
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
    setDrivetrainSpeed(50);
    intake.spin(reverse, 100, percent);
    tracking_wheel_piston.set(true);
    wait(75, msec);
    matchloader.set(true);

    startTime = Brain.Timer.system();

    bool parking = true;

    while (parking) {
        if ((Brain.Timer.system() - startTime) < 500) {
            continue;
        }

        double error = forward_distance_sensor.objectDistance(inches) - 65.0;

        double driveOutput = error * 3;

        if (driveOutput > 40) {
            driveOutput = 40;
        } else if (driveOutput < 40) {
            driveOutput = -40;
        }

        if (error <= 6) {
            matchloader.set(false);
        }

        left_drive.spin(forward, error * 3, percent);
        right_drive.spin(forward, error * 3, percent);

        if (error <= 1.0) {
            parking = false;
            std::cout << "Parked! " << error << ", " << forward_distance_sensor.objectDistance(inches) << std::endl;

            matchloader.set(false);

            left_drive.stop(brake);
            right_drive.stop(brake);
        }
    }*/
}

/*    // NEW SKILLS

    // Clears first park zone

    intake_piston.set(false);
    tracking_wheel_piston.set(true);
    intake.spin(forward, 100, percent);

    wait(500, msec);

    matchload_clear_task();

    wait(200, msec);

    left_drive.stop(brake);
    right_drive.stop(brake); 

    wait(400, msec);

    // Exit park zone

    setDrivetrainSpeed(-80);
    wait(800, msec);
    left_drive.stop(brake);
    right_drive.stop(brake); 
    wait(200, msec);

    // Reset position (Guess first, then use sensors)

    tracking_wheel_piston.set(false);
    position_tracking.SetPosition(42, 0, inertial_sensor.heading());
     
    std::vector<double> positionEstimate = EstimatePositionWithDistance(X_Pos);
    position_tracking.SetPosition(positionEstimate[0], positionEstimate[1], inertial_sensor.heading());
    //position_tracking.SetPosition(position_tracking.GlobalXPos, 0, inertial_sensor.heading());

    // Get final blue block

    driveTo(14.0, 24.0, 60, forward);

    // Drive to middle

    pointAt(0, 6, 100, forward);
    driveFor(16, 100);

    // Score seven in middle

    low_goal_BS.set(true);
    intake_piston.set(true);
    intake.spin(reverse, 60, percent);*/