#include "vex.h"
#include "Robot/odometry.h"

using namespace vex;

brain Brain;
controller Controller;

// Drivetrain

motor motor_FL = motor(PORT16, ratio6_1, true);  // X
motor motor_ML = motor(PORT9, ratio6_1, false);  // X
motor motor_BL = motor(PORT10, ratio6_1, true);  // X

motor_group left_drive = motor_group(motor_FL, motor_ML, motor_BL);

motor motor_FR = motor(PORT15, ratio6_1, false); // X
motor motor_MR = motor(PORT8, ratio6_1, true);   // X
motor motor_BR = motor(PORT7, ratio6_1, false);  // X

motor_group right_drive = motor_group(motor_FR, motor_MR, motor_BR);

// Intake

motor intake_low = motor(PORT11, ratio6_1, false);  // X
motor intake_high = motor(PORT12, ratio6_1, false); // X

motor_group intake = motor_group(intake_low, intake_high);

// Indexer

motor indexer = motor(PORT5, ratio6_1, true);

// Pnematics

digital_out matchloader = digital_out(Brain.ThreeWirePort.B);
digital_out wing = digital_out(Brain.ThreeWirePort.F);
digital_out tracking_wheel_piston = digital_out(Brain.ThreeWirePort.E);
digital_out hood = digital_out(Brain.ThreeWirePort.A);
digital_out indexer_piston = digital_out(Brain.ThreeWirePort.C);
// Intake is D

// Odometry

inertial inertial_sensor = inertial(PORT20); // X
rotation forward_tracking_wheel = rotation(PORT17, true); // X
rotation sideways_tracking_wheel = rotation(PORT19, true); // X

Odometry position_tracking = Odometry(2.75 / 2.0, 0.125, 1.25);

// Sensors

optical color_sensor = optical(PORT8);
distance forward_distance_sensor = distance(PORT14); // X
distance right_distance_sensor = distance(PORT18); // X
distance left_distance_sensor = distance(PORT13); // X
distance back_distance_sensor = distance(PORT6);  // X