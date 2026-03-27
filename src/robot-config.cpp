#include "vex.h"
#include "Robot/odometry.h"

using namespace vex;

brain Brain;
controller Controller;

// Drivetrain

motor motor_FL = motor(PORT13, ratio6_1, true); // ?
motor motor_ML = motor(PORT9, ratio6_1, false);
motor motor_BL = motor(PORT5, ratio6_1, true);

motor_group left_drive = motor_group(motor_FL, motor_ML, motor_BL);

motor motor_FR = motor(PORT14, ratio6_1, false);
motor motor_MR = motor(PORT8, ratio6_1, true);
motor motor_BR = motor(PORT10, ratio6_1, false);

motor_group right_drive = motor_group(motor_FR, motor_MR, motor_BR);

// Intake

motor intake_low = motor(PORT12, ratio6_1, true);
motor intake_high = motor(PORT11, ratio6_1, false);

motor_group intake = motor_group(intake_low, intake_high);

// Indexer

motor indexer = motor(PORT1, ratio6_1, true);

// Pnematics

digital_out matchloader = digital_out(Brain.ThreeWirePort.D);
digital_out wing = digital_out(Brain.ThreeWirePort.E);
digital_out tracking_wheel_piston = digital_out(Brain.ThreeWirePort.C);
digital_out hood = digital_out(Brain.ThreeWirePort.A);
digital_out indexer_piston = digital_out(Brain.ThreeWirePort.B);

// Odometry

inertial inertial_sensor = inertial(PORT10);
rotation forward_tracking_wheel = rotation(PORT16, true);
rotation sideways_tracking_wheel = rotation(PORT15, true);

Odometry position_tracking = Odometry(2.75 / 2.0, 0.125, 1.25);

// Sensors

optical color_sensor = optical(PORT8);
distance forward_distance_sensor = distance(PORT3);
distance right_distance_sensor = distance(PORT7);
distance left_distance_sensor = distance(PORT4);