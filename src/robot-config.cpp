#include "vex.h"
#include "Robot/odometry.h"

using namespace vex;

brain Brain;
controller Controller;

// Drivetrain

motor motor_FL = motor(PORT11, ratio6_1, true); // ?
motor motor_ML = motor(PORT12, ratio6_1, false);
motor motor_BL = motor(PORT13, ratio6_1, true);

motor_group left_drive = motor_group(motor_FL, motor_ML, motor_BL);

motor motor_FR = motor(PORT17, ratio6_1, false);
motor motor_MR = motor(PORT20, ratio6_1, true);
motor motor_BR = motor(PORT19, ratio6_1, false);

motor_group right_drive = motor_group(motor_FR, motor_MR, motor_BR);

// Intake

motor intake_low = motor(PORT9, ratio6_1, false);
motor intake_high = motor(PORT2, ratio6_1, true);

motor_group intake = motor_group(intake_low, intake_high);

// Indexer

motor indexer = motor(PORT1, ratio6_1, true);

// Pnematics

digital_out matchloader = digital_out(Brain.ThreeWirePort.A);
digital_out wing = digital_out(Brain.ThreeWirePort.B);
digital_out tracking_wheel_piston = digital_out(Brain.ThreeWirePort.C);
digital_out hood = digital_out(Brain.ThreeWirePort.D);

// Odometry

inertial inertial_sensor = inertial(PORT10);
rotation forward_tracking_wheel = rotation(PORT16, true);
rotation sideways_tracking_wheel = rotation(PORT15, false);

Odometry position_tracking = Odometry(2.75 / 2.0, 0.25, 1.25);

// Sensors

optical color_sensor = optical(PORT8);

