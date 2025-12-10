#include "Robot/odometry.h"

using namespace vex;

extern brain Brain;
extern controller Controller;

// Drivetrain

extern motor motor_FL;
extern motor motor_ML;
extern motor motor_BL;

extern motor_group left_drive;

extern motor motor_FR;
extern motor motor_MR;
extern motor motor_BR;

extern motor_group right_drive;

// Intake

extern motor intake_low;
extern motor intake_high;

extern motor_group intake;

// Indexer

extern motor indexer;

// Pnematics

extern digital_out matchloader;
extern digital_out wing;
extern digital_out tracking_wheel_piston;
extern digital_out hood;

// Odometry

extern inertial inertial_sensor;
extern rotation forward_tracking_wheel;
extern rotation sideways_tracking_wheel;

extern Odometry position_tracking;

// Sensors

extern optical color_sensor;
extern distance forward_distance_sensor;
extern distance right_distance_sensor;
extern distance left_distance_sensor;
