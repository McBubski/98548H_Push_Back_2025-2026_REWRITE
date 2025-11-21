#pragma once

#include "vex.h"

void turnToHeading(double heading, double turnSpeed);
void pointAt(double x, double y, double turnSpeed, vex::directionType dir);
void driveFor(double distance, double speed);
void setDrivetrainSpeed(double speed);
int CheckMotorStallTask(void);
void driveTo(double x, double y, double speed, vex::directionType dir);
