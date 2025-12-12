#pragma once

#include "RAT/path_generator.h"

extern Path* Display_Path;

void DrawOdometryWindow(void);
void DrawRobotGraphic(void);
void DrawRATPath(Path* path);
void DrawDistanceSensorLines(void);
void DrawDistanceSensorInformation(void);
void SetOdomDisplayModeToDistanceSensor(void);
void SetOdomDisplayToRAT(void);
void CheckOdomButtonPresses(void);