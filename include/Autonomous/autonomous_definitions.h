#pragma once

using AutonFunction = void(*)();

struct Auton {
    const char* name;
    const char* description;
    double startX;
    double startY;
    double startHeading;
    AutonFunction function;
};

extern int auton_path;
extern int num_autons;
extern Auton* autons[];