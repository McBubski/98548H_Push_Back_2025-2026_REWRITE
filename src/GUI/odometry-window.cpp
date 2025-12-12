#include "GUI/odometry-window.h"
#include "GUI_Utility/field_image_buffer.h"
#include "GUI_Utility/button.h"
#include "Autonomous/autonomous_definitions.h"
#include "Robot/color_sorting.h"
#include "Robot/utility_functions.h"

#include <iostream>

float robotSize = 12.5 - 4.0;
float pixelsPerInch = (200.0 / 144.0);
Path* Display_Path = nullptr;

// Displays odometry map, robot, and coordinates on brain screen

bool odomScreenDebounce = false;
const char* odomDisplayMode = "None"; // None, Distance, RAT

// Buttons

Button DistanceSensorDisplayButton = Button(413, 8, 60, 30, "Dist", "#d21ff2", SetOdomDisplayModeToDistanceSensor);
Button RATDisplayButton = Button(413, 48, 60, 30, "RAT", "#d21ff2", SetOdomDisplayToRAT);

void DrawOdometryWindow() {
    // Flips values depending on color
    int colorSide = 1;
    double fixedHeading = inertial_sensor.heading();

    if (colorSortMode == RED) {
        colorSide = -1;
        fixedHeading = fmod(inertial_sensor.heading() + 180, 360);
    } else {
        fixedHeading = inertial_sensor.heading();
    }

    double y_offset_from_starting = (autons[auton_path]->startY - position_tracking.GlobalYPos);
    double adjusted_y = autons[auton_path]->startY - (y_offset_from_starting * colorSide);

    // Heading and Coordinates

    Brain.Screen.setFillColor(black);
    Brain.Screen.setFont(vex::fontType::mono15);
    Brain.Screen.printAt(360, 230, "%.1f°", fixedHeading);

    Brain.Screen.printAt(200, 230, "(%.2f, %.2f)", position_tracking.GlobalXPos * colorSide, position_tracking.GlobalYPos * colorSide);

    // Field and Robot

    Brain.Screen.drawImageFromBuffer(Brain_Scaled_Field, 200, 7, sizeof(Brain_Scaled_Field));

    // Draw Distance Lines if needed

    if (odomDisplayMode == "Distance") {
        DrawDistanceSensorLines();
    }

    // Draw Robot

    DrawRobotGraphic();

    // Check button pressed

    CheckOdomButtonPresses();

    // Buttons

    DistanceSensorDisplayButton.display();
    RATDisplayButton.display();
}   

// Displays visualization of robot

void DrawRobotGraphic(void) {
    // Flips values depending on color
    int colorSide = 1;
    double fixedHeading = inertial_sensor.heading();

    if (colorSortMode == RED) {
        colorSide = -1;
        fixedHeading = fmod(inertial_sensor.heading() + 180, 360);
    } else {
        fixedHeading = inertial_sensor.heading();
    }

    double y_offset_from_starting = (autons[auton_path]->startY - position_tracking.GlobalYPos);
    double adjusted_y = autons[auton_path]->startY - (y_offset_from_starting * colorSide);

    // Calculates where robot should be drawn
    float XOnBrainScreen = 303 + (pixelsPerInch * position_tracking.GlobalXPos * colorSide);
    float YOnbrainScreen = 110 + (-pixelsPerInch * position_tracking.GlobalYPos * colorSide);

    // Calculates line offsets for drawing border
    float headingInRadians = (fixedHeading * M_PI / 180.0);

    float lineOffset1 = sqrt(2) * robotSize * cos(-headingInRadians + M_PI_4);
    float lineOffset2 = sqrt(2) * robotSize * cos(-headingInRadians - M_PI_4);

    float headingX = sin(headingInRadians);
    float headingY = cos(headingInRadians);

    // Fills in robot first
    Brain.Screen.setPenColor(cyan);
    Brain.Screen.setPenWidth(robotSize * 2);
    
    Brain.Screen.drawLine(
        XOnBrainScreen - headingX * robotSize, 
        YOnbrainScreen + headingY * robotSize, 
        XOnBrainScreen + headingX * robotSize, 
        YOnbrainScreen - headingY * robotSize
    );

    // Draws border around robot
    Brain.Screen.setPenColor(black);
    Brain.Screen.setPenWidth(4);

    Brain.Screen.drawLine(XOnBrainScreen + lineOffset1, YOnbrainScreen - lineOffset2, XOnBrainScreen + lineOffset2, YOnbrainScreen + lineOffset1);
    Brain.Screen.drawLine(XOnBrainScreen + lineOffset2, YOnbrainScreen + lineOffset1, XOnBrainScreen - lineOffset1, YOnbrainScreen + lineOffset2);
    Brain.Screen.drawLine(XOnBrainScreen - lineOffset1, YOnbrainScreen + lineOffset2, XOnBrainScreen - lineOffset2, YOnbrainScreen - lineOffset1);
    Brain.Screen.drawLine(XOnBrainScreen - lineOffset2, YOnbrainScreen - lineOffset1, XOnBrainScreen + lineOffset1, YOnbrainScreen - lineOffset2);

    // Draws heading indication arrow
    Brain.Screen.setPenColor(red);
    Brain.Screen.setPenWidth(5);
    
    Brain.Screen.drawLine(XOnBrainScreen, YOnbrainScreen, XOnBrainScreen + headingX * 15, YOnbrainScreen - headingY * 15);
}

void DrawRATPath(Path* path) {
    if (path == nullptr) return;

    for (const auto& point : path->waypoints) {
        // Calculates where robot should be drawn

        double screen_x = 337 + (pixelsPerInch * point.x);
        double screen_y = 110 + (-pixelsPerInch * point.y);

        Brain.Screen.setPenColor(black);

        //double offset = point.curvature * 2000;
        //vex::color c = vex::color(80, std::max(200 - offset, 0.0), std::min(200 + offset, 255.0));
        vex::color c = vex::color(std::min(100 + (point.targetVelocity) * 2.0, 255.0), 50, 50);
        Brain.Screen.setFillColor(c);

        Brain.Screen.setPenWidth(1);
        Brain.Screen.drawCircle(screen_x, screen_y, 3);
    }

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(200, 30, "%d", static_cast<int>(path->size()));
}

void DrawBeam(double distance, double angle) {
    int colorSide = 1;
    double angleColorOffset = 0;

    if (colorSortMode == RED) {
        colorSide = -1;
        angleColorOffset = 180.0;
    } 

    float originX = 303 + (pixelsPerInch * position_tracking.GlobalXPos * colorSide);
    float originY = 110 + (-pixelsPerInch * position_tracking.GlobalYPos * colorSide);

    double beamlength = distance * pixelsPerInch;
    double xDirection = sin(DegToRad(angle + angleColorOffset));
    double yDirection = cos(DegToRad(angle + angleColorOffset));

    Brain.Screen.setPenColor(blue);
    Brain.Screen.setPenWidth(4);

    // Really far, basically no reading

    if (distance >= 390.0) {
        Brain.Screen.setPenColor(yellow);
        beamlength = 14.0 * pixelsPerInch;
    };    

    Brain.Screen.drawLine(
        originX, 
        originY, 
        originX + xDirection * beamlength, 
        originY - yDirection * beamlength
    );
}

void DrawDistanceSensorLines(void) {
    DrawBeam(left_distance_sensor.objectDistance(inches), inertial_sensor.heading() - 90.0);
    DrawBeam(right_distance_sensor.objectDistance(inches), inertial_sensor.heading() + 90.0);
    DrawBeam(forward_distance_sensor.objectDistance(inches), inertial_sensor.heading());

    Brain.Screen.printAt(413, 70, "Fwd:");
}

void SetOdomDisplayModeToDistanceSensor(void) {
    odomDisplayMode = "Distance";
}

void SetOdomDisplayToRAT(void) {
    odomDisplayMode = "RAT";
}

void CheckOdomButtonPresses(void) {
    if (Brain.Screen.pressing()) {
        if (odomScreenDebounce == false) {
            odomScreenDebounce = true;

            int x = Brain.Screen.xPosition();
            int y = Brain.Screen.yPosition(); 

            DistanceSensorDisplayButton.checkPress(x, y);
        }
    } else {
        odomScreenDebounce = false;
    }
}