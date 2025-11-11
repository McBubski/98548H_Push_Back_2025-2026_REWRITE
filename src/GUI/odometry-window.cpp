#include "GUI/odometry-window.h"
#include "GUI_Utility/field_image_buffer.h"

float robotSize = 10;
Path* Display_Path = nullptr;

// Displays odometry map, robot, and coordinates on brain screen

void DrawOdometryWindow() {
    // Heading and Coordinates

    Brain.Screen.setFillColor(black);
    Brain.Screen.setFont(vex::fontType::mono15);
    Brain.Screen.printAt(390, 230, "%.1f°", inertial_sensor.heading(degrees));

    Brain.Screen.printAt(240, 230, "(%.2f, %.2f)", position_tracking.GlobalXPos, position_tracking.GlobalYPos);

    // Field and Robot

    Brain.Screen.drawImageFromBuffer(Brain_Scaled_Field, 234, 7, sizeof(Brain_Scaled_Field));

    DrawRATPath(Display_Path);

    DrawRobotGraphic();
}

// Displays visualization of robot

void DrawRobotGraphic(void) {
    // Calculates where robot should be drawn
    float pixelsPerInch = (200.0 / 144.0);
    float XOnBrainScreen = 337 + (pixelsPerInch * position_tracking.GlobalXPos);
    float YOnbrainScreen = 110 + (-pixelsPerInch * position_tracking.GlobalYPos);

    // Calculates line offsets for drawing border
    float headingInRadians = (floor(inertial_sensor.angle(degrees)) * M_PI / 180.0);

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
    Brain.Screen.setPenWidth(4);
    
    Brain.Screen.drawLine(XOnBrainScreen, YOnbrainScreen, XOnBrainScreen + headingX * 15, YOnbrainScreen - headingY * 15);
}

void DrawRATPath(Path* path) {
    if (path == nullptr) return;

    for (const auto& point : path->waypoints) {
        // Calculates where robot should be drawn

        float pixelsPerInch = (200.0 / 144.0);
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