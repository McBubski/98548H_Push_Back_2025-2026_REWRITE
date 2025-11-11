#include "GUI/GUI-main.h"
#include "GUI_Utility/button.h"

#include "GUI/odometry-window.h"
#include "GUI/robot-window.h"
#include "GUI/auton-window.h"

#include "vex.h"
#include <string.h>

#include "Autonomous/autonomous_definitions.h"

// Button debounce and window state
bool screenDebounce = false;
const char* window = "Odometry";

// Initialize buttons
Button CalibrateButton = Button(10, 140, 180, 40, "Calibrate", "#2ed159", CalibrateInertial); 
Button OdomWindowButton = Button(10, 190, 85, 40, "Odometry", "#2ea3d9", EnterOdomWindow); 
Button RobotWindowButton = Button(105, 190, 85, 40, "Robot", "#f28f2c", EnterRobotWindow); 
Button AutonWindowButton = Button(10, 90, 180, 40, "Select Auton", "#f53b3b", EnterAutonWindow); 

int displaySelector(void) {
    Brain.Screen.render();

    while (true) {
        // Team name and number

        Brain.Screen.setFont(vex::fontType::mono15);
        Brain.Screen.setFillColor(black);
        Brain.Screen.setPenWidth(1);

        Brain.Screen.setPenColor("#e81a1a");
        Brain.Screen.printAt(40, 20, "98548H");
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(98, 20, "REVAMPED");

        Brain.Screen.drawLine(40, 23, 160, 23);
        Brain.Screen.drawLine(40, 8, 160, 8);

        Brain.Screen.setFont(vex::fontType::prop30);

        // Display Auton Name

        const char* name = autons[auton_path]->name;
        int nameLength = Brain.Screen.getStringWidth(name);

        // Display Auton Color

        Brain.Screen.printAt(100 - (nameLength / 2), 65, name);
        Brain.Screen.setPenColor(blue);
        Brain.Screen.setPenWidth(4);
        Brain.Screen.drawLine(100 - (nameLength / 2), 70, 100 - (nameLength / 2) + nameLength, 70);
        Brain.Screen.drawLine(100 - (nameLength / 2), 40, 100 - (nameLength / 2) + nameLength, 40);

        // Display right window information

        Brain.Screen.setPenColor(white);
        Brain.Screen.setFont(vex::fontType::mono40);

        if (strncmp(window, "Odometry", 8) == 0) {
            DrawOdometryWindow();
        } else if (strncmp(window, "Robot", 5) == 0) {
            DrawRobotWindow();
        } else if (strncmp(window, "Auton", 5) == 0) {
            DrawAutonWindow();
        }

        // Display Standard Buttons

        ShowCalibrateButton();
        OdomWindowButton.display();
        RobotWindowButton.display();
        AutonWindowButton.display();

        // Update button pressed

        checkButtonPresses();

        // Print heading to controller screen

        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("%.2f", inertial_sensor.heading(degrees));

        Brain.Screen.render();
        
        wait(20, msec);

        Brain.Screen.clearScreen();
        Controller.Screen.clearLine();
    }

    return 1;
}

void checkButtonPresses(void) {
    if (Brain.Screen.pressing()) {
        if (screenDebounce == false) {
            screenDebounce = true;

            int x = Brain.Screen.xPosition();
            int y = Brain.Screen.yPosition(); 

            CalibrateButton.checkPress(x, y);
            OdomWindowButton.checkPress(x, y);
            RobotWindowButton.checkPress(x, y);
            AutonWindowButton.checkPress(x, y);

            
        }
    } else {
        screenDebounce = false;
    }
}

// Calibrate Button

void ShowCalibrateButton(void) {
    if (inertial_sensor.installed()) {
        if (!inertial_sensor.isCalibrating()) {
            CalibrateButton.text = "Calibrate";
            CalibrateButton.backgroud_color = "#2ec943";
        } else {
            CalibrateButton.text = "Calibrating...";
            CalibrateButton.backgroud_color = "#fcdc26";
        }
    } else {
        CalibrateButton.text = "(!) No Inertial";
        CalibrateButton.backgroud_color = "#dc2f0a";
    }

    CalibrateButton.display();
}

void CalibrateInertial(void) {
    if (!inertial_sensor.isCalibrating()) {
        forward_tracking_wheel.resetPosition();
        sideways_tracking_wheel.resetPosition();

        inertial_sensor.calibrate();
    }

    position_tracking.SetPositionToCurrentAuton();
}

// Window Functions

void EnterOdomWindow(void) {
    window = "Odometry";
}

void EnterRobotWindow(void) {
    window = "Robot";
}
void EnterAutonWindow(void) {
    window = "Auton";
}