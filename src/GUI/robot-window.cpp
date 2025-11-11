#include "GUI/robot-window.h"
#include "vex.h"

void DisplayBatteryLevel(void) {
    int batteryLevel = Brain.Battery.capacity(percent);

    if (batteryLevel >= 60) {
        Brain.Screen.setPenColor(green);
    } else if (batteryLevel < 60 && batteryLevel >= 25) {
        Brain.Screen.setPenColor(yellow);
    } else if (batteryLevel < 25) {
        Brain.Screen.setPenColor(red);
    }

   Brain.Screen.printAt(235, 50, "%d%s", batteryLevel, "%");
}

color GetColorFromMotorTemperature(motor motorToCheck) {
    int temp = motorToCheck.temperature(percent);

    if (temp < 50) {
        return green;
    } else if (temp >= 50 && temp < 60) {
        return yellow;
    } else if (temp >= 60 && temp <= 70) {
        return orange;
    } else if (temp > 70) {
        return red;
    }

    return green;
};

void DisplayMotorInformation(int x, int y, motor motorToDisplay) {
    if (motorToDisplay.installed()) {
         Brain.Screen.setPenColor(GetColorFromMotorTemperature(motorToDisplay));
         Brain.Screen.printAt(x, y, "%.f%c Heat", motorToDisplay.temperature(percent), '%');
    } else {
            Brain.Screen.setPenColor(red);
            Brain.Screen.printAt(x, y, "Error (!)");
    }
}

void DisplayMotorTemp(void) {
    // FL

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(210, 90, "FL Motor: ");

    DisplayMotorInformation(215, 110, motor_FL);

    // ML

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(210, 150, "ML Motor: ");

    DisplayMotorInformation(215, 170, motor_ML);

    // TL

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(210, 210, "BL Motor: ");

    DisplayMotorInformation(215, 230, motor_BL);

    // FR

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(300, 90, "FR Motor: ");

    DisplayMotorInformation(305, 110, motor_FR);

    // MR

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(300, 150, "MR Motor: ");

    DisplayMotorInformation(305, 170, motor_MR);

    // TR

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(300, 210, "BR Motor: ");

    DisplayMotorInformation(305, 230, motor_BR);

    // Intake Full

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(390, 90, "Int. Low: ");

    DisplayMotorInformation(395, 110, intake_low);

    // Intake Half

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(390, 150, "Int. High: ");

    DisplayMotorInformation(395, 170, intake_high);

    // Indexer

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(390, 210, "Indexer: ");

    DisplayMotorInformation(395, 230, indexer);
}

void DrawRobotWindow(void) {
    // Battery

    Brain.Screen.setFont(fontType::mono15);
    Brain.Screen.printAt(230, 30, "Battery:");
    DisplayBatteryLevel();

    // Odom Wheels

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(345, 30, "Odom Wheels: ");   

    if (forward_tracking_wheel.installed() && sideways_tracking_wheel.installed()) {
        Brain.Screen.setPenColor(green);
        Brain.Screen.printAt(350, 50, "Installed");
    } else {
        Brain.Screen.setPenColor(red);
        Brain.Screen.printAt(350, 50, "Error (!)");
    }

    // Motor Temp

    DisplayMotorTemp();
}