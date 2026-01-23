#include "GUI/auton-window.h"
#include "GUI/GUI-main.h"
#include "Autonomous/autonomous_definitions.h"
#include "vex.h"

// Grid information

int rows = 5;
int columns = 2;

int padding = 10;
int strokeWidth = 4;

int windowXSize = 280;// + (strokeWidth * 2) - (padding * 2);
int windowYSize = 240;// + (strokeWidth * 2) - (padding * 2);

bool press_debounce = false;

// Selects the auton based on where the screen was pressed

void SelectAuton(void) {
    if (Brain.Screen.xPosition() >= 480 - windowXSize) {
        int x_coord = floor((Brain.Screen.xPosition() - (480 - windowXSize)) / (windowXSize / columns));
        int y_coord = floor(Brain.Screen.yPosition() / (windowYSize / rows));

        int auton_index = (y_coord * columns) + x_coord; 

        if (auton_index < num_autons) {
            auton_path = auton_index;
            position_tracking.SetPositionToCurrentAuton();

            window = "Odometry";
        }
    }
}

// Draws the auton selection window

void DrawAutonWindow() {
    int auton_index = 0;
    
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < columns; x++) {
            int startX = 480 - windowXSize;
            int startY = padding;

            int buttonSizeX = (windowXSize / columns) - (strokeWidth * 2);
            int buttonSizeY = (windowYSize / rows) - (strokeWidth * 2) - (padding / 2);

            int nameWidth = 15;

            //std::cout << auton_index << std::endl;

            Brain.Screen.setFont(mono15);

            if (auton_index < num_autons) {
                if (auton_index == auton_path) {
                    Brain.Screen.setFillColor("#e277f2");
                } else {
                    Brain.Screen.setFillColor("#b324c9");
                }
                
                Brain.Screen.drawRectangle(startX + (x * buttonSizeX) + (padding * x), startY + (y * buttonSizeY) + (padding * y), buttonSizeX, buttonSizeY);
                Brain.Screen.printAt(startX + (x * buttonSizeX) + (padding * x) + (nameWidth / 2), startY + (y * buttonSizeY) + (padding * y) + 20, autons[auton_index]->name);
            } else {
                Brain.Screen.setFillColor("#949294");
                Brain.Screen.drawRectangle(startX + (x * buttonSizeX) + (padding * x), startY + (y * buttonSizeY) + (padding * y), buttonSizeX, buttonSizeY);
                Brain.Screen.printAt(startX + (x * buttonSizeX) + (padding * x) + (nameWidth / 2), startY + (y * buttonSizeY) + (padding * y) + 20, "Empty");
            }

            auton_index++;
        }
    }

    if (Brain.Screen.pressing()) {
        if (!press_debounce) {
            press_debounce = true;
            SelectAuton();
        }
    } else {
        press_debounce = false;
    }
}