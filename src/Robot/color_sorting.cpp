#include "Robot/color_sorting.h"
#include "vex.h"

#include <iostream>

color_sort_mode colorSortMode = RED;

int ColorSortTask(void) {
    while (1) {
        if (colorSortMode != NONE) {
            if (color_sensor.isNearObject()) {
                color detectedColor = color_sensor.color();
                color targetColor;

                if (colorSortMode == RED) {
                    targetColor = red;
                } else if (colorSortMode == BLUE) {
                    targetColor = blue;
                }

                std::cout << detectedColor << ", " << targetColor << std::endl;

                while (detectedColor != targetColor && color_sensor.isNearObject()) {
                    indexer.spin(reverse, 100, percent);
                }
            } else {
                indexer.stop();
            }
        }

        wait(20, msec);
    }

    return 1;
}

