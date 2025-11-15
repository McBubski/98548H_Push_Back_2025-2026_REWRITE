#include "Robot/color_sorting.h"
#include "vex.h"

#include <iostream>
#include <cmath>

// Current alliance color, whether or not to clor sort, and an override while sorting

color_sort_mode colorSortMode = RED;
bool colorSorting = false;
bool colorSortingIndexerOverride = false;
bool previousColorSortingIndexerOverride = false;
int previousIndexerState = 0;

// Color sensor checks

bool colorTimerRunning = false;
int colorTimerStart = 0;

int ColorSortTask(void) {
    while (1) {

        // Are we color sorting?
        if (colorSorting) {
            // Is there a ball ready to be sorted?
            if (color_sensor.isNearObject()) {
                // Save current color
                color currentColor = color_sensor.color();

                // Get alliance color
                color allianceColor;

                if (colorSortMode == RED) {
                    allianceColor = red;
                } else if (colorSortMode == BLUE) {
                    allianceColor = blue;
                }

                // Checks for edge cases (if it's something weird, just assume to not color sort)

                if (!(currentColor == red || currentColor == blue)) {
                    currentColor = allianceColor;
                }

                // Checks if it's the wrong color

                if (currentColor != allianceColor) {
                    // Start timer to prevent edge cases
                    if (colorTimerRunning == false) {
                        colorTimerRunning = true;
                        colorTimerStart = Brain.Timer.system();
                    }

                    if (Brain.Timer.system() - colorTimerStart >= 25) {
                        colorSortingIndexerOverride = true;

                        // Indexer logic

                        if (previousIndexerState == 100 || previousIndexerState == 0) {
                            indexer.spin(reverse, 100, percent);
                        } else if (previousIndexerState == -100) {
                            indexer.spin(forward, 100, percent);
                        }
                    }
                } else {
                    colorTimerRunning = false;
                    colorSortingIndexerOverride = false;
                    colorTimerStart = Brain.Timer.system();
                }
            } else {
                colorTimerRunning = false;
                colorSortingIndexerOverride = false;
                colorTimerStart = Brain.Timer.system();
            }
        }

                // Returns indexer to original state after sorting

        if (previousColorSortingIndexerOverride == true && colorSortingIndexerOverride == false) {
            indexer.spin(forward, previousIndexerState, percent);
        }

        // Save what the indexer was doing before

        if (previousColorSortingIndexerOverride == false && colorSortingIndexerOverride == true) {
            if (std::abs(indexer.velocity(percent)) > 0) {
                if (indexer.direction() == forward) {
                    previousIndexerState = 100;
                } else if (indexer.direction() == reverse) {
                    previousIndexerState = -100;
                }
            } else {
                previousIndexerState = 0;
            }
        }

        // Used to stop the indexer after

        previousColorSortingIndexerOverride = colorSortingIndexerOverride;

        wait(5, msec);
    }

    return 1;
}

