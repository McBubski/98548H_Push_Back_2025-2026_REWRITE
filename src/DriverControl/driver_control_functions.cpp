#include "DriverControl/driver_control_functions.h"
#include "vex.h"

// Toggles matchload mechanism
void toggleMatchload() {
    matchloader.set(!matchloader.value());
}

// Toggles wing
void toggleWing() {
    wing.set(!wing.value());
}

// Toggles indexer
void toggleIndexer() {
    indexer_piston.set(!indexer_piston.value());
}

// Toggle hood
void toggleHood() {
    hood.set(!hood.value());
}

void toggleIntakeBSIWonderHowLongICanNameAFunctionTurnsOutItsPrettyLongIWonderWhyYoudEverDoThisButThatsCoolAmazeAmazeAmazeEpicFunctionSauceILikeBajaBlastAndLongFunctionNamesEpicSixtyNine() {
    low_goal_BS.set(!low_goal_BS.value());
}