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

// Toggles hood
void toggleHood() {
    indexer_piston.set(!indexer_piston.value());
}