#include "vex.h"

#include "Autonomous/autonomous.h"
#include "DriverControl/driver_control.h"
#include "GUI/GUI-main.h"

using namespace vex;

competition Competition;

int UpdateOdometry(void) {
    while (true) {
        position_tracking.Update();

        wait(20, msec);
    }

    return 1;
}
int DisplayGUI(void) {
    while (true) {
        displaySelector();

        wait(20, msec);
    }

    return 1;
}

void pre_auton(void) {
  // Tasks

  task GUI_Task = task(DisplayGUI);

  // Calibration

  forward_tracking_wheel.resetPosition();
  sideways_tracking_wheel.resetPosition();

  inertial_sensor.calibrate();
  
  while (inertial_sensor.isCalibrating()) { wait(20, msec);}

  // Begins Odometry task

  task Odometry_Task = task(UpdateOdometry);
  position_tracking.SetPositionToCurrentAuton();
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
