#include "teleop.h"
#include "pistons.h"
#include "pros/misc.h"
#include "autos.h"

int DriverControl::times = 0;
bool DriverControl::deScore = false;
bool DriverControl::atStoreHeight = false;
double DriverControl::voltage = 12;

// Updates the chassis motors based on  joystick input
void DriverControl::updateChassis() {
    int fwd = master.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    
    robot.chassis.arcade(-fwd, turn);
};