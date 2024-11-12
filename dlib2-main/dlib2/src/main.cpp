#include "main.h"
#include "au/au.hpp"
#include "declarations.h"
#include "teleop.h"
#include "main.h"
#include "pistons.h"
#include "lift.h"
#include "intakeFuncts.h"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include <string>
#include <iostream>

using namespace au;

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	robot.initialize();

	Routes::initall();
    DriverControl::initAll();
	IntakeHelper::init();
	IntakeHelper::blueExcld(true);

	pros::lcd::set_text(1, "ARMED");
	LiftMngr::initall();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	moClamp.overrideState(1);
    pros::delay(400);
	robot.ffwTurn((au::degrees)(90));

	AutoSelector::updatePath();
	// AutoSelector::run();
	// robot.chassis.move_voltage((au::volts)(7));
	// robot.testStatic();

	// robot.ffwLat((au::inches)(-5), au::milli(au::seconds)(2000));
	// robot.turn_with_pid((au::degrees)(90));
	// robot.ffwTurn((au::degrees)(90));
	// robot.move_with_pid((au::inches)(10));


	// TESTS
	// robot.fwdDynoTest();
	// robot.turnQuasiStaticTest();
	// robot.turnDynoTest();
}

void opcontrol() {

	while(true){

		AutoSelector::updatePath();
		AutoSelector::printPath();
        DriverControl::main();
		pros::delay(10);
		pros::delay(20);
	}
	
	
}