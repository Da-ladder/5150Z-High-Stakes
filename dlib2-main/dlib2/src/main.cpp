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

	// moClamp.overrideState(1);
    // pros::delay(400);
	// robot.turn_with_pid((au::degrees)(90));

	// robot.turn_with_pid(147.8, 200);

	// robot.move_to_point({(au::inches)(53.27), (au::inches)(61.3)}, true, false);

	AutoSelector::updatePath();
	AutoSelector::run();
	// robot.chassis.move_voltage((au::volts)(7));
	// robot.testStatic();

	// robot.ffwLat((au::inches)(-5), au::milli(au::seconds)(2000));
	// robot.turn_with_pid((au::degrees)(90));
	// robot.ffwTurn((au::degrees)(100));
	// robot.move_with_pid((au::inches)(10));


	// TESTS
	// robot.fwdDynoTest();
	//robot.turnQuasiStaticTest();
	// robot.turnDynoTest();
	/*
	auto start_time = pros::millis();

	robot.start_odom();

	while (true) {
		auto left_disp = robot.chassis.left_motors_displacement();
		auto right_disp = robot.chassis.right_motors_displacement();
		auto heading = (left_disp - right_disp) / track_width;

		robot.chassis.turn_voltage(volts(7));
		std::cout << "heading from wheels: " << heading << ", heading from imu: " << imu.get_rotation() << "\n";
		pros::delay(20);
	}
	*/
	
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