#include "main.h"
#include "au/au.hpp"
#include "declarations.h"
#include "teleop.h"
#include "main.h"
#include "lift.h"
#include "intakeFuncts.h"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "mogoDetect.h"
#include <string>

using namespace au;

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	robot.initialize();

	Routes::initall();
    DriverControl::initAll();
	LiftMngr::initall();
	// IntakeHelper::sortState(false);
	// IntakeHelper::blueExcld(true);

	MogoUtils::init();
	RedRingUtil::init();
	IntakeHelper::init();

	pros::lcd::set_text(1, "ARMED");
	AutoSelector::printPath();
	master.rumble("-.");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// MogoUtils::getMogo(7, 3);
	robot.fwdDynoTest();
	/*
	robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);

	// IntakeHelper::voltage(12);

	// MogoUtils::getMogo(10, 2);
	// LiftMngr::setLevel(500);

	// moClamp.overrideState(1);
    // pros::delay(400);
	// robot.turn_with_pid(90, 600);

	// robot.turn_with_pid(147.8, 200);

	// robot.move_to_point({(au::inches)(53.27), (au::inches)(61.3)}, true, false);

	AutoSelector::updatePath(); // UNCOMMENT
	AutoSelector::run(); // UNCOMMENT
	// pros::delay(150);
	// robot.restOdomKeepAngle(4.5,  5.5);

	// robot.chassis.move_voltage((au::volts)(7));
	// robot.testStatic();

	// robot.ffwLat((au::inches)(50), au::milli(au::seconds)(2000));
	// pros::delay(3000);
	// robot.turn_with_pid(70, 2000);
	/*
	robot.ffwTurn((au::degrees)(180)); // >40 deg???
	master.clear();
	pros::delay(150);
	pros::delay(200);
	master.set_text(1, 0, std::to_string(robot.imu.get_rotation().in(au::degrees)));
	*/
	/**/
	
	
	// robot.move_with_pid((au::inches)(10));


	// TESTS
	// robot.fwdDynoTest();
	// robot.fwdQuasiStaticTest();
	// robot.turnQuasiStaticTest();
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

	robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);

	while(true){


		// master.set_text(1, 0, std::to_string(lineRight.get_value()));
		// MogoUtils::refreshMogo(); // CALIBRATION
		// RedRingUtil::refreshRing(); // CALIBRATION
		




		AutoSelector::updatePath();
		AutoSelector::printPath();
        DriverControl::main();
		pros::delay(25);
	}
	
	
}