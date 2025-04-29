#include "main.h"
#include "au/au.hpp"
#include "declarations.h"
#include "pistons.h"
#include "pros/rtos.hpp"
#include "teleop.h"
#include "main.h"
#include "lift.h"
#include "intakeFuncts.h"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "mogoDetect.h"
#include <iostream>
#include <string>

#include "paths.h"

using namespace au;

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	robot.initialize();

	Routes::initall();
    DriverControl::initAll();
	LiftMngr::initall();
	// IntakeHelper::sortState(false);

	MogoUtils::init();
	RedRingUtil::init();
	IntakeHelper::init();
	// IntakeHelper::blueExcld(true);

	pros::lcd::set_text(1, "ARMED");
	AutoSelector::printPath();
	master.rumble("-.");
}

void disabled() {}

void competition_initialize() {}

// PurePursuit pursuitPath;

void autonomous() {
	robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
	
	/*
	double start_time = pros::millis();
	robot.turn_with_pid(90, 2000);
	

	double end_time = pros::millis();
	master.clear();
	pros::delay(100);
	dlib::Pose2d curPos = robot.odom.get_position();
	master.set_text(0, 0, "x:" + std::to_string((curPos.theta).in(au::degrees)));
	pros::delay(50);
	master.set_text(1, 0, "y:" + std::to_string((curPos.y).in(au::inches)));
	pros::delay(50);
	master.set_text(2, 0, "T:" + std::to_string(end_time-start_time));
	*/
	// pros::delay(50);
	

	AutoSelector::updatePath(); // UNCOMMENT
	AutoSelector::run(); // UNCOMMENT

	// robot.chassis.move_voltage((au::volts)(7));
	// robot.testStatic();
	/**/
	
	
	// robot.move_with_pid((au::inches)(10));


	// TESTS
	// robot.fwdDynoTest();
	// robot.fwdQuasiStaticTest();
	// robot.turnQuasiStaticTest();
	// robot.turnDynoTest();
}



void opcontrol() {
	LiftMngr::setLevel(IDLE_HEIGHT);
	IntakeHelper::StopAtColor(false);
	IntakeHelper::blueExcld(true);
	IntakeHelper::sortState(false);
	IntakeHelper::stuckCheckChange(false);
	robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
	// pros::delay(50);

	while(true){
		// master.set_text(1, 0, std::to_string(lineRight.get_value())); hjh
		// MogoUtils::refreshMogo(); // CALIBRATION
		// RedRingUtil::refreshRing(); // CALIBRATION
		// dlib::Pose2d curPos = robot.odom.get_position();
		// std::cout << "{" << "(au::inches)(" << curPos.x.in(au::inches) << "), " << "(au::inches)(" << curPos.y.in(au::inches) << ")}" << std::endl;
		




		AutoSelector::updatePath();
		AutoSelector::printPath();
        DriverControl::main();
		pros::delay(25);
	}
	
	
}