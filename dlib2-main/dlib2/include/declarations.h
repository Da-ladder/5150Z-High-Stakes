#pragma once
#include "main.h"
#include "dlib/dlib.hpp"
#include "pros/distance.hpp"

// externs all controllers
extern pros::Controller master;

// extern button
extern pros::ADIDigitalIn xyBut;
extern pros::ADIDigitalIn turnBut;

// externs kicker and intake so that auto and driver can use them
extern pros::Motor intake;
extern pros::MotorGroup lift;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

// Rotation sensor
extern pros::Rotation liftRot;

extern pros::IMU imu;
extern pros::Rotation vert;

// Color sensors
extern pros::v5::Optical opt;
extern pros::v5::Optical camLight;
extern pros::v5::Distance goalOpt;
extern pros::v5::Distance stakeFinder;
extern pros::v5::Distance ringOpt;

// exp stuff
extern pros::v5::Vision camDetect;

// chassis

// extern dlib::Robot robot;


using namespace au;

class Robot {
    public:
    dlib::Chassis chassis;
	dlib::Imu imu;

	dlib::Odometry odom;
	std::unique_ptr<pros::Task> odom_updater;

	dlib::Pid<Meters> move_pid;
	dlib::ErrorDerivativeSettler<Meters> move_settler;

	dlib::Pid<Degrees> turn_pid;
	dlib::ErrorDerivativeSettler<Degrees> turn_settler;

	dlib::RotationConfig rotRight = {1,
	inches(2.75),
	.8};
	dlib::RotationConfig rotLeft = {9,
	inches(2.75),
	.8};

	dlib::Rotation rotationRight;
	dlib::Rotation rotationLeft;

	dlib::FeedforwardGains ffwdGains;
	dlib::Feedforward<Degrees> ffwd;

	dlib::FeedforwardGains linGains;
	dlib::Feedforward<Meters> linffwd;

	

	Robot(
		dlib::ChassisConfig chassis_config, 
		dlib::ImuConfig imu_config,
		dlib::PidGains move_pid_gains,
		dlib::ErrorDerivativeSettler<Meters> move_pid_settler,
		dlib::PidGains turn_pid_gains,
		dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler,
		dlib::RotationConfig rotRight,
		dlib::RotationConfig rotLeft,
		dlib::FeedforwardGains ffwdGains,
		dlib::FeedforwardGains linGains
	) : 
		chassis(chassis_config), 
		imu(imu_config),
		move_pid(move_pid_gains),
		move_settler(move_pid_settler),
		turn_pid(turn_pid_gains),
		turn_settler(turn_pid_settler),
		odom(),
		rotationRight(rotRight),
		rotationLeft(rotLeft),
		ffwd(ffwdGains),
		linffwd(linGains)
		{

	}

    void initialize();
    void move_with_pid(Quantity<Meters, double> displacement);
    void turnDynoTest();
    void turnQuasiStaticTest();
    void ffwTurn(Quantity<Degrees, double> heading);
    void ffwLat(Quantity<Meters, double> displacement,
              Quantity<Seconds, double> timeout);
    void testStatic();
    void fwdQuasiStaticTest();
    void fwdDynoTest();
    void turn_with_pid(Quantity<Degrees, double> heading);
    void turn_to_point(dlib::Vector2d point);
    void move_to_point(dlib::Vector2d point, bool turn = true, bool fowards = true);
    void start_odom();
};


extern Robot robot;