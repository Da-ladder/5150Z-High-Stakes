#pragma once
#include "dlib/controllers/pid.hpp"
#include "main.h"
#include "dlib/dlib.hpp"
#include "pros/distance.hpp"
#include <cmath>

// externs all controllers
extern pros::Controller master;

// extern button
extern pros::adi::DigitalIn xyBut;
extern pros::adi::DigitalIn turnBut;
// extern pros::adi::DigitalIn intakeTop;

// externs kicker and intake so that auto and driver can use them
extern pros::Motor intake;
extern pros::MotorGroup lift;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

// extern line sensors
extern pros::adi::AnalogIn lineLeft;
extern pros::adi::AnalogIn lineRight;
extern pros::adi::DigitalIn sortLimit;

// Rotation sensor
extern pros::Rotation liftRot;

// extern pros::IMU imu;
extern pros::Rotation vert;

// Color sensors
extern pros::v5::Optical opt;
extern pros::v5::Optical camLight;
extern pros::v5::Distance goalOpt;
extern pros::v5::Distance stakeFinder;
extern pros::v5::Distance ringOpt;

// exp stuff
extern pros::v5::Vision camDetect;
extern pros::v5::Vision camRingDetect;

// chassis

// extern dlib::Robot robot;


using namespace au;

class QuadraticFeedforward {
public:
	double a = 0;
	double b = 0;
	double c = 0;

	QuadraticFeedforward(double a, double b, double c) : a(a), b(c), c(c) {

	}

	inline double calculate(double voltage) {
		return 0;
	}
};


class Robot {
	private:
		static bool detectLine;
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

	dlib::FeedforwardGains ffwdTurnDecelGains;
	dlib::Feedforward<Degrees> ffwdTurnDecel;

	dlib::Pid<Meters> lin_pid;

	

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
		dlib::FeedforwardGains linGains,
		dlib::FeedforwardGains ffwdTurnDecelGains,
		dlib::PidGains linPid
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
		linffwd(linGains),
		ffwdTurnDecel(ffwdTurnDecelGains),
		lin_pid(linPid)
		{

	}

	inline void changeDetectLine(bool value) {
		detectLine = value;
	}
	inline bool getDetectLine() {
		return detectLine;
	}
    void initialize();
    void move_with_pid(Quantity<Meters, double> displacement);
    void turnDynoTest();
    void turnQuasiStaticTest();
    void ffwTurn(Quantity<Degrees, double> heading);
    void ffwLat(Quantity<Meters, double> displacement,
              Quantity<Seconds, double> timeout, double maxAccel = 3.15);
    void testStatic();
    void fwdQuasiStaticTest();
    void fwdDynoTest();
    void turn_with_pid(double heading, int timeoutMS, double maxVolts = 12);
    void turn_to_point(dlib::Vector2d point, bool mogoSide, int to, double maxVolts = 12);
    void move_to_point(dlib::Vector2d point, bool turn = true, bool fowards = true, int to = 1400, double maxAccel = 3.15, double maxTurnVolts = 12);
	void ramseteTest(dlib::Vector2d point, bool fowards = false);
    void start_odom();
	void turn_ffwd(double time);

	void restOdom(double x, double y, double theta);
	void restOdomKeepAngle(double x, double y);

};


extern Robot robot;