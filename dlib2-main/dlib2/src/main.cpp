#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/feedforward.hpp"
#include "dlib/controllers/pid.hpp"
#include "dlib/dlib.hpp"
#include "dlib/hardware/rotation.hpp"
#include "dlib/trajectories/trapezoid_profile.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include <string>
#include <iostream>

using namespace au;

// TODO: Decide if we want to implement helper functions for some of these things
// nehh
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

	void initialize() {
		chassis.initialize();
		imu.initialize();
		start_odom();
	}

	// Use PID to do a relative movement
	void move_with_pid(Quantity<Meters, double> displacement) {
		auto start_displacement = -rotationLeft.get_linear_displacement();
		auto target_displacement = start_displacement + displacement;
		
		
		move_pid.target(target_displacement);

		while (!move_settler.is_settled(move_pid.get_error(), move_pid.get_derivative())) {
			auto reading = -rotationLeft.get_linear_displacement();
			auto voltage = move_pid.update(reading, milli(seconds)(20));
			
			chassis.move_voltage(voltage);

			pros::delay(20);
		}
		std::cout << "thing" << std::endl;
	}

	void turnDynoTest() {
		auto voltage = (au::volts)(7.0);

		auto start_time = pros::millis();
  		auto elapsed_time = 0;
		auto pastRot = imu.get_rotation();
		auto curRot = imu.get_rotation();
		int thing = 0;
		auto vel = (curRot-pastRot)/0.01;
		auto preVel = vel;
		

		while(elapsed_time < 10000) {
			thing++;
			auto current_time = pros::millis();
    		elapsed_time = current_time - start_time;

			// voltage = au::milli(au::volts)(1000.0 * (elapsed_time / 1000.0)); // 1 volts/sec ramp

			chassis.turn_voltage(voltage);  //deltaAngle/delT
			curRot = imu.get_rotation();

			auto vel = (curRot-pastRot)/0.01;
			auto accel = (vel-preVel)/0.01;

			if (thing % 2 == 0) {
				std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", " << (curRot-pastRot)/0.01 << ", " << accel <<std::endl;
			}

			pros::delay(10);
			pastRot = curRot;
			preVel = vel;
		}

		chassis.turn_voltage((au::volts)(0));

	}

	void turnQuasiStaticTest() {

		auto voltage = (au::volts)(0.0);

		auto start_time = pros::millis();
  		auto elapsed_time = 0;
		auto pastRot = imu.get_rotation();
		auto curRot = imu.get_rotation();
		int thing = 0;

		while (voltage < (au::volts)(12)) {
			thing++;
			auto current_time = pros::millis();
    		elapsed_time = current_time - start_time;

			voltage = au::milli(au::volts)(1000.0 * (elapsed_time / 1000.0)); // 1 volts/sec ramp

			chassis.turn_voltage(voltage);  //deltaAngle/delT
			curRot = imu.get_rotation();

			if (thing % 2 == 0) {
				std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", " << (curRot-pastRot)/0.01 <<std::endl;
			}
			
			pros::delay(10);
			pastRot = curRot;
		}

		chassis.turn_voltage((au::volts)(0));

	}

	void ffwTurn(Quantity<Degrees, double> heading) {
		// EXP

		dlib::PidGains turn_pid_gains {
		30, 	// kp, porportional gain
		0, 	// ki, integral gain
		0.0 	// kd, derivative gain
		};
		// EXP END
		
		auto start_time = au::milli(au::seconds)(pros::millis());
  		auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
		auto target_heaidng = heading;
		auto reading = imu.get_rotation();
		auto prevReading = reading;
		auto startHeading = reading;
		int maxAccel = 1400;
		int maxVelo = 520;

		bool pos = true;

		turn_pid.set_gains(turn_pid_gains);

		turn_pid.target(target_heaidng);
		turn_pid.update(reading, milli(seconds)(20));

		double kp = 30;
		double ki = 0;
		double kd = 10;

		if (fabs(turn_pid.get_error().in(au::degrees)) < 40) {
			maxAccel = 1000;
			maxVelo = 450;
			kp = 30;
			ki = 10;
		} else if (fabs(turn_pid.get_error().in(au::degrees)) < 100) {
			kp = 25;
			ki = 0;
			ki = 7;
		}

		dlib::PidGains smol_turn_gains {
		kp, 	// kp, porportional gain
		ki, 	// ki, integral gain
		kd 	// kd, derivative gain
		};

		auto val = turn_pid.get_error();
		

		if (turn_pid.get_error().in(au::degrees) < 0) {
			val = -turn_pid.get_error();
			pos = false;
		}
		dlib::TrapezoidProfile<Degrees> turnTrapProfile = dlib::TrapezoidProfile<Degrees>((au::degrees_per_second_squared)(maxAccel), (au::degrees_per_second)(maxVelo), val);
	

		int cycle = 0;
		while (turnTrapProfile.stage(elapsed_time) != dlib::TrapezoidProfileStage::Done || cycle <= 20) {
			elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
			reading = (au::degrees)(imu.raw.get_rotation());

			auto turnSetpoint = turnTrapProfile.calculate(elapsed_time);

			

			
			
			turn_pid.target(turnSetpoint.position + startHeading);

			auto targVelo = turnSetpoint.velocity;
			auto targAccel = turnSetpoint.acceleration;

			if (!pos) {
				turn_pid.target(-turnSetpoint.position + startHeading);
			}


			// std::cout << targVelo << std::endl;

			
			

			auto ffwdVolts = ffwd.calculate(targVelo, targAccel);

			if (!pos) {
				ffwdVolts = -ffwdVolts;
			}

			auto pidVoltage = turn_pid.update(reading, milli(seconds)(20));

			
			if (turnTrapProfile.stage(elapsed_time) == dlib::TrapezoidProfileStage::Done) {

				if (turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {
					// break;
				}

				turn_pid.set_gains(smol_turn_gains);
				cycle++;

				pidVoltage = turn_pid.update(reading, milli(seconds)(20));
				chassis.turn_voltage(pidVoltage);

				pros::delay(20);
				continue;
			}
			

			
			if (pos && ffwdVolts < (au::volts)(-0.25)) {
				ffwdVolts = (au::volts)(-0.25);
			}

			if (!pos && ffwdVolts > (au::volts)(0.25)) {
				ffwdVolts = (au::volts)(0.25);
			}
			


			auto voltage = ffwdVolts + pidVoltage;
			// Use only feedward output for now

			
			chassis.turn_voltage(voltage);


			

			std::cout 
			<< elapsed_time.in(au::milli(au::seconds)) 
			<< ", " << turnSetpoint.position.in(au::degrees) << ", " 
			<< ((reading-prevReading)/(au::seconds)(0.02)).in(au::degrees_per_second)
			<< ", " << turnSetpoint.velocity.in(au::degrees_per_second)
			<< ", " << turn_pid.get_error()
			<< ", " << voltage << std::endl;

			prevReading = reading;

			pros::delay(20);
		}

		
		/*

		turn_pid.set_gains(smol_turn_gains);

		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative()) || cycle < 10) {
			cycle++;
			reading = (au::degrees)(imu.raw.get_rotation());
			auto pidVoltage = turn_pid.update(reading, milli(seconds)(20));


			chassis.turn_voltage(pidVoltage);

			pros::delay(20);
		}
		*/
		
		
		chassis.turn_voltage((au::volts)(0));
		chassis.brake();
	}

	void ffwLat(Quantity<Meters, double> displacement, Quantity<Seconds, double> timeout) {
		auto start_time = au::milli(au::seconds)(pros::millis());
  		auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
		auto start_displacement = (rotationLeft.get_linear_displacement() + rotationRight.get_linear_displacement())/2.0;
		auto target_displacement = start_displacement + displacement;
		auto reading = start_displacement;
		bool reverse = false;
		

		move_pid.target(target_displacement);
		move_pid.update(reading, milli(seconds)(20));

		// MAX VELO: 1.8222920590859724
		// MAX ACCEL: 6

		if (displacement < au::ZERO) {
			displacement = -displacement;
			reverse = true;
		}
		dlib::TrapezoidProfile<Meters> forwardTrapProfile = dlib::TrapezoidProfile<Meters>((au::meters_per_second_squared)(3), (au::meters_per_second)(1.6), displacement);

		// TODO: MAKE GOOD SETTLE CONDITION
		int cycle = 0;
		while (forwardTrapProfile.stage(elapsed_time) != dlib::TrapezoidProfileStage::Done) {
			cycle++;
			elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
			reading = (rotationLeft.get_linear_displacement() + rotationRight.get_linear_displacement())/2.0;

			auto setpoint = forwardTrapProfile.calculate(elapsed_time);
			
			if (reverse) {
				move_pid.target((-setpoint.position) + start_displacement);
			} else {
				move_pid.target(setpoint.position + start_displacement);
			}
			
			auto pidVoltage = move_pid.update(reading, milli(seconds)(20));


			auto voltage = linffwd.calculate(setpoint.velocity, setpoint.acceleration) + pidVoltage;
			if (reverse) {
				voltage = (-linffwd.calculate(setpoint.velocity, setpoint.acceleration)) + pidVoltage;
			}
			
			// Use only feedward output for now
			

			if (forwardTrapProfile.stage(elapsed_time) == dlib::TrapezoidProfileStage::Decelerating) {
				// vex is cooked?
				chassis.move_voltage(au::volts(0));
			}
			
			chassis.move_voltage(voltage);
			
			pros::delay(20);

			// ERR INFO
			// std::cout << elapsed_time.in(au::milli(au::seconds)) << ", " << move_pid.get_error() << ", " << pidVoltage << ", " << ffwdVoltage << std::endl;

			// DATA
			std::cout 
			<< elapsed_time.in(au::milli(au::seconds)) 
			<< ", " << setpoint.position.in(au::meters) << ", " 
			<< chassis.average_motor_velocity().in(au::meters_per_second) 
			<< ", " << setpoint.velocity.in(au::meters_per_second)
			<< ", " << move_pid.get_error().in(au::inches)
			<< ", " << voltage.in(au::volts) << std::endl;
			
		}

		chassis.move_voltage((au::volts)(0));
		chassis.brake();
		
	}

	void testStatic() {
		auto start_time = au::milli(au::seconds)(pros::millis());
  		auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;

		auto targetvelo= (au::degrees_per_second)(100);

		auto reading = imu.get_rotation();
		auto prevReading = reading;


		while (true) {
			elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
			reading = (au::degrees)(imu.raw.get_rotation());

			auto ffwdVoltage= ffwd.calculate((targetvelo), au::ZERO);
			// auto ffwdVoltage = (au::volts)(1.9);


			chassis.left_motors.raw.move_voltage(ffwdVoltage.in(au::milli(au::volts)));
			chassis.right_motors.raw.move_voltage(-ffwdVoltage.in(au::milli(au::volts)));


			// chassis.turn_voltage(ffwdVoltage);
			// ((reading-prevReading)/(au::seconds)(0.02)).in(au::degrees_per_second)
			std::cout << elapsed_time.in(au::milli(au::seconds)) << ", " 
			<<  ((reading-prevReading)/(au::seconds)(0.02)).in(au::degrees_per_second)
			<< ", " << targetvelo.in(au::degrees_per_second) << ", " 
			<< reading.in(au::degrees) << ", "
			<< ffwdVoltage.in(au::volts) << std::endl;
			prevReading = reading;
			pros::delay(20);
		}
		

	}

	void fwdQuasiStaticTest() {

		auto voltage = (au::volts)(0.0);

		auto start_time = pros::millis();
  		auto elapsed_time = 0;
		auto pastRot = rotationLeft.get_linear_displacement();
		auto curRot = rotationLeft.get_linear_displacement();
		int thing = 0;

		while (voltage < (au::volts)(12)) {
			thing++;
			auto current_time = pros::millis();
    		elapsed_time = current_time - start_time;

			voltage = au::milli(au::volts)(1000.0 * (elapsed_time / 1000.0)); // 1 volts/sec ramp

			chassis.move_voltage(voltage);  //deltaAngle/delT
			curRot = rotationLeft.get_linear_displacement();
			auto vel = rotationLeft.get_linear_velocity();

			if (thing % 2 == 0) {
				std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", " << vel <<std::endl;
			}
			
			pros::delay(10);
			pastRot = curRot;
		}
		chassis.move_voltage((au::volts)(0));

	}


	void fwdDynoTest() {
		auto voltage = (au::volts)(7.0);

		auto start_time = pros::millis();
  		auto elapsed_time = 0;
		auto pastRot = rotationLeft.get_linear_displacement();
		auto curRot = rotationLeft.get_linear_displacement();
		int thing = 0;
		auto vel = rotationLeft.get_linear_velocity();
		auto preVel = vel;
		

		while(elapsed_time < 10000) {
			thing++;
			auto current_time = pros::millis();
    		elapsed_time = current_time - start_time;

			// voltage = au::milli(au::volts)(1000.0 * (elapsed_time / 1000.0)); // 1 volts/sec ramp

			chassis.move_voltage(voltage);  //deltaAngle/delT
			curRot = rotationLeft.get_linear_displacement();

			// auto vel = (curRot-pastRot)/0.01;
			auto vel = rotationLeft.get_linear_velocity();
			auto accel = (vel-preVel)/0.01;

			if (thing % 2 == 0) {
				std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", " << vel << ", " << accel <<std::endl;
			}


			
			pros::delay(10);
			pastRot = curRot;
			preVel = vel;
		}

		chassis.move_voltage((au::volts)(0));

	}

	void turn_with_pid(Quantity<Degrees, double> heading) {
		auto target_heaidng = heading;
		auto reading = imu.get_rotation();

		turn_pid.target(target_heaidng);
		turn_pid.update(reading, milli(seconds)(20));
		int cycle = 0;
		int corCycle = 0;

		pros::delay(20);



		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())/* || corCycle < 2*/) {
			cycle++;
			reading = imu.get_rotation();
			auto voltage = turn_pid.update(reading, milli(seconds)(20));

			if (turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {
				pros::lcd::set_text(6, "TURN SETTLED " + std::to_string(cycle));
				corCycle++;
			} else {
				pros::lcd::set_text(6, "TURN NOT SETTLED " + std::to_string(cycle));
				corCycle = 0;
			}

			chassis.turn_voltage(voltage);

			pros::delay(20);
		}

		chassis.move(0);
	}

	void turn_to_point(dlib::Vector2d point) {
		auto angle = odom.angle_to(point);
		ffwTurn(angle);
	}

	void move_to_point(dlib::Vector2d point) {
		turn_to_point(point);

		auto displacement = odom.displacement_to(point);
		ffwLat(displacement, (au::seconds)(0));
	}

	// Odom task
	void start_odom() {
		rotationLeft.initialize();
		rotationLeft.raw.set_data_rate(10);
		rotationRight.initialize();
		rotationRight.raw.set_data_rate(10);

		
		odom_updater = std::make_unique<pros::Task>([this]() {
			while (true) {
				odom.update(
					rotationLeft.get_linear_displacement(), 
					rotationRight.get_linear_displacement(), 
					imu.get_rotation()
				);

				dlib::Pose2d curPos = odom.get_position();
				pros::lcd::print(0, "x: %f", (curPos.x).in(au::inches)); // print the x position
        		pros::lcd::print(1, "y: %f", (curPos.y).in(au::inches)); // print the y position
    			pros::lcd::print(2, "heading: %f", (curPos.theta).in(au::degrees)); // print the heading

				pros::delay(10);
				
			}
		});
		
	}

};

// Create a config for everything used in the Robot class
dlib::ChassisConfig chassis_config {
	{3, -5, 12},	// left motor ports - + -
	{-16, -17, 18},	// right motor ports + + -
	pros::MotorGearset::blue,
	rpm(480),	// the drivebase rpm
	inches(2.725)	// the drivebase wheel diameter
};

dlib::RotationConfig rotRight {
	1,
	inches(2.75),
	.8
};

dlib::RotationConfig rotLeft {
	9,
	inches(2.75),
	.8
};

//kv: 0.017650692106976215
		// ks: 1.4371121135741225
		// MAX VELO: 598.4404363526928 deg/s
// 0.91 kv for MAX potential

// .79 kv works up to 300 dps
// .35
dlib::FeedforwardGains TurnFFwdGains{
	1.9,
	0.95,
	0.4
};


//1.4724784904435986,
	// 6.225360763050551,
//1.026599683976744,
//	6.021757193809706,
//	0.4782196866053843
dlib::FeedforwardGains LinFFwdGains {
	1.026599683976744,
	6.021757193809706,
	1.1
};

dlib::ImuConfig imu_config {
	11,	// imu port
	1.00961546551	// optional imu scaling constant
};

dlib::PidGains move_pid_gains {
	200, 	// kp, porportional gain 10
	0, 	// ki, integral gain
	0 	// kd, derivative gain
};

dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(1),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.01) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::PidGains turn_pid_gains {
	30, 	// kp, porportional gain
	0, 	// ki, integral gain
	0.0 	// kd, derivative gain
};


dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(0.5),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(0.5)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

Robot robot = Robot(
	chassis_config,
	imu_config,
	move_pid_gains,
	move_pid_settler,
	turn_pid_gains,
	turn_pid_settler,
	rotRight,
	rotLeft,
	TurnFFwdGains,
	LinFFwdGains
);

void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
	robot.initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// robot.chassis.move_voltage((au::volts)(7));
	// robot.testStatic();

	// robot.ffwLat((au::inches)(-5), au::milli(au::seconds)(2000));
	// robot.turn_with_pid((au::degrees)(90));
	robot.ffwTurn((au::degrees)(90));
	// robot.move_with_pid((au::inches)(10));

	// robot.fwdDynoTest();
	// robot.turnQuasiStaticTest();
	// robot.turnDynoTest();

	// robot.turn_with_pid((degrees)(-90));
	// Try a movement!
	// X and Y are in centimeters.
	// robot.move_to_point({centi(meters)(60), centi(meters)(60)});
	
	
}

void opcontrol() {

	auto start_time = au::milli(au::seconds)(pros::millis());
  	auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
	auto curRot = robot.rotationLeft.get_position();
	auto pastRot = curRot;

	auto reading = robot.imu.get_rotation();
	auto prevReading = reading;
	
	// pros::delay(500);
	while(true){
		elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
		reading = robot.imu.get_rotation();
		
		pros::lcd::print(3, "LMTR: %f, RMTR: %f", robot.chassis.left_motors_displacement(), robot.chassis.right_motors_displacement());
		// robot.chassis.left_motors_displacement();
		// robot.chassis.right_motors_displacement();
		// Try arcade drive code!
		// pros::lcd::print(4, "pos: %f, %f, %f", LRAWMOTOR.get_position_all()[0], LRAWMOTOR.get_position_all()[1], LRAWMOTOR.get_position_all()[2]);
		// pros::lcd::set_text(4, std::to_string(robot.chassis.left_motors.raw.get_port_all()[0]) + "   " + std::to_string(robot.chassis.left_motors.raw.get_port_all()[1]) +
							// "   " + std::to_string(robot.chassis.left_motors.raw.get_port_all()[2]));
		
		// pros::lcd::set_text(5, std::to_string(std::vector(chassis_config.left_ports)[0]) + "   " + std::to_string(std::vector(chassis_config.left_ports)[1]) +
							// "   " + std::to_string(std::vector(chassis_config.left_ports)[2]));
		pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);

		// get power and turn
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);
		// arcade movement
		robot.chassis.arcade(power, turn);
		curRot = robot.rotationLeft.get_position();

		//robot.rotationLeft.get_linear_velocity().in(au::meters_per_second)
		//robot.rotationLeft.raw.get_velocity() / 360.0) * (480.0/600.0) * 60.0
		std::cout 
			<< elapsed_time.in(au::milli(au::seconds)) 
			<< ", " << ((reading-prevReading)/(au::seconds)(0.02)).in(au::degrees_per_second)
			<< ", " << turn << std::endl;

		pastRot = curRot;
		prevReading = reading;
		pros::delay(20);
	}
	
	
}