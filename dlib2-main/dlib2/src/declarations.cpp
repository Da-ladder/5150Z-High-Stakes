#include "declarations.h"
#include "pros/distance.hpp"

using namespace pros;

// Declares both controllers (probs move into teleop)
pros::Controller master(pros::E_CONTROLLER_MASTER);

// buttons for helping make autos
pros::ADIDigitalIn xyBut('F');
pros::ADIDigitalIn turnBut('G');

// intake motor go brrr
pros::Motor intake(14);

// lift motor
MotorGroup lift({-2, 10}, pros::v5::MotorGears::green,
                pros::v5::MotorEncoderUnits::degrees);

// Declares the IMU
pros::IMU imu(11);

// Declares color sensor
pros::v5::Optical camLight(0);
pros::v5::Optical opt(1); // Ran out of ports

// Declares distance sensor
pros::v5::Distance goalOpt(13);
pros::v5::Distance ringOpt(6);
pros::v5::Distance stakeFinder(0);

// Declares rot sensors for odom
// vert should increase when moving foward
pros::Rotation vert(-4);
// horz should increase when moving to the right
pros::Rotation horiz(15);

// lift rot
pros::Rotation liftRot(1);

//////// EXPerimental VISION SENSOR ////////
pros::v5::Vision camDetect(2);
//////// EXPerimental VISION SENSOR ////////

using namespace au;

// TODO: Decide if we want to implement helper functions for some of these
// things nehh
void Robot::initialize() {
  chassis.initialize();
  imu.initialize();
  start_odom();
}

void Robot::move_with_pid(Quantity<Meters, double> displacement) {
  auto start_displacement = -rotationLeft.get_linear_displacement();
  auto target_displacement = start_displacement + displacement;

  move_pid.target(target_displacement);
  while (!move_settler.is_settled(move_pid.get_error(),
                                  move_pid.get_derivative())) {
    auto reading = -rotationLeft.get_linear_displacement();
    auto voltage = move_pid.update(reading, milli(seconds)(20));

    chassis.move_voltage(voltage);
    pros::delay(20);
  }
  std::cout << "thing" << std::endl;
}

void Robot::turnDynoTest() {
  auto voltage = (au::volts)(7.0);
  auto start_time = pros::millis();
  auto elapsed_time = 0;
  auto pastRot = imu.get_rotation();
  auto curRot = imu.get_rotation();
  int thing = 0;
  auto vel = (curRot - pastRot) / 0.01;
  auto preVel = vel;

  while (elapsed_time < 10000) {
    thing++;
    auto current_time = pros::millis();
    elapsed_time = current_time - start_time;
    // voltage = au::milli(au::volts)(1000.0 * (elapsed_time / 1000.0)); // 1
    // volts/sec ramp
    chassis.turn_voltage(voltage); // deltaAngle/delT
    curRot = imu.get_rotation();
    auto vel = (curRot - pastRot) / 0.01;
    auto accel = (vel - preVel) / 0.01;
    if (thing % 2 == 0) {
      std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                << (curRot - pastRot) / 0.01 << ", " << accel << std::endl;
    }
    pros::delay(10);
    pastRot = curRot;
    preVel = vel;
  }

  chassis.turn_voltage((au::volts)(0));
}

void Robot::turnQuasiStaticTest() {
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
    voltage = au::milli(au::volts)(1000.0 *
                                   (elapsed_time / 1000.0)); // 1 volts/sec ramp
    chassis.turn_voltage(voltage);                           // deltaAngle/delT
    curRot = imu.get_rotation();
    if (thing % 2 == 0) {
      std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                << (curRot - pastRot) / 0.01 << std::endl;
    }

    pros::delay(10);
    pastRot = curRot;
  }
  chassis.turn_voltage((au::volts)(0));
}

void Robot::ffwTurn(Quantity<Degrees, double> heading) {
  // EXP

  dlib::PidGains turn_pid_gains{
      30, // kp, porportional gain
      0,  // ki, integral gain
      0.0 // kd, derivative gain
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

  dlib::PidGains smol_turn_gains{
      kp, // kp, porportional gain
      ki, // ki, integral gain
      kd  // kd, derivative gain
  };

  auto val = turn_pid.get_error();

  if (turn_pid.get_error().in(au::degrees) < 0) {
    val = -turn_pid.get_error();
    pos = false;
  }
  dlib::TrapezoidProfile<Degrees> turnTrapProfile =
      dlib::TrapezoidProfile<Degrees>(
          (au::degrees_per_second_squared)(maxAccel),
          (au::degrees_per_second)(maxVelo), val);

  int cycle = 0;
  while (turnTrapProfile.stage(elapsed_time) !=
             dlib::TrapezoidProfileStage::Done ||
         cycle <= 20) {
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

    if (turnTrapProfile.stage(elapsed_time) ==
        dlib::TrapezoidProfileStage::Done) {

      if (turn_settler.is_settled(turn_pid.get_error(),
                                  turn_pid.get_derivative())) {
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

    std::cout << elapsed_time.in(au::milli(au::seconds)) << ", "
              << turnSetpoint.position.in(au::degrees) << ", "
              << ((reading - prevReading) / (au::seconds)(0.02))
                     .in(au::degrees_per_second)
              << ", " << turnSetpoint.velocity.in(au::degrees_per_second)
              << ", " << turn_pid.get_error() << ", " << voltage << std::endl;

    prevReading = reading;

    pros::delay(20);
  }

  chassis.turn_voltage((au::volts)(0));
  chassis.brake();
}

void Robot::ffwLat(Quantity<Meters, double> displacement,
              Quantity<Seconds, double> timeout) {
    auto start_time = au::milli(au::seconds)(pros::millis());
    auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
    auto start_displacement = (rotationLeft.get_linear_displacement() +
                               rotationRight.get_linear_displacement()) /
                              2.0;
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
    dlib::TrapezoidProfile<Meters> forwardTrapProfile =
        dlib::TrapezoidProfile<Meters>((au::meters_per_second_squared)(3),
                                       (au::meters_per_second)(1.6),
                                       displacement);

    // TODO: MAKE GOOD SETTLE CONDITION
    int cycle = 0;
    while (forwardTrapProfile.stage(elapsed_time) !=
           dlib::TrapezoidProfileStage::Done) {
      cycle++;
      elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
      reading = (rotationLeft.get_linear_displacement() +
                 rotationRight.get_linear_displacement()) /
                2.0;

      auto setpoint = forwardTrapProfile.calculate(elapsed_time);

      if (reverse) {
        move_pid.target((-setpoint.position) + start_displacement);
      } else {
        move_pid.target(setpoint.position + start_displacement);
      }

      auto pidVoltage = move_pid.update(reading, milli(seconds)(20));

      auto voltage =
          linffwd.calculate(setpoint.velocity, setpoint.acceleration) +
          pidVoltage;
      if (reverse) {
        voltage =
            (-linffwd.calculate(setpoint.velocity, setpoint.acceleration)) +
            pidVoltage;
      }

      // Use only feedward output for now

      if (forwardTrapProfile.stage(elapsed_time) ==
          dlib::TrapezoidProfileStage::Decelerating) {
        // vex is cooked?
        chassis.move_voltage(au::volts(0));
      }

      chassis.move_voltage(voltage);

      pros::delay(20);

      // ERR INFO
      // std::cout << elapsed_time.in(au::milli(au::seconds)) << ", " <<
      // move_pid.get_error() << ", " << pidVoltage << ", " << ffwdVoltage <<
      // std::endl;

      // DATA
      std::cout << elapsed_time.in(au::milli(au::seconds)) << ", "
                << setpoint.position.in(au::meters) << ", "
                << chassis.average_motor_velocity().in(au::meters_per_second)
                << ", " << setpoint.velocity.in(au::meters_per_second) << ", "
                << move_pid.get_error().in(au::inches) << ", "
                << voltage.in(au::volts) << std::endl;
    }

    chassis.move_voltage((au::volts)(0));
    chassis.brake();
  }

void Robot::testStatic() {
    auto start_time = au::milli(au::seconds)(pros::millis());
    auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;

    auto targetvelo = (au::degrees_per_second)(100);

    auto reading = imu.get_rotation();
    auto prevReading = reading;

    while (true) {
      elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
      reading = (au::degrees)(imu.raw.get_rotation());

      auto ffwdVoltage = ffwd.calculate((targetvelo), au::ZERO);
      // auto ffwdVoltage = (au::volts)(1.9);

      chassis.left_motors.raw.move_voltage(
          ffwdVoltage.in(au::milli(au::volts)));
      chassis.right_motors.raw.move_voltage(
          -ffwdVoltage.in(au::milli(au::volts)));

      // chassis.turn_voltage(ffwdVoltage);
      // ((reading-prevReading)/(au::seconds)(0.02)).in(au::degrees_per_second)
      std::cout << elapsed_time.in(au::milli(au::seconds)) << ", "
                << ((reading - prevReading) / (au::seconds)(0.02))
                       .in(au::degrees_per_second)
                << ", " << targetvelo.in(au::degrees_per_second) << ", "
                << reading.in(au::degrees) << ", " << ffwdVoltage.in(au::volts)
                << std::endl;
      prevReading = reading;
      pros::delay(20);
    }
  }

void Robot::fwdQuasiStaticTest() {

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

      voltage = au::milli(au::volts)(
          1000.0 * (elapsed_time / 1000.0)); // 1 volts/sec ramp

      chassis.move_voltage(voltage); // deltaAngle/delT
      curRot = rotationLeft.get_linear_displacement();
      auto vel = rotationLeft.get_linear_velocity();

      if (thing % 2 == 0) {
        std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                  << vel << std::endl;
      }

      pros::delay(10);
      pastRot = curRot;
    }
    chassis.move_voltage((au::volts)(0));
  }

void Robot::fwdDynoTest() {
    auto voltage = (au::volts)(7.0);

    auto start_time = pros::millis();
    auto elapsed_time = 0;
    auto pastRot = rotationLeft.get_linear_displacement();
    auto curRot = rotationLeft.get_linear_displacement();
    int thing = 0;
    auto vel = rotationLeft.get_linear_velocity();
    auto preVel = vel;

    while (elapsed_time < 10000) {
      thing++;
      auto current_time = pros::millis();
      elapsed_time = current_time - start_time;

      // voltage = au::milli(au::volts)(1000.0 * (elapsed_time / 1000.0)); // 1
      // volts/sec ramp

      chassis.move_voltage(voltage); // deltaAngle/delT
      curRot = rotationLeft.get_linear_displacement();

      // auto vel = (curRot-pastRot)/0.01;
      auto vel = rotationLeft.get_linear_velocity();
      auto accel = (vel - preVel) / 0.01;

      if (thing % 2 == 0) {
        std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                  << vel << ", " << accel << std::endl;
      }

      pros::delay(10);
      pastRot = curRot;
      preVel = vel;
    }

    chassis.move_voltage((au::volts)(0));
  }

void Robot::turn_with_pid(Quantity<Degrees, double> heading) {
    auto target_heaidng = heading;
    auto reading = imu.get_rotation();

    turn_pid.target(target_heaidng);
    turn_pid.update(reading, milli(seconds)(20));
    int cycle = 0;
    int corCycle = 0;

    pros::delay(20);

    while (!turn_settler.is_settled(
        turn_pid.get_error(), turn_pid.get_derivative()) /* || corCycle < 2*/) {
      cycle++;
      reading = imu.get_rotation();
      auto voltage = turn_pid.update(reading, milli(seconds)(20));

      if (turn_settler.is_settled(turn_pid.get_error(),
                                  turn_pid.get_derivative())) {
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

void Robot::turn_to_point(dlib::Vector2d point) {
    auto angle = odom.angle_to(point);
    ffwTurn(angle);
  }

void Robot::move_to_point(dlib::Vector2d point, bool turn, bool fowards) {
    if (turn) {
        turn_to_point(point);
    }

    auto displacement = odom.displacement_to(point);

    if (!fowards) {
        ffwLat(-displacement, (au::seconds)(0));
    } else {
        ffwLat(displacement, (au::seconds)(0));
    }
    
  }

  // Odom task
void Robot::start_odom() {
    rotationLeft.initialize();
    rotationLeft.raw.set_data_rate(10);
    rotationRight.initialize();
    rotationRight.raw.set_data_rate(10);

    odom_updater = std::make_unique<pros::Task>([this]() {
      while (true) {
        odom.update(rotationLeft.get_linear_displacement(),
                    rotationRight.get_linear_displacement(),
                    imu.get_rotation());

        dlib::Pose2d curPos = odom.get_position();
        pros::lcd::print(0, "x: %f",
                         (curPos.x).in(au::inches)); // print the x position
        pros::lcd::print(1, "y: %f",
                         (curPos.y).in(au::inches)); // print the y position
        pros::lcd::print(2, "heading: %f",
                         (curPos.theta).in(au::degrees)); // print the heading

        pros::delay(10);
      }
    });
  }




// Create a config for everything used in the Robot class
dlib::ChassisConfig chassis_config{
    {3, -5, 12},    // left motor ports - + -
    {-16, -17, 18}, // right motor ports + + -
    pros::MotorGearset::blue,
    rpm(480),     // the drivebase rpm
    inches(2.725) // the drivebase wheel diameter
};

dlib::RotationConfig rotRight{1, inches(2.75), .8};

dlib::RotationConfig rotLeft{9, inches(2.75), .8};

// kv: 0.017650692106976215
//  ks: 1.4371121135741225
//  MAX VELO: 598.4404363526928 deg/s
// 0.91 kv for MAX potential

// .79 kv works up to 300 dps
// .35
dlib::FeedforwardGains TurnFFwdGains{1.9, 0.95, 0.4};

// 1.4724784904435986,
//  6.225360763050551,
// 1.026599683976744,
//	6.021757193809706,
//	0.4782196866053843
dlib::FeedforwardGains LinFFwdGains{1.026599683976744, 6.021757193809706, 1.1};

dlib::ImuConfig imu_config{
    11,           // imu port
    1.00961546551 // optional imu scaling constant
};

dlib::PidGains move_pid_gains{
    200, // kp, porportional gain 10
    0,   // ki, integral gain
    0    // kd, derivative gain
};

dlib::ErrorDerivativeSettler<Meters> move_pid_settler{
    inches(1), // error threshold, the maximum error the pid can settle at
    meters_per_second(0.01) // derivative threshold, the maximum instantaneous
                            // error over time the pid can settle at
};

dlib::PidGains turn_pid_gains{
    30, // kp, porportional gain
    0,  // ki, integral gain
    0.0 // kd, derivative gain
};

dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler{
    degrees(0.5), // error threshold, the maximum error the pid can settle at
    degrees_per_second(0.5) // derivative threshold, the maximum instantaneous
                            // error over time the pid can settle at
};

Robot robot = Robot(chassis_config, imu_config, move_pid_gains,
                    move_pid_settler, turn_pid_gains, turn_pid_settler,
                    rotRight, rotLeft, TurnFFwdGains, LinFFwdGains);