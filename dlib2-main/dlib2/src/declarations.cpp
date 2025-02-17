#include "declarations.h"
#include "au/au.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>
#include <string>

using namespace pros;

// Declares both controllers (probs move into teleop)
pros::Controller master(pros::E_CONTROLLER_MASTER);

// buttons for helping make autos
pros::adi::DigitalIn xyBut('E');
pros::adi::DigitalIn turnBut('F'); //F
pros::adi::DigitalIn sortLimit('0');


// Line detectors
pros::adi::AnalogIn lineLeft = pros::adi::AnalogIn('H');
pros::adi::AnalogIn lineRight = pros::adi::AnalogIn('D');


// intake motor go brrr
pros::Motor intake(-20, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

// lift motor
MotorGroup lift({4}, pros::v5::MotorGears::red,
                pros::v5::MotorEncoderUnits::degrees);

// Declares the IMU
// pros::IMU imu(0);

// Declares color sensor
pros::v5::Optical camLight(0);
pros::v5::Optical opt(10);

// Declares distance sensor
pros::v5::Distance goalOpt(6); //ohhhhh
pros::v5::Distance ringOpt(0);
pros::v5::Distance stakeFinder(0);

// Declares rot sensors for odom
// vert should increase when moving foward
pros::Rotation vert(0);
// horz should increase when moving to the right
pros::Rotation horiz(0);

// lift rot
pros::Rotation liftRot(9);

//////// EXPerimental VISION SENSOR ////////  
pros::v5::Vision camDetect(5);
pros::v5::Vision camRingDetect(2); // REPLACED BY LB
//////// EXPerimental VISION SENSOR ////////

using namespace au;

// TODO: Decide if we want to implement helper functions for some of these

bool Robot::detectLine = false;

void Robot::initialize() {
  chassis.initialize();
  imu.initialize();
  start_odom();
}

void Robot::move_with_pid(Quantity<Meters, double> displacement) {
  auto start_displacement = rotationLeft.get_linear_displacement();
  auto target_displacement = start_displacement + displacement;

  lin_pid.target(target_displacement);
  while (!move_settler.is_settled(move_pid.get_error(),
                                  move_pid.get_derivative())) {
    auto reading = rotationLeft.get_linear_displacement();
    auto voltage = lin_pid.update(reading, milli(seconds)(20));

    chassis.move_voltage(voltage);
    pros::delay(20);
    // std::cout << voltage << std::endl;
  }
  std::cout << "done" << std::endl;
  
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

  std::cout << "DYNO TEST @ " << voltage.in(au::volts) << " volts for 10 secs" << std::endl;

  while (elapsed_time < 10000) {
    thing++;
    auto current_time = pros::millis();
    elapsed_time = current_time - start_time;
    // voltage = au::milli(au::volts)(1000.0 * (elapsed_time / 1000.0)); // 1
    // volts/sec ramp
    chassis.turn_voltage(voltage); // deltaAngle/delT
    curRot = imu.get_rotation();
    auto vel = (curRot - pastRot) / 0.02;
    auto accel = (vel - preVel) / 0.02;
    if (false) {
      std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                << (curRot - pastRot) / 0.02 << ", " << accel << std::endl;
    }

    if (true) {
        std::cout << "(" << elapsed_time/1000.0 << ", " << (curRot - pastRot) * 50 << ")" << std::endl;
    }
    pros::delay(20);
    pastRot = curRot;
    preVel = vel;
  }

  chassis.turn_voltage((au::volts)(0));
}

void Robot::turnQuasiStaticTest() {
  auto voltage = (au::volts)(0.0);
  auto start_time = pros::millis();
  auto elapsed_time = 0;

  auto linLpastRot = rotationLeft.get_linear_displacement();
  auto linRpastRot = rotationRight.get_linear_displacement();
  auto linLcurRot = rotationLeft.get_linear_displacement();
  auto linRcurRot = rotationRight.get_linear_displacement();
  double linAngle = 0;
  double pastLinAngle = 0;
  double velo = 0;


  auto pastRot = imu.get_rotation();
  auto curRot = imu.get_rotation();
  int thing = 0;

  std::cout << "QUASI TEST @ 0-12 volts for 24 secs" << std::endl;

  while (voltage < (au::volts)(12)) {
    thing++;
    auto current_time = pros::millis();
    elapsed_time = current_time - start_time;
    voltage = au::milli(au::volts)(500.0 * (elapsed_time / 1000.0)); // .5 volts/sec ramp
    
    // voltage = au::milli(au::volts)(4000);

    chassis.turn_voltage(voltage);                           // deltaAngle/delT
    curRot = imu.get_rotation();


    linLcurRot = rotationLeft.get_linear_displacement();
    linRcurRot = rotationRight.get_linear_displacement();

    // linAngle += ((((linRcurRot-linRpastRot) - (linLcurRot-linLpastRot)).in(au::inches)) / 11.25) * (180/3.1415926535897932384626);


    if (false) {
      std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                << (curRot - pastRot) / 0.02 
                << ", "
                << (linAngle-pastLinAngle) / 0.02 << std::endl;
    }

    if (true) {
        std::cout << "(" << voltage.in(au::volts) << ", " << (curRot-pastRot)*50 << ")" << std::endl;
    }

    pros::delay(20);
    linLpastRot = linLcurRot;
    linRpastRot = linRcurRot;
    pastLinAngle = linAngle;
    pastRot = curRot;
  }
  chassis.turn_voltage((au::volts)(0));
}

void Robot::ffwTurn(Quantity<Degrees, double> heading) {
  // EXP
  
  /*
  dlib::PidGains turn_pid_gains{
      0, // kp, porportional gain //30
      0, // ki, integral gain
      0 // kd, derivative gain
  };
  */

  dlib::FeedforwardGains TurnDecelFFwdGains{
  1.75, //1.75
  0.7, //0.88
  0.15}; // ka 0.2
  // EXP END
  auto start_time = au::milli(au::seconds)(pros::millis());
  auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
  auto target_heaidng = heading;
  auto reading = imu.get_rotation();
  auto prevReading = reading;
  auto startHeading = reading;
  int maxAccel = 900; // 1400 MAX NO MOGO 1200
  // 1000
  int maxDecel = 900;
  // 700
  int maxVelo = 400; // 520 MAX NO MOGO

  bool pos = true;
  bool moarBrake = false;

  // turn_pid.set_gains(turn_pid_gains);

  turn_pid.target(target_heaidng);
  turn_pid.update(reading, milli(seconds)(10));

  // double kp = 5;
  // double ki = 0;
  // double kd = 0;
  

  dlib::PidGains decel_turn_gains{
      0, // kp, porportional gain //5
      0, // ki, integral gain
      0 // kd, derivative gain
  };

  auto val = turn_pid.get_error();

  if (turn_pid.get_error().in(au::degrees) < 0) {
    val = -turn_pid.get_error();
    pos = false;
  }
  dlib::TrapezoidProfile<Degrees> turnTrapProfile =
      dlib::TrapezoidProfile<Degrees>(
          (au::degrees_per_second_squared)(maxAccel),
          (au::degrees_per_second_squared)(maxDecel),
          (au::degrees_per_second)(maxVelo), val);

  int cycle = 0;
  // 

  while (turnTrapProfile.stage(elapsed_time) != dlib::TrapezoidProfileStage::Done) {
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
    if (turnTrapProfile.stage(elapsed_time) ==
          dlib::TrapezoidProfileStage::Decelerating) {
        // vex is cooked?
        // ffwd.set_gains(TurnDecelFFwdGains);
        // chassis.turn_voltage(au::volts(0));
    }

    auto ffwdVolts = ffwd.calculate(targVelo, targAccel);
    

    if (ffwdVolts == (au::volts)(1.75)) {
      ffwdVolts = au::Zero();
    }

    if (!pos) {
      ffwdVolts = -ffwdVolts;
    }

    //auto pidVoltage = turn_pid.update(reading, milli(seconds)(20));
    auto pidVoltage = ZERO;
    
    auto voltage = ffwdVolts + pidVoltage;
    chassis.turn_voltage(voltage);
    
    // Use only feedward output for now
    pros::delay(20);

    

    
    
    std::cout << elapsed_time.in(au::milli(au::seconds)) << ", "
              << turnSetpoint.position.in(au::degrees) << ", "
              << ((reading - prevReading) / (au::seconds)(0.02))
                     .in(au::degrees_per_second)
              << ", " << turnSetpoint.velocity.in(au::degrees_per_second)
              << ", " << turn_pid.get_error() 
              << ", " << reading.in(au::degrees)
              << ", " << pidVoltage << std::endl;
    
    prevReading = reading;
  }

  chassis.turn_voltage((au::volts)(0));
  // chassis.move_voltage((au::volts)(0));
  chassis.brake();
}

void Robot::ffwLat(Quantity<Meters, double> displacement,
              Quantity<Seconds, double> timeout, double maxAccel) {
    auto start_time = au::milli(au::seconds)(pros::millis());
    auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
    auto start_displacement = (rotationLeft.get_linear_displacement() +
                               rotationRight.get_linear_displacement()) /
                              2.0;
    auto target_displacement = start_displacement + displacement;
    auto reading = start_displacement;
    bool reverse = false;

    // Write to USB

    // FILE* usd_file_write = fopen("/usd/lastRoute.txt", "a");
	  // fputs("Start ffwLat", usd_file_write);

    move_pid.target(target_displacement); //Making all negative
    move_pid.update(reading, milli(seconds)(20));

    // MAX VELO: 1.8222920590859724
    // MAX ACCEL: 6

    if (displacement < au::ZERO) {
      displacement = -displacement;
      reverse = true;
    }
    dlib::TrapezoidProfile<Meters> forwardTrapProfile =
        dlib::TrapezoidProfile<Meters>(
        (au::meters_per_second_squared)(maxAccel), //3
        (au::meters_per_second_squared)(maxAccel),
        (au::meters_per_second)(1.65), //1.6 kinda low 1.7 med 1.8 high EXP!!!!
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
      // auto pidVoltage = ZERO;

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
        //chassis.move_voltage(au::volts(0));
      }

      chassis.move_voltage(voltage);

      pros::delay(20);

      // ERR INFO
      // std::cout << elapsed_time.in(au::milli(au::seconds)) << ", " <<
      // move_pid.get_error() << ", " << pidVoltage << ", " << ffwdVoltage <<
      // std::endl;

      // LINE DETECT START
      if (detectLine && (lineLeft.get_value() < 700 || lineRight.get_value() < 700)) {
        break;
      }
      
      // LINE DETECT END

      // DATA
      /*
      std::cout << elapsed_time.in(au::milli(au::seconds)) << ", "
                << setpoint.position.in(au::inches) << ", "
                << (rotationLeft.get_linear_velocity().in(au::meters_per_second) +
                rotationRight.get_linear_velocity().in(au::meters_per_second)) / 2.0
                << ", " << setpoint.velocity.in(au::meters_per_second) << ", "
                << move_pid.get_error().in(au::inches) << ", "
                << voltage.in(au::volts) << std::endl;
      /*
      dlib::Pose2d curPos = odom.get_position();
      fprintf(usd_file_write, "%s", (std::to_string(curPos.x.in(au::inches)) + ", " +
                              std::to_string(curPos.y.in(au::inches)) + ", " +
                              std::to_string(curPos.theta.in(au::degrees)) + ", " +
                              std::to_string(elapsed_time.in(au::milli(au::seconds))))
                              .data());
      */
    }
    // fputs(("End ffwLat: " + std::to_string(elapsed_time.in(au::milli(au::seconds)))).data(), usd_file_write);
    // fclose(usd_file_write);
    chassis.move_voltage((au::volts)(0));
    chassis.brake();
  }

void Robot::testStatic() {
    auto start_time = au::milli(au::seconds)(pros::millis());
    auto elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;

    auto targetvelo = (au::meters_per_second)(1); //9.28

    auto reading = imu.get_rotation();
    auto prevReading = reading;

    while (true) {
      elapsed_time = au::milli(au::seconds)(pros::millis()) - start_time;
      reading = (au::degrees)(imu.raw.get_rotation());

      auto ffwdVoltage = linffwd.calculate((targetvelo), au::ZERO);
      // auto ffwdVoltage = (au::volts)(12);

      chassis.left_motors.raw.move_voltage( 
          ffwdVoltage.in(au::milli(au::volts)));
      chassis.right_motors.raw.move_voltage(
          ffwdVoltage.in(au::milli(au::volts)));

      auto vel = rotationLeft.get_linear_displacement();

      if (true) {
        std::cout << "(" << elapsed_time.in(au::milli(au::seconds))/1000.0 << ", " << vel.in(au::meters) << ")" << std::endl;
      }

      // chassis.turn_voltage(ffwdVoltage);
      // ((reading-prevReading)/(au::seconds)(0.02)).in(au::degrees_per_second)
      /*
      std::cout << elapsed_time.in(au::milli(au::seconds)) << ", "
                << ((reading - prevReading) / (au::seconds)(0.1))
                       .in(au::degrees_per_second)
                << ", " << targetvelo.in(au::meters_per_second) << ", "
                << reading.in(au::degrees) << ", " << ffwdVoltage.in(au::volts)
                << std::endl;
      prevReading = reading;
      */
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

      if (false) {
        std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                  << vel << std::endl;
      }

      if (true) {
        std::cout << "(" << voltage.in(au::volts) << ", " << vel.in(au::meters_per_second) << ")" << std::endl;
      }

      pros::delay(20);
      pastRot = curRot;
    }
    chassis.move_voltage((au::volts)(0));
  }

void Robot::fwdDynoTest() {
    auto voltage = (au::volts)(12.0);

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

      if (false) {
        std::cout << elapsed_time << ", " << voltage << ", " << curRot << ", "
                  << vel << ", " << accel << std::endl;
      }

      if (true) {
        std::cout << "(" << elapsed_time/1000.0 << ", " << vel.in(au::meters_per_second) << ")" << std::endl;
      }

      pros::delay(20);
      pastRot = curRot;
      preVel = vel;
    }

    chassis.move_voltage((au::volts)(0));
  }

void Robot::turn_with_pid(double heading, int timeoutMS, double maxVolts) {
    auto target_heaidng = (au::degrees)(heading);
    auto reading = imu.get_rotation();
    

    turn_pid.target(target_heaidng);
    turn_pid.update(reading, milli(seconds)(20));

    // permaTest
    if (fabs(turn_pid.get_error().in(au::degrees)) <= 120 && timeoutMS < 680) {
      timeoutMS = 680;
    }
    int cycle = 0;
    int corCycle = 0;
    chassis.left_motors.raw.set_current_limit_all(2000); //???
    chassis.right_motors.raw.set_current_limit_all(2000); // ???

    // FILE* usd_file_write = fopen("/usd/lastRoute.txt", "a");
	  // fputs("Start turn_with_pid", usd_file_write);

    // pros::delay(20); // wut is this for?

    while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative()) && timeoutMS > cycle*20) {
      cycle++;
      reading = imu.get_rotation();
      auto voltage = turn_pid.update(reading, milli(seconds)(20));
      auto integral = turn_pid.get_integral();
      // std::cout << cycle*20 << ", " << turn_pid.get_error() << ", " << turn_pid.get_integral() << std::endl;

      if (turn_settler.is_settled(turn_pid.get_error(),
                                  turn_pid.get_derivative())) {
        pros::lcd::set_text(6, "TURN SETTLED " + std::to_string(cycle));
        corCycle++;
      } else {
        pros::lcd::set_text(6, "TURN NOT SETTLED " + std::to_string(cycle));
        corCycle = 0;
      }
      if (voltage.in(au::volts) > maxVolts) {
        voltage = (au::volts)(maxVolts);
      } else if (voltage.in(au::volts) < -maxVolts) {
        voltage = (au::volts)(-maxVolts);
      }

      if (voltage.in(au::volts) > 0) {
        chassis.turn_voltage(voltage + (au::volts)(0.6)); //0.75
      } else {
        chassis.turn_voltage(voltage - (au::volts)(0.6));
      }

      /*
      dlib::Pose2d curPos = odom.get_position();
      fprintf(usd_file_write, "%s", (std::to_string(curPos.x.in(au::inches)) + ", " +
                              std::to_string(curPos.y.in(au::inches)) + ", " +
                              std::to_string(curPos.theta.in(au::degrees)) + ", " +
                              std::to_string(cycle*20))
                              .data());
      */
    
      pros::delay(20);
    }
    // fputs(("End turn_with_pid: " + std::to_string(cycle*20)).data(), usd_file_write);
    // fclose(usd_file_write);
  
    chassis.move(0);
    chassis.left_motors.raw.set_current_limit_all(2500); //???
    chassis.right_motors.raw.set_current_limit_all(2500); // ???
  }

void Robot::turn_to_point(dlib::Vector2d point, bool mogoSide, int to, double maxVolts) {
    auto angle = odom.angle_to(point);
    double targAngle = angle.in(au::degrees);

    
    if (!mogoSide) { //270
      targAngle += 180;
      auto curRotation = imu.get_rotation().in(au::degrees);
      double error = targAngle - curRotation;
      
      if (fabs(error) > 180) {
        if (error > 0) {
          while (error > 180) {
            targAngle -= 360;

            error = targAngle - curRotation;
          }
        } else {
          while (error < -180) {
            targAngle += 360;

            error = targAngle - curRotation;
          }
        }
      }

      turn_with_pid(targAngle, to, maxVolts);
    } else {
      auto curRotation = imu.get_rotation().in(au::degrees);
      double error = targAngle - curRotation;

      if (fabs(error) > 180) {
        if (error > 0) {
          while (error > 180) {
            targAngle -= 360;

            error = targAngle - curRotation;
          }
        } else {
          while (error < -180) {
            targAngle += 360;

            error = targAngle - curRotation;
          }
        }
      }

      turn_with_pid(targAngle, to, maxVolts);
    }
    
  }

void Robot::move_to_point(dlib::Vector2d point, bool turn, bool fowards, int to, double maxAccel, double maxTurnVolts) {
    if (turn) {
        turn_to_point(point, fowards, to, maxTurnVolts);
    }

    auto displacement = odom.displacement_to(point);

    if (!fowards) {
        ffwLat(-displacement, (au::seconds)(0), maxAccel);
    } else {
        ffwLat(displacement, (au::seconds)(0), maxAccel);
    }
    
}

// can it use M_PI???
double sinc(double x) {
    if (x == 0.0) {
        return 1.0;
    }
    return std::sin(x) / (x);
}

double signdetect(double num) {
  if (std::signbit(num)) {
    return 1;
  } else {
    return -1;
  }
}

// fwds true = mogo side
// early_exit allows for waypoints to be set up (2 inches for waypoints??? 0.25 should be target for end)
void Robot::ramseteTest(dlib::Vector2d point, bool fowards, double max_voltage, double min_voltage, double early_exit, double slewStart, bool brake) {

  double track_width = 0.25; // stay in meters
  double k_lat = 2.3; // btwn 2.5-3
  chassis.left_motors.raw.set_current_limit_all(2200); //???
  chassis.right_motors.raw.set_current_limit_all(2200); // ???

  // double lastvL = 0, lastvR = 0;

  // setting up PIDs
  lin_pid.target((au::meters)(0)); // I think its supposed to be 0
  turn_pid.target((au::degrees)(0)); // I think its supposed to be 0

  // global errors in meters and degrees
  auto theta = (au::degrees)(0.0);
  auto x_g = (au::meters)(0.0);
  auto y_g = (au::meters)(0.0);

  // local errors in meters and degrees
  auto e_theta = (au::degrees)(0.0);
  auto e_x = (au::meters)(0.0);
  auto e_y = (au::meters)(0.0);

  // error from the point using pythagorean theorem
  auto tri_error = (au::meters)(0.0);

  // pid voltage outputs
  auto v_d = (au::volts)(0.0);
  auto a_velo = (au::volts)(0.0);

  // voltage output difference
  double l_volt = 0.0;
  double voltDiff = 0.0;

  // final left and right voltage output
  double v_L = 0.0;
  double v_R = 0.0;

  // scaling to ratio speed & voltage step rate
  double scaling_factor;
  double voltage_slew = slewStart; //4
  double step = 1.5; // +1 volt per 20 ms so 240 ms for 12 v

  // start the loop
  while (true) {
    dlib::Pose2d curPos = odom.get_position();
    // current theta and x, y error. should be in meters!!!!
    theta = curPos.theta;
    x_g = point.x - curPos.x;
    y_g = point.y - curPos.y;

    // local x & y error
    e_x = (cos(theta.in(au::radians)) * x_g) + (sin(theta.in(au::radians)) * y_g);
    e_y = ((-sin(theta.in(au::radians))) * x_g) + (cos(theta.in(au::radians)) * y_g);

    // local theta error dependent on which side of the bot we want facing the point
    if (!fowards) {
      e_theta = (au::radians)(atan2(-e_y.in(au::meters), -e_x.in(au::meters)));
    } else {
      e_theta = (au::radians)(atan2(e_y.in(au::meters), e_x.in(au::meters)));
    }


    // PID outputs volts
    tri_error = (au::meters)((sqrt(pow(e_x.in(au::meters), 2) + pow(e_y.in(au::meters), 2)) * signdetect(cos(e_theta.in(au::radians)))));
    v_d = lin_pid.update(tri_error, milli(seconds)(20));
    a_velo = turn_pid.update(e_theta, milli(seconds)(20));

    // calculates linear and angular voltages  
    l_volt = (fabs(cos(e_theta.in(au::radians))) * v_d.in(au::volts));
    voltDiff = ((a_velo.in(au::volts)/12.0) * k_lat * sinc(e_theta.in(au::radians)))/track_width;

    if (!fowards) {
      v_L = -l_volt + voltDiff; // l_velo should be - for intake side
      v_R = -l_volt - voltDiff; // l_velo should be - for intake side
    } else {
      v_L = l_volt + voltDiff;
      v_R = l_volt - voltDiff;
    }

    // slew rate control voltage slew over time
    voltage_slew += step;

    if (voltage_slew > max_voltage) {
      voltage_slew = max_voltage;
    }


    // Ratio left and right voltages if it exceeds user max_voltage
    // also ratios it if it is lower than min_voltage
    // possible for voltage to exceed user defined max_voltage or min_voltage
    // but it is more important to keep them at the same ratio than under the same limit.
    if ((fabs(v_L) > voltage_slew) || (fabs(v_R) > voltage_slew)) {
      if (fabs(v_L) > voltage_slew && fabs(v_L) > fabs(v_R)) {
        scaling_factor = voltage_slew / fabs(v_L);
      } else {
        scaling_factor = voltage_slew / fabs(v_R);
      }
      v_L = v_L * scaling_factor;
      v_R = v_R * scaling_factor;
    } else if ((fabs(v_L) < min_voltage) || (fabs(v_R) < min_voltage)) {
      if (v_L < min_voltage) {
        scaling_factor = min_voltage / fabs(v_L);
      } else {
        scaling_factor = min_voltage / fabs(v_R);
      }
      v_L = v_L * scaling_factor;
      v_R = v_R * scaling_factor;
    }

    // move the motors
    chassis.left_motors.raw.move_voltage(v_L*1000);
    chassis.right_motors.raw.move_voltage(v_R*1000);

    
    // "Settle" condition. Try to reduce error to .25 or +-.25 in either x or y
    if (fabs(tri_error.in(au::inches)) < early_exit) {
      break;
    }
    
    // debug statements
    // std::cout << "(" << curPos.x.in(au::inches) << "," << curPos.y.in(au::inches) << ")" << std::endl;
    std::cout << "(" << e_theta << ")" << std::endl;
    pros::delay(20);
  }

  if (brake) {
    chassis.brake(); // test with coast
  }
  chassis.left_motors.raw.set_current_limit_all(2500); //???
  chassis.right_motors.raw.set_current_limit_all(2500); // ???

}

// test how far lookahead should be
void Robot::ramseteFollow(std::vector<dlib::Vector2d>* pointList, int timeout, double lookahead, bool fowards, double maxSpeed, double minSpeed) {
  int start_time = pros::millis();

  robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
  robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);

  // run ramsete for each point
  for (dlib::Vector2d point : *pointList) {
    if (point.x == pointList->back().x && point.y == pointList->back().y) {
      // sets brake to coast
      robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
      robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);

      ramseteTest(point, fowards, maxSpeed, minSpeed, 0.7, maxSpeed); // end off 0.7 inches from the pt
    } else {
      double slewStr = (pros::millis() - start_time)/20.0 * 1.5 + 4;
      if (slewStr > maxSpeed) {
        slewStr = maxSpeed;
      }
      ramseteTest(point, fowards, maxSpeed, minSpeed, lookahead, slewStr, false);
    }
  }
}

// uses path from jerry.io

void Robot::refinedFollow(std::vector<dlib::Pose2d>* pointList, int timeout, double lookahead, bool fowards, double maxSpeed, double minSpeed) {
  int start_time = pros::millis();

  // run ramsete for each point with degrees as the max voltage
  for (dlib::Pose2d point : *pointList) {
    if (point.x == pointList->back().x && point.y == pointList->back().y) {
      maxSpeed = point.theta.in(au::degrees)/100.0 * 12;
      ramseteTest({point.x, point.y}, fowards, maxSpeed, minSpeed, 0.7, maxSpeed); // end off 0.7 inches from the pt
    } else {
      maxSpeed = point.theta.in(au::degrees)/100.0 * 12;
      double slewStr = (pros::millis() - start_time)/20.0 * 1.5 + 4;
      if (slewStr > maxSpeed) {
        slewStr = maxSpeed;
      }
      ramseteTest({point.x, point.y}, fowards, maxSpeed, minSpeed, lookahead, slewStr);
    }
  }
  robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
  robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
  chassis.brake();
}




void Robot::turn_ffwd(double time){
  // chassis.left_motors.raw.set_current_limit_all(2400);
  // chassis.right_motors.raw.set_current_limit_all(2400);

  /*double a = -1844.1699;
  double k = -4.71929;*/ // for ccw 8v
  
  double a = 1984.88275;
  double k = -3.99229;

  time /= 1000.0;
  
  if(time >= 0){
      auto start_time = pros::millis();

      // turn_pid.reset();
      turn_settler.reset();

      while(true) {
          auto elapsed_time = (pros::millis() - start_time)/1000.0;

          auto heading = ((a * (std::exp(k * elapsed_time) - k * elapsed_time))) / (k * k) - (a / (k * k));

          
          auto velocity = (a / k) * (std::exp(k * time)) - a/k;
          auto acceleration = a * (std::exp(k * time));
          
          auto error = degrees(heading) - imu.get_rotation();
          auto voltage = turn_pid.update(error, milli(seconds)(20));
          
          std::cout 
          << elapsed_time << ", " 
          << heading << ", " 
          << imu.get_rotation() << ", "
          << error << ", " 
          << (au::volts)(error.in(au::degrees) * 0.1) << ", "
          << std::endl;
          //std::cout << voltage  << std::endl;
          
          
          if(elapsed_time >= time){
              std::cout << "broke" << std::endl;
              break;
          }

          if (elapsed_time < 0.2) {
            chassis.turn_voltage(volts(8.0));// + (au::volts)(error.in(au::degrees) * 0.8));
          } else {
            // chassis.left_motors.raw.set_current_limit_all(2300);
            // chassis.right_motors.raw.set_current_limit_all(2300);
            chassis.turn_voltage(volts(7.5) + (au::volts)(error.in(au::degrees) * 0.1)); //0.3
          }

          
          
          // (au::volts)(error.in(au::degrees) * 1.5)


          pros::delay(20);
      }

      std::cout << imu.get_rotation() << std::endl;
      //chassis.turn_voltage(volts(12));
      //pros::delay(60);
      chassis.brake();
  }
}

  // Odom task
void Robot::start_odom() {

    chassis.initialize();
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

void Robot::restOdom(double x, double y, double theta) {
  dlib::Pose2d thing = {(au::inches)(x), (au::inches)(y), (au::degrees)(theta)};
  odom.set_position(thing);
}

void Robot::restOdomKeepAngle(double x, double y) {
  dlib::Pose2d curPos = odom.get_position();
  dlib::Pose2d thing = {(au::inches)(x), (au::inches)(y), curPos.theta};
  odom.set_position(thing);
}




// Create a config for everything used in the Robot class
dlib::ChassisConfig chassis_config{
    {11, -12, 13},    // left motor ports - + -
    {15, -16, -14}, // right motor ports + + -
    pros::MotorGearset::blue,
    rpm(600),     // the drivebase rpm
    inches(2.725) // the drivebase wheel diameter
};

dlib::RotationConfig rotRight{-17, inches(2.75), 1};

dlib::RotationConfig rotLeft{8, inches(2.75), 1};

// kv: 0.017650692106976215
//  ks: 1.4371121135741225
//  MAX VELO: 598.4404363526928 deg/s
// 0.91 kv for MAX potential 

// .79 kv works up to 300 dps
// .35
// 0.87
dlib::FeedforwardGains TurnFFwdGains{
  1.75, //1.75
  0.92, //0.88
  0.15}; // ka 0.20

// 1.026599683976744,
//	0.4782196866053843
dlib::FeedforwardGains LinFFwdGains{
  1.2, 
  5.1, //4.47078124536
  1.1 //ka 1.2
};

dlib::ImuConfig imu_config{
    1,           // imu port
    1.00961546551 // optional imu scaling constant
};

dlib::PidGains move_pid_gains{
    200, // kp, porportional gain 200
    0,   // ki, integral gain
    0    // kd, derivative gain
};

dlib::ErrorDerivativeSettler<Meters> move_pid_settler{
    inches(0.5), // error threshold, the maximum error the pid can settle at
    meters_per_second(0.02) // derivative threshold, the maximum instantaneous
                            // error over time the pid can settle at
};

dlib::PidGains lin_pid_gains{
  72, //80
  0,
  10 //10
};

dlib::PidGains turn_pid_gains{
    20, // kp, porportional gain 20
    1.8,  // ki, integral gain //1.4
    1.7, // kd, derivative gain // 1.7
    0.0698132 // This is in radians rip ~10 degrees 0.0698132
};

// 1.5
dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler{
    degrees(1), // error threshold, the maximum error the pid can settle at //1
    degrees_per_second(8) // derivative threshold, the maximum instantaneous //8
                            // error over time the pid can settle at
};

Robot robot = Robot(chassis_config, imu_config, move_pid_gains,
                    move_pid_settler, turn_pid_gains, turn_pid_settler,
                    rotRight, rotLeft, TurnFFwdGains, LinFFwdGains, {}, lin_pid_gains);