#pragma once
#include "autos.h"
#include "au/au.hpp"
#include "mogoDetect.h"
#include "pistons.h"
#include "declarations.h"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pistons.h"
#include "lift.h"
#include "intakeFuncts.h"
#include <concepts>


#define MOGO_SIDE true
#define INTAKE_SIDE false

std::vector<const char*> AutoSelector::routeNames = {};
std::vector<void (*)()> AutoSelector::routePointers = {};

int AutoSelector::indexToRun = 0;

pros::ADIPotentiometer AutoSelector::pot = pros::ADIPotentiometer('E', pros::adi_potentiometer_type_e::E_ADI_POT_EDR);

// Function terminates when goal is detected

void moveToGoal(double speed = 5, double dist = 56, int timeout = 0) {
    int time = 0;
    while (goalOpt.get() > dist && timeout > time) {
        robot.chassis.move_voltage((au::volts)(speed));

        time += 5;
        pros::delay(5);
    }
    robot.chassis.move_voltage((au::volts)(0.5));
}
/*
    @brief manually controls drivebase

*/
void moveManual(int delay, double lspeed, double rspeed = 0.002) {
    if (rspeed == 0.002) {
        robot.chassis.move_voltage((au::volts)(lspeed));
    } else {
        robot.chassis.left_motors.raw.move_voltage(lspeed*1000);
        robot.chassis.right_motors.raw.move_voltage(rspeed*1000);
    }
    pros::delay(delay);
    robot.chassis.move_voltage((au::volts)(0));
}

double fIntVolt = 0;
int fIntDelay = 0;

void intakeDelay() {
    while (true) {
        if (fIntDelay != 0) {
            pros::delay(fIntDelay);
            IntakeHelper::voltage(fIntVolt);
        } else {}
        pros::delay(20);
    }
}

void Routes::skills() {
    LiftMngr::setLevel(250);
    IntakeHelper::blueExcld(true);
    // score on stake
    IntakeHelper::voltage(12);
    pros::delay(700);

    // back off and turn twds mogo
    robot.move_to_point({(au::inches)(-12.61), (au::inches)(0.03)}, false, INTAKE_SIDE);
    robot.turn_to_point({(au::inches)(-14.67), (au::inches)(-22.36)}, MOGO_SIDE, 800);

    // grab mogo
    MogoUtils::getMogo(10, 2);

    // turn to and grab ring
    robot.turn_to_point({(au::inches)(-38.71), (au::inches)(-23.95)}, INTAKE_SIDE, 800);
    RedRingUtil::getRing(true, 11, 2);

    // turn to and grab ring
    robot.move_to_point({(au::inches)(-68.46), (au::inches)(-43.94)}, true, INTAKE_SIDE, 800, 2.5, 8);
    LiftMngr::setLevel(291.62);
    RedRingUtil::getRing(true, 11, 2);
    moveManual(300, 4);

    // turn to and grab ring + stake
    robot.turn_to_point({(au::inches)(-63.21), (au::inches)(-57.33)}, INTAKE_SIDE, 1200);
    pros::delay(600);
    IntakeHelper::voltage(0);
    pros::delay(250);
    IntakeHelper::voltage(12);
    pros::delay(250);
    IntakeHelper::voltage(0);
    LiftMngr::setLevel(250);
    pros::delay(200);
    IntakeHelper::voltage(12);

    robot.move_to_point({(au::inches)(-63.21), (au::inches)(-57.33)}, false, INTAKE_SIDE, 0, 2.5);
    

    robot.turn_with_pid(89.83, 700);
    moveManual(500, -3);
    // IntakeHelper::voltage(12);
    LiftMngr::setLevel(150);
    pros::delay(300);


    // get ring
    robot.move_to_point({(au::inches)(-64.27), (au::inches)(-45.46)}, false, MOGO_SIDE, 0, 3);
    LiftMngr::setLevel(250);
    pros::delay(400);
    robot.turn_to_point({(au::inches)(-44.79), (au::inches)(-47.12)}, INTAKE_SIDE, 900, 12);
    RedRingUtil::getRing(true, 12, 2);

    // get ring
    robot.turn_to_point({(au::inches)(-21.24), (au::inches)(-46.85)}, INTAKE_SIDE, 200, 8);
    RedRingUtil::getRing(true, 9, 2);

    // get ring
    robot.turn_to_point({(au::inches)(-9.39), (au::inches)(-46.58)}, INTAKE_SIDE, 200, 8);
    RedRingUtil::getRing(true, 9, 2);

    // get ring
    robot.turn_with_pid(60.76, 1100, 8);
    RedRingUtil::getRing(true, 9, 2); 

    //
    robot.move_to_point({(au::inches)(-15.15), (au::inches)(-53.80)}, true, MOGO_SIDE, 700, 2.5);
    robot.turn_with_pid(-36.82, 900, 9);
    moveManual(500, 5);
    pros::delay(300);
    IntakeHelper::voltage(0);
    moClamp.overrideState(0);

    // grab ring
    LiftMngr::setLevel(291.62);
    robot.move_to_point({(au::inches)(-15.95), (au::inches)(-52.30)}, false, INTAKE_SIDE, 0, 2.5);
    IntakeHelper::voltage(12);
    /*
    robot.move_to_point({(au::inches)(-108.19), (au::inches)(-52.61)}, false, INTAKE_SIDE, 0, 2.5);


    





    // -44.79 -47.12








    /*

    // turn to and grab ring
    robot.turn_to_point({(au::inches)(-39.98), (au::inches)(49.26)}, INTAKE_SIDE, 800, 8);
    RedRingUtil::getRing(true, 12, 2);
    // pros::delay(500);

    // turn to and grab ring
    robot.turn_to_point({(au::inches)(-18.05), (au::inches)(48.31)}, INTAKE_SIDE, 800, 8);
    RedRingUtil::getRing(true, 12, 2);
    // pros::delay(500);

    // turn to and grab ring
    robot.turn_to_point({(au::inches)(-7.66), (au::inches)(48.13)}, INTAKE_SIDE, 200, 8);
    RedRingUtil::getRing(true, 12, 2);
    // pros::delay(500);

    // turn to and grab ring
    robot.turn_to_point({(au::inches)(-13.82), (au::inches)(58.7)}, INTAKE_SIDE, 800, 8);
    RedRingUtil::getRing(true, 12, 2);
    // pros::delay(500);

    robot.move_to_point({(au::inches)(-4.58), (au::inches)(61.52)}, true, MOGO_SIDE, 900, 2, 8);
    robot.turn_with_pid(37.3, 900);
    moveManual(200, 3);
    moClamp.overrideState(0);



/**/



    

}

void Routes::placehold4() {
    // robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
    // robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);

    // Mogo + preload ring
    robot.move_to_point({(au::inches)(21.65), (au::inches)(0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 400);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    

    // mid ring(s)
    robot.move_to_point({(au::inches)(33.51), (au::inches)(-19.76)}, true, INTAKE_SIDE, 900);
    pros::delay(600);
    robot.turn_with_pid(59.96, 1000);
    // robot.move_to_point({(au::inches)(28.50), (au::inches)(-29.35)}, false, INTAKE_SIDE); //angle:64.36
    robot.move_to_point({(au::inches)(28.30), (au::inches)(-31.79)}, false, INTAKE_SIDE);
    pros::delay(500);


    // ring stack (safe) ring side
    moveManual(1000, 3, 5); ///lol
    pros::delay(200);
    robot.move_to_point({(au::inches)(18.34), (au::inches)(-17.81)}, true, INTAKE_SIDE);
    pros::delay(1100);


    // alliance stake ring
    liftIntake.overrideState(1);
    robot.move_to_point({(au::inches)(19.47), (au::inches)(22.52)}, true, INTAKE_SIDE, 900, 2);
    robot.turn_with_pid(-83.3, 600);
    liftIntake.overrideState(0);
    moveManual(310, -5);
    moveManual(460, 4);
    // robot.move_to_point({(au::inches)(19.47), (au::inches)(22.52)}, false, INTAKE_SIDE);




    // robot.turn_with_pid(-12.66, 1000);
    // robot.move_to_point({(au::inches)(-18.81), (au::inches)(-10.67)}, true, INTAKE_SIDE);
    /**/

}

void Routes::placehold1() {
    IntakeHelper::blueExcld(true);
    robot.move_to_point({(au::inches)(19.0), (au::inches)(-0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 400);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // grab ring stack(safe)
    robot.move_to_point({(au::inches)(18.47), (au::inches)(-14.13)}, true, INTAKE_SIDE, 800);
    pros::delay(1800); //2.2k
    moClamp.overrideState(0);

    // grab center 
    // liftIntake.overrideState(1);
    IntakeHelper::voltage(0);
    robot.move_to_point({(au::inches)(22.83), (au::inches)(34.26)}, true, INTAKE_SIDE, 1200, 2.3);


    // get mogo
    robot.move_to_point({(au::inches)(44.09), (au::inches)(40.81)}, true, MOGO_SIDE, 1200);
    moveToGoal(3, 55, 1500);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // get ring
    robot.move_to_point({(au::inches)(72.13), (au::inches)(61.85)}, true, INTAKE_SIDE, 1300);

    robot.turn_with_pid(-289.33, 1000);

   
    //-289.33
    LiftMngr::setLevel(600);
    moveManual(600, -6);
    moveManual(700, -2);
    /**/

}

void Routes::placehold2() {
    IntakeHelper::blueExcld(false);
    LiftMngr::setMotorPwr(5);
    pros::delay(400);

    LiftMngr::setLevel(130);
    pros::delay(600);
    robot.move_to_point({(au::inches)(28.61), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    robot.turn_with_pid(-86.02, 900);
    LiftMngr::setLevel(250);
    MogoUtils::getMogo(10, 2);
    IntakeHelper::voltage(12);

    // get mid 
    robot.turn_with_pid(-204.41, 800);
    robot.move_to_point({(au::inches)(53.71), (au::inches)(-28.99)}, false, INTAKE_SIDE, 1300, 2.5);
    robot.move_to_point({(au::inches)(60.09), (au::inches)(-29.04)}, true, INTAKE_SIDE, 700, 2.5);
    pros::delay(100);

    // curve back
    moveManual(500, 8, 4);
    IntakeHelper::StopAtColor(true);
    moveManual(400, 6, 6);
    

    // get ring
    robot.turn_with_pid(-165.13, 900);
    // robot.turn_to_point({(au::inches)(46.51), (au::inches)(-17.99)}, INTAKE_SIDE, 800);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(46.51), (au::inches)(-17.99)}, false, INTAKE_SIDE, 900); //OLD
    // RedRingUtil::getRing(false, 10, 5);

    // grab ring from corner
    robot.move_to_point({(au::inches)(44.09), (au::inches)(14.05)}, true, INTAKE_SIDE, 900);
    moveManual(800, -5);
    pros::delay(200);

    // back off
    // IntakeHelper::StopAtColor(true);
    robot.move_to_point({(au::inches)(49.64), (au::inches)(-5.83)}, true, MOGO_SIDE, 500, 2.4);

    // get allaince stake ring
    robot.move_to_point({(au::inches)(20.22), (au::inches)(-9.51)}, true, INTAKE_SIDE, 900);
    // IntakeHelper::StopAtColor(false);
    // IntakeHelper::voltage(12);
    // robot.move_to_point({(au::inches)(-11.40), (au::inches)(-28.74)}, true, INTAKE_SIDE, 900);


    

    /*
    robot.move_to_point({(au::inches)(20.80), (au::inches)(-7.86)}, true, INTAKE_SIDE, 900);

    // to corner
    robot.move_to_point({(au::inches)(-28.01), (au::inches)(-36.63)}, true, INTAKE_SIDE, 900, 2.5);
    // moveManual(400, 6);


    // pros::delay(100);
    /**/
}

void Routes::placehold2Mir() {
    IntakeHelper::blueExcld(true);
    LiftMngr::setMotorPwr(5);
    pros::delay(400);

    LiftMngr::setLevel(130);
    pros::delay(600);
    robot.move_to_point({(au::inches)(28.61), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    robot.turn_with_pid(86.02, 900);
    LiftMngr::setLevel(250);
    MogoUtils::getMogo(10, 2);
    IntakeHelper::voltage(12);

    // get mid 
    robot.turn_with_pid(205.44, 800);
    robot.move_to_point({(au::inches)(51.97), (au::inches)(29.7)}, false, INTAKE_SIDE, 1300, 2.5);
    robot.move_to_point({(au::inches)(64.42), (au::inches)(23.26)}, true, INTAKE_SIDE, 700, 2.5);
    pros::delay(100);
    // curve back
    moveManual(500, 4, 8);
    moveManual(300, 6, 6);
    IntakeHelper::StopAtColor(true);

    // get ring
    robot.turn_with_pid(139.89, 900);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    // robot.move_to_point({(au::inches)(46.87), (au::inches)(16.55)}, false, INTAKE_SIDE, 900); //OLD
    robot.move_to_point({(au::inches)(57.73), (au::inches)(8.65)}, false, INTAKE_SIDE, 900);

    // grab ring from corner
    robot.move_to_point({(au::inches)(48.23), (au::inches)(-16.84)}, true, INTAKE_SIDE, 900, 2.8);
    pros::delay(50);
    moveManual(800, -7);
    

    // back off
    robot.move_to_point({(au::inches)(43.86), (au::inches)(-1.70)}, true, MOGO_SIDE, 500);
    /*
    robot.move_to_point({(au::inches)(20.80), (au::inches)(-7.86)}, true, INTAKE_SIDE, 900);

    // to corner
    robot.move_to_point({(au::inches)(-28.01), (au::inches)(-36.63)}, true, INTAKE_SIDE, 900, 2.5);
    // moveManual(400, 6);


    // pros::delay(100);
    /**/
}

void Routes::placehold3() {
    IntakeHelper::blueExcld(true);
    // RUSH MOGO
    IntakeHelper::voltage(10.5);
    robot.move_to_point({(au::inches)(-36.73), (au::inches)(-0.32)}, false, INTAKE_SIDE, 0, 2);
    IntakeHelper::voltage(0);
    cornerDeploy.overrideState(1);
    pros::delay(300);


    // PULL IT BACK & grab mogo + score ring via through stack
    robot.move_to_point({(au::inches)(-21.88), (au::inches)(-0.32)}, false, MOGO_SIDE, 0, 2);
    cornerDeploy.overrideState(0);
    pros::delay(150);
    robot.turn_with_pid(193.13, 1200);
    moveToGoal(3, 55, 800); //57
    robot.chassis.move_voltage((au::volts)(1.4));
    moClamp.overrideState(1);
    pros::delay(100);
    robot.chassis.move_voltage((au::volts)(0));
    pros::delay(100);
    IntakeHelper::voltage(12);

    // DROP
    robot.move_to_point({(au::inches)(-24.17), (au::inches)(-2.15)}, false, INTAKE_SIDE, 0);
    pros::delay(1500);
    moClamp.overrideState(0);
    pros::delay(200);

    // GET MOGO
    robot.move_to_point({(au::inches)(-31.53), (au::inches)(12.94)}, true, MOGO_SIDE, 1000);
    robot.turn_with_pid(131.21, 500);
    moveToGoal(3, 55, 700); //57
    robot.chassis.move_voltage((au::volts)(1.4));
    moClamp.overrideState(1);
    pros::delay(100);
    robot.chassis.move_voltage((au::volts)(0));
    pros::delay(100);
    LiftMngr::setLevel(500);


    // get other ring
    robot.move_to_point({(au::inches)(-21.23), (au::inches)(3.36)}, false, INTAKE_SIDE, 1000);
    robot.turn_with_pid(124.86, 500);
    moveManual(500, -5);




    

    /**/
}

void Routes::placehold5() {
    robot.move_to_point({(au::inches)(19.0), (au::inches)(-0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 400);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // grab ring stack(safe)
    robot.move_to_point({(au::inches)(18.47), (au::inches)(14.13)}, true, INTAKE_SIDE, 800);
    pros::delay(1800);
    moClamp.overrideState(0);

    // grab center 
    // liftIntake.overrideState(1);
    IntakeHelper::voltage(-12);
    robot.move_to_point({(au::inches)(22.83), (au::inches)(-34.26)}, true, INTAKE_SIDE, 1200);


    // get mogo
    robot.move_to_point({(au::inches)(44.11), (au::inches)(-36.88)}, true, MOGO_SIDE, 1200); //-8.17
    moveToGoal(3, 55, 600);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // get ring
    robot.move_to_point({(au::inches)(62.97), (au::inches)(-55.89)}, true, INTAKE_SIDE, 1300);
    LiftMngr::setLevel(600);

    robot.turn_with_pid(279.39, 1400);
    moveManual(600, -8);
    moveManual(700, -2);

    /**/
}

void Routes::placehold6() {
    // Mogo + preload ring
    // Stake + mogo
    IntakeHelper::blueExcld(false);
    LiftMngr::setMotorPwr(5);
    pros::delay(400);

    LiftMngr::setLevel(130);
    pros::delay(400);
    robot.move_to_point({(au::inches)(28.61), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(250);
    robot.turn_with_pid(-86.02, 800);
    MogoUtils::getMogo(10, 2);
    IntakeHelper::voltage(12);

    // get ring //-163.34
    robot.turn_with_pid(-163.34, 700);
    robot.move_to_point({(au::inches)(45.14), (au::inches)(-14.00)}, false, INTAKE_SIDE, 700); // OLD

    // get ring + drop + get mogo
    // STOP RING AT COLOR
    IntakeHelper::voltage(0);
    robot.turn_to_point({(au::inches)(17.33), (au::inches)(-9.03)}, INTAKE_SIDE, 900);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(17.33), (au::inches)(-9.03)}, false, INTAKE_SIDE); //RE
    

    // robot.move_to_point({(au::inches)(1.35), (au::inches)(-17.70)}, true, INTAKE_SIDE, 600, 3, 8);
    robot.move_to_point({(au::inches)(-22.20), (au::inches)(-33.22)}, true, INTAKE_SIDE, 300, 2.7); //RE
    pros::delay(925);
    // robot.turn_with_pid(179.38, 1200, 6.5);
    moClamp.overrideState(0);
    pros::delay(150);
    robot.turn_with_pid(-51.15, 800);
    // -45.10
    // robot.move_to_point({(au::inches)(-10.77), (au::inches)(-33.39)}, true, INTAKE_SIDE, 500, 2.8, 8);
    // robot.turn_with_pid(268.51, 600); //RE


    // grab mogo + grab ring
    MogoUtils::getMogo(10, 2);
    // robot.turn_with_pid(409.74, 800); //RE
    robot.move_to_point({(au::inches)(-26.72), (au::inches)(-60.81)}, true, INTAKE_SIDE, 900, 3);
    IntakeHelper::StopAtColor(true);


    // touch ladder
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    LiftMngr::setLevel(200);
    robot.turn_with_pid(188.02, 900);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    moveManual(400, -12);
    moveManual(400, -4);
    /**/
}

void Routes::placehold6Mir() {
    IntakeHelper::blueExcld(true);
    LiftMngr::setMotorPwr(5);
    pros::delay(400);

    LiftMngr::setLevel(130);
    pros::delay(400);
    robot.move_to_point({(au::inches)(28.61), (au::inches)(0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(250);
    robot.turn_with_pid(86.02, 800);
    MogoUtils::getMogo(10, 2);
    IntakeHelper::voltage(11);

    // get ring
    robot.move_to_point({(au::inches)(44.64), (au::inches)(13.82)}, true, INTAKE_SIDE, 700);

    // get ring + drop + get mogo
    // STOP RING AT COLOR
    IntakeHelper::StopAtColor(true);
    robot.turn_to_point({(au::inches)(14.94), (au::inches)(5.47)}, INTAKE_SIDE, 900);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(14.94), (au::inches)(5.47)}, false, INTAKE_SIDE); //RE
    

    // robot.move_to_point({(au::inches)(1.35), (au::inches)(-17.70)}, true, INTAKE_SIDE, 600, 3, 8);
    robot.move_to_point({(au::inches)(-19.83), (au::inches)(25.66)}, true, INTAKE_SIDE, 300, 2.7); //RE
    pros::delay(1000);
    // robot.turn_with_pid(74.99, 1200, 6.5);
    moClamp.overrideState(0);
    pros::delay(100);
    robot.turn_with_pid(74.99, 800);
    // -45.10
    // robot.move_to_point({(au::inches)(-10.77), (au::inches)(-33.39)}, true, INTAKE_SIDE, 500, 2.8, 8);
    // robot.turn_with_pid(268.51, 600); //RE


    // grab mogo + grab ring
    MogoUtils::getMogo(10, 2);
    // robot.turn_with_pid(409.74, 800); //RE
    robot.turn_to_point({(au::inches)(-26.85), (au::inches)(62.07)}, INTAKE_SIDE, 900);
    // robot.move_to_point({(au::inches)(-26.85), (au::inches)(62.07)}, true, INTAKE_SIDE, 900, 3);
    RedRingUtil::getRing(true, 10, 4);
    IntakeHelper::StopAtColor(true);


    // touch ladder
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    LiftMngr::setLevel(200);
    robot.turn_with_pid(-183.64, 900);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(11);
    moveManual(400, -12);
    moveManual(400, -4);
    /**/
}


void Routes::placehold7() {
    IntakeHelper::blueExcld(false);
    robot.move_to_point({(au::inches)(-39.35), (au::inches)(0)}, false, INTAKE_SIDE, 0, 2);
    // robot.move_to_point({(au::inches)(-36.74), (au::inches)(-2.03)}, true, INTAKE_SIDE, 700);
    cornerDeploy.overrideState(1);
    // robot.turn_with_pid(20, 500);
    IntakeHelper::voltage(0);
    pros::delay(250);

    // pull back
    robot.move_to_point({(au::inches)(-24.91), (au::inches)(2.57)}, false, MOGO_SIDE, 0, 2);
    cornerDeploy.overrideState(0);
    pros::delay(150);

    // grab pulled back goal
    robot.turn_with_pid(-169.28, 1300);
    moveToGoal(5, 55, 600);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    pros::delay(200);

    // bring goal close & drop
    robot.move_to_point({(au::inches)(-31.36), (au::inches)(-1.11)}, false, INTAKE_SIDE, 0);
    pros::delay(500);
    moClamp.overrideState(0);

    robot.move_to_point({(au::inches)(-14.58), (au::inches)(1.25)}, false, INTAKE_SIDE, 0, 2.5);

    // grab other mogo
    // robot.turn_with_pid(-110.86, 800);
    robot.move_to_point({(au::inches)(-23.5), (au::inches)(-19.99)}, true, MOGO_SIDE, 800);
    robot.turn_with_pid(-105.49, 600);
    moveToGoal(4, 55, 900); //???
    moClamp.overrideState(1);
    pros::delay(200);

    // grab rings on allaince stake 
    liftIntake.overrideState(1);
    robot.move_to_point({(au::inches)(-12.18), (au::inches)(-50.97)}, true, INTAKE_SIDE, 1000);
    robot.turn_with_pid(-230.31, 600);
    liftIntake.overrideState(0);
    pros::delay(200);
    moveManual(400, -4); //-230.31

    moveManual(500, 7); //-230.31





    /*

    
    robot.chassis.move_voltage((au::volts)(5.5));
    pros::delay(800);
    robot.chassis.move_voltage((au::volts)(0));
    cornerDeploy.overrideState(0);
    pros::delay(100);


    robot.turn_with_pid(-167, 1000);
    moveToGoal(5, 55, 600);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    pros::delay(1000);
    moClamp.overrideState(0);

    robot.move_to_point({(au::inches)(-16.71), (au::inches)(3.53)}, false, INTAKE_SIDE);
    robot.turn_with_pid(-114.52, 1200);

    moveToGoal(5, 55, 2000);
    moClamp.overrideState(1);
    pros::delay(200);
    liftIntake.overrideState(1);

    robot.move_to_point({(au::inches)(-22.77), (au::inches)(-43.12)}, true, INTAKE_SIDE);
    robot.turn_with_pid(-215.1, 800);

    robot.chassis.move_voltage((au::volts)(-2.5));
    pros::delay(850); //340
    robot.chassis.move_voltage((au::volts)(0));
    liftIntake.overrideState(0);
    pros::delay(850);
    robot.chassis.move_voltage((au::volts)(2.5));
    pros::delay(850); //340
    robot.chassis.move_voltage((au::volts)(0));
    // robot.move_to_point({(au::inches)(-22.77), (au::inches)(-43.12)}, false, MOGO_SIDE); //back off
    // pros::delay(600);

    

    /*
    // RUSH MID MOGO
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(-37.41), (au::inches)(-0)}, false, INTAKE_SIDE);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(0);
    pros::delay(100);
    robot.turn_with_pid(-2, 400);
    
    // DRAG MOGO BACK
    robot.chassis.move_voltage((au::volts)(5.5));
    pros::delay(350);
    robot.chassis.move_voltage((au::volts)(0));
    cornerDeploy.overrideState(0);
    pros::delay(100);

    // GRAB MOGO
    robot.turn_with_pid(-193.81, 1300);

    moveToGoal(5, 55, 600);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // TURN WITH MOGO
    robot.move_to_point({(au::inches)(-32.93), (au::inches)(1.8)}, false, INTAKE_SIDE);
    robot.turn_with_pid(-273.46, 1300);
    
    // DROP & GRAB OTHER MOGO
    moClamp.overrideState(0);
    robot.move_to_point({(au::inches)(-33.61), (au::inches)(-2.16)}, false, INTAKE_SIDE);
    robot.turn_with_pid(-112.65, 1300);
    moveToGoal(5, 55, 900);
    moClamp.overrideState(1);
    pros::delay(200);

    // GRAB ANOTHER RING
    robot.move_to_point({(au::inches)(-18.79), (au::inches)(11.29)}, true, INTAKE_SIDE);
    */
}

void Routes::placehold8() {
}

void Routes::placehold9() {
}

void Routes::placehold10() {
    IntakeHelper::blueExcld(false);
    // Mogo + preload ring
    robot.move_to_point({(au::inches)(18), (au::inches)(-0.48)}, false, MOGO_SIDE, 0, 2.4);
    moveToGoal(3, 55, 400); //55
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    

    // mid ring(s)
    robot.move_to_point({(au::inches)(33.51), (au::inches)(19.76)}, true, INTAKE_SIDE, 900);
    pros::delay(300);
    robot.turn_with_pid(-69.88, 1000);
    // robot.move_to_point({(au::inches)(31.45), (au::inches)(29.10)}, false, INTAKE_SIDE); //angle:64.36
    robot.move_to_point({(au::inches)(31.45), (au::inches)(29.10)}, false, INTAKE_SIDE);
    pros::delay(800);
    // Turn & point of second ring should be more 90-ish


    // ring stack (safe) ring side
    moveManual(560, 7, 3); ///lol
    pros::delay(100);
    robot.move_to_point({(au::inches)(26.04), (au::inches)(19.63)}, true, INTAKE_SIDE, 900);
    // change point???
    pros::delay(1300);

    
    // alliance stake ring
    // liftIntake.overrideState(1);
    robot.move_to_point({(au::inches)(21.29), (au::inches)(-15.71)}, true, INTAKE_SIDE);
    moveManual(140, 6);
    IntakeHelper::voltage(0);

    /*

    robot.turn_with_pid(90.41, 1000);
    moveManual(600, -4);
    liftIntake.overrideState(0);
    pros::delay(250);
    //moveManual(300, -3);
    moveManual(700, 4);
    // robot.move_to_point({(au::inches)(19.47), (au::inches)(22.52)}, false, INTAKE_SIDE);
    /**/
}

void Routes::placehold11() {
}

void Routes::placehold12() {
    IntakeHelper::blueExcld(true);
    LiftMngr::setLevel(250);
    MogoUtils::getMogo(11, 2);

    // get goal + mid ring
    robot.turn_with_pid(116.00, 900);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(36.13), (au::inches)(-22.37)}, false, INTAKE_SIDE, 0, 2.5);
    robot.move_to_point({(au::inches)(30.10), (au::inches)(-33.72)}, true, INTAKE_SIDE, 600, 2.5, 9);

    IntakeHelper::StopAtColor(true);
    moveManual(500, 4, 8);
    moveManual(300, 8, 8);
    // pros::delay(125);
    

    // get ring
    robot.turn_to_point({(au::inches)(22.39), (au::inches)(-16.89)}, INTAKE_SIDE, 800);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(22.39), (au::inches)(-16.89)}, false, INTAKE_SIDE, 800, 2.5, 10);

    // go to corner
    robot.move_to_point({(au::inches)(-14.97), (au::inches)(-14.66)}, true, INTAKE_SIDE, 800, 2.7);
    moveManual(800, -6);
    pros::delay(100);
    robot.move_to_point({(au::inches)(-2.89), (au::inches)(-15.7)}, false, MOGO_SIDE, 800, 2.4);

    // get alliance stake ring
    liftIntake.overrideState(1);
    robot.move_to_point({(au::inches)(10.20), (au::inches)(15.16)}, true, INTAKE_SIDE, 800, 2.7);
    robot.turn_with_pid(-118.1, 400);
    moveManual(500, -6);
    liftIntake.overrideState(0);
    moveManual(450, 4);
    /**/

}

void Routes::placehold12Mir() {
    IntakeHelper::blueExcld(false);
    LiftMngr::setLevel(230);
    MogoUtils::getMogo(11, 2);

    // get goal + mid ring
    robot.turn_with_pid(-116.00, 900);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(36.19), (au::inches)(24.21)}, false, INTAKE_SIDE, 0, 2.5);
    robot.move_to_point({(au::inches)(34.36), (au::inches)(30.07)}, true, INTAKE_SIDE, 600, 2.5, 9);

    // back off + ring
    moveManual(500, 8, 4);
    // IntakeHelper::voltage(0);
    moveManual(300, 8, 8);
    pros::delay(125);
    // IntakeHelper::voltage(12);

    robot.move_to_point({(au::inches)(21.30), (au::inches)(17.51)}, true, INTAKE_SIDE, 800, 2.5, 8);

    // stack
    robot.turn_with_pid(4.40, 1100, 9);
    robot.move_to_point({(au::inches)(-11.42), (au::inches)(14.43)}, false, INTAKE_SIDE, 900, 2.5);
    // robot.turn_with_pid(-1.69, 600); //???
    moveManual(800, -7);

    // back off + alliance stake ring
    robot.move_to_point({(au::inches)(-3.01), (au::inches)(14.65)}, false, MOGO_SIDE, 900, 2.7);

    robot.turn_with_pid(117.97, 1000);
    liftIntake.overrideState(1);
    robot.move_to_point({(au::inches)(12.89), (au::inches)(-15.26)}, false, INTAKE_SIDE, 900);
    robot.turn_with_pid(119.55, 300);
    moveManual(500, -6);
    liftIntake.overrideState(0);
    moveManual(450, 4);
    // robot.move_to_point({(au::inches)(21.61), (au::inches)(-43.44)}, false, INTAKE_SIDE, 500, 2);
    /**/
    
}

void Routes::placehold13() {
    // Setup
    IntakeHelper::blueExcld(true);
    LiftMngr::setLevel(250);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(12);
    pros::Task intDel(intakeDelay);

    // Rush mid
    IntakeHelper::StopAtColor(true);
    robot.move_to_point({(au::inches)(-38.63), (au::inches)(-0)}, false, INTAKE_SIDE, 0, 2.9);
    rushClamp.overrideState(1);

    // back off & unclamp
    robot.move_to_point({(au::inches)(-26.3), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    rushClamp.overrideState(0);
    cornerDeploy.overrideState(0);

    // turn + grab other mogo + score ring on + drop mogo
    robot.turn_with_pid(117.96, 700);
    IntakeHelper::StopAtColor(false);
    MogoUtils::getMogo(11, 2);
    IntakeHelper::voltage(12);
    pros::delay(500);
    moClamp.overrideState(0);


    // grab ring
    IntakeHelper::StopAtColor(true);
    RedRingUtil::getRing(true, 11, 2);
    // robot.move_to_point({(au::inches)(-16.53), (au::inches)(-7.14)}, true, INTAKE_SIDE, 800, 2.5);
    robot.move_to_point({(au::inches)(-22.49), (au::inches)(-1.17)}, false, MOGO_SIDE, 800, 2.5);

    // grab rushed mogo + score
    robot.turn_with_pid(195.38, 700);
    MogoUtils::getMogo(11, 2);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    // go for corner & get last ring
    fIntVolt = -12;
    fIntDelay = 400;
    robot.move_to_point({(au::inches)(-7.47), (au::inches)(9.16)}, true, INTAKE_SIDE, 800, 2.9);
    robot.turn_with_pid(154.34, 900);
    IntakeHelper::voltage(12);
    moveManual(1100, -6);

    // back off
    robot.move_to_point({(au::inches)(1.02), (au::inches)(4.32)}, false, MOGO_SIDE, 800, 2.9);
    
    //clear corner
    cornerDeploy.overrideState(1);
    pros::delay(500);
    moveManual(900, -6); //-25.91
    robot.turn_with_pid(-25.91, 1500, 8);


    // IntakeHelper::voltage(-12);




    /**/

}

void Routes::placehold13Mir() {
    // Setup
    IntakeHelper::blueExcld(false);
    LiftMngr::setLevel(250);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(12);
    pros::Task intDel(intakeDelay);

    // Rush mid
    IntakeHelper::StopAtColor(true);
    robot.move_to_point({(au::inches)(-38.63), (au::inches)(-0)}, false, INTAKE_SIDE, 0, 2.9);
    rushClamp.overrideState(1);

    // back off & unclamp
    robot.move_to_point({(au::inches)(-26.3), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    rushClamp.overrideState(0);
    cornerDeploy.overrideState(0);

    // get rush MOGO & score
    robot.turn_with_pid(-167.02, 1000);
    MogoUtils::getMogo(9, 3);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    // drop mogo
    robot.move_to_point({(au::inches)(-31.11), (au::inches)(-3.83)}, false, INTAKE_SIDE, 0);
    moClamp.overrideState(0);
    pros::delay(100);

    // grab other mogo
    robot.turn_with_pid(-65.89, 1000);
    MogoUtils::getMogo(10, 3);

    // grab ring
    robot.turn_with_pid(-170.45, 800);
    RedRingUtil::getRing(false, 11, 4);

    // get corner ring
    robot.move_to_point({(au::inches)(-7.10), (au::inches)(14.78)}, true, INTAKE_SIDE, 900, 2.9, 8);
    robot.turn_with_pid(-113.56, 800);
    moveManual(1200, -6);

    // back off & sweep
    robot.move_to_point({(au::inches)(-2.96), (au::inches)(22.41)}, false, MOGO_SIDE, 900, 2.9);
    cornerDeploy.overrideState(1);
    pros::delay(500);
    moveManual(900, -6); //-25.91
    robot.turn_with_pid(-259.66, 1500, 8);


    






}