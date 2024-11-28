#pragma once
#include "autos.h"
#include "au/au.hpp"
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
    robot.move_to_point({(au::inches)(21.65), (au::inches)(-0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 300);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // grab ring stack
    robot.turn_with_pid(76.61, 1000); //req???
    robot.move_to_point({(au::inches)(19.22), (au::inches)(-14.78)}, false, INTAKE_SIDE);

    // go twds safe center stack
    robot.turn_with_pid(-90.22, 1100);
    pros::delay(1800); // more
    // moClamp.overrideState(0);
    // liftIntake.overrideState(1);
    pros::delay(150);
    IntakeHelper::voltage(-12);
    robot.move_to_point({(au::inches)(19.33), (au::inches)(37.1)}, false, INTAKE_SIDE);
    liftIntake.overrideState(0);
    /*

    // grab other mogo
    robot.turn_with_pid(4.11, 1000);
    moveToGoal(5, 55, 2000);
    moClamp.overrideState(1);
    pros::delay(200);

    // grab ring
    IntakeHelper::voltage(12); //-120.13
    robot.move_to_point({(au::inches)(67.37), (au::inches)(56.44)}, true, INTAKE_SIDE); //63.21, 57.4


    robot.turn_with_pid(-294.39, 1500);
    LiftMngr::setLevel(320);
    robot.chassis.move_voltage((au::volts)(-6));
    pros::delay(700);
    robot.chassis.move_voltage((au::volts)(-3));
    pros::delay(1000);


    // touch ladder
    // robot.turn_with_pid(-276.16, 1000);
    // robot.chassis.move_voltage((au::volts)(-6));
    // pros::delay(1300);
    // robot.chassis.move_voltage((au::volts)(-1));

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
    robot.move_to_point({(au::inches)(19.0), (au::inches)(-0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 60, 400);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    

    // mid ring(s)
    robot.move_to_point({(au::inches)(33.96), (au::inches)(19.44)}, true, INTAKE_SIDE);
    pros::delay(600);
    robot.turn_with_pid(-97.97, 1000); //
    /*
    robot.move_to_point({(au::inches)(35.16), (au::inches)(25.81)}, true, INTAKE_SIDE, 1000); //
    pros::delay(500);

    // back off & ring stack (safe)
    robot.move_to_point({(au::inches)(32.67), (au::inches)(7.26)}, false, MOGO_SIDE); //EF
    robot.move_to_point({(au::inches)(25.79), (au::inches)(16.49)}, true, INTAKE_SIDE); //EF
    /*


    // ring stack (safe) ring side
    moveManual(700, 5, 7); ///lol
    pros::delay(400);
    robot.move_to_point({(au::inches)(18.34), (au::inches)(17.81)}, true, INTAKE_SIDE);
    // robot.turn_with_pid(-12.66, 1000);
    robot.move_to_point({(au::inches)(-18.81), (au::inches)(10.67)}, true, INTAKE_SIDE);
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