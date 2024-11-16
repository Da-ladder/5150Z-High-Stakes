#pragma once
#include "autos.h"
#include "pistons.h"
#include "declarations.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pistons.h"
#include "lift.h"
#include "intakeFuncts.h"


#define MOGO_SIDE true
#define INTAKE_SIDE false

std::vector<const char*> AutoSelector::routeNames = {};
std::vector<void (*)()> AutoSelector::routePointers = {};

int AutoSelector::indexToRun = 0;

pros::ADIPotentiometer AutoSelector::pot = pros::ADIPotentiometer('E', pros::adi_potentiometer_type_e::E_ADI_POT_EDR);

// Function terminates when goal is detected

void moveToGoal(double speed = 5, double dist = 55, int timeout = 0) {
    int time = 0;
    while (goalOpt.get() > dist && timeout > time) {
        robot.chassis.move_voltage((au::volts)(speed));

        time += 5;
        pros::delay(5);
    }
    robot.chassis.move_voltage((au::volts)(0));
}

void Routes::skills() {
    
    moveToGoal(5, 55, 400);
    moClamp.overrideState(1);
    IntakeHelper::voltage(12);
    pros::delay(700);

    // get ring
    robot.move_to_point({(au::inches)(29), (au::inches)(10.96)}, true, INTAKE_SIDE);
    pros::delay(1000);
    liftIntake.overrideState(1);
    pros::delay(1000);
    liftIntake.overrideState(0);

    // get other ring
    robot.move_to_point({(au::inches)(18.11), (au::inches)(30.36)}, true, INTAKE_SIDE);
    pros::delay(800);

    // get rings near corner
    robot.move_to_point({(au::inches)(7.54), (au::inches)(26.45)}, true, INTAKE_SIDE);
    pros::delay(800);
    robot.move_to_point({(au::inches)(-4.84), (au::inches)(20.09)}, true, INTAKE_SIDE);
    pros::delay(1000);
    robot.move_to_point({(au::inches)(-3.78), (au::inches)(29.74)}, true, INTAKE_SIDE);
    pros::delay(1000);

    robot.turn_with_pid(-175.98, 700);
    robot.chassis.move_voltage((au::volts)(6));
    pros::delay(800);
    robot.chassis.move_voltage((au::volts)(0));
    moClamp.overrideState(0);
    pros::delay(100);

    robot.move_to_point({(au::inches)(-6.38), (au::inches)(30.04)}, true, INTAKE_SIDE);

    robot.turn_with_pid(-57.34, 1500);
    moveToGoal(11, 55, 1000);
    moClamp.overrideState(1);
    pros::delay(300);

    //

    // put goal in negative -175.98
    // robot.move_to_point({(au::inches)(-18.13), (au::inches)(30.14)}, true, INTAKE_SIDE);
    // moClamp.overrideState(0);
    // pros::delay(100);

}

void Routes::placehold4() {
    // robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
    // robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::brake);
    // Mogo + ring mid
    robot.move_to_point({(au::inches)(21.65), (au::inches)(0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 200);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    
    // robot.turn_with_pid(117.69, 900);
    robot.move_to_point({(au::inches)(30.47), (au::inches)(-21.45)}, true, INTAKE_SIDE);
    pros::delay(200);

    // grab ring stack (safe)
    robot.move_to_point({(au::inches)(38.33), (au::inches)(0.59)}, false, MOGO_SIDE);
    robot.turn_with_pid(74.09, 700);
    robot.move_to_point({(au::inches)(18.16), (au::inches)(-13.98)}, false, INTAKE_SIDE);

    // grab last ring from mid
    robot.turn_with_pid(123.44, 800);
    robot.move_to_point({(au::inches)(26.94), (au::inches)(-27.91)}, false, INTAKE_SIDE);
    pros::delay(300);


    // grab center ring 
    robot.move_to_point({(au::inches)(11.43), (au::inches)(-2.43)}, false, MOGO_SIDE);
    liftIntake.overrideState(1);
    robot.turn_with_pid(257.96, 1000);



    // Get alliance ring from in front of stake
    robot.move_to_point({(au::inches)(15.29), (au::inches)(16.35)}, false, INTAKE_SIDE);
    robot.turn_with_pid(249.26, 1000);
    robot.chassis.move_voltage((au::volts)(-2.5));
    pros::delay(850); //340
    robot.chassis.move_voltage((au::volts)(0));
    liftIntake.overrideState(0);
    pros::delay(850);
    robot.move_to_point({(au::inches)(16.3), (au::inches)(13.74)}, false, MOGO_SIDE);
    pros::delay(600);
    
    /**/

}

void Routes::placehold1() {
    robot.move_to_point({(au::inches)(21.65), (au::inches)(0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 200);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // grab ring stack
    robot.turn_with_pid(-76.61, 1000); //req???
    robot.move_to_point({(au::inches)(19.22), (au::inches)(14.78)}, false, INTAKE_SIDE);

    // go twds safe center stack
    robot.turn_with_pid(90.22, 1100);
    pros::delay(400);
    moClamp.overrideState(0);
    liftIntake.overrideState(1);
    IntakeHelper::voltage(-12);
    robot.move_to_point({(au::inches)(19.33), (au::inches)(-37.1)}, false, INTAKE_SIDE);
    liftIntake.overrideState(0);

    // grab other mogo
    robot.turn_with_pid(-3.23, 1000);
    moveToGoal(4, 55, 2000); // NO TIMEOUT
    moClamp.overrideState(1);
    pros::delay(200);

    // grab ring
    IntakeHelper::voltage(12); //131.44
    robot.move_to_point({(au::inches)(56.82), (au::inches)(-50.55)}, true, INTAKE_SIDE);

    // touch ladder
    robot.turn_with_pid(262.94, 1000);
    robot.chassis.move_voltage((au::volts)(-6));
    pros::delay(1300);
    robot.chassis.move_voltage((au::volts)(-1));
    /**/
}

void Routes::placehold2() {
    robot.move_to_point({(au::inches)(21.65), (au::inches)(-0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 200);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // grab ring stack
    robot.turn_with_pid(76.61, 1000); //req???
    robot.move_to_point({(au::inches)(19.22), (au::inches)(-14.78)}, false, INTAKE_SIDE);

    // go twds safe center stack
    robot.turn_with_pid(-90.22, 1100);
    pros::delay(800);
    moClamp.overrideState(0);
    liftIntake.overrideState(1);
    pros::delay(150);
    IntakeHelper::voltage(-12);
    robot.move_to_point({(au::inches)(19.33), (au::inches)(37.1)}, false, INTAKE_SIDE);
    liftIntake.overrideState(0);

    // grab other mogo
    robot.turn_with_pid(4.11, 1000);
    moveToGoal(5, 55, 2000);
    moClamp.overrideState(1);
    pros::delay(200);

    // grab ring
    IntakeHelper::voltage(12); //-107.32
    robot.move_to_point({(au::inches)(57.13), (au::inches)(58.38)}, true, INTAKE_SIDE); //63.21, 57.4
    

    robot.turn_with_pid(281.14, 1000);
    robot.chassis.move_voltage((au::volts)(-6));
    pros::delay(500);
    robot.chassis.move_voltage((au::volts)(-2));
    pros::delay(500);
    robot.chassis.move_voltage((au::volts)(0));


    // touch ladder
    // robot.turn_with_pid(-276.16, 1000);
    // robot.chassis.move_voltage((au::volts)(-6));
    // pros::delay(1300);
    // robot.chassis.move_voltage((au::volts)(-1));

    /**/
}

void Routes::placehold3() {

    // RUSH MID MOGO
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(-38.41), (au::inches)(0)}, false, INTAKE_SIDE);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(0);
    pros::delay(100);
    robot.turn_with_pid(8, 400);
    
    // DRAG MOGO BACK
    robot.chassis.move_voltage((au::volts)(5.5));
    pros::delay(550);
    robot.chassis.move_voltage((au::volts)(0));
    cornerDeploy.overrideState(0);

    pros::delay(300);

    // GRAB MOGO
    robot.turn_with_pid(193.81, 1300);
    moveToGoal(5, 55, 600);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // TURN WITH MOGO
    robot.move_to_point({(au::inches)(-32.93), (au::inches)(-1.8)}, false, INTAKE_SIDE);
    robot.turn_with_pid(273.46, 1300);
    pros::delay(300);
    
    // DROP & GRAB OTHER MOGO
    moClamp.overrideState(0);
    robot.move_to_point({(au::inches)(-33.61), (au::inches)(2.16)}, false, INTAKE_SIDE);
    robot.turn_with_pid(112.65, 1300);
    moveToGoal(5, 55, 900);
    moClamp.overrideState(1);
    pros::delay(200);

    // GRAB ANOTHER RING
    robot.move_to_point({(au::inches)(-18.79), (au::inches)(-11.29)}, true, INTAKE_SIDE);
    robot.chassis.move_voltage((au::volts)(-5));
    pros::delay(300);
    robot.chassis.move_voltage((au::volts)(5));
    pros::delay(300);
    robot.chassis.move_voltage((au::volts)(0));
    pros::delay(1200);
    // liftIntake.overrideState(1);






    // robot.move_to_point({(au::inches)(-21), (au::inches)(0)}, false, MOGO_SIDE);


}

void Routes::placehold6() {
    robot.move_to_point({(au::inches)(21.65), (au::inches)(-0.48)}, false, MOGO_SIDE);
    moveToGoal(3, 55, 400);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    
    robot.turn_with_pid(-122.04, 800);
    robot.move_to_point({(au::inches)(34.48), (au::inches)(18.92)}, true, INTAKE_SIDE);

    // grab ring stack (safe)
    robot.move_to_point({(au::inches)(23.1), (au::inches)(1.33)}, false, MOGO_SIDE);
    robot.turn_with_pid(-63.71, 700);
    robot.move_to_point({(au::inches)(17.23), (au::inches)(13.55)}, false, INTAKE_SIDE);

    // grab last ring from mid ???
    robot.turn_with_pid(-123.44, 800);
    robot.move_to_point({(au::inches)(26.94), (au::inches)(27.91)}, false, INTAKE_SIDE);
    pros::delay(300);


    // grab center ring 
    robot.move_to_point({(au::inches)(11.43), (au::inches)(2.43)}, false, MOGO_SIDE);
    liftIntake.overrideState(1);
    robot.turn_with_pid(-257.96, 1000);



    // Get alliance ring from in front of stake
    robot.move_to_point({(au::inches)(15.29), (au::inches)(-16.35)}, false, INTAKE_SIDE);
    robot.turn_with_pid(-249.26, 1000);
    robot.chassis.move_voltage((au::volts)(-2.5));
    pros::delay(1050); //340
    robot.chassis.move_voltage((au::volts)(0));
    liftIntake.overrideState(0);
    pros::delay(850);
    robot.move_to_point({(au::inches)(16.3), (au::inches)(-13.74)}, false, MOGO_SIDE);
    pros::delay(600);
    /**/
}

void Routes::placehold7() {
    robot.move_to_point({(au::inches)(-43), (au::inches)(0)}, false, INTAKE_SIDE);
    robot.turn_with_pid(30, 400);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(0);
    pros::delay(250);

    
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
}

void Routes::placehold11() {
}