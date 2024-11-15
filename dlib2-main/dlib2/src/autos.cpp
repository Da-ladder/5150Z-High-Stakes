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
}

void Routes::placehold4() {
    // Mogo + ring mid
    robot.move_to_point({(au::inches)(21.65), (au::inches)(0.48)}, false, true);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);
    robot.turn_with_pid(117.69, 900);
    robot.move_to_point({(au::inches)(30.47), (au::inches)(-21.45)}, false, false);

    // grab ring stack (safe)
    robot.move_to_point({(au::inches)(38.33), (au::inches)(0.59)}, false, true);
    robot.turn_with_pid(74.09, 700);
    robot.move_to_point({(au::inches)(18.16), (au::inches)(-13.98)}, false, false);

    // grab last ring from mid
    robot.turn_with_pid(123.44, 800);
    robot.move_to_point({(au::inches)(27.22), (au::inches)(-26.54)}, false, false);


    // grab center ring
    robot.move_to_point({(au::inches)(11.43), (au::inches)(-2.43)}, false, true);
    robot.turn_with_pid(249.05, 1000);


    robot.move_to_point({(au::inches)(46.09), (au::inches)(78.54)}, false, false);

    // grab ring via neg stack
    // robot.turn_with_pid(-20.63, 1200);
    // robot.move_to_point({(au::inches)(-16.12), (au::inches)(-8.21)}, false, false);
    // cornerDeploy.overrideState(1);
    // pros::delay(200);
    // robot.move_to_point({(au::inches)(-3.97), (au::inches)(-13.38)}, false, false);






    // cornerDeploy.overrideState(0);
    // pros::delay(200);
    // robot.turn_with_pid(39.2, 700);
    // robot.move_to_point({(au::inches)(14.82), (au::inches)(-17.29)}, false, false);

}

void Routes::placehold1() {
    robot.move_to_point({(au::inches)(21.65), (au::inches)(0.48)}, false, MOGO_SIDE);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // grab ring stack
    robot.turn_with_pid(-76.61, 1000); //req???
    robot.move_to_point({(au::inches)(19.22), (au::inches)(14.78)}, false, INTAKE_SIDE);

    // go twds safe center stack
    robot.turn_with_pid(90.22, 1100);
    moClamp.overrideState(0);
    liftIntake.overrideState(1);
    IntakeHelper::voltage(-12);
    robot.move_to_point({(au::inches)(19.33), (au::inches)(-37.1)}, false, INTAKE_SIDE);
    liftIntake.overrideState(0);

    // grab other mogo
    robot.turn_with_pid(2.8, 1000);
    moveToGoal();
    moClamp.overrideState(1);
    pros::delay(200);

    // grab ring
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(48.37), (au::inches)(-52.69)}, true, INTAKE_SIDE);

    // touch ladder
    robot.turn_with_pid(262.94, 1000);
    robot.chassis.move_voltage((au::volts)(-6));
    pros::delay(1300);
    robot.chassis.move_voltage((au::volts)(-1));
}

void Routes::placehold2() {
    robot.move_to_point({(au::inches)(21.65), (au::inches)(-0.48)}, false, MOGO_SIDE);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // grab ring stack
    robot.turn_with_pid(76.61, 1000); //req???
    robot.move_to_point({(au::inches)(19.22), (au::inches)(-14.78)}, false, INTAKE_SIDE);

    // go twds safe center stack
    robot.turn_with_pid(-90.22, 1100);
    moClamp.overrideState(0);
    liftIntake.overrideState(1);
    pros::delay(150);
    IntakeHelper::voltage(-12);
    robot.move_to_point({(au::inches)(19.33), (au::inches)(37.1)}, false, INTAKE_SIDE);
    liftIntake.overrideState(0);

    // grab other mogo
    robot.turn_with_pid(8.0, 1000);
    moveToGoal(5, 55, 1000);
    moClamp.overrideState(1);
    pros::delay(200);

    // grab ring
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(60.58), (au::inches)(56.58)}, true, INTAKE_SIDE);

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
    robot.move_to_point({(au::inches)(-37.41), (au::inches)(0)}, false, INTAKE_SIDE);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(0);
    pros::delay(100);
    robot.turn_with_pid(2, 400);
    
    // DRAG MOGO BACK
    robot.chassis.move_voltage((au::volts)(5.5));
    pros::delay(350);
    robot.chassis.move_voltage((au::volts)(0));
    cornerDeploy.overrideState(0);
    pros::delay(100);

    // GRAB MOGO
    robot.turn_with_pid(193.81, 1300);
    moveToGoal(5, 55, 600);
    moClamp.overrideState(1);
    pros::delay(200);
    IntakeHelper::voltage(12);

    // TURN WITH MOGO
    robot.move_to_point({(au::inches)(-32.93), (au::inches)(-1.8)}, false, INTAKE_SIDE);
    robot.turn_with_pid(273.46, 1300);
    
    // DROP & GRAB OTHER MOGO
    moClamp.overrideState(0);
    robot.move_to_point({(au::inches)(-33.61), (au::inches)(2.16)}, false, INTAKE_SIDE);
    robot.turn_with_pid(112.65, 1300);
    moveToGoal(5, 55, 900);
    moClamp.overrideState(1);
    pros::delay(200);

    // GRAB ANOTHER RING
    robot.move_to_point({(au::inches)(-18.79), (au::inches)(-11.29)}, true, INTAKE_SIDE);





    // robot.move_to_point({(au::inches)(-21), (au::inches)(0)}, false, MOGO_SIDE);


}

void Routes::placehold6() {
}

void Routes::placehold7() {
}

void Routes::placehold8() {
}

void Routes::placehold9() {
}

void Routes::placehold10() {
}

void Routes::placehold11() {
}