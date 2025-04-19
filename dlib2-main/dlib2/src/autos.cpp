#include "autos.h"
#include "au/au.hpp"
#include "declarations.h"
#include "intakeFuncts.h"
#include "liblvgl/draw/lv_img_buf.h"
#include "mogoDetect.h"
#include "pistons.h"
#include "pros/rtos.hpp"
#include <cstdint>


#define ANALOG_SENSOR_PORT 1


#define MOGO_SIDE true
#define INTAKE_SIDE false
#define SCORE_TIME 500

#define STRAIGHT_ARM 145
#define IDLE_ARM 310
#define SCORE_HEIGHT 130
#define STORE_HEIGHT 283 //253
#define ABOVE_IN_HEIGHT 266.5 //222

PurePursuit pursuitPath;


std::vector<const char*> AutoSelector::routeNames = {};
std::vector<void (*)()> AutoSelector::routePointers = {};

int AutoSelector::indexToRun = 0;

pros::adi::Potentiometer AutoSelector::pot = pros::adi::Potentiometer('E', pros::adi_potentiometer_type_e::E_ADI_POT_EDR);


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
bool fIntSortOff = false;

void intakeDelay() {
    while (true) {
        if (fIntDelay != 0) {
            pros::delay(fIntDelay);
            if (fIntSortOff) {
                IntakeHelper::StopAtColor(false);
            }
            IntakeHelper::voltage(fIntVolt);
            fIntDelay = 0;
            fIntVolt = 0;
        } else {}
        pros::delay(20);
    }
}

int fIntPisUpDown = 0;
int fIntPisDelay = 0;
void intakePistonDelay() {
    while (true) {
        if (fIntPisDelay != 0) {
            pros::delay(fIntDelay);
            liftIntake.overrideState(fIntVolt);
            fIntPisUpDown = 0;
            fIntPisDelay = 0;
        } else {}
        pros::delay(20);
    }
}

int fIntMogoUpDown = 0;
int fIntMogoDelay = 0;
void mogoPistonDelay() {
    while (true) {
        if (fIntMogoDelay != 0) {
            pros::delay(fIntMogoDelay);
            moClamp.overrideState(fIntMogoUpDown);
            fIntMogoDelay = 0;
            fIntMogoUpDown = 0;
        } else {}
        pros::delay(20);
    }
}

double fLbLift = 0;
int fLbDelay = 0;

void ladyBrownDelay() {
    while (true) {
        if (fLbDelay != 0) {
            pros::delay(fLbDelay);
            LiftMngr::setLevel(fLbLift);
            fLbDelay = 0;
            fLbLift = 0;
        } else {}
        pros::delay(20);
    }
}

bool finColor = false;
int finColorSortDelay = 0;

void intakeSortDelay() {
    while (true) {
        if (finColorSortDelay != 0) {
            pros::delay(finColorSortDelay);
            IntakeHelper::StopAtColor(finColor);
            finColorSortDelay = 0;
            finColor = false;
        } else {}
        pros::delay(20);
    }
}

void printCords(std::string discriptor) {
    dlib::Pose2d curPos = robot.odom.get_position();
    std::cout << discriptor + ": " << "(" << curPos.x.in(au::inches) << ", " << curPos.y.in(au::inches) << ")" << std::endl;
}

void Routes::skills() {
    int time = 0;
    pros::Task lbThing(ladyBrownDelay);
    pros::Task intakeThing(intakeDelay);
    IntakeHelper::stuckCheckChange(true);
    // score on stake h
    IntakeHelper::blueExcld(true);
    LiftMngr::setLevel(STRAIGHT_ARM + 50);

    while(340 >= time*20) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM + 50);

    // back off and turn twds mogo
    robot.move_to_point({(au::inches)(5.5), (au::inches)(0.0)}, false, MOGO_SIDE);
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(90, 600);

    // grab mogo
    MogoUtils::getMogo(6, 2);
    // printCords("MOGO");

    // move to and grab 1 ring + 1 in lb
    // pros::delay(200);
    IntakeHelper::voltage(12);
    // robot.turn_to_point({(au::inches)(24.82), (au::inches)(20.68)}, INTAKE_SIDE, 600); //OLD
    robot.turn_to_point({(au::inches)(74.69), (au::inches)(44.56)}, INTAKE_SIDE, 750);
    robot.move_to_point({(au::inches)(74.69), (au::inches)(44.56)}, false, INTAKE_SIDE, 0, 2.0);
    IntakeHelper::stuckCheckChange(false);
    LiftMngr::setLevel(STORE_HEIGHT);
    moveManual( 100, -4);
    
    

    // prep for wall stake
    robot.turn_to_point({(au::inches)(54.87), (au::inches)(44.35)}, MOGO_SIDE, 400);

    while(lineLeft.get_value() > 500) {
        moveManual(5, 4);
    }
    moveManual(40, -5);
    robot.chassis.brake();
    pros::delay(100);
    // robot.move_to_point({(au::inches)(54.87), (au::inches)(44.35)}, true, MOGO_SIDE, 700, 2.6); // OLD

    // STAKE
    robot.turn_with_pid(271.4, 600);
    IntakeHelper::voltage(-6);
    LiftMngr::setLevel(ABOVE_IN_HEIGHT);
    IntakeHelper::stuckCheckChange(true);
    pros::delay(150);
    IntakeHelper::voltage(12);
    moveManual(370, -6);
    moveManual(140, -5.5); //240
    robot.restOdomKeepAngle(54.58, 58.48);
    moveManual(10, -5.5);
    pros::delay(50);
    LiftMngr::setLevel(SCORE_HEIGHT);
    pros::delay(150);

    // back up
    robot.move_to_point({(au::inches)(54.4), (au::inches)(46.83)}, true, MOGO_SIDE, 400, 2.4);
    LiftMngr::setLevel(IDLE_ARM);
    IntakeHelper::voltage(12);

    // get ring
    robot.turn_with_pid(361.84, 800);
    robot.move_to_point({(au::inches)(23.8), (au::inches)(48.46)}, false, INTAKE_SIDE, 400, 2.9);

    // get rings (3)
    IntakeHelper::voltage(12);
    robot.refinedFollow(&pursuitPath.skillsANew, 3100, 5.5, INTAKE_SIDE, 12, 1);

    // drop goal
    robot.turn_with_pid(166.05, 400);
    IntakeHelper::voltage(12);
    moveManual(100, 5);
    moveManual(400, 4);
    moClamp.overrideState(0);
    pros::delay(100);

    // back off 
    robot.move_to_point({(au::inches)(5.11), (au::inches)(45.31)}, true, INTAKE_SIDE, 800, 2.9);

    // second mogo
    robot.turn_with_pid(271.75, 900);
    robot.move_to_point({(au::inches)(4.28), (au::inches)(-9.36)}, false, MOGO_SIDE, 800, 2.9);
    MogoUtils::getMogo(6, 2);

    // get ring A
    robot.turn_to_point({(au::inches)(21.18), (au::inches)(-23.03)}, INTAKE_SIDE, 680);
    RedRingUtil::getRing(true, 8, 2);

    // get ring B
    robot.move_to_point({(au::inches)(51.9), (au::inches)(-40.5)}, true, INTAKE_SIDE, 700, 2.8);
    RedRingUtil::getRing(true, 8, 2);
    IntakeHelper::stuckCheckChange(false);
    LiftMngr::setLevel(STORE_HEIGHT);
    moveManual(100, -4);

    // get to 89.41
    robot.turn_with_pid(162.31, 500);

    while(lineLeft.get_value() > 500) {
        moveManual(5, 4);
    }
    moveManual(60, -5); //70
    robot.chassis.brake();
    pros::delay(100);
    
    // get stake
    robot.turn_with_pid(89.41, 600);
    IntakeHelper::voltage(-6);
    LiftMngr::setLevel(ABOVE_IN_HEIGHT);
    IntakeHelper::stuckCheckChange(true);
    pros::delay(150);
    IntakeHelper::voltage(12);
    moveManual(370, -6);
    moveManual(250, -5.5);
    robot.restOdomKeepAngle(54.08, -56.45);
    // LiftMngr::setLevel(SCORE_HEIGHT);
    moveManual(10, -5.5);
    pros::delay(50);
    LiftMngr::setLevel(SCORE_HEIGHT);
    LiftMngr::setVoltage(-10, true);
    pros::delay(100);
    LiftMngr::setVoltage(-10, false);
    pros::delay(50);
    // TEST 
    time = 0;
    while(75 >= time*25) {
        LiftMngr::setVoltage(-12, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);
    pros::delay(100);
    // TEST 

    // back up
    // moClamp.overrideState(1); // REMOVE
    // pros::delay(200); // REMOVE
    // robot.restOdom(54.08, -56.45, 90); // REMOVE 90>???
    robot.move_to_point({(au::inches)(54.05), (au::inches)(-46.39)}, false, MOGO_SIDE, 400, 2.6);
    LiftMngr::setLevel(IDLE_ARM);
    IntakeHelper::voltage(12);
    

    // get ring
    robot.turn_with_pid(-0.34, 700);
    robot.move_to_point({(au::inches)(24.92), (au::inches)(-44.54)}, false, INTAKE_SIDE, 400, 2.9);
    IntakeHelper::voltage(12);
    robot.refinedFollow(&pursuitPath.skillsBNew, 1800, 5.5, INTAKE_SIDE, 12, 0, 4); //1700
    IntakeHelper::voltage(12);

    // place in corner 194.92
    moveManual(200, -4); //-5.98 -54.97
    robot.turn_to_point({(au::inches)(-5.98), (au::inches)(-54.97)}, MOGO_SIDE, 680);
    // robot.turn_with_pid(207.81, 500); // OLD
    IntakeHelper::voltage(12);
    moveManual(350, 2.5, 6);
    moveManual(210, 3, 4);
    pros::delay(200);
    moClamp.overrideState(0);
    pros::delay(100);
    moveManual(200, -9, -7); //TEST
    moveManual(30, 0);
    robot.turn_with_pid(192.5, 700);

    // get ring
    robot.move_to_point({(au::inches)(61.6), (au::inches)(-33.86)}, false, INTAKE_SIDE, 800, 2.8);
    robot.turn_to_point({(au::inches)(77.18), (au::inches)(-23.95)}, INTAKE_SIDE, 100);
    LiftMngr::setLevel(STORE_HEIGHT);
    IntakeHelper::StopAtColor(true);
    RedRingUtil::getRing(true, 8, 2); 
    IntakeHelper::stuckCheckChange(false);
    

    // get goal 
    robot.turn_with_pid(404.52, 900);
    IntakeHelper::StopAtColor(false);
    // robot.turn_to_point({(au::inches)(87.03), (au::inches)(-7.4)}, MOGO_SIDE, 1200);
    moveManual(100, 8);
    IntakeHelper::voltage(12);
    moveManual(100, 8);
    MogoUtils::getMogo(4, 2, 5);
    moveManual(70, 4);
    IntakeHelper::voltage(-6);
    LiftMngr::setLevel(ABOVE_IN_HEIGHT);
    IntakeHelper::stuckCheckChange(true);

    // bounce off alliance 
    robot.turn_with_pid(182.05+360, 1100); // OLD
    robot.move_to_point({(au::inches)(110.94), (au::inches)(0.44)}, false, INTAKE_SIDE, 200, 2.7);
    moveManual(230, -4);
    robot.ffwLat((au::inches)(6), (au::seconds)(1));

    // get alliance stake
    time = 0;
    while(340 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM + 30);
    IntakeHelper::voltage(12);

    // back off
    robot.move_to_point({(au::inches)(92.56), (au::inches)(0.83)}, false, MOGO_SIDE, 800, 2.7); //700
    LiftMngr::setLevel(IDLE_ARM);

    // get ring
    robot.turn_with_pid(311.4+360, 700);
    RedRingUtil::getRing(true, 8, 2);
    IntakeHelper::StopAtColor(true);
    pros::delay(100);
    
    // get ring under ladder
    robot.turn_with_pid(408+360, 700);
    // IntakeHelper::StopAtColor(false);
    // IntakeHelper::voltage(-10);
    // fIntVolt = 0;
    // fIntDelay = 150;
    // fIntSortOff = true;
    RedRingUtil::getRing(true, 6, 2);
    
    // get ring 
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    robot.turn_with_pid(497.64+360, 800, 8);
    // IntakeHelper::StopAtColor(true);
    // pros::delay(100);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(-11);
    pros::delay(100);
    IntakeHelper::voltage(0);
    // fIntVolt = 0;
    // fIntDelay = 150;
    // fIntSortOff = true;
    // robot.move_to_point({(au::inches)(72.11), (au::inches)(-25.68)}, false, INTAKE_SIDE, 300, 2.7);
    

    // get ring (true turn b4)
    fIntVolt = 12;
    fIntDelay = 900;
    IntakeHelper::sortState(true);
    // fIntSortOff = false;
    robot.move_to_point({(au::inches)(93.61), (au::inches)(-46.77)}, false, INTAKE_SIDE, 400, 2.7);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    // get ring & drop off mogo 467.6
    robot.turn_with_pid(457.5+360, 600);
    // robot.turn_to_point({(au::inches)(96.34), (au::inches)(-56.88)}, INTAKE_SIDE, 500);
    cornerDeploy.overrideState(1);
    pros::delay(70);
    moveManual(210, -6);
    // moveManual(450, -6, -10);
    // robot.move_to_point({(au::inches)(96.34), (au::inches)(-56.88)}, false, INTAKE_SIDE, 500, 2.8);
    robot.turn_with_pid(496.65+360, 700);
    moveManual(900, -6.5); //400
    robot.turn_with_pid(679+360, 700);
    robot.turn_with_pid(679+360, 800);
    IntakeHelper::voltage(0);
    moveManual(650, 6); //500
    moClamp.overrideState(0);
    pros::delay(100);
    cornerDeploy.overrideState(0);

    // back off & get in corner
    IntakeHelper::voltage(12);
    // robot.ramseteTest(({(au::inches)(73.98), (au::inches)(-37.34)}, INTAKE_SIDE, 11, 0, 8, 4, true, 2300); // FAST
    // robot.ramseteTest(({(au::inches)(97.53), (au::inches)(-0.25)}, INTAKE_SIDE, 11, 0, 4, 4, true, 2800); // FAST
    // robot.turn_with_pid(770.54+360, 600); // FAST
    robot.move_to_point({(au::inches)(88.38), (au::inches)(-45.88)}, true, INTAKE_SIDE, 100, 2.7); //500
    IntakeHelper::sortState(false);
    IntakeHelper::voltage(12);
    // robot.turn_with_pid(106.5+360, 600); //OLD
    robot.move_to_point({(au::inches)(110.42), (au::inches)(-6.73)}, true, INTAKE_SIDE, 700, 2.7);
    robot.turn_with_pid(762.31+360, 750); //780  -(360*3)
    
    // MogoUtils::angleTwdsMogo(3, 1, 7, 400, 8);
    // moveManual(300, 10, 8);
    MogoUtils::angleTwdsMogo(5, 2, 12, 200, 8);
    // MogoUtils::getMogo(9, 4, 12, 1350, 1);
    moveManual(300, 9); //350
    moveManual(350, 11, 6.5);
    moveManual(540, 11.3); //550 10
    // moveManual(1300, 10);
    
    robot.restOdomKeepAngle(0, 0); // CHANGE ANGLE
    IntakeHelper::voltage(0);
    moveManual(100, -10, -7);
    moveManual(250, -12, -9);
    moveManual(50, 0);

    // get hang 
     //944.54
    robot.turn_to_point({(au::inches)(-33.7), (au::inches)(-25.67)}, MOGO_SIDE, 780);
    fLbLift = 24.5;
    fLbDelay = 200;
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    moveManual(100, 7, 7);
    moveManual(150, 9, 12);
    moveManual(400, 9.75, 10);
    moveManual(125, 12, 12);


    // robot.refinedFollow(&pursuitPath.skillsEnd, 1500, 10, MOGO_SIDE, 11, 4, 5);
    /*
    fLbLift = 24.5;
    fLbDelay = 400;
    robot.ramseteTest({(au::inches)(-33.7), (au::inches)(-25.67)}, MOGO_SIDE, 11.5, 4, 2, 4, true, 1500); //1.5

    // robot.move_to_point({(au::inches)(-36.97), (au::inches)(-19.32)}, true, MOGO_SIDE, 900, 2.7); //1100
    // robot.turn_with_pid(914.47+360, 1070); //OLD
    // moveManual(330, 7, 9);
    // robot.refinedFollow(&pursuitPath.skillsDNew, 1800, 10, MOGO_SIDE, 11, 2, 5);
    // robot.turn_with_pid(944.54+360, 300); //-36.97 -19.32 ANG: 1305.23
    robot.turn_with_pid(1302.96-(360*3), 350); //300
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    moveManual(350, 12);
    // moveManual(440, 12);
    
    // moveManual(800, 4);
    /*
    // robot.ramseteTest({(au::inches)(-33.43), (au::inches)(-18.87)}, MOGO_SIDE, 12, 0, 4, 4, true, 1800);
    robot.turn_with_pid(944.54+360, 300);
    moveManual(240, 12);
    moveManual(800, 4);
    // moveManual(400, 8);

    // MIDDLE: -56.35, -46
    
/**/
}

void Routes::placehold4() {
   IntakeHelper::blueExcld(false);
    
    int time = 0;

    while(340 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    robot.move_to_point({(au::inches)(27.8), (au::inches)(-0)}, false, MOGO_SIDE, 0); //26.3
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(-85.05, 600); //700
    
    MogoUtils::getMogo(5, 3, 7);
    IntakeHelper::voltage(12);

    // get mid {(au::inches)(52.58), (au::inches)(-24.24)}
    // robot.turn_to_point({(au::inches)(52.58), (au::inches)(-24.24)}, INTAKE_SIDE, 900);
    robot.move_to_point({(au::inches)(50.01), (au::inches)(-28.18)}, true, INTAKE_SIDE, 750, 2.9); //900
    robot.turn_with_pid(-144.52, 700); //800
    // robot.changeDetectLine(true);
    robot.move_to_point({(au::inches)(60.04), (au::inches)(-19.26)}, false, INTAKE_SIDE, 0, 3);
    // robot.changeDetectLine(false);


    // curve back
    // moveManual(120, 6, 6); // 2nd RING BACK 120
    // moveManual(400, 9, 3); // 2nd RING BACK
    // moveManual(260, 5, 5);
    //

    robot.ramseteTest({(au::inches)(32.34), (au::inches)(-23.01)}, MOGO_SIDE, 11, 0.8, 0.7);

    // get ring
    robot.turn_with_pid(-142, 300); //-140.99, 600
    IntakeHelper::voltage(12);
    robot.ramseteTest({(au::inches)(47.26), (au::inches)(-12.78)}, INTAKE_SIDE, 10, 0.8, 0.6);
    // robot.move_to_point({(au::inches)(47.26), (au::inches)(-12.78)}, false, INTAKE_SIDE, 0, 2.8); //2.5

    // get corner
    robot.turn_to_point({(au::inches)(43.63), (au::inches)(17.62)},  INTAKE_SIDE, 700);
    robot.move_to_point({(au::inches)(43.63), (au::inches)(17.62)}, false, INTAKE_SIDE, 700, 3); //900
    moveManual(200, -7); // 160, -6
    moveManual(500, -4);

    // go to mid
    robot.move_to_point({(au::inches)(41.96), (au::inches)(18.45)}, true, MOGO_SIDE, 500, 3);
    // robot.turn_with_pid(35.1, 800);

    // ladder
    // robot.turn_to_point({(au::inches)(18.71), (au::inches)(-18.15)}, INTAKE_SIDE, 900);
    robot.move_to_point({(au::inches)(18.71), (au::inches)(-18.15)}, true, INTAKE_SIDE, 900);
    moveManual(150, -7);
    moveManual(370, -3);


    /*

    // get ring @ alliance stack
    robot.move_to_point({(au::inches)(12.79), (au::inches)(-0.58)}, false, INTAKE_SIDE, 500, 3);
    liftIntake.overrideState(1);
    IntakeHelper::voltage(12);
    pros::delay(80);
    moveManual(150, -8, -7);
    liftIntake.overrideState(0);
    moveManual(150, -7);
    robot.turn_with_pid(109.73, 600);
    moveManual(450, -6);
    // moveManual(150, -5);
    



    // robot.move_to_point({(au::inches)(22.74), (au::inches)(24.36)}, true, INTAKE_SIDE, 900, 2.5);

    // robot.turn_with_pid(-140.31, 900);

    /**/
}

void Routes::placehold4Mir() {
    pros::Task lbThing(ladyBrownDelay);
    pros::Task intakeThing(intakeDelay);
    IntakeHelper::blueExcld(true);
    int time = 0;

    while(340 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    robot.move_to_point({(au::inches)(27.12), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(85.05, 700); //.9k
    
    MogoUtils::getMogo(5, 3, 7);
    IntakeHelper::voltage(12);

    // get mid 
    // robot.turn_to_point({(au::inches)(52.81), (au::inches)(27.94)}, INTAKE_SIDE, 800);
    robot.move_to_point({(au::inches)(49.32), (au::inches)(29.78)}, true, INTAKE_SIDE, 800, 2.7);
    pros::delay(80);
    robot.turn_with_pid(150.04, 700);
    // robot.changeDetectLine(true); {(au::inches)(56.32), (au::inches)(27.31)} 54.4, 48.37
    robot.move_to_point({(au::inches)(56.32), (au::inches)(27.31)}, false, INTAKE_SIDE, 700, 3); //3
    // robot.changeDetectLine(false);


    // curve back
    moveManual(50, 6, 6); // 2nd RING BACK 140
    moveManual(400, 3, 9); // 2nd RING BACK
    moveManual(270, 5, 5);

    // get ring
    robot.turn_with_pid(145.4, 700);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(44.53), (au::inches)(17.97)}, false, INTAKE_SIDE, 700); // 3
    
    // get corner
    // robot.move_to_point({(au::inches)(41.3), (au::inches)(-29.36)}, true, INTAKE_SIDE, 700, 3); //900
    // moveManual(300, -10);

    // robot.turn_to_point({(au::inches)(42.55), (au::inches)(-11.09)}, INTAKE_SIDE, 900);
    robot.move_to_point({(au::inches)(42.55), (au::inches)(-11.09)}, true, INTAKE_SIDE, 800, 2.5); //2.9
    moveManual(300, -7, -9); //-10
    moveManual(500, -7); //-10

    // back off
    robot.move_to_point({(au::inches)(47.87), (au::inches)(-17.37)}, true, MOGO_SIDE, 600); //3

    // ladder
    // robot.turn_with_pid(-51.22, 800);
    // LiftMngr::setLevel(125);
    robot.move_to_point({(au::inches)(19.6), (au::inches)(18.64)}, true, INTAKE_SIDE, 900); //2.7
    moveManual(100, -8);
    moveManual(400, -5);
    
    // robot.move_to_point({(au::inches)(42.7), (au::inches)(-23.74)}, false, MOGO_SIDE, 0);
    /**/
}

void Routes::placehold1() {
    pros::Task clampthing(mogoPistonDelay);
    IntakeHelper::blueExcld(false);
    IntakeHelper::sortState(false);
    IntakeHelper::stuckCheckChange(true);
    
    int time = 0;

    while(345 >= time*5) { //320
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(5);
    }
    LiftMngr::setVoltage(0, false);
    LiftMngr::setLevel(STRAIGHT_ARM);

    // get goal + reset lb
    moveManual(290, 7); //250
    LiftMngr::setLevel(IDLE_ARM);
    pros::delay(50);
    robot.turn_with_pid(49.29, 300); //300, 55
    pros::delay(40);
    MogoUtils::getMogo(8, 7, 12, 320, 1);
    MogoUtils::getMogo(5, 3, 6, 1000, 27);
    IntakeHelper::voltage(12);

    // grab mid ring @ alliance stake
    liftIntake.overrideState(1); //-1.49
    // robot.turn_to_point({(au::inches)(5.35), (au::inches)(-20.31)}, INTAKE_SIDE, 650); 
    robot.turn_with_pid(1.49, 650);
    robot.move_to_point({(au::inches)(5.5), (au::inches)(20.72)}, false, INTAKE_SIDE, 600, 2.9);
    liftIntake.overrideState(0);
    
    moveManual(150, 7);
    // liftIntake.overrideState(0);
    pros::delay(50);
    // turn twds other ring
    // robot.turn_to_point({(au::inches)(37.57), (au::inches)(14.81)}, INTAKE_SIDE, 700); /* //800 162.2
    robot.turn_with_pid(165.95, 700); //???
    robot.move_to_point({(au::inches)(38.55), (au::inches)(15.45)}, false, INTAKE_SIDE, 700, 2.9);

    // go twds corner
    robot.turn_with_pid(110, 480);
    cornerDeploy.overrideState(1);
    moveManual(175, -6);
    pros::delay(40);
    robot.turn_with_pid(-10, 275);
    cornerDeploy.overrideState(0); 

    // get corner
    // robot.turn_to_point({(au::inches)(34.21), (au::inches)(-14.79)}, INTAKE_SIDE, 650); /*
    robot.turn_with_pid(68.65, 600); //550
    robot.move_to_point({(au::inches)(31.98), (au::inches)(-11.24)}, false, INTAKE_SIDE, 550, 2.9);
    robot.turn_with_pid(96.9, 450); // 400
    IntakeHelper::stuckCheckChange(false);
    IntakeHelper::blueExcld(true);
    IntakeHelper::sortState(false);
    IntakeHelper::StopAtColor(true);
    

    // first ring
    IntakeHelper::voltage(12);
    moveManual(300, -3); //-2
    moveManual(700, -3); //700
    // moveManual(500, -5.5);

    // get second ring
    moveManual(500, 2); // 400, 2
    liftIntake.overrideState(1);
    // pros::delay(100); //60
    
    moveManual(200, -2);
    moveManual(340, -3.75);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3);

    IntakeHelper::voltage(12);

    
    
    // place goal in corner
    cornerDeploy.overrideState(1);
    pros::delay(250); //500
    moveManual(400, -7, -10);
    IntakeHelper::voltage(0);
    robot.turn_with_pid(283.67, 1350); //1250
    moClamp.overrideState(0);
    moveManual(600, 7.5);

    // get out of corner
    cornerDeploy.overrideState(0);
    moveManual(300, -6);

    // touch ladder 
    // LiftMngr::setLevel(STRAIGHT_ARM+30);
    robot.turn_with_pid(283.01, 550);
    IntakeHelper::voltage(12);
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    moveManual(450, -8); //520
    

    /**/

}

void Routes::placehold1Mir() {
    pros::Task clampthing(mogoPistonDelay);
    IntakeHelper::blueExcld(false);
    IntakeHelper::sortState(false);
    IntakeHelper::stuckCheckChange(true);
    
    int time = 0;

    while(345 >= time*5) { //320
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(5);
    }
    LiftMngr::setVoltage(0, false);
    LiftMngr::setLevel(STRAIGHT_ARM);

    // get goal + reset lb
    moveManual(290, 7); //250
    LiftMngr::setLevel(IDLE_ARM);
    pros::delay(50);
    robot.turn_with_pid(-49.29, 300); //300, 55
    pros::delay(40);
    MogoUtils::getMogo(8, 7, 12, 320, 1);
    MogoUtils::getMogo(5, 3, 6, 1000, 27);
    IntakeHelper::voltage(12);

    // grab mid ring @ alliance stake
    liftIntake.overrideState(1); //-1.49
    // robot.turn_to_point({(au::inches)(5.35), (au::inches)(-20.31)}, INTAKE_SIDE, 650); 
    robot.turn_with_pid(-1.49, 650);
    robot.move_to_point({(au::inches)(5.5), (au::inches)(-20.72)}, false, INTAKE_SIDE, 600, 2.9);
    liftIntake.overrideState(0);
    
    moveManual(150, 7);
    // liftIntake.overrideState(0);
    pros::delay(50);
    // turn twds other ring
    // robot.turn_to_point({(au::inches)(37.57), (au::inches)(14.81)}, INTAKE_SIDE, 700); /* //800 162.2
    robot.turn_with_pid(-165.95, 700); //???
    robot.move_to_point({(au::inches)(38.55), (au::inches)(-15.45)}, false, INTAKE_SIDE, 700, 2.9);

    // go twds corner
    robot.turn_with_pid(-87.72, 500); //480
    cornerDeploy.overrideState(1);
    moveManual(175, -6);
    pros::delay(40);
    robot.turn_with_pid(0, 275);
    cornerDeploy.overrideState(0); 

    // get corner
    // robot.turn_to_point({(au::inches)(34.21), (au::inches)(-14.79)}, INTAKE_SIDE, 650); /*
    robot.turn_with_pid(-76.31, 600); //550
    robot.move_to_point({(au::inches)(32.53), (au::inches)(11.46)}, false, INTAKE_SIDE, 550, 2.9);
    robot.turn_with_pid(-101.60, 450); // 400
    IntakeHelper::stuckCheckChange(false);
    IntakeHelper::blueExcld(true);
    IntakeHelper::sortState(false);
    IntakeHelper::StopAtColor(true);
    

    // first ring
    IntakeHelper::voltage(12);
    moveManual(300, -3); //-2
    moveManual(700, -3); //700
    // moveManual(500, -5.5);

    // get second ring
    moveManual(500, 2); // 400, 2
    liftIntake.overrideState(1);
    // pros::delay(100); //60
    
    moveManual(200, -2);
    moveManual(340, -3.75);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3);

    IntakeHelper::voltage(12);

    
    
    // place goal in corner
    cornerDeploy.overrideState(1);
    pros::delay(250); //500
    moveManual(400, -10, -7);
    IntakeHelper::voltage(0);
    robot.turn_with_pid(59.24, 1350); //1250
    moClamp.overrideState(0);
    moveManual(600, 7.5);

    // get out of corner
    cornerDeploy.overrideState(0);
    moveManual(300, -6);

    // touch ladder 
    // LiftMngr::setLevel(STRAIGHT_ARM+30);
    robot.turn_with_pid(47.5, 550);
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    moveManual(450, -8); //520
    // LiftMngr::setLevel(STRAIGHT_ARM+10);

    /**/
}

void Routes::placehold2() {
    /**/
}

void Routes::placehold2Mir() {
    /**/
}

void Routes::placehold3() {
    pros::Task clampthing(mogoPistonDelay);
    IntakeHelper::blueExcld(false);
    IntakeHelper::sortState(false);
    IntakeHelper::stuckCheckChange(true);
    
    int time = 0;

    while(345 >= time*5) { //320
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(5);
    }
    LiftMngr::setVoltage(0, false);
    LiftMngr::setLevel(STRAIGHT_ARM);

    // get goal + reset lb
    moveManual(250, 6);
    LiftMngr::setLevel(IDLE_ARM);
    moveManual(350, 9, 1);
    moveManual(120, 8, 8); //9
    MogoUtils::getMogo(5, 3, 6, 500);
    IntakeHelper::voltage(12);

    // get mid line
    // robot.turn_to_point({(au::inches)(41.4), (au::inches)(-33.01)},  INTAKE_SIDE, 720); //OLD
    robot.move_to_point({(au::inches)(41.4), (au::inches)(-33.01)}, true, INTAKE_SIDE, 720, 2.9); 
    robot.turn_with_pid(-151.56, 600); //660 x or y -?
    robot.move_to_point({(au::inches)(46.95), (au::inches)(-29.42)}, false, INTAKE_SIDE, 0, 3); 

    // curve back and get other ring
    moveManual(40, 6, 6); // 2nd RING BACK 0
    moveManual(370, 9.5, 1.5); // 2nd RING BACK //400,9,2
    moveManual(300, 5, 5); //260
    pros::delay(70);

    // get ring
    robot.turn_with_pid(-145.00, 680);
    robot.move_to_point({(au::inches)(34.39), (au::inches)(-19.38)}, false, INTAKE_SIDE, 0, 2.8);

    // get corner
    robot.turn_to_point({(au::inches)(27.96), (au::inches)(12.92)},  INTAKE_SIDE, 840); //OLD
    robot.move_to_point({(au::inches)(27.96), (au::inches)(12.92)}, false, INTAKE_SIDE, 700, 3); //900
    robot.turn_with_pid(-102.40, 670);
    IntakeHelper::blueExcld(true);
    IntakeHelper::StopAtColor(true);

    IntakeHelper::voltage(12);
    moveManual(800, -3); //800, -6

    // moveManual(400, -4);

    // get second ring
    moveManual(320, 3); // 350
    liftIntake.overrideState(1);
    // pros::delay(100); //60
    moveManual(300, -4);
    moveManual(200, -4);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3);

    
    // 

    /**/
}

void Routes::placehold3Mir() {
    pros::Task clampthing(mogoPistonDelay);
    IntakeHelper::blueExcld(true);
    IntakeHelper::sortState(false);
    IntakeHelper::stuckCheckChange(true);
    
    int time = 0;

    while(325 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    // get goal + reset lb
    moveManual(350, 1, 9);
    LiftMngr::setLevel(IDLE_ARM);
    moveManual(150, 9, 9);
    MogoUtils::getMogo(5, 3, 6);
    IntakeHelper::voltage(12);

    // get mid line
    robot.move_to_point({(au::inches)(48.61), (au::inches)(30.18)}, true, INTAKE_SIDE, 700, 2.9); 
    robot.turn_with_pid(154.42, 600); //660
    robot.move_to_point({(au::inches)(58.04), (au::inches)(25.14)}, false, INTAKE_SIDE, 0, 3); 

    // curve back and get other ring
    moveManual(150, 6, 6); // 2nd RING BACK 150
    moveManual(400, 2, 9); // 2nd RING BACK
    moveManual(240, 5, 5); //240
    pros::delay(70);

    // get ring
    robot.turn_with_pid(146.47, 640);
    robot.move_to_point({(au::inches)(43.55), (au::inches)(16.32)}, false, INTAKE_SIDE, 0, 2.8);

    // get corner
    robot.turn_to_point({(au::inches)(36.41), (au::inches)(-15.99)},  INTAKE_SIDE, 700); //OLD
    robot.move_to_point({(au::inches)(36.41), (au::inches)(-15.99)}, false, INTAKE_SIDE, 700, 3); //900
    robot.turn_with_pid(99.66, 510);
    IntakeHelper::blueExcld(false);
    IntakeHelper::StopAtColor(true);


    moveManual(1000, -6);
    // moveManual(400, -4);

    // get second ring
    moveManual(250, 3.7); // 300
    liftIntake.overrideState(1);
    // pros::delay(100); //60
    moveManual(300, -4);
    moveManual(200, -4);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3);

    
    IntakeHelper::voltage(12);

    /*
    // moveManual(100, 5);
    // get corner
    // robot.turn_with_pid(-77.14, 700);
    robot.turn_to_point({(au::inches)(38.92), (au::inches)(-17.62)},  INTAKE_SIDE, 700); //OLD
    // robot.ramseteTest({(au::inches)(37.95), (au::inches)(16.54)}, INTAKE_SIDE, 8.5, 0.8, 0.5);
    robot.move_to_point({(au::inches)(38.92), (au::inches)(-17.62)}, false, INTAKE_SIDE, 700, 3); //900
    robot.turn_with_pid(98.27, 440);
    IntakeHelper::stuckCheckChange(false);
    // IntakeHelper::voltage(12);

    moveManual(610, -6); // 500
    moveManual(400, -4);

    // get second ring
    moveManual(300, 3.7); // 300
    liftIntake.overrideState(1);
    pros::delay(60);
    moveManual(300, -6); // 160, -6
    moveManual(200, -4);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3); //340

    
    IntakeHelper::voltage(12);

    // touch ladder
    IntakeHelper::stuckCheckChange(true);
    IntakeHelper::StopAtColor(false); //144.04
    IntakeHelper::voltage(12);
    robot.turn_with_pid(144.04, 850, 9);
    pros::delay(80);

    fIntMogoUpDown = 0;
    fIntMogoDelay = 150;
    // robot.turn_to_point({(au::inches)(21.8), (au::inches)(15.65)}, INTAKE_SIDE, 1000);
    // IntakeHelper::StopAtColor(false);
    moveManual(300, 6);
    moveManual(1200, 10.6);
    // LiftMngr::setLevel(125); //{(au::inches)(21.8), (au::inches)(15.65)}
    // robot.ramseteTest({(au::inches)(21.35), (au::inches)(17.19)}, INTAKE_SIDE, 11.5, 2, 2, 4, true, 1400);
    // moveManual(500, -3);
    /*
    fIntPisUpDown = 1;
    fIntPisDelay = 500;
    
    moveManual(250, -6); // 160, -6
    liftIntake.overrideState(0);
    moveManual(600, 5);
    /**/
}

void Routes::placehold5() {
    // Setup
    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(false);
    LiftMngr::setLevel(275);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(12);

    fIntVolt = 0;
    fIntDelay = 1270;
    

    // Rush mid
    // IntakeHelper::StopAtColor(true);
    // moveManual(700, -12);
    robot.move_to_point({(au::inches)(-35.5), (au::inches)(-0)}, false, INTAKE_SIDE, 0, 2.9);
    rushClamp.overrideState(1);

    // back off & unclamp
    // moveManual(150, 10);
    robot.move_to_point({(au::inches)(-26.3), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    rushClamp.overrideState(0);
    cornerDeploy.overrideState(0);
    moveManual(200, 4);

    // get rush MOGO & score
    robot.turn_with_pid(-167.02, 1000);
    MogoUtils::getMogo(9, 3);
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    // drop mogo
    robot.move_to_point({(au::inches)(-31.11), (au::inches)(-3.83)}, false, INTAKE_SIDE, 0);
    pros::delay(600); //1.1k
    moClamp.overrideState(0);
    pros::delay(100);

    // grab other mogo
    robot.turn_with_pid(-65.89, 1000);
    MogoUtils::getMogo(10, 3);

    // grab ring
    robot.turn_with_pid(-170.45, 800);
    RedRingUtil::getRing(false, 11, 4);
    moveManual(100, -3);

    // drop mogo 
    pros::delay(1100);
    moClamp.overrideState(0);
    robot.move_to_point({(au::inches)(-5.02), (au::inches)(4.60)}, true, INTAKE_SIDE, 900);

    // get corner
    robot.turn_with_pid(-120.41, 900);
    robot.move_to_point({(au::inches)(4.95), (au::inches)(19.59)}, false, INTAKE_SIDE, 900);
    LiftMngr::setLevel(245);
    moveManual(240, -5);

    // back off
    robot.move_to_point({(au::inches)(-7.22), (au::inches)(-5.95)}, false, MOGO_SIDE, 900);
    robot.turn_with_pid(-17.64, 800);
    IntakeHelper::voltage(0);

    // get on mid stake
    LiftMngr::setLevel(140);
    robot.move_to_point({(au::inches)(-40.48), (au::inches)(3.79)}, false, INTAKE_SIDE, 900);

    /**/
}

void Routes::placehold6() {
    pros::Task inSortThing(intakeSortDelay);
    pros::Task pisTing(intakePistonDelay);
    pros::Task pisMogo(mogoPistonDelay);
    IntakeHelper::blueExcld(false);
    // IntakeHelper::sortState(false);
    IntakeHelper::stuckCheckChange(false);
    
    int time = 0;

    while(340 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    robot.move_to_point({(au::inches)(27.8), (au::inches)(-0)}, false, MOGO_SIDE, 0, 3); //26.3
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(-85.05, 500); //600
    
    moveManual(180, 8);
    MogoUtils::getMogo(5, 3, 6);
    IntakeHelper::voltage(12);

    // get mid {(au::inches)(52.58), (au::inches)(-24.24)}
    // robot.turn_to_point({(au::inches)(48.59), (au::inches)(-26.08)}, INTAKE_SIDE, 670);
    robot.move_to_point({(au::inches)(50.71), (au::inches)(-26.45)}, true, INTAKE_SIDE, 670, 2.9); //OLD
    robot.turn_with_pid(-152, 660); //OLD -144.52
    // robot.ramseteTest({(au::inches)(56.27), (au::inches)(-27.5)}, INTAKE_SIDE, 8, 0.3, 0.7);
    // robot.changeDetectLine(true);
    robot.move_to_point({(au::inches)(57.17), (au::inches)(-23.34)}, false, INTAKE_SIDE, 0, 3); //OLD
    // IntakeHelper::stuckCheckChange(true);
    // robot.changeDetectLine(false);


    // curve back
    // moveManual(40, 6, 6); // 2nd RING BACK 120
    moveManual(400, 9, 2); // 2nd RING BACK
    moveManual(300, 5, 5); //260
    pros::delay(70);
    //

    // robot.ramseteTest({(au::inches)(32.34), (au::inches)(-23.01)}, MOGO_SIDE, 9, 0.5, 0.7);

    // get ring
    robot.turn_with_pid(-138.8, 550); //-140.99, 600
    IntakeHelper::voltage(12);
    // robot.ramseteTest({(au::inches)(47.26), (au::inches)(-12.78)}, INTAKE_SIDE, 10, 0.8, 0.6);
    robot.move_to_point({(au::inches)(42.61), (au::inches)(-17.03)}, false, INTAKE_SIDE, 0, 2.8); //2.5
    
    // get ring @ alliance stake
    robot.turn_to_point({(au::inches)(-7.25), (au::inches)(-13.61)}, INTAKE_SIDE, 650);
    liftIntake.overrideState(1);
    // robot.move_to_point({(au::inches)(10.6), (au::inches)(-15.98)}, false, INTAKE_SIDE, 800, 2.6); //2.5
    // liftIntake.overrideState(0);
    fIntMogoUpDown = 0;
    fIntMogoDelay = 800;
    finColor = true;
    finColorSortDelay = 800;
    robot.move_to_point({(au::inches)(-7.25), (au::inches)(-13.61)}, true, INTAKE_SIDE, 200, 2.6); //2.5
    // IntakeHelper::StopAtColor(true);

    // drop & get other mogo
    // moClamp.overrideState(0);
    liftIntake.overrideState(0);


    robot.turn_with_pid(-93.38, 600);
    // IntakeHelper::voltage(0);
    MogoUtils::getMogo(6.8, 3, 8);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    // get other ring
    robot.turn_to_point({(au::inches)(-28.1), (au::inches)(-57.99)}, INTAKE_SIDE, 720); //TEST
    robot.move_to_point({(au::inches)(-28.1), (au::inches)(-57.99)}, true, INTAKE_SIDE, 720, 3); //2.5 

    // set the ang of lb & move twds last pt
    LiftMngr::setLevel(120); //OLD
    // robot.turn_with_pid(-57.8, 500); // {(au::inches)(-7.15), (au::inches)(-55.66)} {(au::inches)(-0.53), (au::inches)(-40.74)} NEW
    //
    // robot.turn_with_pid(-153.57, 200);
    robot.ramseteTest({(au::inches)(-14.28), (au::inches)(-56.21)} , INTAKE_SIDE, 11, 0, 0.6);

    /**/
}

void Routes::placehold6Mir() {
    pros::Task inSortThing(intakeSortDelay);
    pros::Task pisTing(intakePistonDelay);
    pros::Task pisMogo(mogoPistonDelay);
    IntakeHelper::blueExcld(true);
    IntakeHelper::sortState(false);
    
    int time = 0;

    while(340 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    robot.move_to_point({(au::inches)(27.8), (au::inches)(-0)}, false, MOGO_SIDE, 0, 3); //26.3
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(86.5, 500); //600
    
    moveManual(180, 8);
    MogoUtils::getMogo(5, 3, 6); //3 min
    IntakeHelper::voltage(12);

    // get mid {(au::inches)(52.58), (au::inches)(-24.24)}
    // robot.turn_to_point({(au::inches)(45.34), (au::inches)(35.62)}, INTAKE_SIDE, 700);
    robot.move_to_point({(au::inches)(47.34), (au::inches)(32.85)}, true, INTAKE_SIDE, 700, 2.9); //OLD
    robot.turn_with_pid(152.24, 650); //OLD -144.52
    IntakeHelper::stuckCheckChange(true);
    // robot.ramseteTest({(au::inches)(60.04), (au::inches)(-19.26)}, INTAKE_SIDE, 8, 0.3, 0.7);
    // robot.changeDetectLine(true);
    robot.move_to_point({(au::inches)(58.01), (au::inches)(32.01)}, false, INTAKE_SIDE, 200, 2.9); //OLD
    // IntakeHelper::stuckCheckChange(true);
    // robot.changeDetectLine(false);


    // curve back
    moveManual(20, 6, 6); // 2nd RING BACK 120
    moveManual(400, 2, 10); // 2nd RING BACK 3,9
    moveManual(240, 5, 5);
    pros::delay(70); //70
    //

    // robot.ramseteTest({(au::inches)(32.34), (au::inches)(-23.01)}, MOGO_SIDE, 9, 0.5, 0.7);

    // get ring
    robot.turn_with_pid(135.16, 600); // 149.78, 600 OLD
    IntakeHelper::voltage(12);
    // robot.ramseteTest({(au::inches)(47.26), (au::inches)(-12.78)}, INTAKE_SIDE, 10, 0.8, 0.6);
    // {(au::inches)(44.46), (au::inches)(14.28)} OLD
    robot.move_to_point({(au::inches)(44.19), (au::inches)(16.89)}, false, INTAKE_SIDE, 0, 2.8); //2.5]

    // get ring @ alliance stake
    robot.turn_to_point({(au::inches)(-5.99), (au::inches)(18.84)}, INTAKE_SIDE, 700); // {(au::inches)(-5.99), (au::inches)(18.84)}
    liftIntake.overrideState(1);
    // robot.move_to_point({(au::inches)(10.6), (au::inches)(-15.98)}, false, INTAKE_SIDE, 800, 2.6); //2.5
    // liftIntake.overrideState(0);
    fIntMogoUpDown = 0;
    fIntMogoDelay = 800;
    finColor = true;
    finColorSortDelay = 800;
    robot.move_to_point({(au::inches)(-5.99), (au::inches)(18.84)}, true, INTAKE_SIDE, 200, 2.6); //2.5
    // IntakeHelper::StopAtColor(true);
    
    // drop & get other mogo
    // moClamp.overrideState(0);
    liftIntake.overrideState(0);
    // pros::delay(50); //250
    LiftMngr::setLevel(125);

    robot.turn_with_pid(93.38, 650);
    // IntakeHelper::voltage(0);
    MogoUtils::getMogo(5.6, 3, 6.5);
    pros::delay(40);
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    // get other ring
    // robot.turn_to_point({(au::inches)(-30.40), (au::inches)(61.60)}, INTAKE_SIDE,  740);
    robot.move_to_point({(au::inches)(-30.40), (au::inches)(61.60)}, true, INTAKE_SIDE, 740, 3); //2.5 

    // set the ang of lb & move twds last pt
    // robot.turn_with_pid(-57.8, 500); //{(au::inches)(-7.15), (au::inches)(55.66)}
    robot.ramseteTest({(au::inches)(-10.78), (au::inches)(54.84)}, INTAKE_SIDE, 12, 0, 1.5);

    /**/
}

void Routes::oldplacehold6() {
    pros::Task clampthing(mogoPistonDelay);
    IntakeHelper::blueExcld(false);
    IntakeHelper::sortState(false);
    IntakeHelper::stuckCheckChange(true);
    
    int time = 0;

    while(345 >= time*5) { //320
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(5);
    }
    LiftMngr::setVoltage(0, false);
    LiftMngr::setLevel(STRAIGHT_ARM);

    // get goal + reset lb
    moveManual(250, 6);
    LiftMngr::setLevel(IDLE_ARM);
    moveManual(350, 9, 1);
    moveManual(120, 8, 8); //9
    MogoUtils::getMogo(5, 3, 6, 500);
    IntakeHelper::voltage(12);

    // get mid line
    // robot.turn_to_point({(au::inches)(41.4), (au::inches)(-33.01)},  INTAKE_SIDE, 720); //OLD
    robot.move_to_point({(au::inches)(41.4), (au::inches)(-33.01)}, true, INTAKE_SIDE, 720, 2.9); 
    robot.turn_with_pid(-151.56, 600); //660 x or y -?
    robot.move_to_point({(au::inches)(46.95), (au::inches)(-29.42)}, false, INTAKE_SIDE, 0, 3); 

    // curve back and get other ring
    moveManual(40, 6, 6); // 2nd RING BACK 0
    moveManual(370, 9.5, 1.5); // 2nd RING BACK //400,9,2
    moveManual(300, 5, 5); //260
    pros::delay(70);

    // get ring
    robot.turn_with_pid(-145.00, 680);
    robot.move_to_point({(au::inches)(34.39), (au::inches)(-19.38)}, false, INTAKE_SIDE, 0, 2.8);

    // get center stack & curve?
    robot.turn_to_point({(au::inches)(2.75), (au::inches)(-23.26)}, INTAKE_SIDE, 750); //660
    liftIntake.overrideState(1);
    // IntakeHelper::blueExcld(true);
    // IntakeHelper::StopAtColor(true);
    // IntakeHelper::voltage(12); //{(au::inches)(1.53), (au::inches)(-18.3)} //OLD {(au::inches)(1.13), (au::inches)(-22.15)} //NEW
    robot.move_to_point({(au::inches)(2.75), (au::inches)(-23.26)}, false, INTAKE_SIDE, 0, 2.8);

    pros::delay(300); //300
    moveManual(400, -8, -4);
    moveManual(200, -8); //400, -8
    moClamp.overrideState(0);
    IntakeHelper::voltage(0);
    moveManual(250, -8); //250
    IntakeHelper::voltage(-12);
    pros::delay(50);

    // get other mogo
    robot.turn_with_pid(-36.09, 680);
    MogoUtils::getMogo(5, 3, 6);
    liftIntake.overrideState(0);

    // get ring
    IntakeHelper::voltage(12);
    // robot.turn_to_point({(au::inches)(-27.1), (au::inches)(-61.87)}, INTAKE_SIDE, 680);
    robot.move_to_point({(au::inches)(-27.1), (au::inches)(-61.87)}, true, INTAKE_SIDE, 680, 2.8);

    // touch ladder
    robot.turn_with_pid(181.01, 890);
    LiftMngr::setLevel(STRAIGHT_ARM+30);
    robot.move_to_point({(au::inches)(-10.72), (au::inches)(-55.55)}, false, INTAKE_SIDE, 680, 2.8);
    LiftMngr::setLevel(STRAIGHT_ARM+10);



    /**/
}

void Routes::oldplacehold6Mir() {
    pros::Task clampthing(mogoPistonDelay);
    IntakeHelper::blueExcld(false);
    IntakeHelper::sortState(false);
    IntakeHelper::stuckCheckChange(true);
    
    int time = 0;

    while(345 >= time*5) { //320
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(5);
    }
    LiftMngr::setVoltage(0, false);
    LiftMngr::setLevel(STRAIGHT_ARM);

    // get goal + reset lb
    moveManual(250, 6);
    LiftMngr::setLevel(IDLE_ARM);
    moveManual(350, 1, 9);
    moveManual(120, 8, 8); //9
    MogoUtils::getMogo(5, 3, 6, 550, 25);
    IntakeHelper::voltage(12);

    // get mid line
    // robot.turn_to_point({(au::inches)(41.4), (au::inches)(-33.01)},  INTAKE_SIDE, 720); //OLD
    robot.move_to_point({(au::inches)(43.18), (au::inches)(33.73)}, true, INTAKE_SIDE, 720, 2.9);
    robot.turn_with_pid(151.56, 600); //660 x or y -?
    robot.move_to_point({(au::inches)(49.13), (au::inches)(30.11)}, false, INTAKE_SIDE, 0, 3); 

    // curve back and get other ring
    moveManual(40, 6, 6); // 2nd RING BACK 40
    moveManual(370, 1.5, 9.5); // 2nd RING BACK //400,9,2
    moveManual(300, 5, 5); //260
    pros::delay(70);

    // get ring
    robot.turn_with_pid(145.00, 680);
    robot.move_to_point({(au::inches)(34.39), (au::inches)(19.38)}, false, INTAKE_SIDE, 0, 2.8);
    IntakeHelper::voltage(0);

    // get center stack & curve?
    robot.turn_to_point({(au::inches)(3.095), (au::inches)(23.751)}, INTAKE_SIDE, 750); //660
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    
    liftIntake.overrideState(1);
    // IntakeHelper::blueExcld(true);
    // IntakeHelper::StopAtColor(true);
    // IntakeHelper::voltage(12); //{(au::inches)(1.53), (au::inches)(-18.3)} //OLD {(au::inches)(1.13), (au::inches)(-22.15)} //NEW
    
    robot.move_to_point({(au::inches)(3.095), (au::inches)(23.751)}, false, INTAKE_SIDE, 0, 2.8); //2.75, 23.26

    pros::delay(300); //300
    moveManual(400, -4, -8);
    moveManual(200, -8); //400, -8
    moClamp.overrideState(0);
    IntakeHelper::voltage(0);
    moveManual(250, -8); //250
    IntakeHelper::voltage(-12);
    pros::delay(50);

    // get other mogo
    robot.turn_with_pid(36.09, 680);
    MogoUtils::getMogo(5, 3, 6);
    liftIntake.overrideState(0);

    // get ring
    IntakeHelper::voltage(12);
    // robot.turn_to_point({(au::inches)(-27.1), (au::inches)(-61.87)}, INTAKE_SIDE, 680);
    robot.move_to_point({(au::inches)(-27.1), (au::inches)(61.87)}, true, INTAKE_SIDE, 680, 2.8);

    // touch ladder
    robot.turn_with_pid(-181.01, 890);
    LiftMngr::setLevel(STRAIGHT_ARM+30);
    robot.move_to_point({(au::inches)(-10.72), (au::inches)(55.55)}, false, INTAKE_SIDE, 680, 2.8);
    LiftMngr::setLevel(STRAIGHT_ARM+10);



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
    IntakeHelper::sortState(true);
    IntakeHelper::blueExcld(true);
    // LiftMngr::setLevel(230);
    MogoUtils::getMogo(6.3, 4); //8

    // get ring @ alliance stack
    IntakeHelper::voltage(12);
    liftIntake.overrideState(1);
    // robot.turn_to_point({(au::inches)(2.99), (au::inches)(8.78)}, INTAKE_SIDE, 600);
    robot.move_to_point({(au::inches)(4.55), (au::inches)(-7.63)}, true, INTAKE_SIDE, 600, 2.9);
    liftIntake.overrideState(0);
    moveManual(300, 7);
    pros::delay(80);

    // turn twds other ring
    // robot.turn_to_point({(au::inches)(27.03), (au::inches)(-10.73)}, INTAKE_SIDE, 800);
    robot.move_to_point({(au::inches)(27.63), (au::inches)(10.97)}, true, INTAKE_SIDE, 800, 2.9);


    // go twds corner
    robot.turn_with_pid(-50, 450);
    cornerDeploy.overrideState(1);
    moveManual(250, -6); //longer? 350
    pros::delay(60);
    robot.turn_with_pid(20, 280);
    cornerDeploy.overrideState(0); 

    // get corner
    // robot.turn_to_point({(au::inches)(9.60), (au::inches)(31.22)}, INTAKE_SIDE, 650);
    robot.move_to_point({(au::inches)(9.60), (au::inches)(31.22)}, true, INTAKE_SIDE, 650, 2.9);
    robot.turn_with_pid(-73.91, 500);
    /*
    IntakeHelper::stuckCheckChange(false);
    IntakeHelper::blueExcld(false);
    IntakeHelper::sortState(false);
    IntakeHelper::StopAtColor(true);
    
    /*
    // first ring (MODDED)
    IntakeHelper::voltage(12);
    moveManual(125, -6); //-2, 300
    moveManual(700, -3); //700
    // moveManual(500, -5.5);

    // get second ring
    moveManual(400, 2); // 500
    liftIntake.overrideState(1);
    pros::delay(100); //60
    
    moveManual(200, -2);
    moveManual(340, -3.75);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3);
    */
    // first ring
    IntakeHelper::voltage(12);
    moveManual(300, -5); //-2
    moveManual(700, -2); //700
    // moveManual(500, -5.5);

    // get second ring
    moveManual(400, 2); // 500
    liftIntake.overrideState(1);
    pros::delay(100); //60
    
    moveManual(200, -2);
    moveManual(340, -3.75);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3);

    // IntakeHelper::voltage(12);

    
    
    // place goal in corner
    cornerDeploy.overrideState(1);
    pros::delay(500);
    IntakeHelper::voltage(0);
    moveManual(400, -7);
    robot.turn_with_pid(106.4, 1250); //1250
    moClamp.overrideState(0);
    moveManual(600, 6);

    // get out of corner
    moveManual(300, -6);

    // get to mid
    robot.turn_with_pid(207.43, 650); //750 -207.43
    /*
    moveManual(400, -10);
    // robot.move_to_point({(au::inches)(26.79), (au::inches)(-29.20)}, false, INTAKE_SIDE, 650, 2.9);
    robot.turn_with_pid(385.87, 800); //-385.87
    cornerDeploy.overrideState(0);
    /**/

}

void Routes::placehold12Mir() {
    IntakeHelper::sortState(true);
    IntakeHelper::blueExcld(false);
    // LiftMngr::setLevel(230);
    MogoUtils::getMogo(6.5, 4); //8

    // get ring @ alliance stack
    IntakeHelper::voltage(12);
    liftIntake.overrideState(1);
    // robot.turn_to_point({(au::inches)(2.99), (au::inches)(8.78)}, INTAKE_SIDE, 600);
    robot.move_to_point({(au::inches)(4.55), (au::inches)(7.63)}, true, INTAKE_SIDE, 600, 2.9);
    liftIntake.overrideState(0);
    moveManual(300, 7);
    pros::delay(80);

    // turn twds other ring
    // robot.turn_to_point({(au::inches)(27.03), (au::inches)(-10.73)}, INTAKE_SIDE, 800);
    robot.move_to_point({(au::inches)(27.63), (au::inches)(-10.97)}, true, INTAKE_SIDE, 800, 2.9);


    // go twds corner
    robot.turn_with_pid(76.22, 450);
    cornerDeploy.overrideState(1);
    moveManual(350, -6);
    pros::delay(60);
    robot.turn_with_pid(0, 250);
    cornerDeploy.overrideState(0); 

    // get corner
    // robot.turn_to_point({(au::inches)(10.42), (au::inches)(-29.71)}, INTAKE_SIDE, 650);
    robot.move_to_point({(au::inches)(10.42), (au::inches)(-29.71)}, true, INTAKE_SIDE, 650, 2.9);
    robot.turn_with_pid(68.21, 500);
    IntakeHelper::stuckCheckChange(false);
    IntakeHelper::blueExcld(true);
    IntakeHelper::sortState(false);
    IntakeHelper::StopAtColor(true);
    

    // first ring
    IntakeHelper::voltage(12);
    moveManual(300, -3);
    moveManual(700, -3); //700
    // moveManual(500, -5.5);

    // get second ring
    moveManual(400, 2); // 500
    liftIntake.overrideState(1);
    // pros::delay(100); //60
    
    moveManual(200, -2);
    moveManual(340, -3.75);
    liftIntake.overrideState(0);
    moveManual(200, -4);
    moveManual(540, 3);

    IntakeHelper::voltage(12);

    
    // place goal in corner
    cornerDeploy.overrideState(1);
    pros::delay(500);
    IntakeHelper::voltage(0);
    moveManual(400, -7);
    robot.turn_with_pid(255.45, 1250);
    moClamp.overrideState(0);
    moveManual(600, 6);

    // get out of corner
    moveManual(300, -6);

    // get to mid
    robot.turn_with_pid(207.43, 650); //750
    moveManual(400, -10);
    // robot.move_to_point({(au::inches)(26.79), (au::inches)(-29.20)}, false, INTAKE_SIDE, 650, 2.9);
    robot.turn_with_pid(385.87, 800);
    cornerDeploy.overrideState(0);

    /**/
    
}

void Routes::placehold13() {
    // Setup
    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(true);
    LiftMngr::setLevel(ABOVE_IN_HEIGHT-20);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(-3);

    fIntVolt = 12;
    fIntDelay = 300; //1350
    IntakeHelper::stuckCheckChange(true);
    IntakeHelper::StopAtColor(true);

    // Rush mid
    moveManual(50, -8); // -5
    moveManual(50, -11); // -9
    moveManual(550, -12); //500ms // SPEED 10.5
    moveManual(50, -5.5); //50 // SPEED
    moveManual(30, 3); // SPEED 0
    cornerDeploy.overrideState(0);
    moveManual(30, 3); // SPEED 0
    

    // Pull back
    moveManual(60, 6); //3
    moveManual(60, 7, 5);
    moveManual(450, 11.9, 10); //SPEED 600
    // pull back
    // robot.move_to_point({(au::inches)(-12.7), (au::inches)(-0)}, false, MOGO_SIDE, 0, 2.9);
    cornerDeploy.overrideState(1);

    // get other mogo
    pros::delay(200);
    robot.turn_with_pid(168.78, 650);
    cornerDeploy.overrideState(0);
    robot.turn_with_pid(98.05, 550); 
    MogoUtils::getMogo(6, 3, 9);
    
    IntakeHelper::StopAtColor(false);

    IntakeHelper::voltage(12);

    // drop & get other mogo
    // robot.turn_to_point({(au::inches)(-12.88), (au::inches)(28.09)}, MOGO_SIDE, 600);
    robot.move_to_point({(au::inches)(-15.52), (au::inches)(20.94)}, true, MOGO_SIDE, 550);
    moClamp.overrideState(0);
    pros::delay(100);
    IntakeHelper::voltage(0);
    moveManual(150, -7, -7);
    pros::delay(30);
    robot.turn_with_pid(197.04, 700);
    MogoUtils::getMogo(5, 3, 6, 620);
    IntakeHelper::voltage(12);

    // get corner ring
    // robot.turn_to_point({(au::inches)(1.01), (au::inches)(-15.88)}, INTAKE_SIDE, 700);
    robot.move_to_point({(au::inches)(1.01), (au::inches)(-15.88)}, true, INTAKE_SIDE, 700);
    robot.turn_with_pid(106.5, 400);

    IntakeHelper::stuckCheckChange(false);
    moveManual(450, -7); // 6.4
    moveManual(900, -3.5); //350
    robot.restOdomKeepAngle(0, 0);
    moveManual(50, -4);

    // get nu stake
    robot.move_to_point({(au::inches)(-4.54), (au::inches)(43.27)}, true, MOGO_SIDE, 300, 2);
    IntakeHelper::stuckCheckChange(true);
    // 21.22
    // robot.turn_to_point({(au::inches)(-44.52), (au::inches)(22.82)}, INTAKE_SIDE, 600);
    IntakeHelper::voltage(0);
    robot.move_to_point({(au::inches)(-47.69), (au::inches)(21.4)}, true, INTAKE_SIDE, 600, 2.8); // OLD: {(au::inches)(-45.84), (au::inches)(17.33)}
    LiftMngr::setLevel(165);
    pros::delay(400);

    /**/

}

void Routes::placehold13Mir() {
    // Setup
    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(false);
    IntakeHelper::stuckCheckChange(true);
    LiftMngr::setLevel(ABOVE_IN_HEIGHT - 20);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(-3);

    fIntVolt = 12;
    fIntDelay = 300; //1350
    IntakeHelper::StopAtColor(true);
    
    
    // Rush mid
    moveManual(50, -8); // -5
    moveManual(50, -11); // -9
    moveManual(550, -12); //540ms // SPEED 10.5
    moveManual(55, -5.5); //50 // SPEED
    moveManual(30, 3); // SPEED 0
    cornerDeploy.overrideState(0);
    moveManual(30, 3); // SPEED 0
    

    // Pull back
    moveManual(60, 6); //3
    moveManual(60, 7, 5);
    moveManual(550, 11.9, 6); //SPEED 500 8
    
    // moveManual(90, 9, 5);

    // get mogo
    cornerDeploy.overrideState(1);
    pros::delay(250); //250
    robot.turn_with_pid(-144.21, 100);
    // cornerDeploy.overrideState(0);
    robot.turn_with_pid(-231.73, 400);
    cornerDeploy.overrideState(0);
    robot.turn_with_pid(-127.39, 720); //670
    moveManual(200, 8, 8);
    // robot.move_to_point({(au::inches)(-22.94), (au::inches)(-17.52)}, false, MOGO_SIDE, 0, 2.9);
    MogoUtils::getMogo(6, 3, 8);


    // drop mogo
    IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(-39.65), (au::inches)(-27.82)}, true, MOGO_SIDE, 600, 2.9);
    // pros::delay(70);
    // moveManual(180, 5);
    IntakeHelper::voltage(12);
    

    // get rushed mogo
    moClamp.overrideState(0);
    robot.turn_to_point({(au::inches)(-25.9), (au::inches)(-22.94)}, INTAKE_SIDE, 300);
    robot.move_to_point({(au::inches)(-25.9), (au::inches)(-22.94)}, false, INTAKE_SIDE, 300, 2.9);
    robot.turn_with_pid(-259.24, 500);
    IntakeHelper::voltage(-12);
    // moveManual(200, 8, 8); //NEW
    MogoUtils::getMogo(5, 2, 7); //600
    
    // get corner
    // robot.turn_to_point({(au::inches)(6.11), (au::inches)(-4.11)}, INTAKE_SIDE, 500);
    robot.move_to_point({(au::inches)(6.11), (au::inches)(-4.11)}, true, INTAKE_SIDE, 500, 2.8);
    robot.turn_with_pid(-155.25, 500); //-161.06
    IntakeHelper::stuckCheckChange(false);
    IntakeHelper::voltage(12);

    moveManual(450, -7); // 600 6.4
    moveManual(700, -3.5); //900
    robot.restOdomKeepAngle(0, 0);
    moveManual(50, -4);
    // moveManual(150, 4);
    
    
    // get nu wall stake
    robot.move_to_point({(au::inches)(-31.78), (au::inches)(-11.31)}, true, MOGO_SIDE, 580, 2.5);
    IntakeHelper::stuckCheckChange(true);
    // robot.turn_to_point({(au::inches)(-32.86), (au::inches)(11.77)}, INTAKE_SIDE, 590); //OLD
    // robot.move_to_point({(au::inches)(-32.86), (au::inches)(11.77)}, false, INTAKE_SIDE, 740, 2.8); //OLD
    // {(au::inches)(-44.38), (au::inches)(19.67)} OLD PT
    robot.turn_to_point({(au::inches)(-47.69), (au::inches)(15.44)}, INTAKE_SIDE, 650);
    // LiftMngr::setLevel(SCORE_HEIGHT+30);
    robot.move_to_point({(au::inches)(-47.69), (au::inches)(15.44)}, false, INTAKE_SIDE, 700, 2.8);
    LiftMngr::setLevel(SCORE_HEIGHT);
    robot.turn_with_pid(-67.92, 400);

    // put rings on
    /**/
}