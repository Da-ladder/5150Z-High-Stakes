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

#define STRAIGHT_ARM 90
#define IDLE_ARM 290
#define SCORE_HEIGHT 124.45
#define STORE_HEIGHT 255
#define ABOVE_IN_HEIGHT 222



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

void intakeDelay() {
    while (true) {
        if (fIntDelay != 0) {
            pros::delay(fIntDelay);
            IntakeHelper::voltage(fIntVolt);
            fIntDelay = 0;
            fIntVolt = 0;
        } else {}
        pros::delay(20);
    }
}

int fLbLift = 0;
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

void Routes::skills() {
    pros::Task intakeThing(intakeDelay);
    // score on stake
    IntakeHelper::blueExcld(true);
    int time = 0;

    while(340 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    // back off and turn twds mogo
    robot.move_to_point({(au::inches)(5.5), (au::inches)(0.0)}, false, MOGO_SIDE);
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(90, 600);

    // grab mogo
    MogoUtils::getMogo(6, 2);

    // move to and grab rings
    pros::delay(200);
    IntakeHelper::voltage(12);
    // robot.move_to_point({(au::inches)(24.82), (au::inches)(20.68)}, true, INTAKE_SIDE, 600); // PREV ring get
    robot.turn_to_point({(au::inches)(24.82), (au::inches)(20.68)}, INTAKE_SIDE, 600);
    RedRingUtil::getRing(true, 6, 2);

    // move to ring & store it in lb
    robot.move_to_point({(au::inches)(50.39), (au::inches)(40.2)}, true, INTAKE_SIDE, 600);
    // robot.turn_to_point({(au::inches)(57.22), (au::inches)(38.85)}, INTAKE_SIDE, 600); //PREV
    LiftMngr::setLevel(STORE_HEIGHT);
    // robot.move_to_point({(au::inches)(73.73), (au::inches)(44.7)}, true, INTAKE_SIDE, 400, 3); // PREV ring get
    robot.turn_to_point({(au::inches)(73.73), (au::inches)(44.7)}, INTAKE_SIDE, 500);
    RedRingUtil::getRing(true, 6, 2);

    // Wall stake & score while intaking ring
    robot.move_to_point({(au::inches)(54.87), (au::inches)(44.35)}, true, MOGO_SIDE, 700, 2.6);
    IntakeHelper::voltage(0);
    LiftMngr::setLevel(ABOVE_IN_HEIGHT);
    robot.turn_with_pid(271.4, 600);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(54.22), (au::inches)(58.44)}, false, INTAKE_SIDE, 400);
    moveManual(440, -5.5);
    LiftMngr::setLevel(SCORE_HEIGHT);
    pros::delay(200);

    // back up
    robot.move_to_point({(au::inches)(50.66), (au::inches)(46.96)}, true, MOGO_SIDE, 400);

    // get ring
    LiftMngr::setLevel(IDLE_ARM);
    // robot.turn_to_point({(au::inches)(34.54), (au::inches)(49.64)}, INTAKE_SIDE, 700);
    robot.turn_with_pid(359.91, 600);
    // robot.move_to_point({(au::inches)(34.54), (au::inches)(49.64)}, false, INTAKE_SIDE, 600);
    RedRingUtil::getRing(true, 8, 2);
    pros::delay(100);

    // get ring
    robot.turn_with_pid(361.21, 450);
    robot.move_to_point({(au::inches)(11.35), (au::inches)(49.48)}, false, INTAKE_SIDE, 400);
    pros::delay(150);

    // get ring
    RedRingUtil::getRing(true, 8, 2);
    pros::delay(100);

    // get ring
    robot.move_to_point({(au::inches)(4.46), (au::inches)(57.7)}, true, INTAKE_SIDE, 700);
    pros::delay(100);

    // back into corner
    robot.turn_with_pid(139.88, 600);
    moveManual(300, 7);
    moveManual(300, 4);
    moClamp.overrideState(0);

    // get other goal 
    robot.move_to_point({(au::inches)(5.27), (au::inches)(57.81)}, true, INTAKE_SIDE, 300);
    robot.move_to_point({(au::inches)(1.44), (au::inches)(-0.08)}, true, MOGO_SIDE, 900, 3);
    MogoUtils::getMogo(6, 2);

    // get ring
    robot.turn_to_point({(au::inches)(23.35), (au::inches)(-19.44)}, INTAKE_SIDE, 600);
    RedRingUtil::getRing(true, 8, 2);

    // get ring
    robot.move_to_point({(au::inches)(46), (au::inches)(-35.19)}, true, INTAKE_SIDE, 600);
    robot.turn_to_point({(au::inches)(72.11), (au::inches)(-41.92)}, INTAKE_SIDE, 600);
    LiftMngr::setLevel(STORE_HEIGHT);
    RedRingUtil::getRing(true, 6, 2);

    // move back & 2nd wall stake 49.34, -36.24
    robot.move_to_point({(au::inches)(49.34), (au::inches)(-36.24)}, true, MOGO_SIDE, 600, 2.5);
    IntakeHelper::voltage(0);
    LiftMngr::setLevel(ABOVE_IN_HEIGHT);
    robot.turn_with_pid(92.8, 700);
    IntakeHelper::voltage(12);
    // RedRingUtil::getRing(true, 6, 2);
    robot.move_to_point({(au::inches)(51.86), (au::inches)(-51.85)}, false, INTAKE_SIDE, 700);
    moveManual(400, -10);
    moveManual(200, -6);
    LiftMngr::setLevel(SCORE_HEIGHT);
    pros::delay(200);

    // back off
    robot.move_to_point({(au::inches)(51.83), (au::inches)(-41.81)}, true, MOGO_SIDE, 400);
    LiftMngr::setLevel(IDLE_ARM);

    // get ring
    robot.turn_with_pid(0.97, 600);
    RedRingUtil::getRing(true, 8, 2);
    pros::delay(100);

    // get ring
    robot.move_to_point({(au::inches)(12.32), (au::inches)(-42.46)}, true, INTAKE_SIDE, 400);
    pros::delay(100);

    // get ring
    robot.move_to_point({(au::inches)(-4.7), (au::inches)(-43.92)}, false, INTAKE_SIDE, 400);
    // RedRingUtil::getRing(true, 8, 2);
    pros::delay(100);

    // get ring
    robot.move_to_point({(au::inches)(2.88), (au::inches)(-50.98)}, true, INTAKE_SIDE, 700);

    // put goal in
    robot.turn_with_pid(209.11, 600);
    moveManual(300, 7);
    moveManual(300, 4);
    moClamp.overrideState(0);

    // back off 
    robot.move_to_point({(au::inches)(2.88), (au::inches)(-49.68)}, true, INTAKE_SIDE, 400);
    robot.move_to_point({(au::inches)(56.18), (au::inches)(-39.95)}, true, INTAKE_SIDE, 600);

    // get ring
    robot.turn_to_point({(au::inches)(70.7), (au::inches)(-21.64)}, INTAKE_SIDE, 600);
    RedRingUtil::getRing(true, 8, 2);
    fIntVolt = 0;
    fIntDelay = 140;


    // get mogo 3rd 45.74
    robot.turn_with_pid(45.74, 800);
    // robot.turn_to_point({(au::inches)(77.85), (au::inches)(-10.39)}, MOGO_SIDE, 1000);
    // robot.move_to_point({(au::inches)(77.85), (au::inches)(-10.39)}, true, MOGO_SIDE, 1000, 3);
    // robot.move_to_point({(au::inches)(82.81), (au::inches)(-15.22)}, true, MOGO_SIDE, 1300, 2.8); OLD 
    MogoUtils::getMogo(5, 2);
    IntakeHelper::voltage(12);

    // get rings & corner
    robot.move_to_point({(au::inches)(68.38), (au::inches)(-43)}, true, INTAKE_SIDE, 500);

    // get ring
    robot.turn_with_pid(180.65, 800);
    robot.move_to_point({(au::inches)(86.25), (au::inches)(-42.03)}, false, INTAKE_SIDE, 500);

    // back off
    robot.move_to_point({(au::inches)(75.23), (au::inches)(-43.73)}, false, MOGO_SIDE, 500);

    // get ring 
    robot.turn_with_pid(148.55, 450);
    cornerDeploy.overrideState(1);
    moveManual(400, -8);
    moveManual(500, -7);
    fIntVolt = -12;
    fIntDelay = 650;

    // clear corner & drop
    robot.turn_with_pid(327.98, 1300, 11);
    cornerDeploy.overrideState(0);
    moveManual(300, 7);
    moveManual(300, 4);
    moClamp.overrideState(0);

    // back up
    robot.move_to_point({(au::inches)(78.23), (au::inches)(-33.39)}, true, INTAKE_SIDE, 400);

    // get last mogo
    robot.move_to_point({(au::inches)(104.23), (au::inches)(7.11)}, true, MOGO_SIDE, 800, 3);
    MogoUtils::getMogo(6, 2);
    IntakeHelper::voltage(12);

    // clear corner & place
    robot.turn_with_pid(256.09, 1300);
    cornerDeploy.overrideState(1);
    moveManual(400, -8);
    moveManual(500, -7);
    robot.turn_with_pid(404.43, 1300);

    // place
    cornerDeploy.overrideState(0);
    moClamp.overrideState(0);
    moveManual(300, 7);
    moveManual(300, 4);
    
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
    moveManual(120, 6, 6); // 2nd RING BACK 120
    moveManual(400, 9, 3); // 2nd RING BACK
    moveManual(260, 5, 5);

    // get ring
    robot.turn_with_pid(-140.99, 600); //800
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(47.26), (au::inches)(-12.78)}, false, INTAKE_SIDE, 0, 2.8); //2.5

    // get corner
    robot.turn_to_point({(au::inches)(43.63), (au::inches)(17.62)},  INTAKE_SIDE, 700);
    robot.move_to_point({(au::inches)(43.63), (au::inches)(17.62)}, true, INTAKE_SIDE, 700, 3); //900
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
    // moveManual(20, 6, 6); // 2nd RING BACK 140
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


    /*
    // get ring @ center
    robot.turn_with_pid(-32.33, 800);
    robot.move_to_point({(au::inches)(16.57), (au::inches)(2.05)}, false, INTAKE_SIDE, 900); //2.7

    // -87.19

    // ladder
    robot.turn_with_pid(-87.19,700);
    moveManual(200, -8);
    moveManual(400, -5);
    /*
    liftIntake.overrideState(1);
    pros::delay(100);
    moveManual(300, -6);
    moveManual(200, 6);

    /*
    moveManual(300, -6.5, -8);
    moveManual(160, -6);
    
    // robot.move_to_point({(au::inches)(42.7), (au::inches)(-23.74)}, false, MOGO_SIDE, 0);
    /**/
}

void Routes::placehold1() {
    IntakeHelper::blueExcld(true);
    int time = 0;

    // LiftMngr::setLevel(150);
    
    while(300 >= time*25) {
        LiftMngr::setVoltage(-12, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setLevel(130);
    LiftMngr::setVoltage(0, false);

    // back off and get goal
    robot.move_to_point({(au::inches)(-4.86), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(240);
    robot.turn_with_pid(48.56, 600);
    // robot.turn_to_point({(au::inches)(17.6), (au::inches)(13.28)}, MOGO_SIDE, 600);
    // robot.move_to_point({(au::inches)(18.19), (au::inches)(15.13)}, false, MOGO_SIDE, 600);
    // moveManual(500, 8);
    robot.move_to_point({(au::inches)(26.38), (au::inches)(24.93)}, false, MOGO_SIDE, 0, 2);
    moClamp.overrideState(1);
    IntakeHelper::voltage(12);
    pros::delay(100);

    // get mid ring
    robot.move_to_point({(au::inches)(52.23), (au::inches)(30.39)}, true, INTAKE_SIDE, 1400, 3, 9);
    robot.turn_with_pid(147, 650);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(64.67), (au::inches)(23.78)}, false, INTAKE_SIDE, 900);

    // curve back
    moveManual(500, 4.2, 8);
    moveManual(500, 6, 6);

    // get ring
    robot.turn_with_pid(136.76, 550);
    robot.move_to_point({(au::inches)(41.98), (au::inches)(17.26)}, false, INTAKE_SIDE, 900);

    // go to corner
    robot.turn_with_pid(89.97, 550);
    robot.move_to_point({(au::inches)(40.36), (au::inches)(-24.85)}, false, INTAKE_SIDE, 900);
    pros::delay(200);

    // back off
    robot.move_to_point({(au::inches)(45.92), (au::inches)(-9.1)}, false, MOGO_SIDE, 900);

    // get ring
    robot.turn_with_pid(-28.93, 850);
    robot.move_to_point({(au::inches)(10.88), (au::inches)(10.60)}, false, INTAKE_SIDE, 900);
    liftIntake.overrideState(1);
    moveManual(380, -3, -6.5);
    liftIntake.overrideState(0);
    pros::delay(200);
    moveManual(400, 4);

    // moveManual(600, 2);










    //





    /**/

}

void Routes::placehold1Mir() {

}

void Routes::placehold2() {
    IntakeHelper::blueExcld(false);
    int time = 0;
    while(300 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);
    LiftMngr::setLevel(STRAIGHT_ARM);


    robot.move_to_point({(au::inches)(5.81), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(-42.19, 700);
    // robot.move_to_point({(au::inches)(21.19), (au::inches)(-13.29)}, false, MOGO_SIDE, 0, 2.6);
    MogoUtils::getMogo(5, 4.5, 8);
    IntakeHelper::voltage(12);

    // get mid 
    // robot.move_to_point({(au::inches)(53.19), (au::inches)(-32.42)}, true, INTAKE_SIDE, 800, 2.5); //OLD

    // NEW
    robot.move_to_point({(au::inches)(56.75), (au::inches)(-30.86)}, true, INTAKE_SIDE, 1000, 2.5); //49.48, -28.71
    robot.turn_with_pid(-150, 600);
    // robot.changeDetectLine(true);
    robot.move_to_point({(au::inches)(67.99), (au::inches)(-22.90)}, false, INTAKE_SIDE, 600, 2.5); // 62.13, -26.77
    // robot.changeDetectLine(false);
    // NEW
    // robot.move_to_point({(au::inches)(63.50), (au::inches)(-27.21)}, true, INTAKE_SIDE, 600, 2.5); //OLD
    // pros::delay(200);

    // curve back
    moveManual(100, 6);
    moveManual(500, 8, 4);
    // IntakeHelper::StopAtColor(true);
    moveManual(500, 6, 6);
    

    // get ring
    robot.turn_with_pid(-157.27, 900, 10);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(22.40), (au::inches)(-10.99)}, false, INTAKE_SIDE, 900); //OLD
    // ring get!

    // get corner + back off
    robot.turn_with_pid(-86.94, 900);
    robot.move_to_point({(au::inches)(17.62), (au::inches)(8.29)}, false, INTAKE_SIDE, 900, 2.4);
    pros::delay(70);
    robot.move_to_point({(au::inches)(53.67), (au::inches)(12.29)}, false, MOGO_SIDE, 0);

    // turn + get ring @ alliance stake
    robot.turn_with_pid(32.17, 1200);
    robot.move_to_point({(au::inches)(19.49), (au::inches)(-8.68)}, false, INTAKE_SIDE, 900);

    // get ring
    liftIntake.overrideState(1);
    // robot.turn_with_pid(-6.05, 450);

    
    moveManual(330, -5, -8);
    liftIntake.overrideState(0);
    pros::delay(230);
    moveManual(360, 7);
    pros::delay(900);
    IntakeHelper::voltage(0);
    /**/
}

void Routes::placehold2Mir() {
    IntakeHelper::blueExcld(true);
    int time = 0;

    // LiftMngr::setLevel(150);
    
    while(300 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    robot.move_to_point({(au::inches)(27.12), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(91.92, 900); //.9k
    
    MogoUtils::getMogo(5, 4, 7);
    IntakeHelper::voltage(12);

    // get mid 
    // robot.turn_to_point({(au::inches)(48.04), (au::inches)(33.21)}, INTAKE_SIDE, 1100);
    robot.move_to_point({(au::inches)(47.28), (au::inches)(33.17)}, true, INTAKE_SIDE, 1100, 2.5);
    robot.turn_with_pid(144.52, 800);
    // robot.changeDetectLine(true);
    robot.move_to_point({(au::inches)(62.15), (au::inches)(19.87)}, false, INTAKE_SIDE, 700, 2.5);
    // robot.changeDetectLine(false);


    // curve back
    moveManual(220, 6, 6); // 2nd RING BACK 120
    moveManual(400, 3, 9); // 2nd RING BACK
    moveManual(200, 6, 6);

    // get ring
    robot.turn_with_pid(120.46, 900);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(44.46), (au::inches)(12.25)}, false, INTAKE_SIDE, 700, 2.5);


    // get corner + back off
    robot.turn_with_pid(83.37, 900);
    robot.move_to_point({(au::inches)(39.27), (au::inches)(-29.39)}, false, INTAKE_SIDE, 900, 2.0); //2.7
    moveManual(70, -6);
    robot.move_to_point({(au::inches)(41.37), (au::inches)(-16.83)}, false, MOGO_SIDE, 0);


    // turn + get ring @ alliance stake
    robot.turn_with_pid(-34.48, 1200);
    robot.move_to_point({(au::inches)(10.66), (au::inches)(3.75)}, false, INTAKE_SIDE, 900);

    // get ring
    liftIntake.overrideState(1);
    // robot.turn_with_pid(-6.05, 450);

    
    moveManual(330, -6);
    liftIntake.overrideState(0);
    pros::delay(230);
    moveManual(360, 7);



    // RedRingUtil::getRing(true, 10, 4); 
    // moveManual(150, -3);


    /*
    // get allaince stake ring
    robot.move_to_point({(au::inches)(11.82), (au::inches)(17.76)}, true, INTAKE_SIDE, 800, 2.5);
    // robot.turn_with_pid(-9.98, 400);
    liftIntake.overrideState(1);


    moveManual(350, -6);
    liftIntake.overrideState(0);
    // IntakeHelper::voltage(12);
    pros::delay(175);
    IntakeHelper::voltage(12);
    moveManual(400, 6);



    // BACK OFF
    robot.turn_with_pid(-27.90, 900);
    /*
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    moveManual(500, -5);
    moveManual(500, -3);


    // touch ladder cone
    // robot.turn_with_pid(-84.12, 800, 10);
    // moveManual(700, -5);
    // IntakeHelper::voltage(0);



    // moveManual(200, 3); 
    // robot.move_to_point({(au::inches)(57.73), (au::inches)(8.65)}, false, INTAKE_SIDE, 900);

    // grab ring from corner
    /*
    robot.move_to_point({(au::inches)(42.66), (au::inches)(-15.31)}, true, INTAKE_SIDE, 900, 2.8);
    pros::delay(50);
    robot.turn_with_pid(75.99, 700);
    moveManual(300, -5); //600
    moveManual(750, -12); //700
    moveManual(350, 5); //700
    // pros::delay(1300);
    

    // back off
    // robot.move_to_point({(au::inches)(43.86), (au::inches)(-1.70)}, true, MOGO_SIDE, 500, 2);
    /*
    robot.move_to_point({(au::inches)(20.80), (au::inches)(-7.86)}, true, INTAKE_SIDE, 900);

    // to corner
    robot.move_to_point({(au::inches)(-28.01), (au::inches)(-36.63)}, true, INTAKE_SIDE, 900, 2.5);
    // moveManual(400, 6);


    // pros::delay(100);
    /**/
}

void Routes::placehold3() {
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

    // get mid 
    // robot.turn_to_point({(au::inches)(47.24), (au::inches)(-25.99)}, INTAKE_SIDE, 900);
    robot.move_to_point({(au::inches)(52.58), (au::inches)(-24.24)}, true, INTAKE_SIDE, 750, 2.9); //900
    robot.turn_with_pid(-144.52, 700); //800
    // robot.changeDetectLine(true);
    robot.move_to_point({(au::inches)(60.04), (au::inches)(-19.26)}, false, INTAKE_SIDE, 0, 3);
    // robot.changeDetectLine(false);


    // curve back
    moveManual(210, 6, 6); // 2nd RING BACK 120
    moveManual(400, 9, 3); // 2nd RING BACK
    moveManual(260, 5, 5);

    // get ring
    robot.turn_with_pid(-140.99, 600); //800
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(47.55), (au::inches)(-6.18)}, false, INTAKE_SIDE, 0, 2.8); //2.5

    // get corner
    // robot.turn_to_point({(au::inches)(44.2), (au::inches)(24.88)},  INTAKE_SIDE, 700);
    robot.move_to_point({(au::inches)(43.63), (au::inches)(17.62)}, true, INTAKE_SIDE, 700, 3); //900
    moveManual(200, -7); // 160, -6
    moveManual(500, -4);

    // go to mid
    robot.move_to_point({(au::inches)(42.42), (au::inches)(21.31)}, true, MOGO_SIDE, 200, 3);
    robot.turn_with_pid(35.45, 800); //800

    // get ring @ alliance stack
    robot.move_to_point({(au::inches)(11.52), (au::inches)(-0.28)}, false, INTAKE_SIDE, 0);
    liftIntake.overrideState(1);
    IntakeHelper::voltage(12);
    pros::delay(50);
    moveManual(260, -8, -8);
    liftIntake.overrideState(0);
    pros::delay(200);
    moveManual(300, 6);
    moveManual(700, 0);
    IntakeHelper::voltage(0);
    /**/
}

void Routes::placehold3Mir() {
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
    robot.turn_with_pid(150.04, 700);
    // robot.changeDetectLine(true);
    //{(au::inches)(57.48), (au::inches)(25.41)}
    robot.move_to_point({(au::inches)(56.32), (au::inches)(27.31)}, false, INTAKE_SIDE, 700, 3); //3
    // robot.changeDetectLine(false);


    // curve back
    // moveManual(20, 6, 6); // 2nd RING BACK 140
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
    robot.move_to_point({(au::inches)(41.86), (au::inches)(-15.26)}, true, MOGO_SIDE, 200, 3);
    

    // get ring @ center
    robot.turn_with_pid(-32.23, 800);
    robot.move_to_point({(au::inches)(11.84), (au::inches)(1.43)}, false, INTAKE_SIDE, 0);
    
    
    // liftIntake.overrideState(1);
    // pros::delay(100);
    // moveManual(150, -4, -8);
    // moveManual(250, -7);
    // liftIntake.overrideState(0);
    // pros::delay(40);
    // moveManual(470, 6);
    
    liftIntake.overrideState(1);
    IntakeHelper::voltage(12);
    pros::delay(50);
    moveManual(260, -8, -8);
    liftIntake.overrideState(0);
    pros::delay(200);
    moveManual(300, 6);
    moveManual(700, 0);
    IntakeHelper::voltage(0);


    /*
    moveManual(300, -6.5, -8);
    moveManual(160, -6);
    
    // robot.move_to_point({(au::inches)(42.7), (au::inches)(-23.74)}, false, MOGO_SIDE, 0);
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
    // Mogo + preload ring
    // Stake + mogo
    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(false);
    LiftMngr::setMotorPwr(5);
    pros::delay(400);

    LiftMngr::setLevel(130);
    pros::delay(400);
    robot.move_to_point({(au::inches)(28.61), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(175);
    robot.turn_with_pid(-86.02, 800);
    MogoUtils::getMogo(10, 2);
    IntakeHelper::voltage(12);

    // get ring
    robot.turn_with_pid(-163.34, 700);
    robot.move_to_point({(au::inches)(45.14), (au::inches)(-14.00)}, false, INTAKE_SIDE, 700); // OLD
    // RedRingUtil::getRing(false, 6, 3);
    moveManual(160, -3);

    // get ring + drop + get mogo
    // STOP RING AT COLOR
    // IntakeHelper::voltage(0);
    // IntakeHelper::StopAtColor(true);
    fIntVolt = 0;
    fIntDelay = 125;
    // IntakeHelper::voltage(0);
    robot.turn_to_point({(au::inches)(17.33), (au::inches)(-9.03)}, INTAKE_SIDE, 900, 10);
    IntakeHelper::voltage(12);
    // IntakeHelper::voltage(0);
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    IntakeHelper::sortState(false); // STOPS SORT
    robot.move_to_point({(au::inches)(12.34), (au::inches)(-8.03)}, false, INTAKE_SIDE, 0, 2.8); //RE
    robot.turn_to_point({(au::inches)(-22.20), (au::inches)(-33.22)}, INTAKE_SIDE, 350);

    //knock down stack @ alliance stake
    robot.move_to_point({(au::inches)(2.65), (au::inches)(-15.30)}, false, INTAKE_SIDE, 0, 2.1); //RE

    // grab ring @ alliance stake
    // robot.move_to_point({(au::inches)(-12.91), (au::inches)(-25.57)}, false, INTAKE_SIDE, 0); //OLD
    // NEW
        robot.move_to_point({(au::inches)(-19.23), (au::inches)(-28.53)}, false, INTAKE_SIDE, 0, 2.9);
    // NEW


    
    // robot.move_to_point({(au::inches)(-22.20), (au::inches)(-33.22)}, true, INTAKE_SIDE, 300, 2.5); //OLD

    pros::delay(750);
    // robot.turn_with_pid(179.38, 1200, 6.5);
    moClamp.overrideState(0);
    pros::delay(150);
    robot.turn_with_pid(-54.30, 800);
    
    // -70.88
    // robot.move_to_point({(au::inches)(-10.77), (au::inches)(-33.39)}, true, INTAKE_SIDE, 500, 2.8, 8);
    // robot.turn_with_pid(268.51, 600); //RE


    // grab mogo + grab ring
    MogoUtils::getMogo(10, 2);
    // robot.turn_with_pid(409.74, 800); //RE
    robot.move_to_point({(au::inches)(-26.72), (au::inches)(-60.81)}, true, INTAKE_SIDE, 900, 3);
    // IntakeHelper::StopAtColor(true);


    // touch ladder
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    LiftMngr::setLevel(200);
    IntakeHelper::voltage(10);
    // fIntVolt = 0;
    // fIntDelay = 180;
    robot.turn_with_pid(188.02, 900, 9);
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(10);
    moveManual(400, -12);
    moveManual(400, -4);
    moveManual(600, -2);
    /**/
}

void Routes::placehold6Mir() {

    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(true);
    LiftMngr::setMotorPwr(5);
    pros::delay(400);

    LiftMngr::setLevel(130);
    pros::delay(400);
    robot.move_to_point({(au::inches)(28.61), (au::inches)(0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(175);
    robot.turn_with_pid(86.02, 800);
    MogoUtils::getMogo(10, 2);
    IntakeHelper::voltage(11);

    // get ring
    robot.move_to_point({(au::inches)(44.64), (au::inches)(13.82)}, true, INTAKE_SIDE, 700);
    fIntVolt = 0;
    fIntDelay = 150;

    // get ring + drop + get mogo
    // STOP RING AT COLOR
    // IntakeHelper::StopAtColor(true);
    robot.turn_to_point({(au::inches)(14.94), (au::inches)(5.47)}, INTAKE_SIDE, 900);
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    IntakeHelper::sortState(false); // STOPS SORT
    robot.move_to_point({(au::inches)(13.17), (au::inches)(6.36)}, false, INTAKE_SIDE, 0, 2.9); //RE
    

    // get mid ring stack @ stake
    robot.turn_with_pid(-30.21, 500);
    robot.move_to_point({(au::inches)(0.95), (au::inches)(14.62)}, false, INTAKE_SIDE, 300, 2.1);
    robot.move_to_point({(au::inches)(-29.36), (au::inches)(30.55)}, false, INTAKE_SIDE, 300, 3);


    // robot.move_to_point({(au::inches)(-19.83), (au::inches)(25.66)}, true, INTAKE_SIDE, 300, 2.7); //RE
    pros::delay(950);
    // robot.turn_with_pid(74.99, 1200, 6.5);
    moClamp.overrideState(0);
    pros::delay(100);
    robot.turn_with_pid(49.43, 800); //76.45
    // -45.10 49.43
    // robot.move_to_point({(au::inches)(-10.77), (au::inches)(-33.39)}, true, INTAKE_SIDE, 500, 2.8, 8);
    // robot.turn_with_pid(268.51, 600); //RE


    // grab mogo + grab ring
    MogoUtils::getMogo(10, 2);
    // robot.turn_with_pid(409.74, 800); //RE
    // robot.turn_to_point({(au::inches)(-26.85), (au::inches)(62.07)}, INTAKE_SIDE, 900);
    robot.turn_with_pid(-44.02, 800);

    // robot.move_to_point({(au::inches)(-26.85), (au::inches)(62.07)}, true, INTAKE_SIDE, 900, 3);
    RedRingUtil::getRing(true, 10, 4);
    moveManual(150, -3);


    // touch ladder
    robot.chassis.left_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    robot.chassis.right_motors.raw.set_brake_mode_all(pros::MotorBrake::coast);
    LiftMngr::setLevel(180);
    IntakeHelper::voltage(10);
    robot.turn_with_pid(-193.74, 900, 9);
    // IntakeHelper::StopAtColor(false);
    moveManual(400, -12);
    moveManual(400, -4);
    moveManual(600, -2);
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
    LiftMngr::setLevel(175);

    // get goal
    MogoUtils::getMogo(11, 2);

    // mid rings
    robot.turn_with_pid(116.00, 900);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(36.13), (au::inches)(-22.37)}, false, INTAKE_SIDE, 0, 2.5);
    robot.turn_with_pid(59.20, 600);
    robot.move_to_point({(au::inches)(26.71), (au::inches)(-34.48)}, false, INTAKE_SIDE, 600, 2.5, 9);

    // IntakeHelper::StopAtColor(true);
    moveManual(500, 4, 8);
    moveManual(300, 8, 8);
    // pros::delay(125);
    

    // get ring
    robot.turn_to_point({(au::inches)(23.25), (au::inches)(-18.24)}, INTAKE_SIDE, 800);
    /*
    // IntakeHelper::StopAtColor(false);
    // RedRingUtil::getRing(true, 7, 3);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(23.25), (au::inches)(-18.24)}, false, INTAKE_SIDE, 800, 2.5, 10);

    // go to corner
    robot.move_to_point({(au::inches)(-14.97), (au::inches)(-14.66)}, true, INTAKE_SIDE, 800, 2.7);
    moveManual(800, -6);
    pros::delay(100);
    IntakeHelper::voltage(10);
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
    MogoUtils::getMogo(8, 4);

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
    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(true);
    // LiftMngr::setLevel(175);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(12);

    fIntVolt = 0;
    fIntDelay = 1500; //1350

    // Rush mid
    robot.move_to_point({(au::inches)(-35.3), (au::inches)(-0)}, false, INTAKE_SIDE, 0);
    cornerDeploy.overrideState(0);

    // pull back
    robot.move_to_point({(au::inches)(-12.7), (au::inches)(-0)}, false, MOGO_SIDE, 0, 2.9);
    cornerDeploy.overrideState(1);

    // get other mogo
    robot.turn_with_pid(122.35, 250);
    cornerDeploy.overrideState(0);
    robot.turn_with_pid(122.35, 750);
    MogoUtils::getMogo(6, 3);
    IntakeHelper::voltage(12);

    // drop & get other mogo
    robot.move_to_point({(au::inches)(-22.55), (au::inches)(18.25)}, false, MOGO_SIDE, 0);
    pros::delay(650);
    moClamp.overrideState(0);
    IntakeHelper::voltage(0);
    robot.move_to_point({(au::inches)(-33.38), (au::inches)(-0.23)}, false, INTAKE_SIDE, 500);
    robot.turn_with_pid(180.25, 900);
    MogoUtils::getMogo(5, 3, 8, 520);
    IntakeHelper::voltage(12);

    // get ring
    robot.move_to_point({(au::inches)(5.62), (au::inches)(7.77)}, true, INTAKE_SIDE, 600);

    // drop @ opt place
    robot.move_to_point({(au::inches)(-25.32), (au::inches)(2.62)}, false, MOGO_SIDE, 500, 3);
    robot.turn_with_pid(260.84, 900);
    robot.move_to_point({(au::inches)(-32.32), (au::inches)(-18.02)}, false, MOGO_SIDE, 500, 3);
    moClamp.overrideState(0);
    IntakeHelper::voltage(0);

    // goal
    robot.move_to_point({(au::inches)(-30.3), (au::inches)(-3.95)}, false, INTAKE_SIDE, 500, 3);
    robot.turn_with_pid(429.97, 900);
    MogoUtils::getMogo(6, 3);

    // guard center
    robot.turn_with_pid(422.93, 700);

    /**/

}

void Routes::placehold13Mir() {
    // Setup
    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(false);
    // LiftMngr::setLevel(175);
    cornerDeploy.overrideState(1);
    IntakeHelper::voltage(12);

    fIntVolt = 0;
    fIntDelay = 1200; //1150
    

    // Rush mid
    robot.move_to_point({(au::inches)(-35.5), (au::inches)(-0)}, false, INTAKE_SIDE, 0);

    // pull back
    cornerDeploy.overrideState(0);
    robot.move_to_point({(au::inches)(-24), (au::inches)(-0)}, false, MOGO_SIDE, 0, 2.9);

    // get mogo
    cornerDeploy.overrideState(1);
    pros::delay(100);
    robot.turn_with_pid(-123.12, 300);
    cornerDeploy.overrideState(0);
    robot.turn_with_pid(-123.12, 300);
    MogoUtils::getMogo(6, 3);
    IntakeHelper::voltage(12);
    pros::delay(700);
    moClamp.overrideState(0);

    // get other mogo
    robot.move_to_point({(au::inches)(-31.23), (au::inches)(-10.93)}, false, INTAKE_SIDE, 0, 3);
    robot.turn_with_pid(-232.34, 600);
    MogoUtils::getMogo(5, 3, 8, 520);
    // IntakeHelper::voltage(12);

    // get ring
    robot.move_to_point({(au::inches)(-22.03), (au::inches)(4.92)}, true, INTAKE_SIDE, 700);
    robot.move_to_point({(au::inches)(7.57), (au::inches)(-1.77)}, true, INTAKE_SIDE, 600, 2.6);
    moveManual(400, -2);

    // back up
    robot.move_to_point({(au::inches)(-23.49), (au::inches)(5.23)}, false, MOGO_SIDE, 600);
    pros::delay(500);
    moClamp.overrideState(0);
    IntakeHelper::voltage(0);

    // get other mogo
    robot.move_to_point({(au::inches)(-12.64), (au::inches)(2.64)}, false, INTAKE_SIDE, 600);
    robot.move_to_point({(au::inches)(-26.83), (au::inches)(-9.17)}, true, MOGO_SIDE, 800);
    MogoUtils::getMogo(6, 3);

    // turn 4 guard
    robot.turn_with_pid(-61.82, 700);
    /**/

}