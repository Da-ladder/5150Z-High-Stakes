#include "autos.h"
#include "declarations.h"
#include "intakeFuncts.h"
#include "mogoDetect.h"
#include "pistons.h"

#define ANALOG_SENSOR_PORT 1


#define MOGO_SIDE true
#define INTAKE_SIDE false
#define SCORE_TIME 500

#define STRAIGHT_ARM 90
#define IDLE_ARM 290

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

void Routes::skills() {
    IntakeHelper::blueExcld(true);
    // score on stake
    IntakeHelper::voltage(12);
    pros::delay(200);
    IntakeHelper::voltage(12);
    pros::delay(500);

    // back off and turn twds mogo
    robot.move_to_point({(au::inches)(-12.61), (au::inches)(0.03)}, false, INTAKE_SIDE);
    robot.turn_to_point({(au::inches)(-14.67), (au::inches)(-22.36)}, MOGO_SIDE, 800);

    // grab mogo
    MogoUtils::getMogo(8, 2);

    // turn to and grab rings
    robot.turn_to_point({(au::inches)(-38.71), (au::inches)(-23.95)}, INTAKE_SIDE, 800);
    RedRingUtil::getRing(true, 9, 2);
    
    robot.turn_to_point({(au::inches)(-35.22), (au::inches)(-39.58)}, INTAKE_SIDE, 800);
    RedRingUtil::getRing(true, 8, 2);

    robot.turn_to_point({(au::inches)(-55.98), (au::inches)(-55.50)}, INTAKE_SIDE, 800);
    RedRingUtil::getRing(true, 9, 2);
    pros::delay(300);
    moveManual(600, 4);
    
    robot.move_to_point({(au::inches)(-26.76), (au::inches)(-47.80)}, true, INTAKE_SIDE, 800, 1.5);
    RedRingUtil::getRing(true, 8, 3);
    pros::delay(300);

    robot.move_to_point({(au::inches)(-6.22), (au::inches)(-46.01)}, false, INTAKE_SIDE, 800);
    //RedRingUtil::getRing(true, 6, 2);

    robot.turn_with_pid(54.74, 700, 5);
    RedRingUtil::getRing(true, 6, 3);
    moveManual(200, -3);
    pros::delay(300);

    //Drop goal in corner
    robot.move_to_point({(au::inches)(-4.04), (au::inches)(-58.98)}, true, MOGO_SIDE);
    moveManual(800, 5); 
    moClamp.overrideState(0);
    pros::delay(150);
    IntakeHelper::voltage(0);
    //robot.restOdom(0, 0, 0); 
    
    //Move to goal 2
    robot.move_to_point({(au::inches)(-16.81), (au::inches)(-45.34)}, false, INTAKE_SIDE, 1000, 1.5);
    
    robot.move_to_point({(au::inches)(-17.07), (au::inches)(9.38)}, true, MOGO_SIDE, 800, 2);
    //robot.turn_with_pid(88.65, 700, 6);
    
    //Grab second goal
    MogoUtils::getMogo(6, 2);
    
    //Grab second set of rings
    IntakeHelper::voltage(12);

    robot.turn_to_point({(au::inches)(-32.95), (au::inches)(21.18)}, INTAKE_SIDE, 800);
    RedRingUtil::getRing(true, 8, 2);

    robot.turn_to_point({(au::inches)(-38.62), (au::inches)(40.96)}, INTAKE_SIDE, 800);
    RedRingUtil::getRing(true, 6, 2);

    robot.turn_to_point({(au::inches)(-59.78), (au::inches)(54.23)}, INTAKE_SIDE, 800);
    RedRingUtil::getRing(true, 6, 2);
    pros::delay(300);
    moveManual(600, 4);
    
    robot.move_to_point({(au::inches)(-33.10), (au::inches)(50.53)}, true, INTAKE_SIDE, 800, 2);
    //RedRingUtil::getRing(true, 8, 2);
    pros::delay(300);

    robot.move_to_point({(au::inches)(-10.12), (au::inches)(49.69)}, true, INTAKE_SIDE, 800);
    //RedRingUtil::getRing(true, 6, 2);
    
    robot.turn_with_pid(-54.39, 700, 5);
    RedRingUtil::getRing(true, 6, 2);
    pros::delay(500);

    //Drop goal in corner
    robot.move_to_point({(au::inches)(-6.79), (au::inches)(60.02)}, true, MOGO_SIDE);
    moveManual(700, 5); 
    moClamp.overrideState(0);
    pros::delay(150);
    

    // robot.restOdom(-5.19, 63.32, 57.70);  // CHECKPOINT
    // IntakeHelper::voltage(0);
    // robot.restOdom(0, 0, 0); 

    // back off
    robot.move_to_point({(au::inches)(-18.31), (au::inches)(50.46)}, false, INTAKE_SIDE);
    IntakeHelper::voltage(12);

    // get to next goal
    robot.turn_with_pid(4.46, 1300);
    // robot.turn_to_point({(au::inches)(-86.49), (au::inches)(35.77)}, MOGO_SIDE, 800);
    robot.move_to_point({(au::inches)(-73.05), (au::inches)(45.84)}, false, INTAKE_SIDE, 800, 2.5);
    robot.turn_with_pid(49.72, 600); //186.43
    // IntakeHelper::StopAtColor(true);
    RedRingUtil::getRing(true, 7, 3);
    

    

    // grab mogo
    robot.turn_with_pid(230.62, 150); //186.43
    IntakeHelper::voltage(0);
    robot.turn_with_pid(230.62, 900);
    robot.move_to_point({(au::inches)(-104.32), (au::inches)(8.22)}, false, MOGO_SIDE, 800, 2.5);
    MogoUtils::getMogo(6, 3);
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);

    // grab ring 
    robot.turn_with_pid(130.22, 1000); //186.43
    // robot.move_to_point({(au::inches)(-89.87), (au::inches)(-25.15)}, false, INTAKE_SIDE, 800, 2.5);
    RedRingUtil::getRing(true, 7, 3);

    // get ring
    robot.turn_with_pid(96.86, 1000); //186.43
    RedRingUtil::getRing(true, 7, 3);

    // move twds wall
    robot.move_to_point({(au::inches)(-87.1), (au::inches)(-60.18)}, false, INTAKE_SIDE, 800, 2.5);

    // clear corner
    robot.turn_with_pid(8.22, 800); //186.43
    cornerDeploy.overrideState(1);
    moveManual(800, -6);

    //drop mogo
    robot.turn_with_pid(-134.88, 1500, 10); //186.43
    moveManual(800, 5);
    moClamp.overrideState(0);
    cornerDeploy.overrideState(0);
    pros::delay(100);
    IntakeHelper::voltage(-12);

    //back off
    robot.move_to_point({(au::inches)(-95.26), (au::inches)(-39.72)}, false, INTAKE_SIDE, 800, 2.5);

    // fuck
    robot.turn_with_pid(-244.11, 1500, 10); //186.43
    robot.move_to_point({(au::inches)(-118.22), (au::inches)(4.24)}, false, MOGO_SIDE, 800, 2.5);
    MogoUtils::getMogo(8, 3);
    
    // turn into corner
    robot.turn_with_pid(-443.68, 1500, 10); //186.43
    IntakeHelper::voltage(12);
    cornerDeploy.overrideState(1);
    moveManual(900, -6);

    // place into corner
    robot.turn_with_pid(-591.07, 1800, 10); //186.43
    cornerDeploy.overrideState(0);
    moveManual(800, 5);
    // IntakeHelper::voltage(-12);
    moClamp.overrideState(0);
    pros::delay(100);
    IntakeHelper::voltage(-12);
    moveManual(1800, -2.5);
/**/
}

void Routes::placehold4() {
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
    MogoUtils::getMogo(5, 3, 8);
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
    robot.move_to_point({(au::inches)(17.62), (au::inches)(8.29)}, false, INTAKE_SIDE, 900, 2.1);
    moveManual(100, -6);
    robot.move_to_point({(au::inches)(53.67), (au::inches)(12.29)}, false, MOGO_SIDE, 0);

    // turn + get ring @ alliance stake
    robot.turn_with_pid(32.17, 1200);
    robot.move_to_point({(au::inches)(19.49), (au::inches)(-8.68)}, false, INTAKE_SIDE, 900);
    IntakeHelper::voltage(0);
    /*

    // get ring
    liftIntake.overrideState(1);
    // robot.turn_with_pid(-6.05, 450);

    
    moveManual(330, -5, -8);
    liftIntake.overrideState(0);
    pros::delay(230);
    moveManual(360, 7);
    pros::delay(900);
    IntakeHelper::voltage(0);


    /*
    robot.turn_with_pid(59.93, 1100);
    robot.move_to_point({(au::inches)(26.82), (au::inches)(-26.1)}, false, INTAKE_SIDE, 900, 2.4);
    moveManual(300, -3, -7);
    moveManual(400, -3);

    
    /*

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

void Routes::placehold4Mir() {
    IntakeHelper::blueExcld(true);
    int time = 0;

    while(300 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);

    LiftMngr::setLevel(STRAIGHT_ARM);

    robot.move_to_point({(au::inches)(27.12), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    LiftMngr::setLevel(IDLE_ARM);
    robot.turn_with_pid(85.05, 900); //.9k
    
    MogoUtils::getMogo(5, 3, 7);
    IntakeHelper::voltage(12);

    // get mid 
    // robot.turn_to_point({(au::inches)(48.85), (au::inches)(29.85)}, INTAKE_SIDE, 1100);
    robot.move_to_point({(au::inches)(48.85), (au::inches)(29.85)}, true, INTAKE_SIDE, 1100, 2.5);
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
    moveManual(100, -6);
    robot.move_to_point({(au::inches)(41.37), (au::inches)(-16.83)}, false, MOGO_SIDE, 0);

    // turn + get ring @ alliance stake
    robot.turn_with_pid(-34.48, 1200);
    robot.move_to_point({(au::inches)(10.66), (au::inches)(3.75)}, false, INTAKE_SIDE, 900);
    IntakeHelper::voltage(0);
    /*

    // get ring
    liftIntake.overrideState(1);
    // robot.turn_with_pid(-6.05, 450);

    
    moveManual(330, -6);
    liftIntake.overrideState(0);
    pros::delay(230);
    moveManual(360, 7);



    /*
    // robot.turn_with_pid(61.38, 900);
    robot.move_to_point({(au::inches)(16.82), (au::inches)(16.21)}, true, INTAKE_SIDE, 900, 2.3); //2.7
    moveManual(300, -4, -7);
    moveManual(200, -3);


    /*


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
    while(300 >= time*22) {
        LiftMngr::setVoltage(-10, true);
        time++;
        pros::delay(25);
    }
    LiftMngr::setVoltage(0, false);
    LiftMngr::setLevel(125);


    robot.move_to_point({(au::inches)(28.61), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    robot.turn_with_pid(-86.02, 800);
    MogoUtils::getMogo(7, 3);
    IntakeHelper::voltage(12);

    // get mid 
    robot.turn_with_pid(-203.50, 600);
    robot.move_to_point({(au::inches)(53.06), (au::inches)(-30.96)}, false, INTAKE_SIDE, 600, 2.5); //49.48, -28.71
    
    // back off
    robot.move_to_point({(au::inches)(32.04), (au::inches)(-18.1)}, true, MOGO_SIDE, 300, 2.9); //49.48, -28.71

    // get ring
    robot.move_to_point({(au::inches)(45.45), (au::inches)(-15.66)}, true, INTAKE_SIDE, 600); //49.48, -28.71

    // get corner
    robot.turn_with_pid(-89.9, 650);
    // robot.turn_to_point({(au::inches)(43.92), (au::inches)(18.48)}, INTAKE_SIDE, 600); //49.48, -28.71
    robot.move_to_point({(au::inches)(44.35), (au::inches)(21.16)}, false, INTAKE_SIDE, 600, 2.4); //49.48, -28.71
    
    // get ring on bottom stack
    moveManual(250, -7.7);    // pros::delay(200);
    
    // back off
    robot.move_to_point({(au::inches)(42.29), (au::inches)(8.12)}, true, MOGO_SIDE, 300, 2.8); //49.48, -28.71

    // get ring @ alliance stack
    robot.turn_with_pid(30.17, 300); //34.17
    robot.turn_with_pid(30.17, 500); //34.17
    robot.move_to_point({(au::inches)(14.01), (au::inches)(-10.62)}, false, INTAKE_SIDE, 800, 2.8); //49.48, -28.71
    liftIntake.overrideState(1);
    pros::delay(100);
    moveManual(280, -7, -8.5);
    liftIntake.overrideState(0);
    pros::delay(120);
    moveManual(440, 3.5);


    //
    /*
    robot.turn_with_pid(91.50, 800, 9);
    // IntakeHelper::StopAtColor(false);
    // IntakeHelper::voltage(10);
    moveManual(200, -8);
    moveManual(400, -4);
    moveManual(600, -2);

    /**/
}

void Routes::placehold3Mir() {
    IntakeHelper::blueExcld(false);
    LiftMngr::setMotorPwr(5);
    pros::delay(170);

    LiftMngr::setLevel(130);
    pros::delay(400);
    robot.move_to_point({(au::inches)(28.61), (au::inches)(-0)}, false, MOGO_SIDE, 0);
    robot.turn_with_pid(86.02, 700);
    LiftMngr::setLevel(175);
    MogoUtils::getMogo(7, 4);
    IntakeHelper::voltage(12);

    // get mid 
    robot.move_to_point({(au::inches)(50.32), (au::inches)(29.08)}, true, INTAKE_SIDE, 600, 2.5); //49.48, -28.71

    // back off
    robot.move_to_point({(au::inches)(29.12), (au::inches)(17.40)}, false, MOGO_SIDE, 300, 2.9); //49.48, -28.71

    // get ring
    robot.move_to_point({(au::inches)(43.77), (au::inches)(13.16)}, true, INTAKE_SIDE, 600); //49.48, -28.71

    
    // get corner
    robot.move_to_point({(au::inches)(40.66), (au::inches)(-22.61)}, true, INTAKE_SIDE, 600, 2.4); //49.48, -28.71 OLD
    // robot.turn_to_point({(au::inches)(40.66), (au::inches)(-22.61)}, INTAKE_SIDE, 730); //49.48, -28.71


    // get ring on bottom
    // moveManual(850, -10.5);
    moveManual(150, -6);
    // pros::delay(100);
    // pros::delay(200);


    
    // back off
    robot.move_to_point({(au::inches)(42.97), (au::inches)(-9.13)}, true, MOGO_SIDE, 300, 2.8); //49.48, -28.71
    IntakeHelper::voltage(0);

    // get ring @ alliance stack
    robot.turn_with_pid(-28.59, 650);
    IntakeHelper::voltage(12);
    robot.move_to_point({(au::inches)(8.12), (au::inches)(9.55)}, false, INTAKE_SIDE, 700, 2.9); //49.48, -28.71
    liftIntake.overrideState(1);
    pros::delay(150);
    moveManual(350, -6);
    liftIntake.overrideState(0);
    pros::delay(100);
    moveManual(440, 3.5);
    /*


    //
    robot.turn_with_pid(-94.59, 700, 7.95);
    // IntakeHelper::StopAtColor(false);
    // IntakeHelper::voltage(10);
    moveManual(200, -8);
    moveManual(400, -4);
    moveManual(600, -2);

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
    fIntDelay = 1000;

    // Rush mid
    // IntakeHelper::StopAtColor(true);
    // moveManual(700, -12);
    robot.move_to_point({(au::inches)(-36.63), (au::inches)(-0)}, false, INTAKE_SIDE, 0, 2.9); //OLD
    rushClamp.overrideState(1);
    // moveManual(50, -6);

    // back off & unclamp
    robot.move_to_point({(au::inches)(-26.3), (au::inches)(-0)}, false, MOGO_SIDE, 0); //OLD
    // moveManual(500, 7);
    rushClamp.overrideState(0);
    cornerDeploy.overrideState(0);

    // turn + grab other mogo + score ring on + drop mogo
    robot.turn_with_pid(117.96, 700);
    MogoUtils::getMogo(9, 3);
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    pros::delay(1100);
    moClamp.overrideState(0);


    // grab ring
    // IntakeHelper::StopAtColor(true);
    RedRingUtil::getRing(true, 8, 3);
    // robot.move_to_point({(au::inches)(-16.53), (au::inches)(-7.14)}, true, INTAKE_SIDE, 800, 2.5);
    fIntVolt = 0;
    fIntDelay = 300;
    robot.move_to_point({(au::inches)(-22.49), (au::inches)(-1.17)}, false, MOGO_SIDE, 800, 2.5);

    // grab rushed mogo + score
    robot.turn_with_pid(195.38, 700);
    robot.changeDetectLine(true);
    MogoUtils::getMogo(8, 3);
    robot.changeDetectLine(false);
    // IntakeHelper::StopAtColor(false);
    IntakeHelper::voltage(12);
    
    //Go to corner and get ring
    robot.move_to_point({(au::inches)(-1.14), (au::inches)(-4.05)}, true, INTAKE_SIDE, 800, 2.9);
    moveManual(1100, -6);

    //Backoff
    robot.move_to_point({(au::inches)(-9.43), (au::inches)(-2.25)}, false, MOGO_SIDE, 800, 2);

    // turn
    pros::delay(500); //1.2k
    robot.turn_with_pid(18.84, 1200, 11);

    // get to centerish
    robot.move_to_point({(au::inches)(-37.42), (au::inches)(-6.91)}, false, INTAKE_SIDE, 800, 2.4);


    /*

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
    pros::Task intDel(intakeDelay);
    IntakeHelper::blueExcld(false);
    LiftMngr::setLevel(175);
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
    pros::delay(1100);
    moClamp.overrideState(0);
    pros::delay(100);

    // grab other mogo
    robot.turn_with_pid(-65.89, 1000);

    robot.changeDetectLine(true);
    MogoUtils::getMogo(10, 3);
    robot.changeDetectLine(false);

    // grab ring
    robot.turn_with_pid(-170.45, 800);
    RedRingUtil::getRing(false, 11, 4);
    moveManual(100, -3);

    // drop mogo 
    pros::delay(1100);
    robot.turn_with_pid(0.98, 700, 9);
    moClamp.overrideState(0);
    pros::delay(200);

    // face twds other mogo
    robot.turn_with_pid(-179.72, 800, 12);
    robot.move_to_point({(au::inches)(-28.67), (au::inches)(-13.57)}, false, MOGO_SIDE, 0);




    /*


    // go twds wall
    robot.move_to_point({(au::inches)(7.31), (au::inches)(-8.46)}, true, INTAKE_SIDE, 500);
    // corner
    // robot.turn_to_point({(au::inches)(), (au::inches)(21.70)}, INTAKE_SIDE, 900);
    
    robot.move_to_point({(au::inches)(2.56), (au::inches)(18.44)}, true, INTAKE_SIDE, 900);

    // man it
    moveManual(1200, -6, -7);
    moveManual(500, 4.9);
    // cornerDeploy.overrideState(1);


    /*
    IntakeHelper::voltage(-12);

    // get corner ring

    robot.move_to_point({(au::inches)(-7.10), (au::inches)(14.78)}, true, INTAKE_SIDE, 900, 2.9, 8);
    robot.turn_with_pid(-113.56, 800);
    IntakeHelper::voltage(12);
    moveManual(1200, -6);

    // back off & sweep
    robot.move_to_point({(au::inches)(-2.96), (au::inches)(22.41)}, false, MOGO_SIDE, 900, 2.9);
    cornerDeploy.overrideState(1);
    pros::delay(500);
    moveManual(900, -6); //-25.91
    robot.turn_with_pid(-259.66, 1500, 8);
    /**/


    






}