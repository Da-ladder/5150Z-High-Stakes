#pragma once
#include "autos.h"
#include "pistons.h"
#include "declarations.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pistons.h"
#include "lift.h"
#include "intakeFuncts.h"

std::vector<const char*> AutoSelector::routeNames = {};
std::vector<void (*)()> AutoSelector::routePointers = {};

int AutoSelector::indexToRun = 0;

pros::ADIPotentiometer AutoSelector::pot = pros::ADIPotentiometer('E', pros::adi_potentiometer_type_e::E_ADI_POT_EDR);

// Function terminates when goal is detected

void moveToGoal(double speed = 5, double dist = 55, int timeout = 0) {
    int time = 0;
    while (goalOpt.get() > dist) {
    }
}

void Routes::skills() {
}

void Routes::placehold4() {
    
    moClamp.overrideState(1);
    pros::delay(400);
    robot.ffwLat((au::inches)(-19.3), au::milli(au::seconds)(2000));

    // IntakeHelper::voltage(12);

    // robot.ffwTurn((au::degrees)(90));
    // pros::delay(50);
    // robot.ffwTurn((au::degrees)(37.5-180));

    // robot.move_to_point({(au::inches)(36.39), (au::inches)(11.71)}, false, false);



}

void Routes::placehold1() {
}

void Routes::placehold2() {
}

void Routes::placehold3() {
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