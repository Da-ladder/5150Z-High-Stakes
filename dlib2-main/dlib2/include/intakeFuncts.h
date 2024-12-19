#pragma once
#include "declarations.h"
#include "pistons.h"

#define MAIN_LOOP_DELAY 5
#define STUCK_DELAY_MS 300
#define STUCK_DEG_RANGE 7
#define SORT_MS_EXT_DELAY 330



class IntakeHelper {
    private:
        static bool excludeBlue;
        static bool blockSort;
        static int intakePos;
        static int time;
        static int stuckTime;
        static bool feedWall;
        static bool stap;
        static int intakeState; // 0 is off, 1 is fwds, 2 is backwards
        static bool blocking; // to know when to block intake commands
                              // filter requires override.

    public:

    inline static void stuckDetect() {

        int curIntakePos = intake.get_position();

        if (intakeState == 1 && abs(curIntakePos - intakePos) < STUCK_DEG_RANGE) {
            intakePos = curIntakePos;
            stuckTime ++;
        } else {
            intakePos = curIntakePos;
            stuckTime = 0;
        }

        // reverse intake if it detects it is stuck
        if (stuckTime >= (STUCK_DELAY_MS/MAIN_LOOP_DELAY)) {
            intake.move_voltage(-12000);
            pros::delay(100);
            intake.move_voltage(12000);
            stuckTime = 0;
        }


    }

    inline static void main() {
        bool ringLiftSense = false;
        while (true) {

            // stuckDetect();

            if (excludeBlue && (!blockSort)) {
                if (opt.get_hue() >= 200 && opt.get_hue() <= 225/* && opt.get_proximity() >=*/) {
                    colorPistion.overrideState(1); // REJECT blue
                    pros::delay(SORT_MS_EXT_DELAY);
                } else if (opt.get_hue() < 15) {
                    colorPistion.overrideState(0); // ACCEPT red
                }
            } else if ((!excludeBlue) && (!blockSort)) {
                if (opt.get_hue() >= 200 && opt.get_hue() <= 225) {
                    colorPistion.overrideState(0); // ACCEPT blue
                } else if (opt.get_hue() < 15) {
                    colorPistion.overrideState(1); // REJECT red
                    pros::delay(SORT_MS_EXT_DELAY);
                }
            } else {
                colorPistion.overrideState(0); // TURN OFF SORTING
            }

            if (stap) {
                if (excludeBlue) {
                    if (opt.get_hue() < 15) {
                        IntakeHelper::voltage(0);
                    }
                } else if (!excludeBlue) {
                    if (opt.get_hue() >= 200 && opt.get_hue() <= 225) {
                        IntakeHelper::voltage(0);
                    }
                }
            }

                
            pros::delay(MAIN_LOOP_DELAY);
        }
    }

    inline static void StopAtColor(bool var) {
        stap = var;
    }

    inline static bool getWallFeeder() {
        return feedWall;
    }

    inline static void setWallFeeder(bool on) {
        feedWall = on; 
    }

    inline static void blueExcld(bool sort) {
        excludeBlue = sort;
    }

    inline static bool getStortState() {
        return !blockSort;
    }

    inline static void sortState(bool state) {
        blockSort = !state;
    }

    //
    inline static void voltage(double volt) {
        if (!blocking) {
            intake.move_voltage(volt*1000);

            if (volt == 0) {
                intakeState = 0;
            } else if (volt > 0) {
                intakeState = 1;
            } else {
                intakeState = 2;
            }
        } else {
            // do nothing
        }
        
        
    }

    inline static void init() {
        opt.set_integration_time(50);
        pros::delay(50);
        opt.set_led_pwm(100);
        // opt.set_integration_time(10);
        pros::Task IntakeMngr(main);

    }

};