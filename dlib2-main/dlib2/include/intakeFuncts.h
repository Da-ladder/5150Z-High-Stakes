#pragma once
#include "declarations.h"
#include "liblvgl/llemu.hpp"
#include "pistons.h"
#include <string>

#define MAIN_LOOP_DELAY 10 //5
#define STUCK_DELAY_MS 400
#define STUCK_DEG_RANGE 7
#define SORT_MS_EXT_DELAY 100



class IntakeHelper {
    private:
        static bool stuckCheck;
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
            pros::delay(200);
            intake.move_voltage(12000);
            stuckTime = 0;
        }


    }

    inline static void reject() {
        blocking = true;
        while (opt.get_proximity() > 40) {
            while (opt.get_proximity() > 40) {
                intake.move_voltage(11000);
                pros::delay(10);
            }
            intake.move_voltage(-12000);
            pros::delay(140);
        }
        // intake.move_voltage(12000);
        /*
        while (sortLimit.get_value() == 0) {
            pros::delay(10);
        }
        while (sortLimit.get_value() == 1) {
            pros::delay(5);
        }
        pros::delay(10);
        intake.move_voltage(-10000);
        pros::delay(180);
        if (sortLimit.get_value() == 1) {
            intake.move_voltage(-12000);
            pros::delay(300);
        }
        intake.move_voltage(12000);
        */
        blocking = false;
        
        
        /*
        while (sortLimit.get_value() == 0) {
            pros::delay(10);
        }
        if (true) {
            blocking = true;
            pros::delay(100); //80
            intake.move_voltage(-6*1000); //-10
            pros::delay(60);
            intake.move_voltage(12*1000);
            pros::delay(50);
            blocking = false;
        }
        */
    }

    inline static void main() {
        bool ringLiftSense = false;
        while (true) {
            if (stuckCheck) {
                stuckDetect();
            }
            
            
            if (stap) {
                if (excludeBlue) {
                    if (opt.get_hue() < 21) {
                        
                        IntakeHelper::voltage(0);
                        
                    }
                } else if (!excludeBlue) {
                    if (opt.get_hue() >= 200 && opt.get_hue() <= 225) {
                        IntakeHelper::voltage(0);
                        
                    }
                }
            }
            

            if (excludeBlue && (!blockSort)) {
                if (opt.get_hue() >= 200 && opt.get_hue() <= 230 && opt.get_proximity() >= 255) {
                    reject();
                } else if (opt.get_hue() < 21) {
                    // colorPistion.overrideState(0); // ACCEPT red
                }
            } else if ((!excludeBlue) && (!blockSort)) {
                if (opt.get_hue() >= 200 && opt.get_hue() <= 230 && opt.get_proximity() >= 255) {
                    // colorPistion.overrideState(0); // ACCEPT blue
                } else if (opt.get_hue() < 21) {
                    reject();
                }
            }
            pros::lcd::set_text(5, "err: " + std::to_string(opt.get_proximity()));
                
            pros::delay(MAIN_LOOP_DELAY);
        }
    }

    inline static void stuckCheckChange (bool var) {
        stuckCheck = var;
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
        opt.set_integration_time(20); //50 b4
        pros::delay(50);
        opt.set_led_pwm(100);
        // opt.set_integration_time(10);
        pros::Task IntakeMngr(main);

    }

};