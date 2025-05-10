#pragma once
#include "declarations.h"
#include "liblvgl/llemu.hpp"
#include "pistons.h"
#include "pros/device.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <ostream>
#include <string>

#define MAIN_LOOP_DELAY 10 //5
#define STUCK_DELAY_MS 400
#define STUCK_DEG_RANGE 2
#define SORT_MS_EXT_DELAY 100
#define ROT_PER_CYCLE 5.75 //5.73



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
        
        double min_threshold = intake.get_target_velocity()/6.0;
        // std::cout << "MIN_VELO:" << std::to_string(min_threshold) << "|| ACT_VELO:" << std::to_string(intake.get_actual_velocity()) << "|| STUCKTIME:" << std::to_string(stuckTime) <<std::endl;


        if (fabs(intake.get_actual_velocity()) < fabs(min_threshold) && intakeState == 1) {
            stuckTime ++;
        } else {
            stuckTime = 0;
        }

        if (stuckTime >= 10) {
            blocking = true;
            stuckTime = 0;
            intake.move_voltage(-12000);
            pros::delay(100);
            intake.move_voltage(12000);
            blocking = false;
        }
        

        /*
        if (intakeState == 1 && abs(curIntakePos - intakePos) < STUCK_DEG_RANGE) {
            intakePos = curIntakePos;
            stuckTime ++;
            master.set_text(1, 0, std::to_string(stuckTime));
        } else {
            intakePos = curIntakePos;
            stuckTime = 0;
        }

        // reverse intake if it detects it is stuck
        if (stuckTime >= (STUCK_DELAY_MS/MAIN_LOOP_DELAY)) {
            intake.move_voltage(-12000);
            pros::delay(500);
            intake.move_voltage(12000);
            stuckTime = 0;
        }
            */
        


    }

    inline static void reject() {
        blocking = true;
        // intake.move_voltage(0);
        // pros::delay(400);

        //.37 - .67 first eject
        // 2.21 - 2.60 second
        // 4.13 - 4.48 third

        if (pros::competition::is_disabled()) {
            return;
        }

        double current_rot = fmod(intake.get_position(), ROT_PER_CYCLE);
        intake.move_voltage(12000);
        pros::delay(60); //0
        int passed = 0;

        int loopsLooped = 0;

        while(true) {
            loopsLooped++;
            current_rot = fmod(intake.get_position(), ROT_PER_CYCLE);
            if (current_rot >= .57 && current_rot <= .67) {
                if (passed != 1 && passed > 0) {
                    break;
                }
                break;
                passed = 1;
            } else if (current_rot >= 2.41 && current_rot <= 2.60) {
                if (passed != 2 && passed > 0) {
                    break;
                }
                break;
                passed = 2;
            } else if (current_rot >= 4.33 && current_rot <= 4.48) {
                if (passed != 3 && passed > 0) {
                    break;
                }
                break;
                passed = 3;
            }
            if (loopsLooped*5 > 300) { // if over 250 ms, cut it out
                break;
            }
            // std::cout << current_rot << std::endl;
            pros::delay(5);
        }
        intake.move_voltage(-12000);
        pros::delay(100);
        intake.move_voltage(0);

        /*
        while (lightDetect.get_value() < 250) {
            pros::delay(5);
        } 
        // intake.move_voltage(0);
        pros::delay(40);
        intake.move_voltage(-6);
        pros::delay(100);
        intake.move_voltage(0);
        pros::delay(500);
        */
        blocking = false;
        intake.move_voltage(12000);
    }

    inline static void main() {
        bool ringLiftSense = false;
        while (true) {
            // while (pros::competition::is_disabled() || !pros::competition::is_autonomous()) {
                // pros::delay(10);
            // }
            if (stuckCheck) {
                stuckDetect();
            }
            // pros::lcd::set_text(4, "HUE: " + std::to_string(opt.get_hue()));
            
            
            if (stap && !pros::competition::is_disabled()) {
                if (excludeBlue) {
                    if (opt.get_hue() < 20) {
                        // pros::lcd::print(5, "RED STOP TRIGGERED");
                        blocking = true;
                        intake.move_voltage(-4000);
                        pros::delay(25);
                        intake.move_voltage(0);
                        blocking = false;
                        stap = false;
                    }
                } else if (!excludeBlue) {
                    if (opt.get_hue() >= 190 && opt.get_hue() <= 230 && opt.get_proximity() >= 150) {
                        // pros::lcd::print(5, "BLUE STOP TRIGGERED");
                        blocking = true;
                        intake.move_voltage(-4000);
                        pros::delay(25);
                        intake.move_voltage(0);
                        blocking = false;
                        stap = false;
                    }
                }
            }
            
            
            
            if (excludeBlue && (!blockSort) && !pros::competition::is_disabled()) {
                if (opt.get_hue() >= 190 && opt.get_hue() <= 230 && opt.get_proximity() >= 150) {
                    // pros::lcd::print(7, "BLUE EJECT TRIGGERED");
                    reject();
                } else if (opt.get_hue() < 10 && opt.get_proximity() >= 340) {
                    // colorPistion.overrideState(0); // ACCEPT red
                }
            } else if ((!excludeBlue) && (!blockSort) && !pros::competition::is_disabled()) {
                if (opt.get_hue() >= 200 && opt.get_hue() <= 250) {
                    // colorPistion.overrideState(0); // ACCEPT blue
                } else if (opt.get_hue() < 20) {
                    // pros::lcd::print(7, "RED EJECT TRIGGERED");
                    reject();
                }
            }
            
            // pros::lcd::set_text(5, "err: " + std::to_string(opt.get_hue()));
                
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
            // blocking = false;
            // do nothing
        }
        
        
    }

    inline static void init() {
        // master.clear();
        opt.set_integration_time(50); //50 b4
        pros::delay(50);
        opt.set_led_pwm(100);
        // opt.set_integration_time(10);
        pros::Task IntakeMngr(main);
        pros::delay(40);

    }

};