#pragma once
#include "declarations.h"
#include "liblvgl/llemu.hpp"
#include "pistons.h"
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

        double current_rot = fmod(intake.get_position(), ROT_PER_CYCLE);
        intake.move_voltage(12000);
        pros::delay(60); //0
        int passed = 0;

        while(true) {
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
            std::cout << current_rot << std::endl;
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
            if (stuckCheck) {
                stuckDetect();
            }
            
            
            if (stap) {
                if (excludeBlue) {
                    if (opt.get_hue() < 30) {
                        blocking = true;
                        // pros::lcd::print(3, "");
                        intake.move_voltage(-12000);
                        pros::delay(20);
                        intake.move_voltage(0);
                        blocking = false;
                        stap = false;
                    }
                } else if (!excludeBlue) {
                    if (opt.get_hue() >= 200 && opt.get_hue() <= 230 && opt.get_proximity() >= 150) {
                        blocking = true;
                        intake.move_voltage(-12000);
                        pros::delay(20);
                        intake.move_voltage(0);
                        blocking = false;
                        stap = false;
                    }
                }
            }
            
            

            if (excludeBlue && (!blockSort)) {
                if (opt.get_hue() >= 200 && opt.get_hue() <= 230 && opt.get_proximity() >= 150) {
                    reject();
                } else if (opt.get_hue() < 10 && opt.get_proximity() >= 340) {
                    // colorPistion.overrideState(0); // ACCEPT red
                }
            } else if ((!excludeBlue) && (!blockSort)) {
                if (opt.get_hue() >= 200 && opt.get_hue() <= 250) {
                    // colorPistion.overrideState(0); // ACCEPT blue
                } else if (opt.get_hue() < 10) {
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

    }

};