#pragma once
#include "declarations.h"
#include "main.h"



class IntakeHelper {
    private:
        static bool excludeBlue;
        static bool blockLoop; // Not in use
        static int time;
        static int stuckTime;
        static bool feedWall;
        static bool stap;
        static int intakeState; // 0 is off, 1 is fwds, 2 is backwards
        static bool blocking; // to know when to block intake commands
                              // filter requires override.

    public:

    inline static void stuckDetect() {
        if (intake.get_current_draw() > 2000 && intakeState == 1) {
            stuckTime++;
        } else {
            stuckTime = 0;
        }
        if (stuckTime > 60) {
            blocking = true;
            intake.move_voltage(-12000);
            pros::delay(200);
            intake.move_voltage(12000);
            blocking = false;
            // stuckTime = 0;
        }
    }

    inline static void main() {
        bool ringLiftSense = false;
        while (true) {
            
            /*
            if (ringOpt.get() < 80) {
                blocking = true;
                intake.move_voltage(12);
                pros::delay(400);
                intake.move_voltage(0);
                pros::delay(100);
                // intake.move_voltage(12);
                // pros::delay(200);
                blocking = false;
                pros::delay(300);
            }
            */



            /*

            if (blockLoop) {
                pros::delay(5);
                continue;
            }
            // stuckDetect();
            time += 1;
            if (feedWall) {
                if ((opt.get_hue() > 200 && opt.get_hue() < 230) || (opt.get_hue() > 12 && opt.get_hue() < 20)) {
                    intake.move_voltage(4000);
                }
                if (ringOpt.get() < 180) {
                    blocking = true;
                    intake.move_voltage(6000);
                    pros::delay(80);
                    intake.move_voltage(-6000);
                    pros::delay(600);
                    blocking = false;
                } else {
                    // intake.move_voltage(0);
                }
            } else if (stap){
                // stop intake at ring
                if (ringOpt.get() <= 200) {
                    intake.move_voltage(0);
                }
            } else {
                // kicks out rings normally
                if (excludeBlue) {
                    if (intakeState == 1) {
                        // exlude here
                        
                        if (opt.get_hue() > 200 && opt.get_hue() < 230) {
                            
                        }
                    }

                    // do nothing if intake is off or going backwards
                } else {
                    if (intakeState == 1) {
                        
                    }

                    // do nothing if intake is off or going backwards
                }
            }
            */

            
            pros::delay(5);
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

    //
    inline static void voltage(double volt) {
        if (!blocking) {
            intake.move_voltage(volt*1000);
        }
        
        if (volt == 0) {
            intakeState = 0;
        } else if (volt > 0) {
            intakeState = 1;
        } else {
            intakeState = 2;
        }
    }

    inline static void init() {
        opt.set_led_pwm(100);
        // opt.set_integration_time(10); //DNE in pros v5
        pros::Task IntakeMngr(main);

    }

};