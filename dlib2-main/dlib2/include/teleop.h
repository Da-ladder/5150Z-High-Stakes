#pragma once
#include "declarations.h"
#include "pistons.h"
#include "autos.h"
#include "main.h"
#include "lift.h"
#include "intakeFuncts.h"
#include "pros/misc.h"


#define STRAIGHT_ARM 90
#define DESCORE_HEIGHT 155
#define SCORE_HEIGHT 130 //110
#define STORE_HEIGHT 285 //260 //242.31 2nd ary loading height 264.5
// #define STORE_HEIGHT2 266.5
#define SECOND_STORE_HEIGHT 267.5 // for double lady prown 245
#define IDLE_HEIGHT 310 
#define ABOVE_IN_HEIGHT 260 //223


/**
 * @brief Consolidates all driver required functions into one class.
 * Allows for four motor control with PTO enabled.
*/
class DriverControl {
    private:
        static int times; // time since last xy cord was printed
        static bool deScore; // toggle to score deeper
        static double voltage; // voltage to use for LB
        static bool atStoreHeight;
    public:
        static int onOff;
        // Changes button binding for various piston systems 
        inline static void initAll(){
            moClamp.changeButton(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2);
            liftIntake.changeButton(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y);
            // rushClamp.changeButton(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT);
            // hang.changeButton(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X);
            //intake.set_brake_mode(pros::v5::MotorBrake::hold);
        };


        // Allows for control of the chassis even with PTO enabled
        static void updateChassis();


        // Updates all pistons based on controller input
        inline static void updatePistions(){
            moClamp.toggle();
            rushClamp.toggle();
            hang.toggle();
        }


        // Allows for control of intake based on controller input
        inline static void updateIntake(){
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                IntakeHelper::sortState(false);
                IntakeHelper::StopAtColor(false);
                master.rumble(".");
            }
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                IntakeHelper::voltage(12);
                voltage = 12.0;
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                IntakeHelper::voltage(-12);
                voltage = 12.0;
            } else {
                IntakeHelper::voltage(0);
            }
        }

        inline static void updateLift() {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                LiftMngr::setLevel(DESCORE_HEIGHT);
                /*
                deScore = !deScore;
                if (deScore) {
                    master.rumble("--");
                } else {
                    master.rumble(".");
                }
                */
            }
            /*
            if (master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT)) {
                if (voltage == 12.0) {
                    master.rumble(".");
                    voltage = 6.0;
                } else {
                    master.rumble("-");
                    voltage = 12.0;
                }
            }
            */
            
            if (master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A)) {
                LiftMngr::setVoltage(-voltage, true);
                /*
                if (atStoreHeight) {
                    LiftMngr::setLevel(SECOND_STORE_HEIGHT);
                    atStoreHeight = false;
                    IntakeHelper::voltage(-7);
                    pros::delay(20);
                    while (master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A)) {
                        pros::delay(20);
                        IntakeHelper::voltage(0);
                    }
                } else {
                    LiftMngr::setVoltage(-voltage, true);
                }
                */
            } else if (master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B)) {
                LiftMngr::setVoltage(voltage, true);
                atStoreHeight = false;
            } else if (master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1)) {

                if (LiftMngr::getLevel() > 140 && LiftMngr::getLevel() < 290) {
                    if (deScore) {
                        IntakeHelper::voltage(-9);
                        LiftMngr::setMaxVolts(12);
                        LiftMngr::setLevel(DESCORE_HEIGHT);
                        atStoreHeight = false;
                    } else {
                        IntakeHelper::voltage(-9);
                        LiftMngr::setMaxVolts(12);
                        LiftMngr::setLevel(SCORE_HEIGHT); // score ring
                        atStoreHeight = false;
                    }
                } else if (LiftMngr::getLevel() > 300) {
                    LiftMngr::setMaxVolts(12);
                    LiftMngr::setLevel(STORE_HEIGHT); // store ring
                    atStoreHeight = true;
                } else {
                    LiftMngr::setMaxVolts(12);
                    LiftMngr::setLevel(IDLE_HEIGHT); // do nothing
                    atStoreHeight = false;
                }   
            } else if (master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X)) {
                IntakeHelper::voltage(-4);
                LiftMngr::setMaxVolts(12);
                LiftMngr::setLevel(SECOND_STORE_HEIGHT);
            } else {
                LiftMngr::setVoltage(0);
            }
        }

        inline static void openIntake() {
            if (master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP)) {
                if (liftIntake.getPistionState() == 1) {
                    liftIntake.overrideState(0);
                } else {
                    liftIntake.overrideState(1);
                }
                
            } 
        }

        inline static void clearCorner() {
            if (master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y)) {
                if (cornerDeploy.getPistonState() == 1) {
                    cornerDeploy.overrideState(0);
                    // LiftMngr::setLevel(283);
                } else {
                    cornerDeploy.overrideState(1);
                    // LiftMngr::setLevel(180);
                }
            }
        }
        
        inline static void driverMacro() {
            //if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            //    AutoSelector::run();
            //}
        }

        inline static void giveXYTurn() {
            if (xyBut.get_value()) {

                // while (!master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN)) { 
                    // pros::delay(25);
                // }

                dlib::Pose2d curPos = robot.odom.get_position();
                master.clear();
	            pros::delay(100);
	            master.set_text(1, 0, "x:" + std::to_string((curPos.x).in(au::inches)));
                pros::delay(50);
                master.set_text(2, 0, "y:" + std::to_string((curPos.y).in(au::inches)));
            }
            

            if (turnBut.get_value()) {

                // while (!master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN)) { 
                    // pros::delay(25);
                // }

                master.clear();
	            pros::delay(100);
                // master.set_text(1, 0, std::to_string(liftRot.get_position()/100.0));
	            master.set_text(1, 0, std::to_string(robot.imu.get_rotation().in(au::degrees)));
            }
            // master.clear();
            // pros::delay(50);
	        // master.set_text(1, 0, std::to_string(intake.get_current_draw()));
            
        }

        // Calls all functions during driver control
        inline static void main() {
            updateChassis();
            updatePistions();
            updateIntake();
            updateLift();
            clearCorner();
            openIntake();
            // giveXYTurn();
        }
};