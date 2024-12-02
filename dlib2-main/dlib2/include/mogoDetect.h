#pragma once
#include "declarations.h"
#include "pistons.h"
#include "lift.h"
#include "pros/vision.h"

#define MOGO 1
#define RED_RING 1
#define BLUE_RING 2
#define LOW_TRIGGER 0
#define LOWEST_LIFT_ANGLE 0 //156
#define HIGHEST_LIFT_ANGLE 10000 //156
#define SCORE_TRIGGER 209


// currently only set so that one type of vision can be used at once
class MogoUtils {
    private:
        static int mogoX, mogoY;  // Largest MOGO is considered
        
        inline static void moveDrive(double l, double r, int t) {
            robot.chassis.left_motors.raw.move_voltage(l*1000);
            robot.chassis.right_motors.raw.move_voltage(r*1000);
            pros::delay(t);
        }
    public:
        // Sets the vision to have its zero point in the center
        inline static void init() {
            camDetect.set_zero_point(pros::E_VISION_ZERO_CENTER); //set brightness to 14 (calibrate sensor on SKILLS FIELD)
            //camDetect.set_auto_white_balance(false);
            camDetect.set_exposure(31);

            // Sets mogo sig
            pros::vision_signature_s_t MOGO_SIG = pros::c::vision_signature_from_utility(1, -1679, -969, -1324, -5931, -5061, -5496, 2.800, 0);
            camDetect.set_signature(MOGO, &MOGO_SIG);
        }

        // Refreshes the x and y cord of the mogo
        inline static void refreshMogo() {
            pros::vision_object_s_t  mogo = camDetect.get_by_sig(0, MOGO);

            mogoX = mogo.x_middle_coord;
            mogoY = mogo.y_middle_coord;

            pros::lcd::print(6, "mogoX #: %i", mogoX);
            pros::lcd::print(7, "mogoY #: %i", mogoY);

        }

        // Returns the x cord of the middle of the mogo
        inline static int getMidX() {
            return mogoX;
        }

        // Returns the y cord of the middle of the mogo
        inline static int getMidY() {
            return mogoY;
        }

        // Move to the mogo
        inline static void getMogo(double baseSpeed = 8, double minSpeed = 1.5) {
            double 
            tkP=0.05, // Turn P
            tkD=0.0, // Turn D
            lkP=-0.02; // lateral P

            double yErr=0, lastYerr=0, yTarg=0;

            // xTarg is the bottom of the camera
            double xErr=0, lastXerr=0, xTarg=200;

            double flatSpeed, lspeed, rspeed;
            double disError;

            int mmDis = 9999; // default for when nothing is detected

            // keep looping while mogo has not been detected in the clamp
            while (mmDis > 53) {
                MogoUtils::refreshMogo();

                disError = 50 - mmDis; // 50 is most in mogo can go on corner

                // X ERROR IS THE LEFT & RIGHT ERROR: LEFT::+ RIGHT::-
                xErr = 0 - getMidX();
                // Y ERROR GIVES distance (SWITCH TO DISTANCE FOR FOWARDS CONTROL HIGHER THAN -40) or when yErr is smaller than zero!
                // NOTE: -50 & 50 is range of distance sensor
                yErr = -10 - getMidY();


                // base speed limiter
                xErr = 0 - getMidX();
                if ((yErr < 0) && (fabs(xErr) <= 50)) {
                    flatSpeed = disError * lkP;
                } else {
                    flatSpeed = baseSpeed;
                }


                
                // Turning controls
                lspeed = flatSpeed - (xErr*tkP + ((xErr-lastXerr) * tkD));
                rspeed = flatSpeed + (xErr*tkP + ((xErr-lastXerr) * tkD));


                

                if (lspeed < minSpeed) { lspeed = minSpeed; }
                if (rspeed < minSpeed) { rspeed = minSpeed; }

                pros::lcd::print(6, "LSPEED #: %f", lspeed);
                pros::lcd::print(7, "RSPEED #: %f", rspeed);

                moveDrive(lspeed, rspeed, 10); //bc mogo clamp is forwards
                lastXerr = xErr;
                lastYerr = yErr;
                mmDis = goalOpt.get();

            }

            // Lets the drive contuine under residual power and clamps after 75 ms
            moveDrive(2, 2, 30);
            moveDrive(1.5, 1.5, 20);
            moClamp.overrideState(1);
            moveDrive(0, 0, 150);

        }
          
};

// INSERT RING TRACKER BELOW
// TODO: add filter so that it does not take objects with small width (to prevent stake tracking)
class RedRingUtil {
    private:
        static int ringX, ringY;  // Largest RED ring is considered
        
        inline static void moveDrive(double l, double r, int t) {
            robot.chassis.left_motors.raw.move_voltage(l*1000);
            robot.chassis.right_motors.raw.move_voltage(r*1000);
            pros::delay(t);
        }
    public:
        // Sets the vision to have its zero point in the center
        inline static void init() {
            camRingDetect.set_zero_point(pros::E_VISION_ZERO_CENTER); //set brightness to 14 (calibrate sensor on SKILLS FIELD)
            //camRingDetect.set_auto_white_balance(false);
            camRingDetect.set_exposure(31);

            // Sets red & blue ring sig (calibration required)
            pros::vision_signature_s_t RED_RING_SIG = pros::c::vision_signature_from_utility(1, 8561, 10445, 9504, -2371, -1133, -1752, 4.000, 0);
            camRingDetect.set_signature(RED_RING, &RED_RING_SIG);

            pros::vision_signature_s_t BLUE_RING_SIG = pros::c::vision_signature_from_utility(2, -4153, -3497, -3826, 6599, 8631, 7614, 5.200, 0);
            camRingDetect.set_signature(BLUE_RING, &BLUE_RING_SIG);
        }

        // Refreshes the x and y cord of the mogo
        inline static void refreshRing(bool red = true) {
            pros::vision_object_s_t ring;
            if (red) {
                ring = camRingDetect.get_by_sig(0, RED_RING);
            } else {
                ring = camRingDetect.get_by_sig(0, BLUE_RING);
            }
             

            ringX = ring.x_middle_coord;
            ringY = ring.y_middle_coord;

            pros::lcd::print(6, "ringX #: %i", ringX);
            pros::lcd::print(7, "ringY #: %i", ringY);

        }

        // Returns the x cord of the middle of the mogo
        inline static int getMidX() {
            return ringX;
        }

        // Returns the y cord of the middle of the mogo
        inline static int getMidY() {
            return ringY;
        }

        // Move to the mogo
        inline static void getRing(bool red = true, double baseSpeed = 8, double minSpeed = 1.5) {
            double 
            tkP=0.04, // Turn P
            tkD=0.0, // Turn D
            lkP=0.18; // lateral P

            double yErr=0, lastYerr=0, yTarg=0;

            // xTarg is the bottom of the camera
            double xErr=0, lastXerr=0, xTarg=200;

            double flatSpeed, lspeed, rspeed;


            // keep looping while mogo has not been detected in the clamp
            while (yErr >= -10) {

                if (red) {
                    RedRingUtil::refreshRing();
                } else {
                    RedRingUtil::refreshRing(false);
                }

                // X ERROR IS THE LEFT & RIGHT ERROR: LEFT::+ RIGHT::-
                xErr = -10 - getMidX();

                // Y ERROR GIVES distance (SWITCH TO DISTANCE FOR FOWARDS CONTROL HIGHER THAN -40) or when yErr is smaller than zero!
                yErr = 60 - getMidY();

                // base speed limiter
                xErr = 0 - getMidX();
                if ((yErr < 90)) {
                    flatSpeed = yErr * lkP;
                } else {
                    flatSpeed = baseSpeed;
                }


                
                // Turning controls
                lspeed = flatSpeed + (xErr*tkP + ((xErr-lastXerr) * tkD));
                rspeed = flatSpeed - (xErr*tkP + ((xErr-lastXerr) * tkD));


                

                if (lspeed < minSpeed) { lspeed = minSpeed; }
                if (rspeed < minSpeed) { rspeed = minSpeed; }

                pros::lcd::print(6, "LSPEED #: %f", lspeed);
                pros::lcd::print(7, "RSPEED #: %f", rspeed);

                moveDrive(-lspeed, -rspeed, 10); //bc intake is reverse
                lastXerr = xErr;
                lastYerr = yErr;

            }

            // Lets the drive contuine under residual power so intake can succc
            moveDrive(-5, -5, 150);
            moveDrive(-2, -2, 40);
            moveDrive(0, 0, 30);
        }
          
};


class StakeVision {
    private:
        static int objectsDetected;
        static int stakeLowestY[10];
        static int stakeMidY[10];
        static int stakeMidX[10];
        static pros::vision_object_s_t stakeArray[10];

        inline static void refreshStakeLowY() {
            for (int i = 0; i < 10; i++) {
                if (stakeArray[i].signature == 255) {
                    objectsDetected = i;
                    return;
                } else {
                    stakeMidX[i] = stakeArray[i].left_coord + (stakeArray[i].width/2);
                    stakeMidY[i] = stakeArray[i].top_coord + (stakeArray[i].height/2);
                    stakeLowestY[i] = stakeArray[i].top_coord+(stakeArray[i].height);
                }
            }
            objectsDetected = 10;
        }

        inline static int filterSigs() {
            bool found = false;
            int highestY = 0;

            // ASSUME LIFT IS TRUE
            if (true) {
                for (int i = 0; i < 10; i++) {
                    if (stakeMidY[i] >= stakeMidY[highestY]) {
                        highestY = i;
                        found = true;
                    }
                }
            }
            
            if (found) {
                return highestY;
            } else {
                return -1;
            }
            
        }

        // Might bug out if some are red
        inline static void refreshBlueStake() {
            camDetect.read_by_sig(0, BLUE_RING, 10, stakeArray);
        }

        // Might bug out if some are blue
        inline static void refreshRedStake() {
            camDetect.read_by_sig(0, RED_RING, 10, stakeArray);
        }

        inline static void moveDrive(double l, double r, int t) {
            left_motors.move_voltage(l*1000);
            right_motors.move_voltage(r*1000);
            pros::delay(t);
        }
    public:
        //camDetect.read_by_sig(0, BLUE_STAKE, 10, array)
        // RUN INIT WHEN PROGRAM STARTS
        inline static void init() {
            // Sets blue STAKE sig RED SETUP REQUIRED
            pros::vision_signature_s_t blueStkSig = pros::c::vision_signature_from_utility(1, -4125, -3789, -3958, 9519, 10505, 10012, 8.000, 0);
            camDetect.set_signature(BLUE_RING, &blueStkSig);

            pros::vision_signature_s_t redStkSig = pros::c::vision_signature_from_utility(2, 8109, 9981, 9046, -1683, -645, -1164, 5.400, 0);
            camDetect.set_signature(RED_RING, &redStkSig);


            camDetect.set_zero_point(pros::E_VISION_ZERO_CENTER); //set brightness to 14 (calibrate sensor)
            camDetect.set_exposure(25);
        }

        inline static void devFeedBack(bool targetBlue = true) {
            if (targetBlue) {
                refreshBlueStake();
                refreshStakeLowY();
            } else {
                refreshRedStake();
                refreshStakeLowY();
            }

            int index = filterSigs();
            if (index != -1) {
                pros::lcd::print(5, "LOWEST STAKE Y: %i", stakeLowestY[index]);
                // x_middle_coord
                pros::lcd::print(6, "X MID #: %i", stakeMidX[index]);
                pros::lcd::print(7, "Y MID #: %i", stakeMidY[index]); //top to bottom coord
            } else {
                pros::lcd::print(6, "NO OBJECTS DETECTED");
            }
        }

        inline static void scoreOnStake(double baseSpeed = 5, double maxSpeed = 8, double minSpeed = 0, bool targetBlue = true) {
            bool score = false;
            double rspeed, lspeed, xErr, lastxerr;
            double tkD = 0.00, tkP = 0.05;
            while (!score) {
                devFeedBack();
                if (targetBlue) {
                    refreshBlueStake();
                    refreshStakeLowY();
                } else {
                    refreshRedStake();
                    refreshStakeLowY();
                }

                int index = filterSigs();

                if (index = -1) {
                    index = 0;
                }

                if (index != -1 && stakeLowestY[index] < SCORE_TRIGGER) {
                        xErr = 160 - stakeMidX[index];
                        
                        
                        lspeed = baseSpeed + (xErr*tkP + ((xErr-lastxerr) * tkD));
                        rspeed = baseSpeed - (xErr*tkP + ((xErr-lastxerr) * tkD));

                        if (lspeed < minSpeed) { lspeed = minSpeed; }
                        if (rspeed < minSpeed) { rspeed = minSpeed; }

                        if (lspeed > maxSpeed) { lspeed = maxSpeed; }
                        if (rspeed > maxSpeed) { rspeed = maxSpeed; }

                        lastxerr = xErr;

                        // pros::lcd::print(6, "LSPEED #: %f", lspeed);
                        // pros::lcd::print(7, "RSPEED #: %f", rspeed);


                        moveDrive(lspeed, rspeed, 10); //bc mogo clamp is the reverse
                } else if (index != -1 && stakeLowestY[index] >= SCORE_TRIGGER) {
                    // assumes the bot will be at the correct angle when the stake is close enough
                    // score it!
                    LiftMngr::setLevel(100);
                    moveDrive(7, 7, 70);
                    moveDrive(-3, -3, 200);
                    score = true;
                    moveDrive(0, 0, 400);
                } else {
                    // run both sides at same speed
                    moveDrive(minSpeed, minSpeed, 10);
                }
            }
        }





};
