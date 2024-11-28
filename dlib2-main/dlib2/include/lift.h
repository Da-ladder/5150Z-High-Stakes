#pragma once
#include "declarations.h"
#include "api.h"

// LiftMngr controls the lift
class LiftMngr{
    private:
        static bool blockLiftThread;
        static double holdLvl; // level to hold to
        static double voltReq; // voltage that external calls request from setVoltage
        static int time; // time since voltReq was executed and at zero
        static double prevVolt; // previous voltage, used for slew rate
                                // accel and decel control

    public:
        inline static void setMotorPwr(double voltage) {
            
            if ((prevVolt/1000) > voltage) {
                double volt = lift.get_voltage() - 1000;
                if (volt < voltage*1000) {
                    lift.move_voltage(voltage);
                    prevVolt = voltage;
                } else {
                    lift.move_voltage(volt);
                    prevVolt = volt;
                }
            } else if ((prevVolt/1000) < voltage) {
                double volt = lift.get_voltage() + 1000;
                if (volt > voltage*1000) {
                    lift.move_voltage(voltage);
                    prevVolt = voltage;
                } else {
                    lift.move_voltage(volt);
                    prevVolt = volt;
                }
            } else {
                lift.move_voltage(voltage*1000);
                prevVolt = voltage;
            }
        }

        inline static void main() {
            double kp = 0.2, ki = 0.00, kd = 0.1; //kp = 0.08, ki = 0.02, kd = 0.2
            double accumlator = 0;
            double lastErr = 0;
            double perVolt = 0;
            while (true) {
                if (blockLiftThread) {
                    setMotorPwr(voltReq);
                    time = 0;
                    pros::delay(5);
                    continue;
                }
                    time += 1;

                    if (time < (50)) {
                        holdLvl = liftRot.get_position()/100.0;
                        setMotorPwr(voltReq);
                        pros::delay(5);
                        continue;
                    }
                    float error = holdLvl - (liftRot.get_position()/100.0);

                    pros::lcd::print(5, "err: %f", error);

                    if (true) { //270
                        double wantVolt = 0;
                        if (fabs(error) > 5) {
                            wantVolt = (error * kp) + (accumlator * ki) + ((error-lastErr) * kd);
                        } else {
                            wantVolt = (error * kp) + (accumlator * ki);
                        }
                        
                        
                        setMotorPwr(wantVolt);

                        if (fabs(error) < 5 && fabs(error) > 0) {
                            if (error > 0) {
                                accumlator += error;
                            } else {
                                accumlator += error;
                            }
                            
                        } else if (fabs(error) > 5 || (fabs(error) <= 0.4)){
                            accumlator = 0;
                        }

                        lastErr = error;
                    }
                
                

                

                pros::delay(5); //slightly longer than main thread delay
            }
        }

        inline static int getCurLevel() {
            return lift.get_position();
        }

        inline static double getLevel() {
            return holdLvl;
        }

        //sets hold level to the user defined lvl
        inline static void setLevel(double lvl) {
            holdLvl = lvl;
        }

        //Requests voltage upon next turn 
        inline static void setVoltage(double volt, bool block = false) {
            blockLiftThread = block;
            voltReq = volt;
        }

        inline static void initall() {
            //sets pos to zero
            liftRot.reset();
            // lift.tare_position_all();

            // divides by 100 to get regular degrees
            holdLvl = liftRot.get_position()/100.0;

            pros::Task liftManagement(main);
        }

        
};