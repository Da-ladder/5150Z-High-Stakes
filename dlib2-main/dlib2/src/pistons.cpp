#include "pistons.h"
#include "pros/adi.hpp"

// Declares all solenoids
pros::ADIDigitalOut mogoClamp('A', LOW); // 0 is placeholder REPLACE WITH LETTER
pros::ADIDigitalOut liftTake('C', LOW); // 0 is placeholder REPLACE WITH LETTER
pros::ADIDigitalOut cornerSweep('D', LOW);
pros::ADIDigitalOut odomLift('B', LOW); ///?

// Declares all members of PistonControl which is the wrapper class
PistonControl moClamp(&mogoClamp);
PistonControl liftIntake(&liftTake);
PistonControl cornerDeploy(&cornerSweep);
PistonControl odomExtract(&odomLift);