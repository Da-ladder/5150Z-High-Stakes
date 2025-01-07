#include "pistons.h"
#include "pros/adi.hpp"

// Declares all solenoids
pros::ADIDigitalOut mogoClamp('A', LOW); // 0 is placeholder REPLACE WITH LETTER
pros::ADIDigitalOut liftTake('B', LOW); // 0 is placeholder REPLACE WITH LETTER
pros::ADIDigitalOut colorEject('0', LOW);
pros::ADIDigitalOut cornerSweep('C', LOW);
pros::ADIDigitalOut odomLift('0', LOW); ///?
pros::ADIDigitalOut rushingClamp('G', LOW);
pros::ADIDigitalOut hangOut('0', LOW);


// Declares all members of PistonControl which is the wrapper class
PistonControl moClamp(&mogoClamp);
PistonControl liftIntake(&liftTake);
PistonControl cornerDeploy(&cornerSweep);
PistonControl rushClamp(&rushingClamp);
PistonControl odomExtract(&odomLift);
PistonControl colorPistion(&colorEject);
PistonControl hang(&hangOut);