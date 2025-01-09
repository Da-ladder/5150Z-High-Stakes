#include "pistons.h"
#include "pros/adi.hpp"

// Declares all solenoids
pros::adi::DigitalOut mogoClamp('A', LOW); // 0 is placeholder REPLACE WITH LETTER
pros::adi::DigitalOut liftTake('B', LOW); // 0 is placeholder REPLACE WITH LETTER
pros::adi::DigitalOut colorEject('0', LOW);
pros::adi::DigitalOut cornerSweep('C', LOW);
pros::adi::DigitalOut odomLift('0', LOW);
pros::adi::DigitalOut rushingClamp('G', LOW);
pros::adi::DigitalOut hangOut('0', LOW);


// Declares all members of PistonControl which is the wrapper class
PistonControl moClamp(&mogoClamp);
PistonControl liftIntake(&liftTake);
PistonControl cornerDeploy(&cornerSweep);
PistonControl rushClamp(&rushingClamp);
PistonControl odomExtract(&odomLift);
PistonControl colorPistion(&colorEject);
PistonControl hang(&hangOut);