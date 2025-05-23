#pragma once
#include "declarations.h"
#include "pros/llemu.hpp"
#include "au/au.hpp"
#include "mogoDetect.h"
#include "pistons.h"
#include "declarations.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pistons.h"
#include "lift.h"
#include "intakeFuncts.h"
#include <concepts>

/**
 * @brief Creates a class so autos can be selected from the
 * potentiometer. Also includes the ability to add autos and
 * rename them independently of the function name on the fly. 
*/
class AutoSelector {
  private:
    // All functions and variables are static as these routes
    // should stay the same no matter what
    static pros::adi::Potentiometer pot; // The potentiometer to use
    static std::vector<const char*> routeNames; // Vector of route names
    static std::vector<void (*)()> routePointers; // Vector of route function pointers
    static int indexToRun; // The index for the route it is currently on
  
  public:
    /**
     * @brief Adds a new route to the selector
     *
     * @param routeName The name to use for the route which appears on the brain screen
     * @param routeCall The function to use for the route
    */
    inline static void add(const char *routeName, void routeCall()) {
        routeNames.push_back(routeName);
        routePointers.push_back(routeCall);
    }


    /**
     * @brief Updates the index which controls what route to run
     * this must be called to refresh the route it is on
     */
    inline static void updatePath() {
        if (xyBut.get_new_press()) {
            indexToRun += 1;
        }
        if (turnBut.get_new_press()) {
            indexToRun -= 1;
        }
        // indexToRun = pot.get_value_calibrated()/100;
    }


    // Returns indexToRun
    inline static int getRunInt() {
        return indexToRun;
    }


    /**
     * @brief Returns the name of the route that is selected based
     * on the index which controls which ro ute it is on. The function
     * will print OOB (Out of bounds) if the current index is larger
     * than the size of the vector which would cause a data abort
     * otherwise.
    */
    inline static void printPath() {
        pros::lcd::print(3, routeNames[0]);
        if (indexToRun >= 0 && indexToRun < routeNames.size()) {
            // pros::lcd::print(3, routeNames[indexToRun]);
        } else {
            // pros::lcd::print(3, "OOB || CURR VALUE: %i", indexToRun);
        }
    }
    

    // Initializes the drivebase
    inline static void startAll() {
    }


    // Prints out the cordinates of the bot depending on the PTO state.
    inline static void printCords() {
    }
    

    // Runs the current route index it is on
    inline static void run() {
        pros::Clock::time_point start = pros::Clock::now();
        routePointers[0]();
        
        pros::Clock::time_point end = pros::Clock::now();

        pros::Clock::duration duration = end - start;
        pros::delay(50);
        master.clear();
        pros::delay(50);
        master.set_text(0, 1, std::to_string(duration.count()));
        
    }
};

class Routes{
    public:
        // Creates static functions for autonomous routes.
        // Not all are named according to the routes they represent.
        // They are generic names so that a name can be attached to them
        // during auto selection.
        void static placehold1(); 
        void static placehold1Mir(); 
        void static placehold2(); 
        void static placehold2Mir(); 
        void static placehold3();
        void static placehold3Mir();
        void static placehold4();
        void static placehold4Mir();
        void static placehold5();
        void static placehold5Mir();
        void static placehold6();
        void static placehold6Mir();
        void static oldplacehold6();
        void static oldplacehold6Mir();
        void static placehold7();
        void static placehold7Mir();
        void static placehold8();
        void static placehold8Mir();
        void static placehold9();
        void static placehold10();
        void static placehold11();
        void static placehold12();
        void static placehold12Mir();
        void static placehold13();
        void static placehold13Mir();
        void static skills();

    
     inline static void initall() {
        // Autos added below are self explanatory due to the route name attached to them.
        // The names attached are within quotes
        // AutoSelector::add("RED Goal SIDE 5 ring (ELIM)", placehold12); // DON'T RUN
        // AutoSelector::add("BLUE Goal SIDE 5 ring (ELIM)", placehold12Mir); // DON'T RUN

        AutoSelector::add("Ring side RED (ELIM 6+1)", placehold5Mir); // done plz | need verification
        AutoSelector::add("Ring side BLUE (ELIM 6+1 OLD)", placehold5); // done plz | test body

        AutoSelector::add("Alliance Stake Goal Side BLUE (ELIM 1+4)", placehold7); //done plz
        // Corner in is 100ms less | Back out of corner is 12v instead of 11v
        AutoSelector::add("Alliance Stake Goal Side RED (ELIM 1+4)", placehold7Mir); // done plz

        AutoSelector::add("Ring side RED (QUAL 6+1)", placehold3Mir); // ending sketch. done? | need verification
        AutoSelector::add("Ring side BLUE (QUAL 6+1 NEW)", placehold3); // good

        AutoSelector::add("Regional Solo BLUE (HOME)", oldplacehold6); // done plz | need verification      
        AutoSelector::add("Regional Solo Red (HOME)", oldplacehold6Mir); // done plz

        AutoSelector::add("Alliance Stake Goal Side BLUE (QUAL 1+4)", placehold1); // good
        AutoSelector::add("Alliance Stake Goal Side RED (QUAL 1+4)", placehold1Mir); // doinker drop delay?

        AutoSelector::add("RED Goal SIDE 5 ring (QUAL)", placehold2); // Done plz
        AutoSelector::add("BLUE Goal SIDE 5 ring (QUAL)", placehold2Mir); // Done plz

        AutoSelector::add("skills", skills); // done plz

        AutoSelector::add("Goal Rush RED (ELIM)", placehold13); // end pt? 
        AutoSelector::add("Goal Rush BLUE (ELIM)", placehold13Mir); //good sometimes miss first ring 

        AutoSelector::add("RED Goal SIDE 5 ring (ELIM WALL STAKE END)", placehold8); // ???
        AutoSelector::add("BLUE Goal SIDE 5 ring (ELIM WALL STAKE END)", placehold8Mir); // ???
         

        // vvvvvvvv NOT IN USE vvvvvvvv

        AutoSelector::add("Ring side BLUE (STAKE AWP 5+1)", placehold4); 
        AutoSelector::add("Ring side RED (STAKE AWP 5+1)", placehold4Mir);

        AutoSelector::add("Mogo side BLUE (END STAKE)", placehold5);         
        
        AutoSelector::add("EZ BLUE", placehold11); // ???'
        

        
        
        // AutoSelector::add("Solo BLUE", placehold1);
        // AutoSelector::add("Solo RED MIRROR SHORT", placehold2);        

        
     }
     
};