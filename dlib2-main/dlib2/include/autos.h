#pragma once
#include "declarations.h"
#include "pros/llemu.hpp"
#include "mogoDetect.h"
#include "lift.h"

/**
 * @brief Creates a class so autos can be selected from the
 * potentiometer. Also includes the ability to add autos and
 * rename them independently of the function name on the fly. 
*/
class AutoSelector {
  private:
    // All functions and variables are static as these routes
    // should stay the same no matter what
    static pros::ADIPotentiometer pot; // The potentiometer to use
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
     * on the index which controls which route it is on. The function
     * will print OOB (Out of bounds) if the current index is larger
     * than the size of the vector which would cause a data abort
     * otherwise.
    */
    inline static void printPath() {
        if (indexToRun >= 0 && indexToRun < routeNames.size()) {
            pros::lcd::print(3, routeNames[indexToRun]);
        } else {
            pros::lcd::print(3, "OOB || CURR VALUE: %i", indexToRun);
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
        routePointers[indexToRun]();
    }
};

class Routes{
    public:
        // Creates static functions for autonomous routes.
        // Not all are named according to the routes they represent.
        // They are generic names so that a name can be attached to them
        // during auto selection.
        void static placehold1(); 
        void static placehold2(); 
        void static placehold3();
        void static placehold4();
        void static placehold5();
        void static placehold6();
        void static placehold7();
        void static placehold8();
        void static placehold9();
        void static placehold10();
        void static placehold11();
        void static skills();

    
     inline static void initall() {
        // Autos added below are self explanatory due to the route name attached to them.
        // The names attached are within quotes
        
        // AutoSelector::add("Blue elim right hard", placehold1);
        // AutoSelector::add("Blue right EZ", placehold7); //placehold3

        AutoSelector::add("skills", skills);
        AutoSelector::add("EZ BLUE", placehold11);

        AutoSelector::add("ELIM Red RUSH", placehold3); //good
        AutoSelector::add("ELIM blue RUSH MIRROR", placehold7); //good

        AutoSelector::add("Red Solo AWP", placehold1); //good
        AutoSelector::add("Blue Solo AWP", placehold5);

        AutoSelector::add("Elims BLUE MIRROR (5 ring)", placehold10); //good-ish
        AutoSelector::add("Elims RED (5 ring)", placehold4); //good

        

        

        

        
        
        // AutoSelector::add("Solo BLUE", placehold1);
        // AutoSelector::add("Solo RED MIRROR SHORT", placehold2);        

        
     }
     
};