#pragma once
#include "declarations.h"
#include "pros/misc.h"

/**
 * @brief Wraps all pistons into a class which checks for button toggles and
 * can be controlled by auton
*/
class PistonControl {
  private:
    pros::Controller* controller; // The controller to track the button presses on 
    pros::ADIDigitalOut* piston; // The piston to control
    pros::controller_digital_e_t button; // The button to track
    int pistionState = 0; // Keeps -= of the state of the piston

  public:
    /**
     * @brief Sets up a wrapper for pistons and gives various utility functions for them.
     *
     * @param pistonbind The piston to control
     * @param control The controller to use or will default to the master controller
     * @param digitalPress The button to monitor for presses or will default to the left button
    */
    PistonControl(pros::ADIDigitalOut* pistonbind, pros::Controller* control = &master , 
                  pros::controller_digital_e_t digitalPress = pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT) {
      controller = control;
      piston = pistonbind;
      button = digitalPress;
    }

    
    /**
     * @brief Changes the button which is being monitered for a press
     *
     * @param digitalPress The button to change to
    */
    inline void changeButton(pros::controller_digital_e_t digitalPress) {
      button = digitalPress;
    }


    // Sets piston state using the stored pistionState variable
    inline void setPistionState() {
      piston->set_value(pistionState);
    }


    // Gets piston state using the stored pistionState variable
    inline int getPistionState() {
      return this->pistionState;
    }


    /**
     * @brief Allows piston state to be changed at will in the program
     * 
     * @param state The state to set the piston to
    */
    inline void overrideState(int state) {
      this->pistionState = state;
      setPistionState();
    }


    // Allows the piston to be toggled on and off through button presses
    inline void toggle() {
      if (controller->get_digital_new_press(button)) {
        if(pistionState == 1) {
          this->pistionState = 0;
        } else {
          this->pistionState = 1;
        }
        setPistionState();
      }
    }


    // Allows pistons to be enabled as long as the button is being held down
    inline void hold() {
      if (controller->get_digital(button)) {
        this->pistionState = 1;
      } else {
        this->pistionState = 0;
      }
        setPistionState();
    }


    // gets the current piston state
    inline int getPistonState() {
      return this->pistionState;
    }
};


// externs all pistons so that the program can control them from anywhere
extern PistonControl moClamp;
extern PistonControl liftIntake;
extern PistonControl cornerDeploy;
extern PistonControl odomExtract;
extern PistonControl colorPistion;
extern PistonControl rushClamp;