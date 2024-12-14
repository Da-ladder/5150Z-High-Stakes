#include "mogoDetect.h"
#include <cmath>




int MogoUtils::mogoX = 0;
int MogoUtils::mogoY = 0;
int RedRingUtil::ringX = 0;
int RedRingUtil::ringY = 0;

pros::vision_object_s_t StakeVision::stakeArray[10];
int StakeVision::stakeLowestY[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int StakeVision::stakeMidX[10];
int StakeVision::stakeMidY[10];
int StakeVision::objectsDetected = 0;


/*
    vision_object_s_t includes:
    
    /// Object signature
	    uint16_t signature;
	/// Object type, e.g. normal, color code, or line detection
	    vision_object_type_e_t type;
	/// Left boundary coordinate of the object
	    int16_t left_coord;
	/// Top boundary coordinate of the object
	    int16_t top_coord;
	/// Width of the object
	    int16_t width;
	/// Height of the object
	    int16_t height;
	/// Angle of a color code object in 0.1 degree units (e.g. 10 -> 1 degree, 155 -> 15.5 degrees)
	    uint16_t angle;
	/// Coordinates of the middle of the object (computed from the values above)
	    int16_t x_middle_coord;
	/// Coordinates of the middle of the object (computed from the values above)
	    int16_t y_middle_coord;
*/

    