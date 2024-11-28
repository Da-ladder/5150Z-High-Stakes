#include "mogoDetect.h"


int MogoUtils::mogoX = 0;
int MogoUtils::mogoY = 0;
int RedRingUtil::ringX = 0;
int RedRingUtil::ringY = 0;

pros::vision_object_s_t StakeVision::stakeArray[10];
int StakeVision::stakeLowestY[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int StakeVision::stakeMidX[10];
int StakeVision::stakeMidY[10];
int StakeVision::objectsDetected = 0;
