#include "lift.h"

double LiftMngr::voltReq = 0;
bool LiftMngr::blockLiftThread = false;
double LiftMngr::holdLvl = 0;
int LiftMngr::time = 0;
double LiftMngr::prevVolt = 0;