#include "global.h"
#include "Parameter.h"
#include <EEPROM.h>
#include <Wire.h>
#include "pid.h"
#include "AHRS.h"
#include "outputs.h"

//#include "RemoteXY_Control.h"
//#include "WEP_APP.h"
//#include "NeoPixel.h"
/*
  #include "bme280.h"
*/

ESPCOPTER esp;

PID oto;
PID xOpt;
PID yOpt;

PID xMulti;
PID yMulti;

PID xMultiM;
PID yMultiM;

PID xLps;
PID yLps;

PID xGOT;
PID yGOT;

AHRS ahrs;


PID roll;
PID pitch;
PID yaw;




#include "autoOpt.h"
#include "setup.h"
#include "FlightControl.h"
// #include "loop.h"

