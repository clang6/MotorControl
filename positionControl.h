#ifndef POSITIONCONTROL__H__
#define POSITIONCONTROL__H__

#include "currentControl.h"

#define MAXTRACKLENGTH 2000 // at 200 Hz and 10s max time duration

volatile float PositionKp;
volatile float PositionKi;
volatile float PositionKd;
volatile int PositionActualDeg; // 1/10ths of degrees
volatile int PositionError;
volatile int PositionErrInt;
volatile int PositionErrPrev;
volatile int PositionErrDot;
volatile int PosDeg;

// global variables used in TRACK MODE:
volatile int TRACKPosRef[MAXTRACKLENGTH];
volatile int TRACKPosAct[MAXTRACKLENGTH];
volatile int TRACKLength;
volatile int TRACKcount;

void PositionControlSetup();

#endif