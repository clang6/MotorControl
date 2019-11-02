#include "positionControl.h"
#include "currentControl.h"
//#include "utilities.h"
#include <xc.h>
#include <sys/attribs.h>

static void timer4Setup(void) {
  // Timer 2 setup:
  T4CONbits.TCKPS = 0b011;        // Timer 4 prescale factor of 1:8
  PR4 = 49999;                    // period = (PR4+1) * N * 12.5 ns = 0.005 s, 200 Hz
  TMR4 = 0;                       // initialize TMR2 to 0
  T4CONbits.ON = 1;               // turn on Timer 2
}

static void PositionInterruptSetup(void) {
    // here is the setup for the Timer 2 interrupt:
    IPC4bits.T4IP = 6;              // step 4: interrupt priority 6, higher priority than current control ISR
    IPC4bits.T4IS = 0;              // step 4: interrupt subpriority 0 (NU32 doesn't care about subpriority)
    IFS0bits.T4IF = 0;              // step 5: clear the int flag
    IEC0bits.T4IE = 1;              // step 6: enable T2 interrupt by setting IEC0<8>
}

void PositionControlSetup(void) {
	timer4Setup();
	PositionInterruptSetup();
  	PositionError=0;
  	PositionErrInt = 0;
  	PositionErrPrev = 0;
  	PositionErrDot = 0;
  	CurrentRefHold = 0;	// this is not a typo!
  	TRISFbits.TRISF1 = 0;	// digital output
  	LATFbits.LATF1 = 0;		// initialize low
}