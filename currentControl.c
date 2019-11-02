#include "currentControl.h"
//#include "utilities.h"
#include <xc.h>
#include <sys/attribs.h>

static void timer2Setup(void) {
  // Timer 2 setup:
  T2CONbits.TCKPS = 0b001;        // Timer 2 prescale factor of 1: (N=2)
  PR2 = 15999;                    // period = (PR2+1) * N * 12.5 ns = 0.0002 s, 5 kHz
  TMR2 = 0;                       // initialize TMR2 to 0
  T2CONbits.ON = 1;               // turn on Timer 2
}

static void PWMSetup(void) {
  // Timer3 and OC1 setup --> PWM setup:
  T3CONbits.TCKPS = 0;            // Timer3 prescaler N=0 (1:1)
  PR3 = 3999;                     // period = (PR3+1) * N * 12.5 ns = 50 us, 20 kHz
  TMR3 = 0;                       // initial TMR3 count is 0
  OC1CONbits.OCTSEL = 1;          // use Timer 3 for output comparison
  OC1CONbits.OCM = 0b110;         // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 3000;                   // duty cycle = OC1RS/(PR3+1) = 75%
  OC1R = 3000;                    // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;               // turn on Timer3
  OC1CONbits.ON = 1;              // turn on OC1
}

static void CurrentInterruptSetup(void) {
    // here is the setup for the Timer 2 interrupt:
    IPC2bits.T2IP = 5;              // step 4: interrupt priority 5
    IPC2bits.T2IS = 0;              // step 4: interrupt subpriority 0 (NU32 doesn't care about subpriority)
    IFS0bits.T2IF = 0;              // step 5: clear the int flag
    IEC0bits.T2IE = 1;              // step 6: enable T2 interrupt by setting IEC0<8>
}

void CurrentControlSetup(void) {
	timer2Setup();
	PWMSetup();
	CurrentInterruptSetup();
	DutyCycle = 1000; // 25% duty cycle
	TRISFbits.TRISF0 = 0; // pin F0 is a digital output
	LATFbits.LATF0 = 0; // initialize F0 to digital low
  CurrentError = 0;
  CurrentErrInt = 0;
  ITESTcounter = 0;
}

void makeITESTwaveform(void) {
  int i = 0, center = 0, A = 200; // 2 full cycles of square wave reference current trajectory, +/- 200 mA
  for (i = 0; i < IMAX; ++i) {
    if (i < 3*IMAX/4) {
      CurrentWaveform[i] = center + A;      // 1st half of 2nd cycle
      if (i < IMAX/2) {
        CurrentWaveform[i] = center - A;    // 2nd half of 2st cycle
        if (i < IMAX/4) {
          CurrentWaveform[i] = center + A;  // 1st half of 1st cycle
        }
      }
    }
    else {
      CurrentWaveform[i] = center - A;      // 2nd half of 2nd cycle
    }
  }
}
