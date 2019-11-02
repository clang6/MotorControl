#ifndef CURRENTCONTROL__H__
#define CURRENTCONTROL__H__

#define IMAX 100

// Global variables:
volatile int DutyCycle; // must be less than PR3+1 (PR3 = 4000)
volatile int Direction; // 0 (FWD) or 1 (REV)
volatile float CurrentKp;		// Current controller proportional gain
volatile float CurrentKi;		// Current controller integral gain
volatile int CurrentWaveform[IMAX]; // ITEST waveform
//Global variables used for ITEST:
volatile int ITESTcounter;	// counter used in current control ISR for ITEST
volatile int CurrentReference[IMAX];
volatile int CurrentActual[IMAX];

// Global variables used by HOLD:
volatile int CurrentRefHold;
volatile int CurrentActHold;
volatile int CurrentErrorHold;

volatile int CurrentError;
volatile int CurrentErrInt;
// HOLD re-uses CurrentError and CurrentErrInt

// Global variables used by TRACK:
volatile int CurrentRefTrack;
volatile int CurrentActTrack;
volatile int CurrentErrorTrack;

void CurrentControlSetup();
void makeITESTwaveform();
// void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentController();

#endif