#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include <stdio.h>
// include other header files here
#include "encoder.h"
#include "utilities.h"
#include "adc.h"
#include "currentControl.h"
#include "positionControl.h"


#define BUF_SIZE 200

unsigned int adc_sample_convert(int pin);

void __ISR(_TIMER_4_VECTOR, IPL6SOFT) PositionController(void) { // _TIMER_4_VECTOR = 16
  IFS0bits.T4IF = 0;              // clear the interrupt flag
  switch (getMODE()) {
    case IDLE:
    {
      break;
    }
    case PWM:
    {
      break;
    }
    case ITEST:
    {
      break;
    }
    case HOLD: // MODE = HOLD
    {
      // go to angle PosDeg:
      PositionActualDeg = encoder_angle();
      PositionError = PosDeg - PositionActualDeg;
      PositionErrInt = PositionError + PositionErrInt;

      if (PositionErrInt > 400) {
        PositionErrInt = 400;
      }
      else if (PositionErrInt < -400) {
        PositionErrInt = -400;
      }

      PositionErrDot = PositionError - PositionErrPrev;
      CurrentRefHold = PositionKp*PositionError + PositionKi*PositionErrInt + PositionKd*PositionErrDot;

      PositionErrPrev = PositionError;
      break;
    }
    case TRACK: // MODE = TRACK
    {
      if (TRACKcount < TRACKLength) {
      // go to angle :
      TRACKPosAct[TRACKcount] = encoder_angle();
      PositionError = TRACKPosRef[TRACKcount] - TRACKPosAct[TRACKcount];
      PositionErrInt = PositionError + PositionErrInt;

      if (PositionErrInt > 400) {
        PositionErrInt = 400;
      }
      else if (PositionErrInt < -400) {
        PositionErrInt = -400;
      }

      PositionErrDot = PositionError - PositionErrPrev;
      CurrentRefTrack = PositionKp*PositionError + PositionKi*PositionErrInt + PositionKd*PositionErrDot;

      PositionErrPrev = PositionError;

      TRACKcount++;
      }
      else {
        // clear errors, again!
        PositionErrPrev = 0;
        PositionErrDot = 0;
        PositionErrInt = 0;
        CurrentError = 0;
        CurrentErrInt = 0;
        DutyCycle = 0;
        PosDeg = TRACKPosAct[TRACKcount]; // after TRACKing, HOLD to the last position in the reference trajectory
        setMODE(HOLD);
        TRACKcount = 0;
      }
      break;
    }
    default:
    {
      break;
    }
  }
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentController(void) { // _TIMER_4_VECTOR = 16
  int u; // PI current control action

  IFS0bits.T2IF = 0;              // clear the interrupt flag

  switch (getMODE()) { // MODEs are IDLE, PWM, ITEST, HOLD, TRACK
    case IDLE: // MODE = IDLE
      {
      OC1RS = 0; // H-bridge ENABLE = 0.  25%???
      LATFbits.LATF0 = 0; // H-bridge PHASE doesn't matter when ENABLE = 0
      DutyCycle = 0;
      break;
    }
    case PWM: // MODE = PWM
      {
      LATFbits.LATF0 = Direction;
      OC1RS = DutyCycle;
      break;
    }
    case ITEST: // MODE = ITEST
    {
      if (ITESTcounter < IMAX) {
        CurrentReference[ITESTcounter] = CurrentWaveform[ITESTcounter];
        CurrentActual[ITESTcounter] = adc_current(0);

        CurrentError = CurrentReference[ITESTcounter] - CurrentActual[ITESTcounter];
        CurrentErrInt = CurrentErrInt + CurrentError; // error integral
        u = CurrentKp*CurrentError + CurrentKi*CurrentErrInt; // PI current controller, u can be negative!

        Direction = 0;                // Direction = 0 spins the motor forward, set this as default
        if (u<0) {
          u = -u;                     // make u positive
          Direction = 1;              // Direction = 1 spins the motor in reverse
        }

        LATFbits.LATF0 = Direction;
        DutyCycle = u;
        OC1RS = DutyCycle;
        ITESTcounter++;
      }
      else {                          // ITEST is over
        setMODE(IDLE);
        ITESTcounter = 0;             // reset ITESTcounter
        CurrentError = 0;                    // reset error
        CurrentErrInt = 0;                   // reset error integral
      }
      break;
    }
    case HOLD: // MODE = HOLD
    {
      CurrentActHold = adc_current(0);

      CurrentError = CurrentRefHold - CurrentActHold;
      CurrentErrInt = CurrentErrInt + CurrentError; // error integral

      // Limit CurrentErrInt to prevent integrator windup:
        if (CurrentErrInt > 600) {
          CurrentErrInt = 600;
        }
        else if (CurrentErrInt < -600) {
          CurrentErrInt = -600;
        }

      u = CurrentKp*CurrentError + CurrentKi*CurrentErrInt; // PI current controller, u can be negative!

      Direction = 0;                // Direction = 0 spins the motor forward, set this as default
      if (u<0) {
        u = -u;                     // make u positive
        Direction = 1;              // Direction = 1 spins the motor in reverse
      }

      LATFbits.LATF0 = Direction;
      DutyCycle = u;

      // Deadband: duty cycles between -15% and +15% cannot overcome motor friction.
      // If DutyCycle is in this range, set it to 0.
      if (DutyCycle < 600) { // 600/4000 = 15%
        DutyCycle = 0;
      }

      // Duty cycle saturation:
      if (DutyCycle > 4000) {
        DutyCycle = 4000;
      }

      OC1RS = DutyCycle;
      break;
    }
    case TRACK: // MODE = TRACK
    {
      CurrentActTrack = adc_current(0);

      CurrentError = CurrentRefTrack - CurrentActTrack;
      CurrentErrInt = CurrentErrInt + CurrentError; // error integral

      // Limit CurrentErrInt to prevent integrator windup:
        if (CurrentErrInt > 600) {
          CurrentErrInt = 600;
        }
        else if (CurrentErrInt < -600) {
          CurrentErrInt = -600;
        }

      u = CurrentKp*CurrentError + CurrentKi*CurrentErrInt; // PI current controller, u can be negative!

      Direction = 0;                // Direction = 0 spins the motor forward, set this as default
      if (u<0) {
        u = -u;                     // make u positive
        Direction = 1;              // Direction = 1 spins the motor in reverse
      }

      LATFbits.LATF0 = Direction;
      DutyCycle = u;

      // Deadband: duty cycles between -15% and +15% cannot overcome motor friction.
      // If DutyCycle is in this range, set it to 0.
      if (DutyCycle < 600) { // 600/4000 = 15%
        DutyCycle = 0;
      }

      // Duty cycle saturation:
      if (DutyCycle > 4000) {
        DutyCycle = 4000;
      }

      OC1RS = DutyCycle;
      break;
    }
    default:
    {
      break;
    }
  }
}

int main()
{
  char buffer[BUF_SIZE];
  char lcdbuf[16];
  unsigned int adcval = 0;
  float kptemp = 0, kitemp = 0, kdtemp = 0;
  unsigned int duration = 0;
  int j;

  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;
  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  int degrees = 0;
  encoder_init();
  adc_init();
  setMODE(IDLE);
  CurrentControlSetup();
  makeITESTwaveform();


  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'a':                      // display ADC count
      {
        adcval = adc_counts(0); // read the averaged ADC value from pin AN0 (RB0) - 0-1023
        sprintf(buffer, "%d\r\n", adcval);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'b':                      // display ADC value from pin AN0 (RB0) as a current in mA
      {
        sprintf(buffer, "%d\r\n", adc_current(0));
        NU32_WriteUART3(buffer);
        break;
      }
      case 'c':                      // read encoder (counts)
      {
        sprintf(buffer, "%d\r\n", encoder_ticks());
        NU32_WriteUART3(buffer);     // send encoder count to client
        break;
      }
      case 'd':                      // read encoder (degrees)
      {
        degrees = encoder_angle();
        sprintf(buffer, "%d\r\n", degrees);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'e':                        // reset encoder count
      {
        encoder_reset();
        break;
      }
      case 'f':                        // prompt for PWM value (-100 to 100)
      {
        int n = 0;
        setMODE(PWM);
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &n);
        if (n<0) {
          DutyCycle = -40*n; // scale to 0-4000
          Direction = 1; // REV
        }
        else {
          DutyCycle = 40*n; // scale to 0-4000
          Direction = 0; // FWD
        }
        sprintf(buffer, "%d %d\r\n", DutyCycle, Direction);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'g':                        // set current gains
      {
        __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f %f", &kptemp, &kitemp);
        CurrentKp = kptemp;                  // copy local variables to globals used by ISR
        CurrentKi = kitemp;
        CurrentError = 0;
        CurrentErrInt = 0; // reset the error integral after getting new controller gains
        __builtin_enable_interrupts();  // only 2 simple C commands while ISRs disabled
        break;
      }
      case 'h':                        // get current gains
      {
        sprintf(buffer, "%f %f\r\n", CurrentKp, CurrentKi);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'i':                         // set position gains
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f %f %f", &kptemp, &kitemp, &kdtemp);
        __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
        PositionKp = kptemp;                  // copy local variables to globals used by ISR
        PositionKi = kitemp;
        PositionKd = kdtemp;
        PositionError = 0;
        PositionErrPrev = 0;
        PositionErrDot = 0;
        PositionErrInt = 0; // reset the error integral after getting new controller gains
        CurrentError = 0;
        CurrentErrInt = 0;
        DutyCycle = 0;
        __builtin_enable_interrupts();  // only 2 simple C commands while ISRs disabled
        break;
      }
      case 'j':                         // get position gains
      {
        sprintf(buffer, "%f %f %f\r\n", PositionKp, PositionKi, PositionKd);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'k':
      {
        int i;
        setMODE(ITEST);
        while(getMODE()==2) { // ITEST = 2
          ; // do nothing.
        }
        sprintf(buffer, "%d\r\n", IMAX);
        NU32_WriteUART3(buffer);
        for (i=0; i<IMAX; i++) {   // send plot data to MATLAB
                                  // when first number sent = 1, MATLAB knows we're done
        sprintf(buffer, "%d %d \r\n", CurrentActual[i], CurrentReference[i]);
        NU32_WriteUART3(buffer);
        }
        break;
      }
      case 'l':                        // go to angle (deg)
      {
        setMODE(IDLE);
        // encoder_reset();
        // PositionError = 0;
        PositionErrPrev = 0;
        PositionErrDot = 0;
        PositionErrInt = 0;
        CurrentError = 0;
        CurrentErrInt = 0;
        DutyCycle = 0;
        __builtin_disable_interrupts();
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &PosDeg);
        PosDeg = 10*PosDeg; // 1/10ths of a degree
        setMODE(HOLD);
        __builtin_enable_interrupts();
        break;
      }
      case 'm': // get a step trajectory from the client program
      {
        setMODE(IDLE);
        __builtin_disable_interrupts();
        for (j = 0; j < MAXTRACKLENGTH; ++j)
        {
          TRACKPosRef[j] = 0; // initialize the TRACK reference position array
          TRACKPosAct[j] = 0; // initialize the TRACK actual position array
        }
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer,"%d",&TRACKLength);
        for (j = 0; j < TRACKLength; ++j)
        {
          NU32_ReadUART3(buffer, BUF_SIZE);
          sscanf(buffer,"%d",&TRACKPosRef[j]);
        }
        __builtin_enable_interrupts();
        break;
      }
      case 'n':
      {
        setMODE(IDLE);
        __builtin_disable_interrupts();
        for (j = 0; j < MAXTRACKLENGTH; ++j)
        {
          TRACKPosRef[j] = 0; // initialize the TRACK reference position array
          TRACKPosAct[j] = 0; // initialize the TRACK actual position array
        }
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer,"%d",&TRACKLength);
        for (j = 0; j < TRACKLength; ++j)
        {
          NU32_ReadUART3(buffer, BUF_SIZE);
          sscanf(buffer,"%d",&TRACKPosRef[j]);
        }
        __builtin_enable_interrupts();
        break;
      }
      case 'o':
      {
        __builtin_disable_interrupts();
        setMODE(TRACK);
        TRACKcount = 0; // start counting
        PositionErrPrev = 0;
        PositionErrDot = 0;
        PositionErrInt = 0;
        CurrentError = 0;
        CurrentErrInt = 0;
        DutyCycle = 0;
        __builtin_enable_interrupts();
        while(getMODE()==4) { // TRACK = 4
          ; // do nothing.
        }
        // __builtin_disable_interrupts();
        sprintf(buffer, "%d\r\n", TRACKLength);
        NU32_WriteUART3(buffer);
        for (j=0; j<TRACKLength; j++) {   // send plot data to MATLAB
                                  // when first number sent = 1, MATLAB knows we're done
        sprintf(buffer, "%d %d\r\n", TRACKPosRef[j], TRACKPosAct[j]);
        NU32_WriteUART3(buffer);
        }
        // __builtin_enable_interrupts();
        break;
      }
      case 'z': // display position reference, actual, error NOW
      {
        sprintf(buffer, "Curr:%d, PWM:%d, RefP:%d, MeasP:%d, ErrP:%d, ErrInt:%d\r\n", CurrentRefHold, DutyCycle/4000, PosDeg, PositionActualDeg, PositionError, PositionErrInt);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'p':                        // unpower the motor
      {
        setMODE(IDLE);
        DutyCycle = 0;
        PositionErrInt = 0;
        PositionErrPrev = 0;
        PositionErrDot = 0;
        break;
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here.
        setMODE(IDLE);
        break;
      }
      case 'r':                        // get the present MODE
      {
        sprintf(buffer, "%d\r\n", getMODE());
        NU32_WriteUART3(buffer);
        break;
      }
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        setMODE(IDLE);
        break;
      }
    }

    return 0;
  }
}
