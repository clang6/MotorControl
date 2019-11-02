#ifndef UTILITIES__H__
#define UTILITIES__H__

typedef enum {IDLE, PWM, ITEST, HOLD, TRACK} mode;    // define data structure containing modes

// MODE
mode getMODE();                                       		// Return the current operating mode
void setMODE(mode newMODE);                                 // Set operating mode

#endif
