#include <stdio.h>
#include "NU32.h"
#include "utilities.h"

static volatile mode MODE;            	// Operating mode

void setMODE(mode newMODE) {  // Set mode
    MODE = newMODE;     // Update global MODE
}

mode getMODE() {  // Return mode
    return MODE;
}