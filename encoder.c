#include "encoder.h"                   
#include <xc.h>

static int encoder_command(int read) { // send a command to the encoder chip
                                       // 0 = reset count to 32,768, 1 = return the count
  SPI4BUF = read;                      // send the command
  while (!SPI4STATbits.SPIRBF) { ; }   // wait for the response
  SPI4BUF;                             // garbage was transferred, ignore it
  SPI4BUF = 5;                         // write garbage, but the read will have the data
  while (!SPI4STATbits.SPIRBF) { ; }
  return SPI4BUF;
}

int encoder_ticks(void) {
  int ticks;
  ticks = encoder_command(1); // call it twice!
  ticks = encoder_command(1);
  if (ticks > 35456) {
    ticks = ticks - 3584;
  }
  else if (ticks < 30080) {
    ticks = ticks + 3584;
  }
  return ticks;
}

// int encoder_counts(void) {
//   return encoder_command(1);
// }

void encoder_init(void) {
  // SPI initialization for reading from the decoder chip
  SPI4CON = 0;              // stop and reset SPI4
  SPI4BUF;                  // read to clear the rx receive buffer
  SPI4BRG = 0x4;            // bit rate to 8 MHz, SPI4BRG = 80000000/(2*desired)-1
  SPI4STATbits.SPIROV = 0;  // clear the overflow
  SPI4CONbits.MSTEN = 1;    // master mode
  SPI4CONbits.MSSEN = 1;    // slave select enable
  SPI4CONbits.MODE16 = 1;   // 16 bit mode
  SPI4CONbits.MODE32 = 0; 
  SPI4CONbits.SMP = 1;      // sample at the end of the clock
  SPI4CONbits.ON = 1;       // turn SPI on
}

// you write functions to reset encoder and return angle in 1/10th degrees
// reset encoder count to 32768:
void encoder_reset(void) {
  encoder_command(0);
  // return encoder_command(0);
}

int encoder_angle(void) { // return angle in 1/10ths of degrees
  int angle;
  angle = (((int) (encoder_ticks()-32768))*2009)/1000; // 0.2009 degrees/count
  // wrap angle to [-180, 180]
  if (angle > 5400) {
    angle = angle - 7200;
  }
  else if (angle < -5400) {
    angle = angle + 7200;
  }
  return angle;
}