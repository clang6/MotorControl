#include "adc.h"
#include <xc.h>


void adc_init(void) {
AD1PCFGbits.PCFG0 = 0;          // AN0 (RB0) is an adc pin
AD1CON3bits.ADCS = 2;           // ADC clock period is Tad = 2*(ADCS+1)*Tpb =
                                  //                          2*3*12.5ns = 75 ns
AD1CON1bits.ADON = 1;           // turn on A/D converter
}


float adc_current(int pin) {
	int adcval;
	adcval = adc_counts(pin);
	return (((float) (2.304*adcval)) - 1036.4);
}

unsigned int adc_counts(int pin){ // take the average of the last 5 adc values
  unsigned int adc1 = adc_sample_convert(pin);
  unsigned int adc2 = adc_sample_convert(pin);
  unsigned int adc3 = adc_sample_convert(pin);
  unsigned int adc4 = adc_sample_convert(pin);
  unsigned int adc5 = adc_sample_convert(pin);
  return (adc1+adc2+adc3+adc4+adc5)/5;
}

unsigned int adc_sample_convert(int pin) {  // sample & convert the value on the given
                                            // adc pin; the pin should be configured as an
                                            // analog input in AD1PCFG
  unsigned int elapsed = 0, finish_time = 0;
  AD1CHSbits.CH0SA = pin;                   // connect chosen pin to MUXA for sampling
  AD1CON1bits.SAMP = 1;                     // start sampling
  elapsed = _CP0_GET_COUNT();
  finish_time = elapsed + SAMPLE_TIME;
  while (_CP0_GET_COUNT() < finish_time) {
    ;                                       // sample for more than 250 ns
  }
  AD1CON1bits.SAMP = 0;                     // stop sampling and start converting
  while(!AD1CON1bits.DONE) {
    ;                                       // wait for the conversion process to finish
  }
  return ADC1BUF0;                          // read the buffer with the result
}
