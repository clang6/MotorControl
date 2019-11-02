#ifndef ENCODER__H__
#define ENCODER__H__

// Constants used by ADC:
// #define VOLTS_PER_COUNT (3.3/1024)
// #define CORE_TICK_TIME 25               // nanoseconds between core ticks
#define SAMPLE_TIME 10                  // 10 core timer ticks = 250 ns

void adc_init(); // initialize the ADC

unsigned int adc_sample_convert(int); // sample and convert

unsigned int adc_counts(int);	// take the average of the last 5 adc values

float adc_current(int pin); // convert ADC value to current in mA

#endif
