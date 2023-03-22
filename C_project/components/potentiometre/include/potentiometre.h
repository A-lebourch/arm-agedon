#include <stdint.h>
#include "driver/adc.h"

#define NB_POT 2 // number of potentiometers accepted

// if init impossible return NULL
// else return potentiometer num
int pot_init(int adc_port); 

int16_t get_pot_value(int num_pot);