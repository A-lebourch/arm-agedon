#include "potentiometre.h"

static int pot_list[NB_POT]; // list of adc port 
static int last_pot_index = 0;

int pot_init(int adc_port) {
    if (last_pot_index >= sizeof(pot_list)/sizeof(int))
        return NULL;
    pot_list[last_pot_index] = adc_port;
    return last_pot_index++;
}

int get_pot_value(int num_pot) {
    int pot_value;

    adc2_get_raw(pot_list[num_pot], ADC_WIDTH_BIT_12, &pot_value);
    return pot_value;
}
