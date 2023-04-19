#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NB_POT 2 // number of potentiometers accepted

// if init impossible return NULL
// else return potentiometer num

void my_task();