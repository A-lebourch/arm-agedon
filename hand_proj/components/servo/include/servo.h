#ifdef __cplusplus
extern "C"
{
#endif

#ifndef servo_h
#define servo_h

#include <stdio.h>
#include <math.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define SERVO1_GPIO 12
#define SERVO2_GPIO 13
#define SERVO3_GPIO 14
#define SERVO4_GPIO 15
#define SERVO5_GPIO 25
#define SERVO6_GPIO 26
#define SERVO7_GPIO 27

#define SERVO_FREQ 50
#define MIN_PULSE_WIDTH 400
#define MAX_PULSE_WIDTH 1000

#define MIN_VALUE_POT 0
#define MAX_VALUE_POT 4095
#define NOMBRE_SERVO_ET_POT 7

// int list_potar[NOMBRE_SERVO_ET_POT] = {8, 9}; // valeur adc
// #ifndef list_potar

// #endif

int verifDirectionServo(int posActu, int posApres);

void servoConfig(void);

void mouvement_servo(int valeurTest, int num_servo, int value);

int arrondi(float n);

int map(float valeur, int valMinDepart, int valMaxDepart, int valMinArrive, int valMaxArrive);

#endif

#ifdef __cplusplus
}
#endif