
#include "servo.h"
ledc_channel_config_t servo_config[7] = {{.channel = 1,
                                          .gpio_num = SERVO1_GPIO,
                                          .speed_mode = LEDC_HIGH_SPEED_MODE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .duty = 0,
                                          .hpoint = 0},
                                         {.channel = 2,
                                          .gpio_num = SERVO2_GPIO,
                                          .speed_mode = LEDC_HIGH_SPEED_MODE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .duty = 0,
                                          .hpoint = 0},
                                          {//index
                                            .channel = 3,
                                          .gpio_num = SERVO3_GPIO,
                                          .speed_mode = LEDC_HIGH_SPEED_MODE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .duty = 0,
                                          .hpoint = 0},
                                          {.channel = 4,
                                          .gpio_num = SERVO4_GPIO,
                                          .speed_mode = LEDC_HIGH_SPEED_MODE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .duty = 0,
                                          .hpoint = 0},
                                          {.channel = 5,
                                          .gpio_num = SERVO5_GPIO,
                                          .speed_mode = LEDC_HIGH_SPEED_MODE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .duty = 0,
                                          .hpoint = 0},
                                          {.channel = 6,
                                          .gpio_num = SERVO6_GPIO,
                                          .speed_mode = LEDC_HIGH_SPEED_MODE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .duty = 0,
                                          .hpoint = 0},
                                          {.channel = 7,
                                          .gpio_num = SERVO7_GPIO,
                                          .speed_mode = LEDC_HIGH_SPEED_MODE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .duty = 0,
                                          .hpoint = 0}};
ledc_timer_config_t servo_timer_config = {
    .freq_hz = SERVO_FREQ,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_num = LEDC_TIMER_0};





int verifDirectionServo(int posActu, int posApres)
{
    int value = 0;

    if (posActu > posApres)
        value = -1; // diminuer la valeur

    else if (posActu < posApres)
        value = 1; // augmenter la valeur

    else if (posActu == posApres)
        value = 0; // ne pas bouger

    return value;
}

void servoConfig()
{
    ledc_channel_config(&servo_config[0]);
    ledc_channel_config(&servo_config[1]);
    ledc_channel_config(&servo_config[2]);
    ledc_channel_config(&servo_config[3]);
    ledc_channel_config(&servo_config[4]);
    ledc_channel_config(&servo_config[5]);
    ledc_channel_config(&servo_config[6]);
    ledc_timer_config(&servo_timer_config);
}

void mouvement_servo(int valeurTest,int num_servo, int value)
{
    ledc_set_duty(servo_config[num_servo].speed_mode, servo_config[num_servo].channel, value);
    ledc_update_duty(servo_config[num_servo].speed_mode, servo_config[num_servo].channel);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}




// fonction pour arrondir à la 10zaine
int arrondi(float n)
{
    // Smaller multiple
    int a = n / 10;
    a *= 10;

    // Larger multiple
    int b = a + 10;

    // Return of closest of two
    return (n - a > b - n) ? b : a;
}

// transforme la valeur du potar en la valeur du servo
int map(float valeur, int valMinDepart, int valMaxDepart, int valMinArrive, int valMaxArrive)
{
    float etenduDepart = valMaxDepart - valMinDepart;
    float etenduArrive = valMaxArrive - valMinArrive;
    float ratio = etenduArrive / etenduDepart;
    valeur -= valMinDepart;
    valeur *= ratio;
    valeur += valMinArrive;
    int value = round(valeur);
    value = arrondi(value);

    return value;
}
