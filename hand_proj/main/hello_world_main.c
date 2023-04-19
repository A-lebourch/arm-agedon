/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "mqtt_subscriber.h"
#include "servo.h"
void app_main(void)
{   
    servoConfig();
    // mqtt_app_start();
    
    // servo_values = [1,0,0,0,0,0,0];
    int maVariable = 400;

    // for (int j = 0; j < 15; j++)
    while(1)
    {
        maVariable += 150;
        for (int i = 0; i < NOMBRE_SERVO_ET_POT; i++)
        {
            // int valeurTest = verifDirectionServo(ServoPosActuelle[i], maVariable);
            printf("mvt\n");
            mouvement_servo(1, i,maVariable);
            
            if (maVariable > 1000) maVariable = 400;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    while(1)
    {
        for (int i = 0; i < NOMBRE_SERVO_ET_POT; i++)
        {
            // int valeurTest = verifDirectionServo(ServoPosActuelle[i], maVariable);
            printf("\n c'est la : %d pour %d",get_servo_value(i),i);
            mouvement_servo(1, i,(int) get_servo_value(i));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
        }
    }

}



// void app_main()
// {
//     servoConfig();
//     // uint8_t list_potar[NOMBRE_SERVO_ET_POT] = {8, 9};

//     // adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_DB_11); // CONFIG LE CHANNEL SUR LEQUEL ON ECRIT

    
//     // int ServoPosActuelle[NOMBRE_SERVO_ET_POT];
//     // for (int i = 0; i < NOMBRE_SERVO_ET_POT; i++)
//     // {
//     //     ServoPosActuelle[i] =0;
//     // }
//     // int potentiometer_value = 22;
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);
//     // printf("Fonctionnement avec %d potar et %d servo \n", (sizeof(list_potar) / 4), (sizeof(list_potar) / 4));
//     int maVariable = 400;

//     for (int j = 0; j < 50; j++)
//     {
//         maVariable += 150;
//         for (int i = 0; i < NOMBRE_SERVO_ET_POT; i++)
//         {
//             // int valeurTest = verifDirectionServo(ServoPosActuelle[i], maVariable);
//             printf("mvt\n");
//             mouvement_servo(1, i,maVariable);
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//             if (maVariable > 1000) maVariable = 50;
//         }
//     }
// }
