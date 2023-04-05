#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "accel.h"
// #include "potentiometre.h"
#include "mqtt_publisher.h"

#define PUSH_BUTTON  GPIO_NUM_0
#define MIN_SERVO    0
#define MAX_SERVO    180
Gyro gyro;
Angle position; // en degrés
Gyro diffs;

static const char *TAG = "MAIN";

// map(-250 -> 250, 0 -> 180) for gyro
// void mapGyro(Gyro *gyro)
// {
//     int16_t ratio = abs(MAX_GYRO - MIN_GYRO) / abs(MAX_SERVO - MIN_SERVO);
//     gyro->X -= MIN_GYRO;
//     gyro->X *= ratio;
//     gyro->X += MAX_SERVO;
//     gyro->Y -= MIN_GYRO;
//     gyro->Y *= ratio;
//     gyro->Y += MAX_SERVO;
//     gyro->Z -= MIN_GYRO;
//     gyro->Z *= ratio;
//     gyro->Z += MAX_SERVO;
// }

// void update_position_angle(Gyro *gyro, Angle *pos) {
//     accel_read_gyro_values(gyro);
//     mapGyro(gyro);
//     diffs.X = gyro->X;
//     diffs.Y = gyro->Y;
//     diffs.Z = gyro->Z;
//     vTaskDelay(10/portTICK_PERIOD_MS);
//     accel_read_gyro_values(gyro);
//     mapGyro(gyro);
//     diffs.X -= gyro->X;
//     diffs.Y -= gyro->Y;
//     diffs.Z -= gyro->Z;
//     ESP_LOGI(TAG, "DIFFS - \t X: %d Y: %d Z: %d\n", diffs.X, diffs.Y, diffs.Z);
//     // diviser par 100 car 10/1000 secondes = 1/100
//     position.X = (position.X + diffs.X/5);
//     position.Y = (position.Y + diffs.Y/5);
//     position.Z = (position.Z + diffs.Z/5);
//     // position cap to 0, 180
//     if (position.X < MIN_SERVO)
//         position.X = MIN_SERVO;
//     if (position.Y < MIN_SERVO)
//         position.Y = MIN_SERVO;
//     if (position.Z < MIN_SERVO)
//         position.Z = MIN_SERVO;

//     if (position.X > MAX_SERVO)
//         position.X = MAX_SERVO;
//     if (position.Y > MAX_SERVO)
//         position.Y = MAX_SERVO;
//     if (position.Z > MAX_SERVO)
//         position.Z = MAX_SERVO;

// }
// prendre deux mesures espacées de 10ms puis faire la différence
// diviser cette différence par le temps mis pour avoir l'angle à ajouter à la position 

void app_main(void)
{   
    

    mqtt_init();
    
    // vTaskDelay(500/portTICK_PERIOD_MS);
    // printf("\n\n\nstart init pot\n");
    // vTaskDelay(1500/portTICK_PERIOD_MS);
    // int pot = pot_init(8);
    // int val = get_pot_value(pot);
    // printf("my value : %d\n",val);
    
    // printf("enter loop\n");
    // while(1)
    // {
    //     val = get_pot_value(pot);
    //     printf("my value : %d\n",val);
    //     vTaskDelay(500/portTICK_PERIOD_MS);
    // } 
    // //------------- GPIO INIT ---------------//
    // gpio_set_direction(PUSH_BUTTON, GPIO_MODE_INPUT);

    // //------------- ACCEL INIT ---------------//
    // accel_init();


    // ESP_LOGI(TAG, "Press the button on the board to set initial position. ");
    // while(gpio_get_level(PUSH_BUTTON) != 0) {
        
    //     /* Gyro values read */
    //     accel_read_gyro_values(&gyro);

    //     ESP_LOGI(TAG, "GYRO - \t X: %d Y: %d Z: %d\n", gyro.X, gyro.Y, gyro.Z);
    //     vTaskDelay(10/portTICK_PERIOD_MS);
    // }

    // accel_deinit();
    // return;
}
