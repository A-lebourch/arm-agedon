#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "accel.h"

#define PUSH_BUTTON  GPIO_NUM_0
#define MIN_SERVO    0
#define MAX_SERVO    180
Gyro gyro;
Angle position; // en degrés
Gyro diffs;

static const char *TAG = "MAIN";

void update_position_angle(Gyro *gyro, Angle *pos) {
    accel_read_gyro_values(&gyro);
        diffs.X = gyro->X;
        diffs.Y = gyro->Y;
        diffs.Z = gyro->Z;
        vTaskDelay(10/portTICK_PERIOD_MS);
        accel_read_gyro_values(&gyro);
        diffs.X -= gyro->X;
        diffs.Y -= gyro->Y;
        diffs.Z -= gyro->Z;
        ESP_LOGI(TAG, "DIFFS - \t X: %d Y: %d Z: %d\n", diffs.X, diffs.Y, diffs.Z);
        // map diffs (ou gyro) avec position (-250 -> 250, 0 -> 180)
        
        position.X = (position.X + diffs.X);
        position.Y = (position.Y + diffs.Y);
        position.Z = (position.Z + diffs.Z);
        // position capé à 0, 180
        if (position.X < MIN_SERVO)
            position.X = MIN_SERVO;
        if (position.Y < MIN_SERVO)
            position.Y = MIN_SERVO;
        if (position.Z < MIN_SERVO)
            position.Z = MIN_SERVO;

        if (position.X > MAX_SERVO)
            position.X = MAX_SERVO;
        if (position.Y > MAX_SERVO)
            position.Y = MAX_SERVO;
        if (position.Z > MAX_SERVO)
            position.Z = MAX_SERVO;

}
// prendre deux mesures espacées de 10ms puis faire la différence
// diviser cette différence par le temps mis pour avoir l'angle à ajouter à la position 

void app_main(void)
{   
    //------------- GPIO INIT ---------------//
    gpio_set_direction(PUSH_BUTTON, GPIO_MODE_INPUT);

    //------------- ACCEL INIT ---------------//
    accel_init();

    ESP_LOGI(TAG, "Press the button on the board to set initial position. ");
    while(gpio_get_level(PUSH_BUTTON) != 0) {

        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    position.X = 0;
    position.Y = 0; 
    position.Z = 0;
    ESP_LOGI(TAG, "Position set, program begin. (Release press on the button)");
    vTaskDelay(2000/portTICK_PERIOD_MS);

    while(gpio_get_level(PUSH_BUTTON) != 0) {

        accel_read_gyro_values(&gyro);
        diffs.X = gyro.X;
        diffs.Y = gyro.Y;
        diffs.Z = gyro.Z;
        vTaskDelay(10/portTICK_PERIOD_MS);
        accel_read_gyro_values(&gyro);
        diffs.X -= gyro.X;
        diffs.Y -= gyro.Y;
        diffs.Z -= gyro.Z;
        ESP_LOGI(TAG, "DIFFS - \t X: %d Y: %d Z: %d\n", diffs.X, diffs.Y, diffs.Z);
        // map diffs (ou gyro) avec position (-250 -> 250, 0 -> 180)
        
        position.X = (position.X + diffs.X);
        position.Y = (position.Y + diffs.Y);
        position.Z = (position.Z + diffs.Z);
        // position capé à 0, 180
        // servo (servo1, position.X)
        // servo (servo2, position.Y)
        // Z on s'en fout on le bouge que sur 2 axes
        // poser l'accel sur le dessus de la main (axe x et y spécifiés sur le servo)
        ESP_LOGI(TAG, "POSITION - \t X: %d Y: %d Z: %d\n", position.X, position.Y, position.Z);
    }

    accel_deinit();
    return;
}
