#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "accel.h"

#define PUSH_BUTTON  GPIO_NUM_0
Gyro gyro;

static const char *TAG = "MAIN";

void app_main(void)
{   
    //------------- GPIO INIT ---------------//
    gpio_set_direction(PUSH_BUTTON, GPIO_MODE_INPUT);

    //------------- ACCEL INIT ---------------//
    accel_init();

    while(gpio_get_level(PUSH_BUTTON) != 0) {
        
        /* Gyro values read */
        accel_read_gyro_values(&gyro);

        ESP_LOGI(TAG, "GYRO - \t X: %d Y: %d Z: %d\n", gyro.X, gyro.Y, gyro.Z);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    accel_deinit();
    return;
}
