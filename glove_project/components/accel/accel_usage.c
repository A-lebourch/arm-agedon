#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "accel.h"

#define PUSH_BUTTON  GPIO_NUM_0
Accel gyro; // gyro values

static const char *TAG = "MAIN";

void accel_usage(void)
{   
    //------------- GPIO INIT ---------------//
    gpio_set_direction(PUSH_BUTTON, GPIO_MODE_INPUT);
    
    //------------- ACCEL INIT ---------------//
    accel_init();

    while(gpio_get_level(PUSH_BUTTON) != 0) {
        
        /* Gyro values read */
        accel_read_gyro_values(&gyro);

        ESP_LOGI(TAG, "GYRO - \t X: %f \t Y: %f \t Z: %f\n", gyro.x, gyro.y, gyro.z);

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    accel_deinit();
    return;
}
