#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "accel.h"
// #include "potentiometre.h"
#include "mqtt_publisher.h"
#include <math.h>

#define PUSH_BUTTON  GPIO_NUM_0


Accel accel;
Accel gyro;

static const char *TAG = "MAIN";

#define dt 0.500   // Temps d'échantillonnage en secondes
#define Q_angle 0.5 // Erreur de mesure de l'angle
#define Q_gyro 0.1 // Erreur de mesure de la vitesse angulaire
#define R_angle 0.03 // Erreur du modèle d'angle

float x_angle = 0, x_bias = 0; // État du filtre de X
float PX[2][2] = {{0, 0},{0, 0}}; // Matrice de covariance de l'état de X
float y_angle = 0, y_bias = 0; // État du filtre de Y
float PY[2][2] = {{0, 0},{0, 0}}; // Matrice de covariance de l'état de Y

void kalman_filter(float acc_angle, float gyro_rate, float P[2][2], float *angle, float *bias)
{
    // Prédiction de l'état
    *angle += dt * (gyro_rate - *bias);
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1]+= dt * Q_gyro;

    // Mise à jour de l'état en utilisant la mesure de l'accéléromètre
    float y = acc_angle - *angle;
    float S = P[0][0] + R_angle;
    float K_0 = P[0][0] / S;
    float K_1 = P[1][0] / S;
    *angle += K_0 * y;
    *bias += K_1 * y;
    P[0][0] -= K_0 * P[0][0];
    P[0][1] -= K_0 * P[0][1];
    P[1][0] -= K_1 * P[0][0];
    P[1][1] -= K_1 * P[0][1];
}

void app_main(void)
{   
    //------------- GPIO INIT ---------------//
    gpio_set_direction(PUSH_BUTTON, GPIO_MODE_INPUT);

    //------------- ACCEL INIT ---------------//
    accel_init();
    double accel_x_angle, accel_y_angle;

    //------------- MQTT INIT ---------------//
    mqtt_init();

    //------------- POTENTIOMETER INIT ---------------//
    int potentiometer_value3, potentiometer_value4, potentiometer_value5, 
    potentiometer_value6, potentiometer_value7;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

    ESP_LOGI(TAG, "Press the button on the board to start.");
    while(gpio_get_level(PUSH_BUTTON) != 0) 
    {
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Program will start. (Release press on the button) \n Press again button to stop when prgram executes");
    vTaskDelay(2000/portTICK_PERIOD_MS);

    while(gpio_get_level(PUSH_BUTTON) != 0) 
    {
        potentiometer_value3 = adc1_get_raw(ADC1_CHANNEL_3);
        potentiometer_value4 = adc1_get_raw(ADC1_CHANNEL_4);
        potentiometer_value5 = adc1_get_raw(ADC1_CHANNEL_5);
        potentiometer_value6 = adc1_get_raw(ADC1_CHANNEL_6);
        potentiometer_value7 = adc1_get_raw(ADC1_CHANNEL_7);
        send_value(2,potentiometer_value3);
        send_value(3,potentiometer_value4);
        send_value(4,potentiometer_value5);
        send_value(5,potentiometer_value6);
        send_value(6,potentiometer_value7);

        accel_read_gyro_values(&gyro);
        accel_read_values(&accel);
        // get angle from accelerometer values
        accel_y_angle = atan2((double) accel.y, sqrt(pow((double) accel.x, 2) + pow((double) accel.z, 2))) * 180.0 / M_PI;
        accel_x_angle = atan2((double) accel.x, sqrt(pow((double) accel.y, 2) + pow((double) accel.z, 2))) * 180.0 / M_PI;
        // kalman filter application
        kalman_filter(accel_y_angle, gyro.y, PY, &y_angle, &y_bias);
        kalman_filter(accel_x_angle, gyro.x, PX, &x_angle, &x_bias);
        // print angle
        ESP_LOGI(TAG, "X : %3.4f \t Y : %3.4f", x_angle, y_angle);
        int x_value_to_send = (int) (x_angle + 180); // -180/180 -> 0/360
        int y_value_to_send = (int) (y_angle + 180);
        send_value(0, x_value_to_send);
        send_value(1, y_value_to_send);

        // wait
        vTaskDelay(dt*1000 / portTICK_PERIOD_MS); // in milliseconds
        
    }

    accel_deinit();
    return;
}
