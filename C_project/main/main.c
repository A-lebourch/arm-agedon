#include <stdio.h>
#include <time.h>
#include "accel.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

//-------------ACCEL SAMPLING---------------//
int16_t AcX=0,AcY=0,AcZ=1,Tmp,GyX=0,GyY=0,GyZ=0,count=0;
unsigned long Start = 0,loop_start,temp2;
float delta,wX,wY,wZ;
euler_angles angles;
vector_ijk fused_vector;
Quaternion q_acc;

//-------------I2C---------------//
#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

//-------------MPU6050---------------//
#define MPU6050_SLAVE_ADDR                  0b1101000   /*!< Slave address of the MPU6050 (The LSB bit of the 7 bit address is determined by the logic level on pin AD0) */

#define MPU6050_GYRO_SENSOR_ADDR            0x43        /*!< Slave address of the MPU6050 gyroscope sensor */
#define MPU6050_GYRO_CONFIG_ADDR            0x1B        /*!< Slave address of the MPU6050 gyroscope config */

#define MPU6050_ACCEL_SENSOR_ADDR           0x3B        /*!< Slave address of the MPU6050 accelerometer sensor */
#define MPU6050_ACCEL_CONFIG_ADDR           0x1C        /*!< Slave address of the MPU6050 accelerometer config */

#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7

#define PUSH_BUTTON                         GPIO_NUM_0
static const char *TAG = "i2c-accel";

static esp_err_t i2c_master_init(void){
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Read a sequence of bytes from a MPU6050 gyroscope registers
 */
static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SLAVE_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SLAVE_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

uint16_t convert_gyro_value(uint8_t high, uint8_t low){
    return ((high << 8) | low) / 131;
}

uint16_t convert_accel_value(uint8_t high, uint8_t low){
    return ((high << 8) | low) / 16384;
}
void app_main(void)
{   
    //------------- GPIO INIT ---------------//
    gpio_set_direction(PUSH_BUTTON, GPIO_MODE_INPUT);

    //------------- I2C INIT ---------------//
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "I2C initialized successfully");
    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x68 */
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, data, sizeof(data[0])));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%X", data[0]);
    ESP_LOGI(TAG, "%s", data[0]==0x68 ? "I2C communication OK" : "I2C communication KO");

    //------------- ACCEL/GYRO INIT -------------//
    mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0); // enable captor
    // mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_ADDR, 0x10); // +- 2g
    mpu6050_register_write_byte(MPU6050_GYRO_CONFIG_ADDR, 0x10); // +- 250°/s
    uint8_t gyro_measurement[6];
    // uint8_t accel_measurement[6];

    while(gpio_get_level(PUSH_BUTTON) != 0){

        /* Gyro values read */
        ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_GYRO_SENSOR_ADDR, gyro_measurement, sizeof(gyro_measurement)));
        GyX = convert_gyro_value(gyro_measurement[0], gyro_measurement[1]);
        GyY = convert_gyro_value(gyro_measurement[2], gyro_measurement[3]); 
        GyZ = convert_gyro_value(gyro_measurement[4], gyro_measurement[5]);
        /* Accel values read */
        // ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_SENSOR_ADDR, accel_measurement, sizeof(accel_measurement)));
        // AcX = convert_accel_value(accel_measurement[0], accel_measurement[1]);
        // AcY = convert_accel_value(accel_measurement[2], accel_measurement[3]); 
        // AcZ = convert_accel_value(accel_measurement[4], accel_measurement[5]);
        /* print */
        // ESP_LOGI(TAG, "AcX: %d \t AcY: %d \t AcZ: %d\n", AcX, AcY, AcZ);
        ESP_LOGI(TAG, "GyX: %d \t GyY: %d \t GyZ: %d\n", GyX, GyY, GyZ);

        vTaskDelay(100/portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");

    // while (1){
    //     delta = 0.001*(clock()-Start);
    //     fused_vector = update_fused_vector(fused_vector,AcX,AcY,AcZ,wx,wy,wz,delta);
    //     printf("Rotation x : %f \nRotation y : %f\nRotation z : %f\n", wx, wy, wz);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
}
