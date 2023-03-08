#include "accel.h"
#include "esp_log.h"

//-------------I2C---------------//
#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

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

//-----------------------------------------//

void accel_init(void) {

    //------------- I2C INIT ---------------//
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "I2C initialized successfully");
    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x68 */
    uint8_t who_am_i;
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, &who_am_i, sizeof(who_am_i)));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%X", who_am_i);
    ESP_LOGI(TAG, "%s", who_am_i==0x68 ? "I2C communication OK" : "I2C communication KO");

    //------------- ACCEL/GYRO INIT -------------//
    mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0); // enable accelerator
    mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_ADDR, 0x10); // +- 2g
    mpu6050_register_write_byte(MPU6050_GYRO_CONFIG_ADDR, 0x10); // +- 250°/s
}

static int8_t convert_gyro_value(uint8_t high, uint8_t low) {
    return ((high << 8) | low) / 131;
}

static int8_t convert_accel_value(uint8_t high, uint8_t low) {
    return ((high << 8) | low) / 16384;
}

void accel_read_gyro_values(Gyro *gyro) {

    uint8_t gyro_measurement[6];
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_GYRO_SENSOR_ADDR, gyro_measurement, sizeof(gyro_measurement)));
    gyro->X = convert_gyro_value(gyro_measurement[0], gyro_measurement[1]) + X_ERROR;
    gyro->Y = convert_gyro_value(gyro_measurement[2], gyro_measurement[3]) + Y_ERROR;
    gyro->Z = convert_gyro_value(gyro_measurement[4], gyro_measurement[5]) + Z_ERROR;
}

void accel_print_gyro_errors(void) {
    float Xerror=0, Yerror=0, Zerror=0;
    Gyro gyro;
    for (size_t i = 0; i < 200; i++)
    {
        accel_read_gyro_values(&gyro);
        Xerror += gyro.X;
        Yerror += gyro.Y; 
        Zerror += gyro.Z; 
    }
    Xerror/=200;
    Yerror/=200;
    Zerror/=200;
    ESP_LOGI(TAG, "Xerror: %lf \t Yerror: %lf \t Zerror: %lf\n", Xerror, Yerror, Zerror);
}

void accel_deinit(void) {
    mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0b00100000); // disable/sleep accelerator
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}