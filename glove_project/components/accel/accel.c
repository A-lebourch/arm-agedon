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
    mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0); // enable accelerometer
    /* uncomment to set specific range using 2nd parameter refering to datasheet */
    //mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_ADDR, 0x10); // +- 8g
    //mpu6050_register_write_byte(MPU6050_GYRO_CONFIG_ADDR, 0x10); // +- 1000°/s
}

/**
 * @brief Used to assembly high and low adress register to form the right value
 */
static float convert_gyro_value(uint8_t high, uint8_t low) {
    return (float) ((high << 8) | low) / 131;
}

/**
 * @brief Used to assembly high and low adress register to form the right value
 */
static float convert_accel_value(uint8_t high, uint8_t low) {
    return (float) ((high << 8) | low) / 16384;
}

static float map_accel_value(float n, uint16_t max_input, uint16_t max_output) {
    if (n > max_output)
        return -1*(max_input-n);
    else if (n == max_output)
        return 0;
    return n;
}

/**
 * @brief Map for accel conversions to negatives values
 */
static void map_accel(Accel *accel)
{
    accel->x = map_accel_value(accel->x, 4, 2);
    accel->y = map_accel_value(accel->y, 4, 2);
    accel->z = map_accel_value(accel->z, 4, 2);
}

/**
 * @brief Map for gyro conversions to negatives values
 */
static void map_gyro(Accel *gyro)
{
    gyro->x = map_accel_value(gyro->x, 500, 250);
    gyro->y = map_accel_value(gyro->y, 500, 250);
    gyro->z = map_accel_value(gyro->z, 500, 250);
}

void accel_read_values(Accel *accel) {

    uint8_t accel_measurement[6];
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_SENSOR_ADDR, accel_measurement, sizeof(accel_measurement)));
    accel->x = convert_accel_value(accel_measurement[0], accel_measurement[1]) - X_ACCEL_ERROR;
    accel->y = convert_accel_value(accel_measurement[2], accel_measurement[3]) - Y_ACCEL_ERROR;
    accel->z = convert_accel_value(accel_measurement[4], accel_measurement[5]) - Z_ACCEL_ERROR;
    map_accel(accel);
}

void accel_read_gyro_values(Accel *gyro) {

    uint8_t gyro_measurement[6];
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_GYRO_SENSOR_ADDR, gyro_measurement, sizeof(gyro_measurement)));
    gyro->x = convert_gyro_value(gyro_measurement[0], gyro_measurement[1]) - X_GYRO_ERROR;
    gyro->y = convert_gyro_value(gyro_measurement[2], gyro_measurement[3]) - Y_GYRO_ERROR;
    gyro->z = convert_gyro_value(gyro_measurement[4], gyro_measurement[5]) - Z_GYRO_ERROR;
    map_gyro(gyro);
}

void accel_print_accel_errors(void) {
    float Xerror=0, Yerror=0, Zerror=0;
    uint16_t nb_iter = 200;
    Accel accel;
    for (uint16_t i = 0; i < nb_iter; i++) {
        accel_read_values(&accel);
        Xerror += accel.x;
        Yerror += accel.y; 
        Zerror += accel.z; 
    }
    Xerror/=nb_iter;
    Yerror/=nb_iter;
    Zerror/=nb_iter;
    Zerror--; // car à la mesure, 1G sur l'axe Z
    ESP_LOGI(TAG, "Xerror: %lf \t Yerror: %lf \t Zerror: %lf\n", Xerror, Yerror, Zerror);
}

void accel_print_gyro_errors(void) {
    float Xerror=0, Yerror=0, Zerror=0;
    uint16_t nb_iter = 200;
    Accel gyro;
    for (uint16_t i = 0; i < nb_iter; i++) {
        accel_read_gyro_values(&gyro);
        Xerror += gyro.x;
        Yerror += gyro.y; 
        Zerror += gyro.z; 
    }
    Xerror/=nb_iter;
    Yerror/=nb_iter;
    Zerror/=nb_iter;
    ESP_LOGI(TAG, "Xerror: %lf \t Yerror: %lf \t Zerror: %lf\n", Xerror, Yerror, Zerror);
}


void accel_deinit(void) {
    mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0b00100000); // disable/sleep accelerometer
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}