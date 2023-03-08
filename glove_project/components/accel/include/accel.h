#ifdef __cplusplus
extern "C" {
#endif

#ifndef ACCEL_H
#define ACCEL_H

#include "driver/i2c.h"

//-------------MPU6050---------------//
#define MPU6050_SLAVE_ADDR                  0b1101000   /*!< Slave address of the MPU6050 (The LSB bit of the 7 bit address is determined by the logic level on pin AD0) */

#define MPU6050_GYRO_SENSOR_ADDR            0x43        /*!< Slave address of the MPU6050 gyroscope sensor */
#define MPU6050_GYRO_CONFIG_ADDR            0x1B        /*!< Slave address of the MPU6050 gyroscope config */

#define MPU6050_ACCEL_SENSOR_ADDR           0x3B        /*!< Slave address of the MPU6050 accelerometer sensor */
#define MPU6050_ACCEL_CONFIG_ADDR           0x1C        /*!< Slave address of the MPU6050 accelerometer config */

#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7

#define X_GYRO_ERROR                        0.6
#define Y_GYRO_ERROR                        -2
#define Z_GYRO_ERROR                        1.3

#define X_ACCEL_ERROR                       0.05
#define Y_ACCEL_ERROR                       0
#define Z_ACCEL_ERROR                       -0.05

#define MIN_GYRO                            -250
#define MAX_GYRO                            250

#define G                                   9.8067
#define MIN_ACCEL                           -2*G
#define MAX_ACCEL                           2*G

typedef struct Accel_t {
    float x, y, z;
} Accel;

typedef struct Angle_T {
    int16_t x, y, z;
} Angle;

//-------------FUNCTIONS---------------//

void accel_init(void);

void accel_read_values(Accel *accel);

void accel_read_gyro_values(Accel *gyro);

void accel_print_gyro_errors(void);

void accel_print_accel_errors(void);

void accel_deinit(void);

#endif // ACCEL_H

#ifdef __cplusplus
}
#endif
