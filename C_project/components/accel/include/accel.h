#ifdef __cplusplus
extern "C" {
#endif

// VECTOR_3D

#ifndef VECTOR_3D_INCLUDED
#define VECTOR_3D_INCLUDED

#include <math.h>

typedef struct vector_ijk {

    float a;
    float b;
    float c;

    // v = ai + bj + ck

} vector_ijk;

vector_ijk vector_3d_initialize(float a, float b, float c);
vector_ijk vector_3d_sum(vector_ijk v1, vector_ijk v2);
vector_ijk vector_3d_difference(vector_ijk v1, vector_ijk v2);
float vector_3d_dot_product(vector_ijk v1, vector_ijk v2);
vector_ijk vector_3d_cross_product(vector_ijk v1, vector_ijk v2);
vector_ijk vector_3d_normalize(vector_ijk v);
vector_ijk vector_3d_scale(vector_ijk v1, float scale);
float InvSqrt(float x);

#endif

// QUATERNION

#ifndef QUATERNION_INCLUDED
#define QUATERNION_INCLUDED

typedef struct Quaternion {

    float a;
    float b;
    float c;
    float d;

    // q = a + bi + cj + dk

} Quaternion;

typedef struct euler_angles {

    float roll;
    float pitch;
    float yaw;

} euler_angles;

Quaternion quaternion_initialize(float a, float b, float c, float d);
Quaternion quaternion_product(Quaternion q1, Quaternion q2);
Quaternion quaternion_conjugate(Quaternion q);
Quaternion quaternion_normalize(Quaternion q);
Quaternion quaternion_between_vectors(vector_ijk v1, vector_ijk v2);
vector_ijk quaternion_rotate_vector(vector_ijk v, Quaternion q);
euler_angles quaternion_to_euler_angles(Quaternion q);

#endif

// SENSOR_PROCESSING

#ifndef SENSOR_PROCESSING_INCLUDED
#define SENSOR_PROCESSING_INCLUDED

#include<stdint.h>
#include<stdlib.h>

#define ACCEL_LOWER_LIMIT 6144
#define ACCEL_UPPER_LIMIT 10240

Quaternion quaternion_from_accelerometer(float ax, float ay, float az);
Quaternion quaternion_from_gyro(float wx, float wy, float wz, float time);
float fusion_coeffecient(vector_ijk virtual_gravity, vector_ijk sensor_gravity);
vector_ijk sensor_gravity_normalized(int16_t ax, int16_t ay, int16_t az);
vector_ijk fuse_vector(vector_ijk virtual_gravity, vector_ijk sensor_gravity);
vector_ijk update_gravity_vector(vector_ijk gravity_vector,float wx,float wy,float wz,float delta);
vector_ijk update_fused_vector(vector_ijk fused_vector, int16_t ax, int16_t ay, int16_t az,float wx,float wy,float wz,float delta);

#endif // SENSOR_FUSION_LIB_H_INCLUDED

#ifdef __cplusplus
}
#endif
