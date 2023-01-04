#include <stdio.h>
#include <stdint.h>
#include "accel.h"

// VECTOR_3D

vector_ijk vector_3d_initialize(float a, float b, float c)
{
    vector_ijk v;
    v.a = a;
    v.b = b;
    v.c = c;
    return v;
}

vector_ijk vector_3d_sum(vector_ijk v1, vector_ijk v2)
{
    vector_ijk v;
    v.a = v1.a + v2.a;
    v.b = v1.b + v2.b;
    v.c = v1.c + v2.c;
    return v;
}

vector_ijk vector_3d_difference(vector_ijk v1, vector_ijk v2)
{
    vector_ijk v;
    v.a = v1.a - v2.a;
    v.b = v1.b - v2.b;
    v.c = v1.c - v2.c;
    return v;
}

float vector_3d_dot_product(vector_ijk v1, vector_ijk v2)
{
    return (v1.a*v2.a + v1.b*v2.b + v1.c*v2.c);

}


vector_ijk vector_3d_cross_product(vector_ijk v1, vector_ijk v2)
{
    vector_ijk v;
    v.a = v1.b*v2.c - v1.c*v2.b;
    v.b = v1.c*v2.a - v1.a*v2.c;
    v.c = v1.a*v2.b - v1.b*v2.a;
    return v;
}

vector_ijk vector_3d_normalize(vector_ijk v1)
{
    vector_ijk v2;
    float one_by_sqrt;
    one_by_sqrt = InvSqrt(v1.a*v1.a + v1.b*v1.b + v1.c*v1.c);
    v2.a = v1.a*one_by_sqrt;
    v2.b = v1.b*one_by_sqrt;
    v2.c = v1.c*one_by_sqrt;
    return v2;
}

vector_ijk vector_3d_scale(vector_ijk v1, float scale)
{
    vector_ijk v2;
    v2.a = v1.a*scale;
    v2.b = v1.b*scale;
    v2.c = v1.c*scale;
    return v2;
}

float InvSqrt(float x)
{
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float tmp = *(float*)&i;
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

// QUATERNION

Quaternion quaternion_initialize(float a, float b, float c, float d)
{
    Quaternion q;
    q.a = a;
    q.b = b;
    q.c = c;
    q.d = d;
    return q;
}

Quaternion quaternion_product(Quaternion q1, Quaternion q2)
{
    //q = q1*q2
    Quaternion q;
    q.a = (q1.a*q2.a) - (q1.b*q2.b) - (q1.c*q2.c) - (q1.d*q2.d);
    q.b = (q1.a*q2.b) + (q1.b*q2.a) + (q1.c*q2.d) - (q1.d*q2.c);
    q.c = (q1.a*q2.c) - (q1.b*q2.d) + (q1.c*q2.a) + (q1.d*q2.b);
    q.d = (q1.a*q2.d) + (q1.b*q2.c) - (q1.c*q2.b) + (q1.d*q2.a);
    return q;
}

Quaternion quaternion_conjugate(Quaternion q1)
{
    Quaternion q2;
    q2.a = q1.a;
    q2.b = -q1.b;
    q2.c = -q1.c;
    q2.d = -q1.d;
    return q2;
}

Quaternion quaternion_normalize(Quaternion q1)
{
    Quaternion q2;
    float one_by_sqrt;
    one_by_sqrt = InvSqrt(q1.a*q1.a + q1.b*q1.b + q1.c*q1.c + q1.d*q1.d);
    q2.a = q1.a*one_by_sqrt;
    q2.b = q1.b*one_by_sqrt;
    q2.c = q1.c*one_by_sqrt;
    q2.d = q1.d*one_by_sqrt;
    return q2;
}

Quaternion quaternion_between_vectors(vector_ijk v1, vector_ijk v2)
{
    // rotates from v1 to v2
    vector_ijk v1_norm = vector_3d_normalize(v1);
    vector_ijk v2_norm = vector_3d_normalize(v2);
    vector_ijk half_way_vector = vector_3d_normalize(vector_3d_sum(v1_norm,v2_norm));
    float angle = vector_3d_dot_product(v1_norm, half_way_vector);
    vector_ijk axis = vector_3d_cross_product(v1_norm, half_way_vector);
    Quaternion result = quaternion_initialize(angle, axis.a, axis.b,axis.c);
    return result;
}

vector_ijk quaternion_rotate_vector(vector_ijk v, Quaternion q)
{
    Quaternion quaternion_vector = quaternion_initialize(0.0, v.a, v.b, v.c);
    Quaternion q_inverse = quaternion_conjugate(q);
    Quaternion quaternion_rotated_vector = quaternion_product(quaternion_product(q, quaternion_vector),q_inverse);
    vector_ijk rotated = vector_3d_initialize(quaternion_rotated_vector.b,quaternion_rotated_vector.c,quaternion_rotated_vector.d);
    return rotated;
}

euler_angles quaternion_to_euler_angles(Quaternion q)
{
    euler_angles result;
    double q0 = q.a;
    double q1 = q.b;
    double q2 = q.c;
    double q3 = q.d;
    result.roll = atan2(2*(q0*q1 + q2*q3),1 - 2*(q1*q1 + q2*q2))*180/3.14;
    result.pitch = asin(2*(q0*q2 - q3*q1))*180/3.14;
    if (q.d==0)
        result.yaw = 0.0;
    else
        result.yaw = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3))*180/3.14;
    return result;
}

// SENSOR_PROCESSING_LIB


Quaternion quaternion_from_accelerometer(float ax, float ay, float az)
{
    /*vector_ijk gravity = vector_3d_initialize(0.0f, 0.0f, -1.0f);
    vector_ijk accelerometer = vector_3d_initialize(ax, ay, az);
    Quaternion orientation = quaternion_between_vectors(gravity,accelerometer);
    return orientation;*/
    float norm_u_norm_v = 1.0;
    float cos_theta = -1.0*az;
    //float half_cos = sqrt(0.5*(1.0 + cos_theta));
    float half_cos = 0.7071*sqrt(1.0 + cos_theta);
    Quaternion orientation;
    orientation.a = half_cos;
    //float temp = 1/(2.0*half_cos);
    float temp = 0.5/half_cos;
    orientation.b = -ay*temp;
    orientation.c = ax*temp;
    orientation.d = 0.0;
    return orientation;
}

Quaternion quaternion_from_gyro(float wx, float wy, float wz, float time)
{
    // wx,wy,wz in radians per second: time in seconds
    float alpha = 0.5*time;
    float a,b,c,d;
    b = alpha*(-wx);
    c = alpha*(-wy);
    d = alpha*(-wz);
    a = 1 - 0.5*(b*b+c*c+d*d);
    Quaternion result = quaternion_initialize(a,b,c,d);
    return result;
}

float fusion_coeffecient(vector_ijk virtual_gravity, vector_ijk sensor_gravity)
{
    float dot = vector_3d_dot_product(sensor_gravity,virtual_gravity);

    if (dot<=0.96)
        return 40.0;

    return 10.0;
}

vector_ijk sensor_gravity_normalized(int16_t ax, int16_t ay, int16_t az)
{
    vector_ijk result;
    result.a = ax;
    result.b = ay;
    result.c = az;
    result = vector_3d_normalize(result);
    return result;
}

vector_ijk fuse_vector(vector_ijk virtual_gravity, vector_ijk sensor_gravity)
{
    float fusion = fusion_coeffecient(virtual_gravity, sensor_gravity);
    virtual_gravity = vector_3d_scale(virtual_gravity,fusion);
    vector_ijk result = vector_3d_sum(virtual_gravity,sensor_gravity);
    result = vector_3d_normalize(result);
    return result;
}

vector_ijk update_gravity_vector(vector_ijk gravity_vector,float wx,float wy,float wz,float delta)
{
    Quaternion q_gyro = quaternion_from_gyro(wx,wy,wz,delta);
    gravity_vector = quaternion_rotate_vector(gravity_vector,q_gyro);
    return gravity_vector;
}

vector_ijk update_fused_vector(vector_ijk fused_vector, int16_t ax, int16_t ay, int16_t az,float wx,float wy,float wz,float delta)
{
    vector_ijk virtual_gravity = update_gravity_vector(fused_vector,wx,wy,wz,delta);
    vector_ijk sensor_gravity = sensor_gravity_normalized(ax,ay,az);
    fused_vector = fuse_vector(virtual_gravity,sensor_gravity);
    return fused_vector;
}
