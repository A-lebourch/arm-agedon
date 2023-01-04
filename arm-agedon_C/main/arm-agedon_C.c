#include <stdio.h>
#include "../includes/Quaternions/quaternion.h"
#include "../includes/Quaternions/sensor_processing_lib.h"
#include "../includes/Quaternions/vector_3d.h"

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX=0,AcY=0,AcZ=1,Tmp,GyX=0,GyY=0,GyZ=0,count=0;
unsigned long Start = 0,loop_start,temp2;
float delta,wx,wy,wz;
euler_angles angles;
vector_ijk fused_vector;
Quaternion q_acc;

void app_main(void)
{
    printf("Hello world!\n");
}
