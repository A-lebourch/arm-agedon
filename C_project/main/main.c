#include <stdio.h>
#include "quaternion.h"
#include "sensor_processing_lib.h"
#include "vector_3d.h"

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
    while (1){
        delta = 0.001*(millis()-Start);
        fused_vector = update_fused_vector(fused_vector,AcX,AcY,AcZ,wx,wy,wz,delta);
        printf("Rotation x : %d \nRotation y : %d\nRotation z : %d \n", wx, wy, wz);
    }
}
