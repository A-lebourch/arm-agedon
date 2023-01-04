#include <stdio.h>
#include <time.h>
#include "accel.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

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
        delta = 0.001*(clock()-Start);
        fused_vector = update_fused_vector(fused_vector,AcX,AcY,AcZ,wx,wy,wz,delta);
        printf("Rotation x : %f \nRotation y : %f\nRotation z : %f\n", wx, wy, wz);
    }
}
