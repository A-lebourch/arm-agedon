from mpu6050 import accel
from machine import SoftI2C, Pin

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=400000)
accel = accel(i2c)

accel.val_test()