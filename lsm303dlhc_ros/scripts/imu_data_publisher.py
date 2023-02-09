#!/usr/bin/env python3
import sys
import os
import time
import board
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag

script_dir = os.path.dirname( __file__ )
mymodule_dir = os.path.join( script_dir, '..', 'l3g')
sys.path.append( mymodule_dir )

import adafruit_l3g

import numpy as np
import rospy
from sensor_msgs.msg import Imu

# I2C connection:
i2c = board.I2C()
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
gyro = adafruit_l3g.L3GD20_I2C(i2c)


def read_data():
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    imu_msg = Imu()
    
    while not rospy.is_shutdown():
        # Read acceleration, magnetometer, gyroscope, temperature.
        accel_x, accel_y, accel_z = accel.acceleration
        gyro_x, gyro_y, gyro_z = gyro.gyro
        mag_x, mag_y, mag_z = mag.magnetic
        
        #Header
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        #linear accelerations from accelerometer
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        # angular velocities
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        pub.publish(imu_msg)
        rate.sleep()

        
if __name__ == '__main__':
    try:
        read_data()
    except rospy.ROSInterruptException:
        pass
