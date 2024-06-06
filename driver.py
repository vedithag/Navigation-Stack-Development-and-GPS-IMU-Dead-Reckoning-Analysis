#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import serial
import numpy as np
from imu_driver.msg import imu_msg
from std_msgs.msg import Header

ser = serial.Serial('/dev/ttyUSB0', 115200)

def euler_to_quaternion(roll, pitch, yaw):
    # Converts roll, pitch, yaw (in radians) to quaternion (w, x, y, z) representation.
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz

def imu_data_recorder():
    rospy.init_node('imu_data_publisher')
    pub = rospy.Publisher('imu_data', imu_msg, queue_size=10)
    rate = rospy.Rate(40)

    while not rospy.is_shutdown():
        string_data = ser.readline().decode('utf-8').strip()
        if string_data.startswith('$VNYMR'):
            data = string_data.split(',')
            if len(data) == 13:
                header = Header()
                header.frame_id = 'IMU1_Frame'
                header.stamp = rospy.Time.now()

                imu_msg_data = imu_msg()
                imu_msg_data.header = header

                imu_msg_data.magnetic_field.x = float(data[4]) * 0.0001
                imu_msg_data.magnetic_field.y = float(data[5]) * 0.0001
                imu_msg_data.magnetic_field.z = float(data[6]) * 0.0001

                imu_msg_data.linear_acceleration.x = float(data[7])
                imu_msg_data.linear_acceleration.y = float(data[8])
                imu_msg_data.linear_acceleration.z = float(data[9])

                imu_msg_data.angular_velocity.x = float(data[10])
                imu_msg_data.angular_velocity.y = float(data[11])
                imu_msg_data.angular_velocity.z = float(data[12].split('*')[0])

                roll = float(data[3])
                pitch = float(data[2])
                yaw = float(data[1])
                qw, qx, qy, qz = euler_to_quaternion(np.radians(roll), np.radians(pitch), np.radians(yaw))
                imu_msg_data.orientation.x = qx
                imu_msg_data.orientation.y = qy
                imu_msg_data.orientation.z = qz
                imu_msg_data.orientation.w = qw

                imu_msg_data.raw_data = string_data

                rospy.loginfo(imu_msg_data)
                pub.publish(imu_msg_data)
        else:
            rospy.logwarn('Invalid data received: %s', string_data)

        rate.sleep()

if __name__ == '__main__':
    try:
        imu_data_recorder()
    except rospy.ROSInterruptException:
        pass

