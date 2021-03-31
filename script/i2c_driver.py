#!/usr/bin/env python

import rospy
import math
import smbus2
import struct

from sensor_msgs.msg import Imu, MagneticField, NavSatFix

class WITMotionI2cDriver:
    def __init__(self):
        self.__bus_id = rospy.get_param('~bus_id', 8)
        self.__address = int(rospy.get_param('~address', '0x50'), 16)

        self.__frame_id = rospy.get_param('~frame_id', "imu_frame")

        self.__g = rospy.get_param('~g', 9.80665)

        self.__bus = smbus2.SMBus(self.__bus_id)

        self.__imu_pub = rospy.Publisher(self.__frame_id+'/raw', Imu, queue_size=10)
        self.__gps_pub = rospy.Publisher(self.__frame_id+'/gps', NavSatFix, queue_size=10)
        self.__magnetic_pub = rospy.Publisher(self.__frame_id+'/magnetic', MagneticField, queue_size=10)

    def exec_loop(self):
        imu_msg = Imu()

        data = self.__bus.read_i2c_block_data(self.__address, 0x34, 6)
        data_str = ''
        for i in xrange(6):
            data_str += chr(data[i])
        data_word = struct.unpack('<hhh', data_str)
        imu_msg.linear_acceleration.x = data_word[0] / 32768.0 * 16.0 * self.__g
        imu_msg.linear_acceleration.y = data_word[1] / 32768.0 * 16.0 * self.__g
        imu_msg.linear_acceleration.z = data_word[2] / 32768.0 * 16.0 * self.__g

        data = self.__bus.read_i2c_block_data(self.__address, 0x37, 6)
        data_str = ''
        for i in xrange(6):
            data_str += chr(data[i])
        data_word = struct.unpack('<hhh', data_str)
        imu_msg.angular_velocity.x = data_word[0] / 32768.0 * 2000.0 * math.pi / 180.0
        imu_msg.angular_velocity.y = data_word[1] / 32768.0 * 2000.0 * math.pi / 180.0
        imu_msg.angular_velocity.z = data_word[2] / 32768.0 * 2000.0 * math.pi / 180.0

        data = self.__bus.read_i2c_block_data(self.__address, 0x51, 8)
        data_str = ''
        for i in xrange(8):
            data_str += chr(data[i])
        data_word = struct.unpack('<hhhh', data_str)
        imu_msg.orientation.x = data_word[0] / 32768.0
        imu_msg.orientation.y = data_word[1] / 32768.0
        imu_msg.orientation.z = data_word[2] / 32768.0
        imu_msg.orientation.w = data_word[3] / 32768.0

        imu_msg.header.frame_id = self.__frame_id
        imu_msg.header.stamp = rospy.Time.now()
        self.__imu_pub.publish(imu_msg)

        magnetic_msg = MagneticField()
        data = self.__bus.read_i2c_block_data(self.__address, 0x3a, 6)
        data_str = ''
        for i in xrange(6):
            data_str += chr(data[i])
        data_word = struct.unpack('<hhh', data_str)
        magnetic_msg.magnetic_field.x = data_word[0]
        magnetic_msg.magnetic_field.y = data_word[1]
        magnetic_msg.magnetic_field.z = data_word[2]

        magnetic_msg.header.frame_id = self.__frame_id
        magnetic_msg.header.stamp = rospy.Time.now()
        self.__magnetic_pub.publish(magnetic_msg)


def main():
    rospy.init_node('jy901_i2c_node')

    driver = WITMotionI2cDriver()

    while not rospy.is_shutdown():
        driver.exec_loop()
        rospy.Rate(50).sleep()

if __name__ == '__main__':
    main()
