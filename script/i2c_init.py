#!/usr/bin/env python

import rospy
import math
import smbus2
import struct

def main():
    rospy.init_node('jy901_i2c_init_node')

    bus_id = rospy.get_param('~bus_id', 8)
    address = int(rospy.get_param('~address', '0x50'), 16)

    bus = smbus2.SMBus(self.__bus_id)

    # TODO: unfinished

if __name__ == '__main__':
    main()
