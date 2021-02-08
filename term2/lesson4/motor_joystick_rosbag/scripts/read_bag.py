#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosbag
from std_msgs.msg import Int32

def bag_read():
# for a specific time read, rosbag play 'motor_data.bag'

    rospy.init_node('motor_data_read')

    bag = rosbag.Bag('/home/maniosvm/catkin_ws/src/rc2_robotics/term2/lesson4/motor_joystick_rosbag/bagfiles/motor_data.bag')

    for topic, msg, t in bag.read_messages(topics = ['motor_value_topic']):
        print (str(msg) + ", and timestamp: " + str(t))

    bag.close()

if __name__ == '__main__':
    try:
        bag_read()
    except rospy.ROSInterruptException:
        pass
