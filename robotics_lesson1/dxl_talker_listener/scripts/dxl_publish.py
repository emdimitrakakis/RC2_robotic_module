#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32

def dxl_talker():
    pub = rospy.Publisher('motor_value', Int32, queue_size=10)
    rospy.init_node('dxl_talker', anonymous=True)
    
    rate = rospy.Rate(0.5) # 10hz
    
    m_value = 100

    while not rospy.is_shutdown():
    	rospy.loginfo("Motor value is %d", m_value)
        pub.publish(m_value)

        if m_value < 3000:
        	m_value += 100

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl_talker()
    except rospy.ROSInterruptException:
        pass
