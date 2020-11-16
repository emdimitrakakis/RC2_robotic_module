#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32

def dxl_talker():
    pub = rospy.Publisher('motor_pwm_value', Int32, queue_size=10)
    rospy.init_node('dxl_talker', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        m_value = input("Enter motor PWM value (enter -1 to terminate):")

        if m_value == -1:
            break;

        while m_value > 885 or m_value < -885:
            print("Motor PWM value should be between -885 and 885")
            m_value = input("Enter motor PWM value (enter -1 to terminate):")

    	rospy.loginfo("Motor PWM value is %d", m_value)
        pub.publish(m_value)

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl_talker()
    except rospy.ROSInterruptException:
        pass
