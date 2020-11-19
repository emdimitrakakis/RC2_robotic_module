#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32MultiArray

def dxl_talker():
    pub = rospy.Publisher('motor_pwm_values', Int32MultiArray, queue_size=10)
    rospy.init_node('dxl_talker', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz

    m_value_array = Int32MultiArray()
    m_value_array.data = []

    while not rospy.is_shutdown():

        m_value1 = input("Enter motor 1 PWM value (enter -1 to terminate):")

        if m_value1 == -1:
            break;

        while m_value1 > 885 or m_value1 < -885:
            print("Motor 1 PWM value should be between -885 and 885")
            m_value1 = input("Enter motor PWM value (enter -1 to terminate):")

        m_value2 = input("Enter motor 2 PWM value (enter -1 to terminate):")

        if m_value2 == -1:
            break;

        while m_value2 > 885 or m_value2 < -885:
            print("Motor 2 PWM value should be between -885 and 885")
            m_value2 = input("Enter motor 2 PWM value (enter -1 to terminate):")
        
        m_value_array.data = [m_value1, m_value2]

        rospy.loginfo("Motor PWM values are %d, %d", m_value_array.data[0], m_value_array.data[1])
        pub.publish(m_value_array)

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl_talker()
    except rospy.ROSInterruptException:
        pass
