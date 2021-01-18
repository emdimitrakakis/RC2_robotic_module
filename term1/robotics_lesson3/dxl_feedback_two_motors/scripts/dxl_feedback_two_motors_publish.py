#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import rospy
from std_msgs.msg import Int32

def dxl_talker():
    pub = rospy.Publisher('motor_1_pwm_value', Int32, queue_size=10)
    rospy.init_node('dxl_talker', anonymous=True)
    
    rate = rospy.Rate(1000) # 10hz

    m_value = 0

    print("Press 'w' to increase PWM, 's' to decrease it, and 'ESC' to exit.")

    while not rospy.is_shutdown():

        user_input = getch()

        if user_input == chr(0x77) and m_value < 800:
            m_value = m_value + 50
        if user_input == chr(0x73) and m_value > 0:
            m_value = m_value - 50
        if user_input == chr(0x1b):        
            break;

        rospy.loginfo("Motor 1 PWM value is %d", m_value)
        pub.publish(m_value)

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl_talker()
    except rospy.ROSInterruptException:
        pass
