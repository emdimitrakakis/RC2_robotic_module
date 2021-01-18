#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

def callback(data):
    print("X:" + str(data.x) + ", Y:" + str(data.y) + ", Z: " + str(data.z))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("read_joystick", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
