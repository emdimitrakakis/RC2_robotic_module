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
import rosbag
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_OPERATING_MODE    = 11		  # Control table value for operating mode
ADDR_PRO_TORQUE_ENABLE = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_PWM      = 100              # Control table address for goal PWM
ADDR_PRO_PRESENT_PWM   = 124              # Control table address for present PWM

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Define the rosbag
bag = rosbag.Bag('/home/maniosvm/catkin_ws/src/rc2_robotics/term2/lesson4/motor_joystick_rosbag/bagfiles/motor_data.bag', 'w')

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def dxl_operating_mode(operating_mode_value):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, operating_mode_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("PWM operating mode enabled.")

def dxl_torque_enable():
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

    print("Torque enabled.")

def dxl_write(sub_motor_pwm_value):
    # Write goal pwm position
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, sub_motor_pwm_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read present position
    dxl_present_pwm, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_PWM)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    rosbag_write(dxl_present_pwm)
    #rospy.loginfo("Present PWM is: %03d", dxl_present_pwm)

def dxl_torque_disable():
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()

    print("Torque disabled.")

def callback(data):
    #rospy.loginfo("I heard %d", data.data)
    if data.x > 600 and data.x <= 700:
	dxl_write(75)
    elif data.x > 700 and data.x <=800:
	dxl_write(150)
    elif data.x > 800:
	dxl_write(250)

    elif data.x < 400 and data.x >= 300:
	dxl_write(-75)
    elif data.x < 300 and data.x >= 200:
	dxl_write(-150)
    elif data.x < 200:
	dxl_write(-250)

    else:
	dxl_write(0)

def dxl_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_joystick_rosbag', anonymous=True)

    rospy.Subscriber("/read_joystick", Point, callback)
    
    print("Press Ctrl+C multiple times to quit!")

    #instead of spin(), keep node running until ESC is pressed
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
        if getch() == chr(0x1b):
            break

def rosbag_write(value):
    motor_value = Int32()
    motor_value.data = value
    bag.write('motor_value_topic', motor_value)
        
if __name__ == '__main__':

    dxl_operating_mode(16) # this is the operating mode value for PWM

    dxl_torque_enable()

    dxl_listener()

    dxl_torque_disable()

    bag.close()
