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
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_OPERATING_MODE    = 11		  # The Control table address for operating
ADDR_PRO_TORQUE_ENABLE = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_PWM      = 100              # Control table address for goal PWM
ADDR_PRO_PRESENT_PWM   = 124              # Control table address for present PWM

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = [1, 2]             # Dynamixel ID for the two motors
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
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_OPERATING_MODE, operating_mode_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("PWM operating mode for ID %03d enabled" % DXL_ID[0])

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_OPERATING_MODE, operating_mode_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("PWM operating mode for ID %03d enabled" % DXL_ID[1])

def dxl_torque_enable():
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque for ID %03d enabled" % DXL_ID[0])

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque for ID %03d enabled" % DXL_ID[1])

def dxl_write(motor_values):
    motor_pwm_value_1 = motor_values[0]
    motor_pwm_value_2 = motor_values[1]

    # Write goal pwm position for ID 1
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_GOAL_PWM, motor_pwm_value_1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read present position for ID 1
    dxl_present_pwm_1, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_PRESENT_PWM)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("Present PWM for ID %03d is: %03d" % (DXL_ID[0], dxl_present_pwm_1))

    # Write goal pwm position for ID 2
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_GOAL_PWM, motor_pwm_value_2)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read present position for ID 2
    dxl_present_pwm_2, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_PRESENT_PWM)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("Present PWM for ID %03d is: %03d" % (DXL_ID[1], dxl_present_pwm_2))

def dxl_torque_disable():
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    print("Torque for ID %03d disabled" % DXL_ID[0])

    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    print("Torque for ID %03d disabled" % DXL_ID[1])

    # Close port
    portHandler.closePort()

def callback(data):
    #rospy.loginfo("I heard %d", data.data)
    dxl_write(data.data)
    
def dxl_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('dxl_listener', anonymous=True)

    rospy.Subscriber("motor_pwm_values", Int32MultiArray, callback)
    
    print("Press ESC to quit!")

    #instead of spin(), keep node running until ESC is pressed
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
        if getch() == chr(0x1b):
            break
    

if __name__ == '__main__':
    
    dxl_operating_mode(16) # this is the operating mode value for PWM

    #enable torque for both motors
    dxl_torque_enable()

    dxl_listener()

    dxl_torque_disable()
