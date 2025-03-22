#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Float32
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Dynamixel Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_POSITION = 116
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

# Position Limits
MIN_POSITION = 106  # Fully Opened
MAX_POSITION = 190  # Fully Closed

# Force to Current Conversion Factor (To be calibrated)
K = 100  # Example: 100 mA per 1 Newton

# Auto-Return Parameters
AUTO_RETURN_SPEED = 10  # How fast to return to the open position (lower = faster)
FORCE_THRESHOLD = 0.8  # Minimum force to consider the gripper active

# Initialize PortHandler & PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)


def set_dynamixel_current(goal_current):
    goal_current = int(goal_current)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_current)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr(f"Failed to write goal current: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        rospy.logerr(f"Dynamixel Error: {packetHandler.getRxPacketError(dxl_error)}")


def read_dynamixel_position():
    position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr(f"Failed to read position: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return None
    return position * (360.0 / 4096.0)  # Convert to degrees


def force_control_callback(force_msg):
    finger_force = force_msg.data  # Force from load cell
    position = read_dynamixel_position()

    if position is None:
        return

    # Check position limits
    if position < MIN_POSITION or position > MAX_POSITION:
        rospy.logwarn("Motor position out of safe range! Stopping motor.")
        set_dynamixel_current(0)  # Stop motor
        return

    if finger_force > FORCE_THRESHOLD:  # Normal gripping
        desired_current = finger_force * K  # Map force to motor current
        set_dynamixel_current(desired_current)
    else:  # No force detected, auto-return
        if position > MIN_POSITION:  # Only return if not fully open
            return_current = -AUTO_RETURN_SPEED  # Negative to open gripper
            set_dynamixel_current(return_current)

    rospy.loginfo(f"Finger Force: {finger_force} N, Motor Position: {position} deg")


def dynamixel_force_control():
    rospy.init_node('dynamixel_force_controller', anonymous=True)

    if portHandler.openPort():
        rospy.loginfo("Succeeded to open the port")
    else:
        rospy.logerr("Failed to open the port")
        return

    if portHandler.setBaudRate(BAUDRATE):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.logerr("Failed to change the baudrate")
        return

    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    rospy.Subscriber("gripper_force_trigger", Float32, force_control_callback)
    rospy.spin()

    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()


if __name__ == "__main__":
    try:
        dynamixel_force_control()
    except rospy.ROSInterruptException:
        pass
