import rospy
from std_msgs.msg import Float32
from dynamixel_sdk import *  # Uses Dynamixel SDK
import time
import numpy as np

# Dynamixel settings
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126
ADDR_GOAL_CURRENT = 102
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_OPERATING_MODE = 11
ADDR_REBOOT = 8

PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE_DXL = 57600
DEVICENAME = '/dev/ttyUSB0'  # Adjust according to your setup

# Limits
MIN_POSITION = 106
MAX_POSITION = 190
NEUTRAL_POSITION = 120
FULLY_CLOSED_POSITION = 190
FORCE_THRESHOLD = 0.5  # Adjust as needed
K = 0.5  # Control gain for current scaling
AUTO_RETURN_SPEED = 0.1  # Increased Speed factor for returning to neutral
MIN_RETURN_CURRENT = 150  # Minimum current applied during auto-return (mA)
TAU = 64 # angle offset, depends on motor/gripper mounting [degree]
K_MOTOR = 1.769 # motor constant [Nm/A]
B_MOTOR = 0.2214 # negative of motor offset 
L = 0.038 # length of lever arm [m]

force_value = None  # Global variable to store the force value
last_log_time = 0  # Timestamp for controlling logging rate


def force_callback(msg):
    global force_value
    force_value = msg.data


# Initialize ROS node
rospy.init_node('gripper_force_controller')
rate = rospy.Rate(50)  # 10 Hz

# Subscribe to the force topic
rospy.Subscriber('/gripper_force_trigger', Float32, force_callback)

# Initialize PortHandler and PacketHandler for Dynamixel
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if not portHandler.openPort():
    rospy.logerr('Failed to open the port')
    exit(1)

# Set baudrate
if not portHandler.setBaudRate(BAUDRATE_DXL):
    rospy.logerr('Failed to set baudrate')
    exit(1)

# Set Operating Mode to Current-based Position Control Mode (mode 5)
result, error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 5)
if result != 0 or error != 0:
    rospy.logerr('Failed to set Current-based Position Control Mode')

# Enable Dynamixel Torque
result, error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
if result != 0 or error != 0:
    rospy.logerr('Failed to enable torque. Attempting reboot...')
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_REBOOT, 1)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)


def log_info(message):
    global last_log_time
    if time.time() - last_log_time >= 1.0:  # Log once per second
        rospy.loginfo(message)
        last_log_time = time.time()


def log_params():
    log_info(f"Parameters - FORCE_THRESHOLD: {FORCE_THRESHOLD}, K: {K}, AUTO_RETURN_SPEED: {AUTO_RETURN_SPEED}, MIN_RETURN_CURRENT: {MIN_RETURN_CURRENT} mA")


def read_position():
    raw_position, result, error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if result != 0 or error != 0:
        rospy.logwarn('Failed to read position.')
        return None

    position_degrees = raw_position * 0.088  # Convert from raw to degrees
    log_info(f"Motor Position: {position_degrees:.2f} degrees")
    return position_degrees


def read_current():
    current, result, error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
    if result != 0 or error != 0:
        rospy.logwarn('Failed to read current.')
        return None
    current_mA = current * 2.69  # Convert from raw to mA
    log_info(f"Motor Current: {current_mA:.6f} mA")
    return current_mA


def move_motor(current, position):
    current_value = int(current / 2.69)  # Conversion factor for current in mA to SDK unit
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, int(position / 0.088))
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, current_value)
    log_info(f"Applying Current: {current:.6f} mA | Target Position: {position:.2f} degrees")
    read_current()  # Read current immediately after applying to confirm it's set


def move_to_neutral():
    current_position = read_position()
    if current_position is None:
        return
    move_motor(MIN_RETURN_CURRENT, NEUTRAL_POSITION)

def grip_object():
    I = force_to_current_mapping(force_value)
    move_motor(I, FULLY_CLOSED_POSITION)

def force_to_current_mapping(force):

    position = read_position()
    if position is None:
        return
    gripper_angle = position - TAU
    I = (((force * L / np.sin(np.deg2rad(gripper_angle))) + B_MOTOR) / K_MOTOR) * 1000
    return I

try:
    while not rospy.is_shutdown():
        if force_value is not None:
            log_info(f"Force Value: {force_value:.2f} N")
            log_params()
            current_position = read_position()
            read_current()

            if current_position is None:
                continue

            if MIN_POSITION <= current_position <= MAX_POSITION:
                if force_value > FORCE_THRESHOLD:
                    grip_object()
                else:
                    move_to_neutral()
            else:
                move_to_neutral()

        rate.sleep()

finally:
    # Disable Dynamixel Torque before exit
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
