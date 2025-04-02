#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_TEMP = 146

PROTOCOL_VERSION = 2.0

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

# Initialize ROS node
rospy.init_node('gripper_force_controller')
rate = rospy.Rate(50)

BAUDRATE_DXL = rospy.get_param('baudrate_dxl', 57600)
DEVICENAME = rospy.get_param('devicename', '/dev/ttyUSB0')
DXL_ID = rospy.get_param('dxl_id', 1)

# Subscribe to the force topic
rospy.Subscriber('/gripper_force_trigger', Float32, lambda msg: globals().__setitem__('force_value', msg.data))

joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
force_pub = rospy.Publisher('/gripper_force', Float32, queue_size=10)
diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

# Initialize PortHandler and PacketHandler for Dynamixel
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    rospy.logerr('Failed to open the port')
    exit(1)
if not portHandler.setBaudRate(BAUDRATE_DXL):
    rospy.logerr('Failed to set baudrate')
    exit(1)

packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 5)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

def log_info(message):
    global last_log_time
    if time.time() - last_log_time >= 1.0:
        rospy.loginfo(message)
        last_log_time = time.time()

def log_params():
    log_info(f"Parameters - FORCE_THRESHOLD: {FORCE_THRESHOLD}, K: {K}, AUTO_RETURN_SPEED: {AUTO_RETURN_SPEED}, MIN_RETURN_CURRENT: {MIN_RETURN_CURRENT} mA")

def read_value(address, length):
    if length == 4:
        return packetHandler.read4ByteTxRx(portHandler, DXL_ID, address)[0]
    elif length == 2:
        return packetHandler.read2ByteTxRx(portHandler, DXL_ID, address)[0]
    elif length == 1:
        return packetHandler.read1ByteTxRx(portHandler, DXL_ID, address)[0]
    return None

def read_position():
    raw = read_value(ADDR_PRESENT_POSITION, 4)
    if raw is None:
        rospy.logwarn('Failed to read position.')
        return None
    deg = raw * 0.088
    log_info(f"Motor Position: {deg:.2f} degrees")
    return deg

def read_current():
    raw = read_value(ADDR_PRESENT_CURRENT, 2)
    if raw is None:
        rospy.logwarn('Failed to read current.')
        return None

    # Convert from unsigned to signed 16-bit value
    if raw > 32767:
        raw -= 65536

    mA = raw * 2.69
    log_info(f"Motor Current: {mA:.6f} mA")
    return mA

# Publishes joint state and diagnostic info for the motor
def publish_joint_and_diag():
    pos = read_value(ADDR_PRESENT_POSITION, 4)
    vel = read_value(ADDR_PRESENT_VELOCITY, 4)
    cur = read_value(ADDR_PRESENT_CURRENT, 2)
    volt = read_value(ADDR_PRESENT_VOLTAGE, 2)
    temp = read_value(ADDR_PRESENT_TEMP, 1)
    goal_cur = read_value(ADDR_GOAL_CURRENT, 2)
    if None in (pos, vel, cur, volt, temp, goal_cur):
        rospy.logwarn("Incomplete telemetry")
        return

    deg = pos * 0.088
    rad = np.deg2rad(deg)
    current_mA = cur * 2.69
    voltage = volt * 0.1
    temperature = temp
    goal_current = goal_cur * 2.69

    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = ['gripper']
    js.position = [rad]
    js.velocity = [0.0]
    js.effort = [current_mA / 1000.0]
    joint_pub.publish(js)

    diag = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = "Gripper Motor"
    status.level = 0
    status.message = "Nominal"
    status.hardware_id = "XM430-W350"
    status.values = [
        KeyValue("Position (deg)", str(deg)),
        KeyValue("Velocity", str(vel)),
        KeyValue("Current (mA)", str(current_mA)),
        KeyValue("Voltage (V)", str(voltage)),
        KeyValue("Temperature (\u00b0C)", str(temperature)),
        KeyValue("Goal Current (mA)", str(goal_current))
    ]
    diag.status.append(status)
    diag_pub.publish(diag)

def move_motor(current, position):
    pos_val = int(position / 0.088)
    cur_val = int(current / 2.69)
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, pos_val)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, cur_val)
    log_info(f"Applying Current: {current:.6f} mA | Target Position: {position:.2f} degrees")
    read_current()

def move_to_neutral():
    pos = read_position()
    if pos is None:
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
            force_pub.publish(Float32(force_value))
            log_info(f"Force Value: {force_value:.2f} N")
            log_params()
            current_position = read_position()
            current_current = read_current()
            publish_joint_and_diag()

            if current_position is None:
                continue

            within_bounds = MIN_POSITION <= current_position <= MAX_POSITION
            if within_bounds and force_value > FORCE_THRESHOLD:
                grip_object()
            else:
                move_to_neutral()

        rate.sleep()
finally:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
