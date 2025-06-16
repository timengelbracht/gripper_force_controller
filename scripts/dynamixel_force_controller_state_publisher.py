#!/usr/bin/env python3
"""
Gripper-force controller (ROS 1, Python 3.8) for a single Dynamixel XM-series servo.
Handles Present/Goal-Current registers as **signed 16-bit two’s-complement**
so all readings and commands use proper units.
"""

import time
from typing import Optional

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from dynamixel_sdk import *  # Dynamixel SDK

# ──────────────────────────────────────────────────────────────────────────────
# Dynamixel addresses & constants
# ──────────────────────────────────────────────────────────────────────────────
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT  = 126
ADDR_GOAL_CURRENT     = 102
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_OPERATING_MODE   = 11
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_VOLTAGE  = 144
ADDR_PRESENT_TEMP     = 146

PROTOCOL_VERSION = 2.0

# ──────────────────────────────────────────────────────────────────────────────
# Helper constants / conversion utilities
# ──────────────────────────────────────────────────────────────────────────────
CURRENT_LSB_MA = 2.69   # mA per LSB for Current registers (XM430 datasheet)
DEG_PER_TICK   = 0.088  # ° per position tick (360° / 4096)


def _to_signed(val: Optional[int], bits: int = 16) -> int:
    """Return signed two-complement representation of an unsigned word."""
    if val is None:
        return 0
    if val >= 1 << (bits - 1):          # top bit set ⇒ negative
        val -= 1 << bits
    return val

# ──────────────────────────────────────────────────────────────────────────────
# Application parameters
# ──────────────────────────────────────────────────────────────────────────────
# MIN_POSITION          = 106
# MAX_POSITION          = 190
# NEUTRAL_POSITION      = 120
# FULLY_CLOSED_POSITION = 190

FORCE_THRESHOLD    = 0.5    # N
AUTO_RETURN_SPEED  = 0.1
MIN_RETURN_CURRENT = 150    # mA

TAU      = 64          # angle offset [deg]
K_MOTOR  = 1.769       # Nm/A
B_MOTOR  = 0.2214      # Nm (negative motor offset)
L        = 0.038       # lever arm [m]

# ──────────────────────────────────────────────────────────────────────────────
# ROS node setup
# ──────────────────────────────────────────────────────────────────────────────
rospy.init_node("gripper_force_controller")
rate = rospy.Rate(50)

BAUDRATE_DXL = rospy.get_param("~baudrate_dxl", 57600)
DEVICENAME   = rospy.get_param("~devicename", "/dev/ttyUSB0")
DXL_ID       = rospy.get_param("~dxl_id", 1)

MIN_POSITION = rospy.get_param("~min_ticks")
MAX_POSITION = rospy.get_param("~max_ticks")
NEUTRAL_POSITION = MIN_POSITION + 15   # if you like
FULLY_CLOSED_POSITION = MAX_POSITION    

force_value: Optional[float] = None  # cache of latest force trigger
_last_log: float = 0.0               # throttled log timer

rospy.Subscriber(
    "/gripper_force_trigger",
    Float32,
    lambda msg: globals().__setitem__("force_value", msg.data),
)

joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
force_pub = rospy.Publisher("/gripper_force_stamped", WrenchStamped, queue_size=10)
diag_pub  = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

# ──────────────────────────────────────────────────────────────────────────────
# Dynamixel port / packet handlers
# ──────────────────────────────────────────────────────────────────────────────
portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    rospy.logfatal("Failed to open the port %s", DEVICENAME)
    raise SystemExit(1)
if not portHandler.setBaudRate(BAUDRATE_DXL):
    rospy.logfatal("Failed to set baudrate %d", BAUDRATE_DXL)
    raise SystemExit(1)

# Current-based position mode
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 5)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

# ──────────────────────────────────────────────────────────────────────────────
# Utility functions
# ──────────────────────────────────────────────────────────────────────────────

def _throttle_log(msg: str, interval: float = 1.0) -> None:
    global _last_log
    now = time.time()
    if now - _last_log >= interval:
        rospy.loginfo(msg)
        _last_log = now


def read_value(address: int, length: int):
    if length == 4:
        return packetHandler.read4ByteTxRx(portHandler, DXL_ID, address)[0]
    if length == 2:
        return packetHandler.read2ByteTxRx(portHandler, DXL_ID, address)[0]
    if length == 1:
        return packetHandler.read1ByteTxRx(portHandler, DXL_ID, address)[0]
    return None


def read_position_deg() -> Optional[float]:
    raw = read_value(ADDR_PRESENT_POSITION, 4)
    if raw is None:
        rospy.logwarn("Failed to read position")
        return None
    deg = raw * DEG_PER_TICK
    _throttle_log(f"Motor Position: {deg:.2f}°")
    return deg


def read_current_mA() -> float:
    raw = read_value(ADDR_PRESENT_CURRENT, 2)
    if raw is None or raw >= 0xFFFE:  # 0xFFFE/0xFFFF = invalid
        return 0.0
    signed = _to_signed(raw)
    mA = signed * CURRENT_LSB_MA
    _throttle_log(f"Motor Current: {mA:.2f} mA ({signed} LSB)")
    return mA

# ──────────────────────────────────────────────────────────────────────────────
# Publishing helpers
# ──────────────────────────────────────────────────────────────────────────────

def publish_joint_and_diag():
    pos      = read_value(ADDR_PRESENT_POSITION, 4)
    vel      = read_value(ADDR_PRESENT_VELOCITY, 4)
    cur      = read_value(ADDR_PRESENT_CURRENT, 2)
    volt     = read_value(ADDR_PRESENT_VOLTAGE, 2)
    temp     = read_value(ADDR_PRESENT_TEMP, 1)
    goal_cur = read_value(ADDR_GOAL_CURRENT, 2)

    if None in (pos, vel, cur, volt, temp, goal_cur):
        rospy.logwarn("Incomplete telemetry")
        return

    deg        = pos * DEG_PER_TICK
    current_mA = _to_signed(cur) * CURRENT_LSB_MA
    goal_mA    = _to_signed(goal_cur) * CURRENT_LSB_MA

    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name         = ["gripper"]
    js.position     = [np.deg2rad(deg)]
    js.velocity     = [0.0]
    js.effort       = [current_mA / 1000.0]  # Amps
    joint_pub.publish(js)

    diag               = DiagnosticArray()
    diag.header.stamp  = rospy.Time.now()
    status             = DiagnosticStatus()
    status.name        = "Gripper Motor"
    status.level       = 0
    status.message     = "Nominal"
    status.hardware_id = "XM430-W350"
    status.values      = [
        KeyValue("Position (deg)", f"{deg:.2f}"),
        KeyValue("Velocity", str(vel)),
        KeyValue("Current (mA)", f"{current_mA:.1f}"),
        KeyValue("Voltage (V)", f"{volt * 0.1:.2f}"),
        KeyValue("Temperature (°C)", str(temp)),
        KeyValue("Goal Current (mA)", f"{goal_mA:.1f}"),
    ]
    diag.status.append(status)
    diag_pub.publish(diag)

# ──────────────────────────────────────────────────────────────────────────────
# Motion helpers
# ──────────────────────────────────────────────────────────────────────────────

def move_motor(current_mA: float, position_deg: float):
    pos_val = int(position_deg / DEG_PER_TICK)
    cur_val = int(current_mA / CURRENT_LSB_MA) & 0xFFFF  # two’s-complement encode

    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, pos_val)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, cur_val)
    _throttle_log(
        f"Commanded {current_mA:.1f} mA → pos {position_deg:.1f}°  (raw {cur_val})"
    )


def move_to_neutral():
    if read_position_deg() is not None:
        move_motor(MIN_RETURN_CURRENT, NEUTRAL_POSITION)


def force_to_current_mA(force_N: float) -> float:
    position = read_position_deg()
    if position is None:
        return 0.0
    angle = position - TAU
    sin_term = np.sin(np.deg2rad(angle)) or 1e-6  # protect against 0
    Nm = force_N * L / sin_term + B_MOTOR
    return (Nm / K_MOTOR) * 1000  # → mA


def grip_object():
    current_mA = force_to_current_mA(force_value)
    move_motor(current_mA, FULLY_CLOSED_POSITION)

# ──────────────────────────────────────────────────────────────────────────────
# Main loop
# ──────────────────────────────────────────────────────────────────────────────
try:
    while not rospy.is_shutdown():
        if force_value is not None:
            # publish wrench
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = rospy.Time.now()
            wrench_msg.header.frame_id = "gripper"
            wrench_msg.wrench.force.x = force_value
            force_pub.publish(wrench_msg)

            _throttle_log(f"Force Value: {force_value:.2f} N")
            publish_joint_and_diag()

            pos_deg = read_position_deg()
            if pos_deg is None:
                rate.sleep()
                continue

            within_bounds = MIN_POSITION <= pos_deg <= MAX_POSITION
            if within_bounds and force_value > FORCE_THRESHOLD:
                grip_object()
            else:
                move_to_neutral()

        rate.sleep()

finally:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
