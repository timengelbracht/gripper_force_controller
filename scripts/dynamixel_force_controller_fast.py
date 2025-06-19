#!/usr/bin/env python3
"""
Force‐driven gripper loop using raw register ticks from
/dynamixel_workbench/dynamixel_state to reproduce your original angle readings.
"""

import threading
import numpy as np
import rospy
from std_msgs.msg import Float32
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest

# ──────────────────────────────────────────────────────────────────────────────
# Constants (from your original script)
# ──────────────────────────────────────────────────────────────────────────────
FORCE_THRESHOLD    = 0.5    # N
TAU                = 64     # deg
K_MOTOR            = 1.769  # Nm/A
B_MOTOR            = 0.2214 # Nm (negative motor offset)
L                  = 0.038  # lever arm [m]
CURRENT_LSB_MA     = 2.69   # mA per LSB
DEG_PER_TICK       = 0.088  # ° per position tick (360°/4096)

# ──────────────────────────────────────────────────────────────────────────────
# Internal state
# ──────────────────────────────────────────────────────────────────────────────
_lock        = threading.Lock()
latest_force = 0.0
latest_pos   = 0.0   # in degrees, from raw ticks

# placeholder for the service proxy
joint_cmd_srv: rospy.ServiceProxy

# ──────────────────────────────────────────────────────────────────────────────
# Helper: write a single register via the DynamixelCommand service
# ──────────────────────────────────────────────────────────────────────────────
def send_dxl_command(addr_name: str, value: int):
    req = DynamixelCommandRequest()
    req.command   = ""       # unused for simple writes
    req.id        = 1
    req.addr_name = addr_name
    req.value     = value
    print(value)
    try:
        res = joint_cmd_srv(req)
        if not res.comm_result:
            rospy.logwarn(f"[SERVICE] {addr_name}={value} failed")
    except rospy.ServiceException as e:
        rospy.logerr(f"[SERVICE] call failed: {e}")

# ──────────────────────────────────────────────────────────────────────────────
# Callback: update latest_pos (deg) from raw DynamixelStateList
# ──────────────────────────────────────────────────────────────────────────────
def state_cb(msg: DynamixelStateList):
    global latest_pos
    for st in msg.dynamixel_state:
        if st.id == 1:
            with _lock:
                latest_pos = st.present_position * DEG_PER_TICK
            return

# ──────────────────────────────────────────────────────────────────────────────
# Callback: on force trigger, compute & send current+position
# ──────────────────────────────────────────────────────────────────────────────
def force_cb(msg: Float32):
    with _lock:
        f   = msg.data
        pos = latest_pos

    rospy.loginfo(f"[FORCE_CB] raw force = {f:.3f} N, pos = {pos:.1f}°")

    # force → torque → current (mA)
    angle   = pos - TAU
    sin_t   = np.sin(np.deg2rad(angle)) or 1e-6
    torque  = f * L / sin_t + B_MOTOR    # Nm
    curr_mA = (torque / K_MOTOR) * 1000  # mA

    # choose goal position tick
    goal_deg = FULLY_CLOSED_POSITION if f > FORCE_THRESHOLD else NEUTRAL_POSITION
    goal_tick = int(goal_deg /  DEG_PER_TICK)

    # convert to register LSBs
    cur_val = int(curr_mA / CURRENT_LSB_MA) & 0xFFFF
    pos_val = int(goal_tick)

    #rospy.loginfo(f"[CTRL] f={f:.2f}N pos={pos:.1f}° → cur={cur_val} tick={pos_val}")
    rospy.loginfo_throttle(1.0, f"[CTRL] f={f:.2f}N pos={pos:.1f}° → cur={cur_val} tick={pos_val}")


    send_dxl_command("Goal_Current",  cur_val)
    send_dxl_command("Goal_Position", pos_val)

# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    rospy.init_node("gripper_force_controller_simple")

    # soft‐limits
    min_t = rospy.get_param("~min_ticks", 106)
    max_t = rospy.get_param("~max_ticks", 190)
    NEUTRAL_POSITION      = min_t + 15
    FULLY_CLOSED_POSITION = max_t

    with _lock:
        latest_pos = NEUTRAL_POSITION * DEG_PER_TICK

    # connect to the verified service
    srv_name = "/dynamixel_workbench/dynamixel_command"
    rospy.loginfo(f"Waiting for service {srv_name}...")
    rospy.wait_for_service(srv_name)
    joint_cmd_srv = rospy.ServiceProxy(srv_name, DynamixelCommand)
    rospy.loginfo(f"Connected to {srv_name}.")

    # subscribe
    rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, state_cb)
    rospy.Subscriber("/gripper_force_trigger",         Float32,               force_cb)
    rospy.loginfo("Subscribed to /dynamixel_workbench/dynamixel_state and /gripper_force_trigger")

    rospy.spin()
