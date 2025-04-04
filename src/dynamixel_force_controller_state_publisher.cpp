#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cmath>

// Dynamixel Addresses and Constants
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126
#define ADDR_GOAL_CURRENT 102
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_OPERATING_MODE 11
#define PROTOCOL_VERSION 2.0

#define MIN_POSITION 106.0
#define MAX_POSITION 190.0
#define NEUTRAL_POSITION 120.0
#define FULLY_CLOSED_POSITION 190.0
#define FORCE_THRESHOLD 0.5
#define K_MOTOR 1.769
#define B_MOTOR 0.2214
#define LEVER_ARM_LENGTH 0.038
#define TAU 64.0
#define AUTO_RETURN_CURRENT 150.0

// ROS and Dynamixel variables
float force_value = 0.0;
ros::Time last_log_time;

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Utility functions
inline float ticksToDegrees(int32_t ticks) {
    return ticks * 0.088f;
}

inline int32_t degreesToTicks(float degrees) {
    return static_cast<int32_t>(degrees / 0.088f);
}

inline int16_t currentToRaw(float current_mA) {
    return static_cast<int16_t>(current_mA / 2.69f);
}

inline float rawToCurrent(int16_t raw_current) {
    return raw_current * 2.69f;
}

void logThrottled(const std::string &msg) {
    if ((ros::Time::now() - last_log_time).toSec() > 1.0) {
        ROS_INFO_STREAM(msg);
        last_log_time = ros::Time::now();
    }
}

void forceCallback(const std_msgs::Float32::ConstPtr &msg) {
    force_value = msg->data;
}

float readPosition() {
    uint32_t pos_raw;
    packetHandler->read4ByteTxRx(portHandler, 1, ADDR_PRESENT_POSITION, &pos_raw);
    return ticksToDegrees(pos_raw);
}

float readCurrent() {
    uint16_t raw;
    packetHandler->read2ByteTxRx(portHandler, 1, ADDR_PRESENT_CURRENT, &raw);
    int16_t current_signed = raw > 32767 ? raw - 65536 : raw;
    return rawToCurrent(current_signed);
}

void moveMotor(float current_mA, float position_deg) {
    packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, degreesToTicks(position_deg));
    packetHandler->write2ByteTxRx(portHandler, 1, ADDR_GOAL_CURRENT, currentToRaw(current_mA));
}

float forceToCurrentMapping(float force, float gripper_angle_deg) {
    float angle_rad = (gripper_angle_deg - TAU) * M_PI / 180.0f;
    float torque = force * LEVER_ARM_LENGTH / sin(angle_rad);
    float current_A = (torque + B_MOTOR) / K_MOTOR;
    return current_A * 1000.0f;  // mA
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_force_controller");
    ros::NodeHandle nh("~");

    std::string device_name;
    int baudrate;
    nh.param<std::string>("devicename", device_name, "/dev/ttyUSB0");
    nh.param("baudrate_dxl", baudrate, 57600);

    portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        ROS_FATAL("Failed to open the port!");
        return 1;
    }

    if (!portHandler->setBaudRate(baudrate)) {
        ROS_FATAL("Failed to set baudrate!");
        return 1;
    }

    packetHandler->write1ByteTxRx(portHandler, 1, ADDR_OPERATING_MODE, 5);
    packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, 1);

    ros::Subscriber force_sub = nh.subscribe("/gripper_force_trigger", 1, forceCallback);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    ros::Rate rate(50);

    while (ros::ok()) {
        float position = readPosition();
        float current = readCurrent();

        if (force_value > FORCE_THRESHOLD && position >= MIN_POSITION && position <= MAX_POSITION) {
            float target_current = forceToCurrentMapping(force_value, position);
            moveMotor(target_current, FULLY_CLOSED_POSITION);
            logThrottled("Gripping object...");
        } else {
            moveMotor(AUTO_RETURN_CURRENT, NEUTRAL_POSITION);
            logThrottled("Returning to neutral...");
        }

        // Publish Joint State
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = {"gripper"};
        js.position = {position * M_PI / 180.0};
        js.effort = {current / 1000.0};
        joint_pub.publish(js);

        // Diagnostics (basic example)
        diagnostic_msgs::DiagnosticArray diag_array;
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "Gripper Motor";
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "Operational";
        status.values.resize(2);
        status.values[0].key = "Position (deg)";
        status.values[0].value = std::to_string(position);
        status.values[1].key = "Current (mA)";
        status.values[1].value = std::to_string(current);
        diag_array.status.push_back(status);
        diag_pub.publish(diag_array);

        ros::spinOnce();
        rate.sleep();
    }

    packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, 0);
    portHandler->closePort();

    return 0;
}

