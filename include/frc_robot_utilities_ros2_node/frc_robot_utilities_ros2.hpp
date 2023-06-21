#pragma once
#include <rclcpp/rclcpp.hpp>

#include <frc_robot_utilities_ros2_node/BufferedROSMsgHandler.hpp>
#include <frc_robot_utilities_ros2_node/RobotStatusHelper.hpp>


#include <ck_ros2_msgs_node/msg/hmi_signals.hpp>
#include <ck_ros2_base_msgs_node/msg/robot_status.hpp>


extern BufferedROSMsgHandler<ck_ros2_msgs_node::msg::HMISignals> hmi_updates;
extern RobotStatusHelper robot_status;

void register_for_robot_updates();
bool reset_robot_pose(Alliance alliance, double x_inches=0, double y_inches=0, double heading_degrees=0);
