#include "frc_robot_utilities_ros2_node/frc_robot_utilities_ros2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <atomic>
#include <sys/stat.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <robot_localization/srv/set_pose.hpp>

#include "frc_robot_utilities_ros2_node/BufferedROSMsgHandler.hpp"
#include <ck_ros2_msgs_node/msg/hmi_signals.hpp>
#include <ck_ros2_base_msgs_node/msg/robot_status.hpp>

#include <ck_utilities_ros2_node/node_handle.hpp>

BufferedROSMsgHandler<ck_ros2_msgs_node::msg::HMISignals> hmi_updates;
BufferedROSMsgHandler<ck_ros2_base_msgs_node::msg::RobotStatus> robot_updates_internal;
RobotStatusHelper robot_status(robot_updates_internal);

rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr _set_pose_client;

inline double __deg2rad(double deg)
{
	return deg * M_PI / 180.0;
}

inline double __inches_to_meters(double inches)
{
	return inches * 0.0254;
}

void register_for_robot_updates()
{
	hmi_updates.register_for_updates("/HMISignals");
	robot_updates_internal.register_for_updates("/RobotStatus");
	_set_pose_client = node_handle->create_client<robot_localization::srv::SetPose>("/set_pose", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
}

bool reset_robot_pose(Alliance alliance, double x_inches, double y_inches, double heading_degrees)
{    

	if (node_handle)
	{
		auto initial_pose = std::make_shared<robot_localization::srv::SetPose::Request>();
		initial_pose->pose.header.stamp = node_handle->get_clock()->now();
		initial_pose->pose.header.frame_id = "odom";

		initial_pose->pose.pose.pose.position.x = __inches_to_meters(x_inches);
		initial_pose->pose.pose.pose.position.y = __inches_to_meters(y_inches);
		initial_pose->pose.pose.pose.position.z = 0;

		double heading_rad = __deg2rad(heading_degrees);
		if (alliance == Alliance::BLUE)
		{
        	heading_rad += M_PI;
		}

		tf2::Quaternion q;
		q.setRPY(0,0,heading_rad);
		initial_pose->pose.pose.pose.orientation.w = q.getW();
		initial_pose->pose.pose.pose.orientation.x = q.getX();
		initial_pose->pose.pose.pose.orientation.y = q.getY();
		initial_pose->pose.pose.pose.orientation.z = q.getZ();

		initial_pose->pose.pose.covariance =
		{ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0174533,};

		if(_set_pose_client && _set_pose_client->wait_for_service(std::chrono::milliseconds(250)))
		{
			auto result = _set_pose_client->async_send_request(initial_pose);
			RCLCPP_INFO(node_handle->get_logger(), "Resetting robot pose to %s", initial_pose->pose.header.frame_id.c_str());
			if (rclcpp::spin_until_future_complete(node_handle, result) != rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_ERROR(node_handle->get_logger(), "Failed to call service set_pose");
			}
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_handle->get_logger(), "FAILED TO RESET ROBOT POSE!");
			return false;
		}
	}
	RCLCPP_ERROR(node_handle->get_logger(), "NODE HANDLE NOT INITIALIZED!");
	return false;
}