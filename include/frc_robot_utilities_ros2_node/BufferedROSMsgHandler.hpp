#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <mutex>
#include <memory>

#include <ck_utilities_ros2_node/node_handle.hpp>

template <typename T>
class BufferedROSMsgHandler
{
private:
    T msg_update_tmp;
    T msg_buf;
    std::recursive_mutex msg_lock;
    std::atomic_bool update_occurred {false};
    typename rclcpp::Subscription<T>::SharedPtr subscriber_;

    void update_func(const T& callback_msg)
    {
        std::lock_guard<std::recursive_mutex> lock(msg_lock);
        msg_update_tmp = callback_msg;
        update_occurred = true;
    }

public:
    BufferedROSMsgHandler()
    {

    };

    /**
     * @brief Register for hmi data updates. Call once before spin or spin_once
     * @param node The pointer to the instantiated NodeHandle object
     * @param topic The name of the ROS topic to subscribe to
     * @param queue_size The ROS msg queue size. Defaults to 1
    */
    void register_for_updates(std::string topic, int queue_size = 1)
    {
        subscriber_ = node_handle->create_subscription<T>(topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort().durability_volatile(), std::bind(&BufferedROSMsgHandler<T>::update_func, this, std::placeholders::_1));
    };
    
    /**
     * @brief Get the latest message data
     * @return The latest ROS msg data that this instance is subscribed to
    */
    const T& get()
    {
        if (update_occurred)
        {
            std::lock_guard<std::recursive_mutex> lock(msg_lock);
            msg_buf = msg_update_tmp;
            update_occurred = false;
        }
        return msg_buf;
    };

    bool has_updated()
    {
        return update_occurred;
    }
};