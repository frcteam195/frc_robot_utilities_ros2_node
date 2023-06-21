#include "frc_robot_utilities_ros2_node/RobotStatusHelper.hpp"

RobotStatusHelper::RobotStatusHelper(BufferedROSMsgHandler<ck_ros2_base_msgs_node::msg::RobotStatus>& buffered_msg_obj)
{
    buf_handler_ptr = &buffered_msg_obj;
}

void RobotStatusHelper::update()
{
    if (buf_handler_ptr->has_updated())
    {
        std::lock_guard<std::recursive_mutex> lock(robot_lock);
        const ck_ros2_base_msgs_node::msg::RobotStatus& r_stat = buf_handler_ptr->get();
        robot_state = (RobotMode) r_stat.robot_state;
        alliance = (Alliance) r_stat.alliance;
        match_time = r_stat.match_time;
        game_data = r_stat.game_data;
        selected_auto = r_stat.selected_auto;
        m_is_connected = r_stat.is_connected;
    }
}

RobotMode RobotStatusHelper::get_mode()
{
    update();
    return robot_state;
}

Alliance RobotStatusHelper::get_alliance()
{
    update();
    return alliance;
}

double RobotStatusHelper::get_match_time()
{
    update();
    return match_time;
}

std::string RobotStatusHelper::get_game_data()
{
    update();
    return game_data;
}

int RobotStatusHelper::get_selected_auto()
{
    update();
    return selected_auto;
}

bool RobotStatusHelper::is_connected()
{
    update();
    return m_is_connected;
}