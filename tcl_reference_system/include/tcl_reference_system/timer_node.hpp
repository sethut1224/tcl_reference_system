#ifndef TCL_REFERENCE_SYSTEM__TIMER_NODE_HPP_
#define TCL_REFERENCE_SYSTEM__TIMER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "tcl_reference_system_msg/msg/dynamic_message.hpp"
#include "tcl_reference_system_msg/msg/tcl_dynamic_message.hpp"

#include <random>
#include <time.h>
#include <unordered_map>

using tcl_reference_system_msg::msg::DynamicMessage;
using tcl_reference_system_msg::msg::TCLDynamicMessage;
namespace tcl_reference_system
{

class TimerNode : public rclcpp::Node
{
private:
    std::unordered_map<std::string, rclcpp::Publisher<TCLDynamicMessage>::SharedPtr> pub_map_;
    std::unordered_map<std::string, rclcpp::Subscription<TCLDynamicMessage>::SharedPtr> sub_map_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::normal_distribution<float> execution_time_generator_;
    std::normal_distribution<float> message_size_generator_;
    std::mt19937 * gen_;

public:
    explicit TimerNode(const rclcpp::NodeOptions& options);

    ~TimerNode();
    
    void
    normal_topic_callback(const TCLDynamicMessage::SharedPtr msg);

    void
    blocking_topic_callback(const TCLDynamicMessage::SharedPtr msg);

    void
    timer_callback();

    void 
    do_something(int execution_time);

    void 
    publish(int message_size);

    void
    execute();
};
}

#endif