#include "tcl_reference_system/spin_node.hpp"

namespace tcl_reference_system
{
    SpinNode::SpinNode(const rclcpp::NodeOptions& options)
    : Node("spin_node", options)
    {
        int execution_time_mean = this->declare_parameter("execution_time_mean", 0);
        int message_size_mean = this->declare_parameter("message_size_mean", 0);
        
        double execution_time_deviation = this->declare_parameter("execution_time_deviation", 0.0);
        double message_size_deviation = this->declare_parameter("message_size_deviation", 0.0);
        
        std::vector<std::string> sub_topics = this->declare_parameter("subscribe_topics", std::vector<std::string>());
        std::vector<std::string> pub_topics = this->declare_parameter("publish_topics", std::vector<std::string>());

        auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));
        
        auto blocking_topics = this->get_node_timing_coordination_interface()->get_blocking_topics();

        std::for_each(sub_topics.begin(), sub_topics.end(), [&](auto& topic)
        {   
            if(std::find(blocking_topics.begin(), blocking_topics.end(), topic) == blocking_topics.end())
            {
                sub_map_[topic] = this->create_subscription<DynamicMessage>(topic, qos, std::bind(&SpinNode::normal_topic_callback, this, std::placeholders::_1));
            }
            else
            {
                sub_map_[topic] = this->create_subscription<DynamicMessage>(topic, qos, std::bind(&SpinNode::blocking_topic_callback, this, std::placeholders::_1));

                ++blocking_topic_num_;
            }
        });

        std::for_each(pub_topics.begin(), pub_topics.end(), [&](auto& topic)
        {
            pub_map_[topic] = this->create_publisher<DynamicMessage>(topic, qos);    
        });

        execution_time_generator_ = std::normal_distribution<float>(
            execution_time_mean,
            execution_time_deviation);
        
        message_size_generator_ = std::normal_distribution<float>(
            message_size_mean,
            message_size_deviation);

        // uncomment to use a deteministic seed
        //      std::random_device rd;
        //      gen_ = new std::mt19937(1701);

        //non-deterministic seed
        std::random_device rd;
        gen_ = new std::mt19937(rd());
    }

    void
    SpinNode::normal_topic_callback(const DynamicMessage::SharedPtr msg)
    {
        (void)msg;
    }

    void
    SpinNode::blocking_topic_callback(const DynamicMessage::SharedPtr msg)
    {   
        static uint8_t count = 0;
        // RCLCPP_INFO(this->get_logger(), "Subscribe");
        int sum = 0;

        ++count;

        for(size_t i = 0; i < msg->length; ++i){
            sum += msg->data[i];
        }

        if(count == blocking_topic_num_)
        {
            execute();
            count = 0;
        }
    }

    void 
    SpinNode::do_something(int execution_time)
    {
        int64_t start = this->now().nanoseconds();

        while(true)
        {
            auto elapsed_ms = (this->now().nanoseconds() - start) * 1.0e-6;
            if(elapsed_ms >= execution_time)
                break;
        }
    }

    void
    SpinNode::publish(int message_size)
    {
        DynamicMessage::UniquePtr msg(new DynamicMessage());
        auto size_per_data = sizeof(decltype(msg->data)::value_type);

        int length = message_size / size_per_data;
        
        msg->length = length;
        msg->data.reserve(length);

        for(int i = 0; i < length; ++i)
            msg->data.emplace_back();
        
        this->get_node_timing_coordination_interface()->propagate_timing_message();
        
        std::for_each(pub_map_.begin(), pub_map_.end(), [&](auto& iter)
        {
            iter.second->publish(std::move(msg));
        });
        // RCLCPP_INFO(this->get_logger(), "Publish");
    }

    void
    SpinNode::execute()
    {
        int execution_time = (int)execution_time_generator_(*gen_);
        int message_size = (int)message_size_generator_(*gen_);

        do_something(execution_time);

        publish(message_size);
    }

    SpinNode::~SpinNode()
    {
        delete gen_;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(tcl_reference_system::SpinNode)