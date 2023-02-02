#include "tcl_reference_system/spin_node_main.hpp"
#include "reference_tcl_interface/reference_tcl_interface.hpp"

namespace tcl_reference_system
{
    SpinNodeMain::SpinNodeMain(const rclcpp::NodeOptions& options)
    : Node("spin_node_main", options)
    {
        int execution_time_mean = this->declare_parameter("execution_time_mean", 0);
        int message_size_mean = this->declare_parameter("message_size_mean", 0);
        
        double execution_time_deviation = this->declare_parameter("execution_time_deviation", 0.0);
        double message_size_deviation = this->declare_parameter("message_size_deviation", 0.0);
        
        std::vector<std::string> sub_topics = this->declare_parameter("subscribe_topics", std::vector<std::string>());
        std::vector<std::string> pub_topics = this->declare_parameter("publish_topics", std::vector<std::string>());

        auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));
        
        auto blocking_topics = this->get_node_timing_interface()->get_blocking_topics();

        std::for_each(sub_topics.begin(), sub_topics.end(), [&](auto& topic)
        {   
            if(std::find(blocking_topics.begin(), blocking_topics.end(), topic) == blocking_topics.end())
            {
                sub_map_[topic] = this->create_subscription<TCLDynamicMessage>(topic, qos, std::bind(&SpinNodeMain::normal_topic_callback, this, std::placeholders::_1));
            }
            else
            {
                sub_map_[topic] = this->create_subscription<TCLDynamicMessage>(topic, qos, std::bind(&SpinNodeMain::blocking_topic_callback, this, std::placeholders::_1));
                ++blocking_topic_num_;
            }
        });

        std::for_each(pub_topics.begin(), pub_topics.end(), [&](auto& topic)
        {
            pub_map_[topic] = this->create_publisher<TCLDynamicMessage>(topic, qos);    
        });

        execution_time_generator_ = std::normal_distribution<float>(
            execution_time_mean,
            execution_time_deviation);
        
        message_size_generator_ = std::normal_distribution<float>(
            message_size_mean,
            message_size_deviation);

        // uncomment to use a deteministic seed
        // std::random_device rd;
        // gen_ = new std::mt19937(1701);

        //non-deterministic seed
        std::random_device rd;
        gen_ = new std::mt19937(rd());
    }

    void
    SpinNodeMain::normal_topic_callback(const TCLDynamicMessage::SharedPtr tcl_msg)
    {
        // (void)msg;
        RCLCPP_INFO(this->get_logger(), "%s : Normal Topic Sub", this->get_name());
        DynamicMessage::SharedPtr msg(new DynamicMessage());
        reference_tcl_interface::messageInterface<TCLDynamicMessage, DynamicMessage>(
            *tcl_msg, *msg, this->get_node_timing_interface());
    }

    void
    SpinNodeMain::blocking_topic_callback(const TCLDynamicMessage::ConstSharedPtr tcl_msg)
    {   
        RCLCPP_INFO(this->get_logger(), "Blocking Topic Sub");
        static uint8_t count = 0;

        DynamicMessage::SharedPtr msg(new DynamicMessage());
        reference_tcl_interface::messageInterface<TCLDynamicMessage, DynamicMessage>(
            *tcl_msg, *msg, this->get_node_timing_interface());
        
        int sum = 0;
        for(size_t i = 0; i < msg->length; ++i){
            sum += msg->data[i];
        }

        ++count;

        if(count == blocking_topic_num_)
        {
            execute();
            count = 0;
        }
    }

    void 
    SpinNodeMain::do_something(int execution_time)
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
    SpinNodeMain::publish(int message_size)
    {
        DynamicMessage::SharedPtr msg(new DynamicMessage());
        auto size_per_data = sizeof(decltype(msg->data)::value_type);

        int length = message_size / size_per_data;
        
        msg->length = length;
        msg->data.reserve(length);

        for(int i = 0; i < length; ++i)
            msg->data.emplace_back();

        TCLDynamicMessage::SharedPtr tcl_msg(new TCLDynamicMessage());
        reference_tcl_interface::messageInterface<DynamicMessage, TCLDynamicMessage>(
            *msg, *tcl_msg, this->get_node_timing_interface());

        std::for_each(pub_map_.begin(), pub_map_.end(), [&](auto& iter)
        {
            iter.second->publish(*tcl_msg);
        });
        // RCLCPP_INFO(this->get_logger(), "%s : SpinNode Publish ", this->get_name());
    }

    void
    SpinNodeMain::execute()
    {
        int execution_time = (int)execution_time_generator_(*gen_);
        int message_size = (int)message_size_generator_(*gen_);

        do_something(execution_time);

        publish(message_size);
    }

    SpinNodeMain::~SpinNodeMain()
    {
        delete gen_;
    }
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<tcl_reference_system::SpinNodeMain>(rclcpp::NodeOptions());

    rclcpp::executors::SingleThreadedExecutor exec;

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
}