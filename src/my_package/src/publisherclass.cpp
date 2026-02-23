#include"rclcpp/rclcpp.hpp"
#include"std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class publisher_node : public rclcpp::Node
{
    private:
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std_msgs::msg::Int32 msg_;
    
    public:
        void timer_callback()
        {
            msg_.data++;
            publisher_->publish(msg_);
        }
        publisher_node(std::string name) : Node(name)
        {
            publisher_ = this->create_publisher<std_msgs::msg::Int32>("int_topic", 10);
            timer_ = this->create_wall_timer(500ms,
                                             [this]()
                                             { this->timer_callback(); });
        }
};

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<publisher_node>("publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}