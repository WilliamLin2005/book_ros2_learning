#include"rclcpp/rclcpp.hpp"
#include"std_msgs/msg/int32.hpp"

class subscriberclass_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
public:
    subscriberclass_node(const std::string name) :  rclcpp::Node(name)
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Int32>("int_topic", 10, [this](std_msgs::msg::Int32::SharedPtr msg_)
                                                                      { this->callback(msg_); });
    }

    void callback(std_msgs::msg::Int32::SharedPtr mes_)
    {
        RCLCPP_INFO(rclcpp::get_logger("subscriber"),"Hello %d",mes_->data);
    }
};

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<subscriberclass_node>("subscriber");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}