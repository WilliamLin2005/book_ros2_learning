#include"rclcpp/rclcpp.hpp"
#include"std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

//publisher_node
class publisher_node : public rclcpp::Node
{
    private:
        std_msgs::msg::Int32 mes_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    public:
        void timer_callback()
        {
            publisher_->publish(mes_);
            RCLCPP_INFO_STREAM(this->get_logger(), std::endl
                                                       << "Hello: " << mes_.data);
            mes_.data++;
        }
        publisher_node(const std::string name) :Node(name)
        {
            mes_.data = 0;
            timer_ = this->create_wall_timer(500ms, [this]()
                                             { this->timer_callback(); });
            publisher_ = this->create_publisher<std_msgs::msg::Int32>("int_topic", 10);
        }
};

//subscruber_node
class subscriber_node : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
    public:
        void callback(const std_msgs::msg::Int32::SharedPtr message_)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), std::endl<<"GET: " << message_->data);
        }
        subscriber_node(const std::string name) : Node(name)
        {
            subscriber_ = this->create_subscription<std_msgs::msg::Int32>("int_topic", 10, [this](std_msgs ::msg::Int32::SharedPtr mes_)
                                                                          { callback(mes_); });
        }
};

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor_;
    auto node_pub = std::make_shared<publisher_node>("publisher");
    auto node_sub = std::make_shared<subscriber_node>("subscriber");
    executor_.add_node(node_pub);
    executor_.add_node(node_sub);
    executor_.spin();
    rclcpp::shutdown();
    return 0;
}
