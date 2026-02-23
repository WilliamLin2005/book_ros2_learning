#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class loggernode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    int counter;

public:
    loggernode(std::string name): Node(name)
    {
        counter = 0;
        timer_ = this->create_wall_timer(500ms,
                                         [this]()
                                         { this->timer_callback(); });
    }

    void timer_callback()
    {
        std::cout << "HELLO" << (counter++) << std::endl;
    }
};

int main(int argc,char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<loggernode>("logger");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
