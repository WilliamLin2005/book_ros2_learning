#include<vector>
#include<string>
#include "rclcpp/rclcpp.hpp"

class parameter_node : public rclcpp::Node
{
    private:
        int num_particles_;
        std::vector<std::string> topic_name_;
        std::vector<std::string> topic_type_;

    public:
        parameter_node(const std::string name) : Node(name)
        {
            this->declare_parameter("num_particles", 500);
            this->declare_parameter("topic_name", std::vector<std::string>());
            this->declare_parameter("topic_type", std::vector<std::string>());

            this->get_parameter("num_particles", num_particles_);
            RCLCPP_INFO_STREAM(this->get_logger(), "num_particles: " << num_particles_);

            this->get_parameter("topic_name", topic_name_);
            this->get_parameter("topic_type", topic_type_);

            if(topic_name_.size() != topic_type_.size())
            {
                RCLCPP_WARN(this->get_logger(), "number of topic_name: %zu != number of topic_type: %zu",topic_name_.size(),topic_type_.size());
            }
            else
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "the number of topic_name:" << topic_name_.size());
                for (size_t i = 0; i < topic_name_.size();i++)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), std::endl
                                                               << topic_name_[i] << " : " << topic_type_[i]);
                }
            }
        }
};

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<parameter_node>("parameter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}