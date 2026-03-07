#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("vff_avoidance_node");

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("Stop_bt_node"));
    factory.registerFromPlugin(loader.getOSName("Back_bt_node"));
    factory.registerFromPlugin(loader.getOSName("Turn_bt_node"));
    factory.registerFromPlugin(loader.getOSName("vff_bt_node"));
    factory.registerFromPlugin(loader.getOSName("ready_to_run_vff_bt_node"));

    std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_vff");
    std::string xml_file_path = pkgpath + "/behavior_tree_xml/bt_vff.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node_name", node);
    BT::Tree tree = factory.createTreeFromFile(xml_file_path, blackboard);

    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

    rclcpp::Rate rate(10);

    bool finish = false;

    while(!finish && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
