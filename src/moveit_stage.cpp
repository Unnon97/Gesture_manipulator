#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace mtc = moveit::task_constructor;

class MoveitTaskNode : public rclcpp::Node
{
    public:
        MoveitTaskNode() : Node("Moveit_Task_Node")
        {
            stage_subscriber = this->create_subscription<std_msgs::msg::String>(
                "/move_manipulator",10, std::bind(&MoveitTaskNode::commandCallback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
        void commandCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            std::string command = msg->data;

            if (command == "run_stage1") {
                runstage1();
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Unknown command received: %s", command.c_str());
            }
        }
}


// Add the stages or containers for Moveit manipulator
void runstage1()
{
    RCLCPP_INFO(this->get_logger(), "Executing Stage 1")
    moveit::task_constructor::Task task;
    task.stages()->push_back(std::make_unique<moveit::task_constructor::stages::CurrentState>("current state"));

    task.plan()
}






int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MoveitTaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}






















