#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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


static const rclcpp::Logger LOGGER = rclcpp::get_logger("manipulator");
namespace mtc = moveit::task_constructor;


class MoveitTaskNode : public rclcpp::Node
{
    public:
        MoveitTaskNode();
        void doTask();

        void setupPlanningScene();

    private:

        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_subscriber;
        void commandCallback(const std_msgs::msg::String::SharedPtr msg)
        {   

            if (msg->data == "open_gripper") {
              RCLCPP_INFO(this->get_logger(),"Opening gripper now.....");
            }
            else if (msg->data == "pick_object") {
              RCLCPP_INFO(this->get_logger(),"Picking object now.....");
            }
            else if (msg->data == "turn_right") {
              RCLCPP_INFO(this->get_logger(),"Turning right now.....");
            }
            else if (msg->data == "runs_stage1") {
              RCLCPP_INFO(this->get_logger(),"WILL run stage 1 now");
            } 
            else {
                RCLCPP_INFO(this->get_logger(), "Inside Unknown command received: %s", msg->data.c_str());
            }
        }
};

MoveitTaskNode::MoveitTaskNode() : Node("moveit_task_node")
{
  stage_subscriber = this->create_subscription<std_msgs::msg::String>(
                "/move_manipulator",10, std::bind(&MoveitTaskNode::commandCallback, this, std::placeholders::_1));
        
}
void MoveitTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1, 0.02 };

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = -0.25;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}


void MoveitTaskNode::doTask()
{
  task_ = createTask();
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}


mtc::Task MoveitTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));
  
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>
      ("move_to_pick",mtc::stages::Connect::GroupPlannerVector{ {arm_group_name, sampling_planner} });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick_object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef","group","ik_frame"});

    {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.1, 0.15);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
    }

    {
    // Sample grasp pose
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose("open");
    stage->setObject("object");
    stage->setAngleDelta(M_PI / 12);
    stage->setMonitoredStage(current_state_ptr);  // Hook into current state

    Eigen::Isometry3d grasp_frame_transform;
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    grasp_frame_transform.linear() = q.matrix();
    grasp_frame_transform.translation().z() = 0.1;
     // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object", task.getRobotModel()->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }
  return task;
}




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto mtc_task_node = std::make_shared<MoveitTaskNode>();
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.spin();
  });

  mtc_task_node->setupPlanningScene();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
