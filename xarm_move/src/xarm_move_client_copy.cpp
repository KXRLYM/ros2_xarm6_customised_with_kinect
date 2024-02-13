#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>   
#include <moveit/kinematic_constraints/utils.h>

#include <memory>
#include <moveit_msgs/msg/robot_state.h>

using namespace std::chrono_literals;
class XarmMoveClientNode : public rclcpp::Node {
private:
    rclcpp::Node::SharedPtr node_handle_;
    bool service_done_ = false;


public:
    XarmMoveClientNode(const rclcpp::NodeOptions & options) : Node("xarm_move_client_node", options),
    node_handle_(std::shared_ptr<XarmMoveClientNode>(this, [](auto *){}))
    {
        RCLCPP_FATAL(this->get_logger(), "yahoooo");
        execute();
    }

    void execute() 
    {
        robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader(node_handle_, "robot_description"));

        planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(node_handle_, robot_model_loader));

        psm->startSceneMonitor();
        psm->startWorldGeometryMonitor();
        psm->startStateMonitor();

        /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
        moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

        /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
        for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
        RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

        std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
        planning_interface::PlannerManagerPtr planner_instance;
        std::vector<std::string> planner_plugin_names;
        if (!this->get_parameter("ompl.planning_plugins", planner_plugin_names))
            RCLCPP_FATAL(this->get_logger(), "Could not find planner plugin names");
        try
        {
            planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", "planning_interface::PlannerManager"));
        }
        catch (pluginlib::PluginlibException& ex)
        {
            RCLCPP_FATAL(this->get_logger(), "Exception while creating planning plugin loader %s", ex.what());
        }

        if (planner_plugin_names.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                        "No planner plugins defined. Please make sure that the planning_plugins parameter is not empty.");
        }

        const auto& planner_name = planner_plugin_names.at(0);
        try
        {
            planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));
            if (!planner_instance->initialize(robot_model, node_handle_,
                                            this->get_namespace()))
            RCLCPP_FATAL(this->get_logger(), "Could not initialize planner instance");
            RCLCPP_INFO(this->get_logger(), "Using planning interface '%s'", planner_instance->getDescription().c_str());
        }
        catch (pluginlib::PluginlibException& ex)
        {
            const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
            std::stringstream ss;
            for (const auto& cls : classes)
            ss << cls << " ";
            RCLCPP_ERROR(this->get_logger(), "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_name.c_str(),
                        ex.what(), ss.str().c_str());
        }


        RCLCPP_ERROR(this->get_logger(), "here");
        planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle_, "xarm6"));
        RCLCPP_ERROR(this->get_logger(), "there");
        planning_interface::MotionPlanRequest req;
        req.pipeline_id = "xarm6";
        req.planner_id = "RRTConnectkConfigDefault";
        req.allowed_planning_time = 1.0;
        req.max_velocity_scaling_factor = 1.0;
        req.max_acceleration_scaling_factor = 1.0;
        planning_interface::MotionPlanResponse res;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "xarm_gripper_base_link";
        pose.pose.position.x = 0.3;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.75;
        pose.pose.orientation.w = 1.0;
        std::vector<double> tolerance_pose(3, 0.1);
        std::vector<double> tolerance_angle(3, 0.1);

        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
        moveit_msgs::msg::DisplayTrajectory display_trajectory;

        /* Visualize the trajectory */
        RCLCPP_INFO(this->get_logger(), "Visualizing the trajectory");
        moveit_msgs::msg::MotionPlanResponse response;
        res.getMessage(response);

        req.group_name = "xarm6";
        moveit_msgs::msg::Constraints pose_goal =
            kinematic_constraints::constructGoalConstraints("kinect", pose, tolerance_pose, tolerance_angle);
        req.goal_constraints.push_back(pose_goal);

        // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
        // representation while planning
        {
            planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
            /* Now, call the pipeline and check whether planning was successful. */
            /* Check that the planning was successful */
            if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code.val != res.error_code.SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not compute plan successfully");
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<XarmMoveClientNode>(node_options);
    RCLCPP_INFO(node->get_logger(), "Starting Xarm Move Client Node...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Shutting down servive..");
    rclcpp::shutdown();

    return 0;
} 