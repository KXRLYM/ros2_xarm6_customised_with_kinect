#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "xarm_move/srv/get_target_point.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/point2_d.hpp"

#include "xarm_api/xarm_ros_client.h"

using namespace std::chrono_literals;

typedef float fp32;
struct Point {
    float x;
    float y;
    float z;
};


class XarmMoveClientNode : public rclcpp::Node {

private:

    using GetTargetPoint = xarm_move::srv::GetTargetPoint;
    using GoalHandle = rclcpp_action::ClientGoalHandle<GetTargetPoint>;

    rclcpp::Node::SharedPtr node_handle_;
    //rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    std::thread execution_thread_;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    xarm_api::XArmROSClient xarm_ros_client_;
    Point target_point;
    std::atomic<bool> is_thread_running_;  
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools;

    rclcpp::Client<GetTargetPoint>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    XarmMoveClientNode(const rclcpp::NodeOptions& options) : Node("xarm_move_client_node", options),
    node_handle_(std::shared_ptr<XarmMoveClientNode>(this, [](auto *){})), move_group_interface(node_handle_, "arm_group"),
    planning_scene_interface(""),
    xarm_ros_client_(),
    is_thread_running_(false),
    target_point{0.0f, 0.0f, 0.0f},
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    moveit_visual_tools(node_handle_, "link1", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel())
    {

        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.loadRemoteControl();
        RCLCPP_INFO(this->get_logger(), "Starting Xarm Move Client Node...");

        removeAllCollisionObjects();

        addCollisionObject();

        xarm_ros_client_.init(node_handle_, "xarm");
        
        fp32 gripper_position;
        int result = xarm_ros_client_.get_gripper_position(&gripper_position);
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "Gripper Position: %f", static_cast<float>(gripper_position));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get gripper position");
        }

        /**
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_point",
            10,
            std::bind(&XarmMoveClientNode::detectionCallback, this, std::placeholders::_1)
        );
        **/
        client_ = this->create_client<GetTargetPoint>("get_target_point");
        execution_thread_ = std::thread(&XarmMoveClientNode::execute, this, target_point);

        /**
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(5),  // Set the timer interval to 5 seconds
            [this]() {
                this->sendServiceRequest();
            }
        );
        **/
    }

    int execute(const Point& target_point)
        {
            RCLCPP_INFO(this->get_logger(), "starting executor!");
            auto const logger = rclcpp::get_logger("xarm_move_client_node");
            // auto move_group_interface = MoveGroupInterface(node_handle_, "arm_group");
            move_group_interface.startStateMonitor();
            move_group_interface.setPlanningTime(20.0);
            move_group_interface.setGoalJointTolerance(0.1);
            move_group_interface.setGoalPositionTolerance(0.01);
            move_group_interface.setGoalOrientationTolerance(0.1); 
            auto current_pose = move_group_interface.getCurrentPose();
        
            // Set a target Pose
            auto const target_pose = [&target_point, &current_pose]{
            geometry_msgs::msg::Pose msg;
            msg.orientation.x = current_pose.pose.orientation.x;
            msg.orientation.y = current_pose.pose.orientation.y;
            msg.orientation.z = current_pose.pose.orientation.z;
            msg.orientation.w = current_pose.pose.orientation.w;
            msg.position.x = target_point.x;
            msg.position.y = target_point.y;
            msg.position.z = 0.32;
            return msg;
            }();

            move_group_interface.setPoseTarget(target_pose);
            RCLCPP_INFO(logger, "Target Pose: [%f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);
            auto const prompt = [this](auto text) {
                this->moveit_visual_tools.prompt(text);
            };

            if (rclcpp::ok()) {
                RCLCPP_INFO(logger, "Current Pose: [%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
                RCLCPP_INFO(logger, "Current Orientation: [%f, %f, %f, %f]", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
                // Create a plan to that target pose

                auto const [success, plan] = [&, move_group_interface = std::ref(this->move_group_interface)]() mutable {
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok = static_cast<bool>(move_group_interface.get().plan(msg));
                return std::make_pair(ok, msg);
                }();

                RCLCPP_INFO(logger, "Target Pose: [%f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);

                // Execute the plan
                if(success) {
                    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
                    moveit_visual_tools.trigger();
                    move_group_interface.execute(plan);
                    return 0;
                } else {
                    RCLCPP_ERROR(logger, "Planning failed!");
                    return 1;
                }
            }
            return 1;
        }    

    ~XarmMoveClientNode() {
        // Make sure to join the thread in the destructor to avoid issues
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
    }
    
    void sendServiceRequest() 
    {
        // this->timer_->cancel();
        auto request = std::make_shared<GetTargetPoint::Request>();
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        RCLCPP_INFO(this->get_logger(), "Service avaliable!");
        auto result = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_handle_, result) 
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed");
        } else {
            RCLCPP_INFO(this->get_logger(), "succeeded with %f", result.get()->target_point.point.x);
        }


        // timer_->reset();
        return;
    }

    /**
    void detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        geometry_msgs::msg::PointStamped point_out;
        try {
            if (tf_buffer_.canTransform("world", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(3.0))) {
                tf_buffer_.transform(*msg, point_out, "world", tf2::durationFromSec(1.0));
                target_point.x = point_out.point.x;
                target_point.y = point_out.point.y;
                target_point.z = point_out.point.z;
                // RCLCPP_INFO(this->get_logger(), "Camera frame: [%f, %f, %f]", target_point.x, target_point.y, target_point.z);
                //execution_thread_ = std::thread(&XarmMoveClientNode::execute, this, target_point);
            }
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error %s", ex.what());
            return;
        }         

    }
    **/

    void removeAllCollisionObjects()
    {
        std::vector<std::string> object_ids;
        planning_scene_interface.getObjects(object_ids);

        for (const auto& object_id : object_ids)
        {
            moveit_msgs::msg::CollisionObject remove_object;
            remove_object.id = object_id;
            remove_object.operation = remove_object.REMOVE;

            std::vector<moveit_msgs::msg::CollisionObject> remove_objects;
            remove_objects.push_back(remove_object);

            planning_scene_interface.applyCollisionObjects(remove_objects);
        }
    }

    void addCollisionObject()
    {
        moveit_msgs::msg::CollisionObject desk;
        desk.header.frame_id = move_group_interface.getPlanningFrame();
        desk.id = "desk1";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.6;
        primitive.dimensions[1] = 1.8;
        primitive.dimensions[2] = 0.2;

        /* A pose for the box (specified relative to frame_id) */
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0.1;
        box_pose.position.y = -0.15;
        box_pose.position.z = -0.1676;
        
        desk.primitives.push_back(primitive);
        desk.primitive_poses.push_back(box_pose);
        desk.operation = desk.ADD;        
        
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(desk);
        planning_scene_interface.addCollisionObjects(collision_objects);
        /** to do later: disable collision between slider_link and the desk.
        robot_model_loader::RobotModelLoader::Options options("","");
        robot_model_loader::RobotModelLoader robot_model_loader(node_handle_, options);
        // Create a PlanningSceneMonitor
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node_handle_, &robot_model_loader, "monitor");
        planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();

        this->planning_scene_interface.addCollisionObjects(collision_objects);
        collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
        acm.setEntry("desk1", "slider_link", true);
        **/
        sleep(2.0);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<XarmMoveClientNode>(node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
}