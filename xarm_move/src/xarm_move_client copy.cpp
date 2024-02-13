#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

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

typedef float fp32;
struct Point {
    float x;
    float y;
};

class XarmMoveClientNode : public rclcpp::Node {
private: 
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    //rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr subscription_;
    std::thread execution_thread_;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    xarm_api::XArmROSClient xarm_ros_client_;
    Point target_point;
    std::atomic<bool> is_thread_running_;  
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools;
public:
    XarmMoveClientNode(const rclcpp::NodeOptions& options) : Node("xarm_move_client_node", options),
    node_handle_(std::shared_ptr<XarmMoveClientNode>(this, [](auto *){})), move_group_interface(node_handle_, "arm_group"),
    planning_scene_interface(""),
    xarm_ros_client_(),
    is_thread_running_(false),
    target_point{0.0f, 0.0f},
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

        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_point",
            10,
            std::bind(&XarmMoveClientNode::detectionCallback, this, std::placeholders::_1)
        )
        /**
        subscription_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
            "/detection_transformed",
            10,
            std::bind(&XarmMoveClientNode::detectionCallback, this, std::placeholders::_1)
        );
        **/
    }

    ~XarmMoveClientNode() {
        // Make sure to join the thread in the destructor to avoid issues
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
    }

    void detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        geometry_msgs::msg::PointStamped point_out;
        try {
            if (tf_buffer_.canTransform("world", msg.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(3.0))) {
                tf_buffer_.transform(point_in, point_out, "world", tf2::durationFromSec(1.0));
                target_point.x = point_out.point.x;
                target_point.y = point_out.point.y;
                RCLCPP_INFO(this->get_logger(), "Camera frame: [%f, %f]", target_point.x, target_point.y);
                //execution_thread_ = std::thread(&XarmMoveClientNode::execute, this, target_point);
                if (!is_thread_running_.exchange(true)) {
                    
                    execution_thread_ = std::thread(&XarmMoveClientNode::execute, this, target_point);

                }

            }
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error %s", ex.what());
            return;
        }     
    }

    /**

    void detectionCallback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg){
        float sum_x = 0.0;
        float sum_y = 0.0;
        int num_point = 0; 
        std::string detection_name = "cat";

        for (const auto& detection : msg->detections) {
            if (!detection.mask.data.empty() && detection.class_name == detection_name) {
                for (const auto& point : detection.mask.data) {  
                    num_point++;
                    sum_x += point.x;
                    sum_y += point.y;
                }
            }

            float average_x = (num_point > 0) ? sum_x / num_point : 0.0;  // Added a check to avoid division by zero
            float average_y = (num_point > 0) ? sum_y / num_point : 0.0;  // Added a check to avoid division by zero

            if (!(average_x == 0.0 && average_y == 0.0)) {
                RCLCPP_INFO(this->get_logger(), "Centeroid: [%f, %f]", average_x, average_y);
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header.stamp = this->now();
                point_in.header.frame_id = "depth_camera_link";
                
                std::array<float, 2> camera_coords = pixelToCameraFrame(average_x, average_y);
                point_in.point.x = camera_coords[0];
                point_in.point.y = camera_coords[1];
                point_in.point.z = 0.0;

                try {
                    if (tf_buffer_.canTransform("world", point_in.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(3.0))) {
                        tf_buffer_.transform(point_in, point_out, "world", tf2::durationFromSec(1.0));
                        target_point.x = point_out.point.x;
                        target_point.y = point_out.point.y;
                        RCLCPP_INFO(this->get_logger(), "Camera frame: [%f, %f]", target_point.x, target_point.y);
                        //execution_thread_ = std::thread(&XarmMoveClientNode::execute, this, target_point);
                        if (!is_thread_running_.exchange(true)) {
                            
                            execution_thread_ = std::thread(&XarmMoveClientNode::execute, this, target_point);

                        }

                    }
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(this->get_logger(), "Transform error %s", ex.what());
                    return;
                }
            }
        }

    }



    std::array<float, 2> pixelToCameraFrame(float pixel_x, float pixel_y, float fx = 973.252, float fy = 972.92, float cx = 1025.34, float cy = 775.605)
    {
        // Apply the pinhole camera model
        float camera_x = ((pixel_x - cx) / fx);
        float camera_y = ((pixel_y - cy) / fy);
        std::array<float, 2> camera_coords = {camera_x, camera_y};
        return camera_coords;
    }
    **/
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
        RCLCPP_INFO(this->get_logger(), "dying?");  
        auto current_pose = move_group_interface.getCurrentPose();
    
        // Set a target Pose
        auto const target_pose = [&target_point, &current_pose]{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = current_pose.pose.orientation.x;
        msg.orientation.y = current_pose.pose.orientation.y;
        msg.orientation.z = current_pose.pose.orientation.z;
        msg.orientation.w = current_pose.pose.orientation.w;
        msg.position.x = target_point.y;
        msg.position.y = target_point.x;
        msg.position.z = 0.32;
        return msg;
        }();

        move_group_interface.setPoseTarget(target_pose);
        RCLCPP_INFO(logger, "Target Pose: [%f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);

        /**
        auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
            node_handle_, "link1", rviz_visual_tools::RVIZ_MARKER_TOPIC,
            move_group_interface.getRobotModel()};
        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.loadRemoteControl();

        auto const draw_title = [&moveit_visual_tools](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);
        };

        **/
        auto const prompt = [this](auto text) {
            this->moveit_visual_tools.prompt(text);
        };

        /**
        auto const draw_trajectory_tool_path =
            [&moveit_visual_tools,
            jmg = move_group_interface.getRobotModel()->getJointModelGroup(
                "arm_group")](auto const trajectory) {
            moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
            };
        **/
        
        if (rclcpp::ok()) {
            RCLCPP_INFO(logger, "Current Pose: [%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
            RCLCPP_INFO(logger, "Current Orientation: [%f, %f, %f, %f]", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
            // Create a plan to that target pose

            /**
            prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
            draw_title("Planning");
            moveit_visual_tools.trigger();
            **/
            auto const [success, plan] = [&, move_group_interface = std::ref(this->move_group_interface)]() mutable {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.get().plan(msg));
            return std::make_pair(ok, msg);
            }();

            RCLCPP_INFO(logger, "Target Pose: [%f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);

            // Execute the plan
            if(success) {
                //draw_trajectory_tool_path(plan.trajectory);
                //moveit_visual_tools.trigger();
                prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
                // draw_title("Executing");
                moveit_visual_tools.trigger();
                move_group_interface.execute(plan);
                is_thread_running_ = false;
                return 0;
            } else {
                RCLCPP_ERROR(logger, "Planning failed!");
                return 1;
            }
        }
        is_thread_running_ = false;
        return 1;
    }

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