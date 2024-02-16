#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "xarm_msgs/srv/get_target_point.hpp"

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

#include "action_msgs/msg/goal_status_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


typedef float fp32;
struct Point {
    float x;
    float y;
    float z;
};

'''
This node receives a target point through ROS2 service "get_target_point" and publishes it. 
Initially using servic was intended such that the target point is only obtained when asked for. 
Then it publishes that target point to move the robot. 
Right now, get_target_point is requested in a while loop. See main().
'''
class XarmServiceNode : public rclcpp::Node {
private:
    using GetTargetPoint = xarm_msgs::srv::GetTargetPoint;
    using GoalHandle = rclcpp_action::ClientGoalHandle<GetTargetPoint>;

    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Client<GetTargetPoint>::SharedPtr client_;
    geometry_msgs::msg::PointStamped target;

public:
    XarmServiceNode() : Node("xarm_service_node"),
    node_handle_(std::shared_ptr<XarmServiceNode>(this, [](auto *){})),    
    {
        // create a node, publisher and a service
        RCLCPP_INFO(this->get_logger(), "Starting Xarm Service Node...!!!!!!!!!!!!!!!!!!!!!!!!!!");
        client_ = this->create_client<GetTargetPoint>("get_target_point");
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_for_execution",10);
        target = geometry_msgs::msg::PointStamped();
    }
    
    // this function sents a request to obtain a target point to move to. 
    void sendServiceRequest() 
    {

        auto request = std::make_shared<GetTargetPoint::Request>();
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        //RCLCPP_INFO(this->get_logger(), "Service avaliable!");
        
        auto result = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_handle_, result) 
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed");
            return;
        } else {
            // if successfully obtained, publish the point
            target = result.get()->target_point;
            RCLCPP_INFO(this->get_logger(), "publishing target point...");
            publisher_->publish(target);
            return;
        }
        return;
    }

    geometry_msgs::msg::PointStamped getTarget()
    {
        return target;
    }

};

'''
This node receives the target point and move the robot.
Receiving the node and moving the robot needed to be in a separate thread (one spinning, one not spinning). 
'''
class XarmMoveClientNode : public rclcpp::Node {
private:

    using GetTargetPoint = xarm_msgs::srv::GetTargetPoint;
    using GoalHandle = rclcpp_action::ClientGoalHandle<GetTargetPoint>;

    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    std::thread execution_thread_;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    xarm_api::XArmROSClient xarm_ros_client_;
    std::shared_ptr<geometry_msgs::msg::PointStamped> target_point;
    std::atomic<bool> is_thread_running_;  
    std::atomic<bool> is_joinable_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools;

    rclcpp::Client<GetTargetPoint>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_subscription_;
    int nothing_count;

public:
    explicit XarmMoveClientNode(const rclcpp::NodeOptions& options) : Node("xarm_move_client_node", options),
    node_handle_(std::shared_ptr<XarmMoveClientNode>(this, [](auto *){})), move_group_interface(node_handle_, "xarm6"),
    planning_scene_interface(""),
    xarm_ros_client_(),
    is_thread_running_(false),
    is_joinable_(false),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    moveit_visual_tools(node_handle_, "link1", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()),
    nothing_count(0)
    {
        // empty the rviz space
        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.loadRemoteControl();
        RCLCPP_INFO(this->get_logger(), "Starting Xarm Move Client Node...");

        // this is not used right now. Planned to use it for overriding the current trajectory with "stop"
        goal_subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>("/xarm6_traj_controller/follow_joint_trajectory/_action/status", 10, std::bind(&XarmMoveClientNode::goalCallback, this, _1));
        
        // populate the rviz workspace with a table
        removeAllCollisionObjects();

        addCollisionObject();

        xarm_ros_client_.init(node_handle_, "xarm");
        
        // testing gripper connection + get a position
        fp32 gripper_position;
        int result = xarm_ros_client_.get_gripper_position(&gripper_position);
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "Gripper Position: %f", static_cast<float>(gripper_position));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get gripper position");
        }

        client_ = this->create_client<GetTargetPoint>("get_target_point");

        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/target_for_execution", 10, std::bind(&XarmMoveClientNode::executionCallBack, this, _1));
    }

    void goalCallback(const action_msgs::msg::GoalStatusArray msg) {
        RCLCPP_INFO(this->get_logger(), "received something................");
    }

    void executionCallBack(const geometry_msgs::msg::PointStamped target_point) {
        // target point is always received. if it is zero, human hand is detected
        if (target_point.point.x + target_point.point.y + target_point.point.z < -0.1) {
            // hand is detected
            RCLCPP_ERROR(this->get_logger(), "HAND DETECTED, NOT MOVING");
            return;
        }

        // thread handling. In order to move the robot, a new thread needs to be created
        if (is_joinable_) {
            execution_thread_.join();
        }
    
        if (!is_thread_running_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(), "creating a thread");
            execution_thread_ = std::thread(&XarmMoveClientNode::execute, this, target_point);
        }
        RCLCPP_INFO(this->get_logger(), "thread is finished!");   
    }

    // thread starts here. The goal is to move the robot to the target point. 
    int execute(const geometry_msgs::msg::PointStamped& target_point)
        {
            is_joinable_ = false;
            RCLCPP_INFO(this->get_logger(), "starting executor!");
            auto const logger = rclcpp::get_logger("xarm_move_client_node");

            // setting the goal tolerances
            move_group_interface.startStateMonitor();
            move_group_interface.setPlanningTime(3.0);
            move_group_interface.setGoalJointTolerance(0.03);
            move_group_interface.setGoalPositionTolerance(0.001);
            move_group_interface.setGoalOrientationTolerance(0.005); 

            // Initialise a target pose transformation. Depth camera link to world so that the robot knows where to move to in the world frame. 
            auto current_pose = move_group_interface.getCurrentPose();
            float x, y, z;
            geometry_msgs::msg::PointStamped point_in = geometry_msgs::msg::PointStamped();
            geometry_msgs::msg::PointStamped point_out;

            // when target point is -1 -1 -1, nothing is detected. If that happens over 5 loop iterations according to 'nothing_count', the robot moves to a default position (hardcoded)
            if (target_point.point.x + target_point.point.y + target_point.point.z < 0.001 && -0.001 < target_point.point.x + target_point.point.y + target_point.point.z ) {
                // nothing is detected
                if (nothing_count < 5) {
                    nothing_count++;
                    RCLCPP_INFO(this->get_logger(), "nothing count: %i", nothing_count);
                    is_thread_running_ = false;
                    is_joinable_ = true;
                    return 0;
                } 
                nothing_count = 0;
                RCLCPP_INFO(this->get_logger(), "Nothing detected! gotta go home");
                point_out = geometry_msgs::msg::PointStamped();
                // hardcoded default position
                point_out.point.x = 0.032821;
                point_out.point.y = 0.270264;
                point_out.point.z = 0.24;
            } else {
                // if some object is detected. Then do transformation then move the robot arm. 
                nothing_count = 0;
                x = target_point.point.x;
                y = target_point.point.y;
                z = target_point.point.z;

                point_in.point.x = x;
                point_in.point.y = y;
                point_in.point.z = z;
                point_in.header.frame_id = "depth_camera_link";

                try {
                    if (tf_buffer_.canTransform("world", "depth_camera_link", tf2::TimePointZero, tf2::durationFromSec(3.0))) {
                        tf_buffer_.transform(point_in, point_out, "world", tf2::durationFromSec(1.0));
                    }
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(this->get_logger(), "Transform error %s", ex.what());
                    return 1;
                }                 
            }
            // Set a target Pose
            auto const target_pose = [this, target_point, &current_pose, point_out]{
                geometry_msgs::msg::Pose msg;
  
                msg.orientation.x = current_pose.pose.orientation.x;
                msg.orientation.y = current_pose.pose.orientation.y;
                msg.orientation.z = current_pose.pose.orientation.z;
                msg.orientation.w = current_pose.pose.orientation.w;
                msg.position.x = point_out.point.x;
                msg.position.y = point_out.point.y;

                // fixing the height to be constant for now.
                msg.position.z = 0.24;
                return msg;
            }();

            
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
                
                // Constraint is used to ensure the robot arm orientation doesn't deviate too much from the initial orientation. 
                moveit_msgs::msg::OrientationConstraint orientation_constraint;
                orientation_constraint.header.frame_id = move_group_interface.get().getPoseReferenceFrame();
                orientation_constraint.link_name = move_group_interface.get().getEndEffectorLink();

                orientation_constraint.orientation = current_pose.pose.orientation;
                orientation_constraint.absolute_x_axis_tolerance = 0.4;
                orientation_constraint.absolute_y_axis_tolerance = 0.4;
                orientation_constraint.absolute_z_axis_tolerance = 0.4;
                orientation_constraint.weight = 1.0;

                moveit_msgs::msg::Constraints orientation_constraints;
                orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
                move_group_interface.get().setPathConstraints(orientation_constraints);
                move_group_interface.get().setPoseTarget(target_pose);
                auto const ok = static_cast<bool>(move_group_interface.get().plan(msg));
                return std::make_pair(ok, msg);
                }();

                RCLCPP_INFO(logger, "Target Pose: [%f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);
                // Execute the plan
                if(success) {
                    move_group_interface.execute(plan);
                    move_group_interface.setStartStateToCurrentState();
                    RCLCPP_INFO(logger, "sucess and now flag is false");
                    is_thread_running_ = false;
                    is_joinable_ = true;
                    return 0;
                } else {
                    RCLCPP_ERROR(logger, "Planning failed!");
                    is_thread_running_ = false;
                    is_joinable_ = true;
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
    
    // Remove all objects from the current scene
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

    // Add collision object. Used to add a table. Dimension is hardcoded.
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
        box_pose.position.y = -0.40;
        box_pose.position.z = -0.11;
        
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

        // wait till the object is loaded.
        sleep(2.0);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Rate loop_rate(1);

    auto node = std::make_shared<XarmMoveClientNode>(node_options);
    auto service_node = std::make_shared<XarmServiceNode>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);

    std::thread executor_thread([executor]() {
        executor->spin();
    });


    std::thread loop_thread([&]() {
        while (rclcpp::ok()) {
            service_node->sendServiceRequest(); 
            loop_rate.sleep();
        }
    });

    loop_thread.join();
    executor_thread.join();
    rclcpp::shutdown();
}