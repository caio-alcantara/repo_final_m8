#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <array>

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class Go2Base : public rclcpp::Node {
public:
    Go2Base() : Node("go2_base"), offset_calculated_(false) {
        // Init Sport Client
        sport_client_ = std::make_shared<SportClient>();

        // Init TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to Pose
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/utlidar/robot_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                handlePose(msg);
            });

        // Subscribe to Cmd Vel
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handleVelocity(msg);
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    }

private:
    void handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        rclcpp::Time current_ros_time = this->now();
        rclcpp::Time msg_time(msg->header.stamp);

        // Calculate time offset ONCE on the first message
        if (!offset_calculated_) {
            time_offset_ = current_ros_time.nanoseconds() - msg_time.nanoseconds();
            offset_calculated_ = true;
            RCLCPP_INFO(this->get_logger(), "Time offset calculated: %.9f seconds", time_offset_ / 1e9);
        }

        // Apply offset to incoming timestamp
        rclcpp::Time corrected_time(msg_time.nanoseconds() + time_offset_);

        // Prepare TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = corrected_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg->pose.position.x;
        t.transform.translation.y = msg->pose.position.y;
        t.transform.translation.z = msg->pose.position.z;
        t.transform.rotation = msg->pose.orientation;

        // Publish TF
        tf_broadcaster_->sendTransform(t);

        // Prepare Odom
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = corrected_time;
        odom_msg.header.frame_id = t.header.frame_id;
        odom_msg.child_frame_id = t.child_frame_id;
        odom_msg.pose.pose = msg->pose;
        odom_pub_->publish(odom_msg);
    }

    void handleVelocity(const geometry_msgs::msg::Twist::SharedPtr msg) {
        unitree_api::msg::Request req;
        sport_client_->Move(req, msg->linear.x, msg->linear.y, msg->angular.z);
        sport_pub_->publish(req);

        if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0) {
            stopRobot();
        }
    }

    void stopRobot() {
        unitree_api::msg::Request req;
        sport_client_->StopMove(req);
        sport_pub_->publish(req);
    }

    std::shared_ptr<SportClient> sport_client_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
    
    // Time sync variables
    bool offset_calculated_;
    int64_t time_offset_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2Base>());
    rclcpp::shutdown();
    return 0;
}
