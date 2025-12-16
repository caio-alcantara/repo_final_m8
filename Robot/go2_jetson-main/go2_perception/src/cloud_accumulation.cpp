#include <deque>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CloudAccumulationNode : public rclcpp::Node
{
public:
    CloudAccumulationNode() : Node("cloud_accumulation"), offset_calculated_(false)
    {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.reliable();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_deskewed", qos,
            std::bind(&CloudAccumulationNode::cloudCallback, this, std::placeholders::_1));
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. Time Synchronization Strategy (Must match go2_base)
        rclcpp::Time current_ros_time = this->now();
        rclcpp::Time msg_time(msg->header.stamp);

        // Calculate offset once
        if (!offset_calculated_) {
            time_offset_ = current_ros_time.nanoseconds() - msg_time.nanoseconds();
            offset_calculated_ = true;
        }

        // Apply offset to create a corrected timestamp
        rclcpp::Time corrected_time(msg_time.nanoseconds() + time_offset_);

        // 2. Accumulation Logic
        clouds_.push_back(*msg);
        if (clouds_.size() > 20) {
            clouds_.pop_front();
        }

        auto merged_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
        *merged_cloud = clouds_.front(); 

        size_t total_data = 0;
        size_t total_width = 0;
        for (const auto& c : clouds_) {
            total_data += c.data.size();
            total_width += c.width;
        }
        
        if (clouds_.size() > 1) {
            merged_cloud->data.reserve(total_data);
            merged_cloud->width = 0;
            for (const auto& c : clouds_) {
                merged_cloud->data.insert(merged_cloud->data.end(), c.data.begin(), c.data.end());
                merged_cloud->width += c.width;
            }
        }
        
        // 3. APPLY CORRECTED TIMESTAMP
        merged_cloud->header.stamp = corrected_time;
        merged_cloud->header.frame_id = "odom"; 

        // 4. Filter Z
        auto filtered_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
        filtered_cloud->header = merged_cloud->header;
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = false;
        filtered_cloud->is_bigendian = merged_cloud->is_bigendian;
        filtered_cloud->fields = merged_cloud->fields;
        filtered_cloud->point_step = merged_cloud->point_step;
        filtered_cloud->row_step = 0;
        filtered_cloud->width = 0;

        size_t point_step = merged_cloud->point_step;
        size_t z_offset = merged_cloud->fields[2].offset;

        for (size_t i = 0; i < merged_cloud->width * merged_cloud->height; ++i)
        {
            float z;
            if (i * point_step + z_offset + sizeof(float) <= merged_cloud->data.size()) {
                memcpy(&z, &merged_cloud->data[i * point_step + z_offset], sizeof(float));
                if (z >= 0.2 && z <= 1.0) {
                    filtered_cloud->data.insert(
                        filtered_cloud->data.end(),
                        &merged_cloud->data[i * point_step],
                        &merged_cloud->data[(i + 1) * point_step]
                    );
                    filtered_cloud->width++;
                }
            }
        }
        filtered_cloud->row_step = filtered_cloud->data.size();
        pub_->publish(std::move(filtered_cloud));
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::deque<sensor_msgs::msg::PointCloud2> clouds_;
    bool offset_calculated_;
    int64_t time_offset_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudAccumulationNode>());
  rclcpp::shutdown();
  return 0;
}
