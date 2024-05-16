#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class LivoxRePubNode : public rclcpp::Node {
public:
    LivoxRePubNode() : rclcpp::Node("livox_repub") {
        std::string lid_topic, pcl_topic;
        this->declare_parameter<std::string>("lid_topic", "/livox/lidar");
        this->declare_parameter<std::string>("pcl_topic", "/livox_pcl");
        this->get_parameter<std::string>("lid_topic", lid_topic);
        this->get_parameter<std::string>("pcl_topic", pcl_topic);

        RCLCPP_INFO_STREAM(this->get_logger(), "lid_topic: " << lid_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "pcl_topic: " << pcl_topic);

        sub_livox_msg =
            this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, std::bind(&LivoxRePubNode::LivoxMsgCbk, this, std::placeholders::_1));
        pub_pcl_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(pcl_topic, 100);
    }

    void LivoxMsgCbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in) {
        livox_data.push_back(livox_msg_in);
        if (livox_data.size() < TO_MERGE_CNT) return;

        pcl::PointCloud<PointType> pcl_in;

        for (size_t j = 0; j < livox_data.size(); j++) {
            auto& livox_msg = livox_data[j];
            auto time_end = livox_msg->points.back().offset_time;
            for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
                PointType pt;
                pt.x = livox_msg->points[i].x;
                pt.y = livox_msg->points[i].y;
                pt.z = livox_msg->points[i].z;
                float s = livox_msg->points[i].offset_time / (float)time_end;

                pt.intensity = livox_msg->points[i].line +
                               livox_msg->points[i].reflectivity / 10000.0;  // The integer part is line number and the decimal part is timestamp
                pt.curvature = s * 0.1;
                pcl_in.push_back(pt);
            }
        }

        unsigned long timebase_ns = livox_data[0]->timebase;

        sensor_msgs::msg::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(pcl_in, pcl_ros_msg);
        pcl_ros_msg.header.stamp = rclcpp::Time(timebase_ns, RCL_SYSTEM_TIME);
        pcl_ros_msg.header.frame_id = "/livox";
        pub_pcl_out->publish(pcl_ros_msg);
        livox_data.clear();
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_msg;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out;
    std::vector<livox_ros_driver2::msg::CustomMsg::SharedPtr> livox_data;
    uint64_t TO_MERGE_CNT = 1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxRePubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
