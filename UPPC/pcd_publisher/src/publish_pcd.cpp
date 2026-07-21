#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
 
class PCDPublisher : public rclcpp::Node
{
public:
    PCDPublisher() : Node("pcd_publisher")
    {
        // 创建发布者，话题名为"pcd_topic"，队列长度10
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_topic", 10);
        
        // 定时器，1秒发布一次
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PCDPublisher::timer_callback, this)
        );
 
        // 设置PCD文件路径（替换为你的文件路径）
        // /home/ek/RC2026/mapss/scans.pcd
        pcd_path_ = "/home/ek/RC2026/mapss/scans.pcd";

        // pcd_path_ = "/home/wangce/RvizData/cat.pcd";
        RCLCPP_INFO(this->get_logger(), "PCD文件路径: %s", pcd_path_.c_str());
    }

private:
    void timer_callback()
    {
        // 读取PCD文件
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "无法读取PCD文件！");
            return;
        }
 
        // 转换为ROS 2的PointCloud2消息
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
 
        // 设置消息头（坐标系需与RViz2的Fixed Frame一致）
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->get_clock()->now();
 
        // 发布消息
        publisher_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "发布点云数据 (点数: %d)", cloud->size());
    }
 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string pcd_path_;
};
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPublisher>());
    rclcpp::shutdown();
    return 0;
}