#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class FastLivoMapConverter : public rclcpp::Node
{
public:
    FastLivoMapConverter() : Node("fast_livo_map_converter"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // 参数配置
        this->declare_parameter("map_resolution", 0.05);
        this->declare_parameter("map_width", 1000);
        this->declare_parameter("map_height", 1000);
        this->declare_parameter("origin_x", -25.0);
        this->declare_parameter("origin_y", -25.0);
        this->declare_parameter("z_min", -0.1);
        this->declare_parameter("z_max", 0.5);
        this->declare_parameter("voxel_size", 0.1);
        this->declare_parameter("map_update_rate", 1.0);
        
        // 订阅FAST-LIVO2的点云话题
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&FastLivoMapConverter::pointcloudCallback, this, std::placeholders::_1));
        
        // 备用：订阅FAST-LIVO2的地图点话题
        map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map_points", 10,
            std::bind(&FastLivoMapConverter::mapPointsCallback, this, std::placeholders::_1));
        
        // 发布2D占据栅格地图
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        
        // 发布机器人初始位置
        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        
        // 初始化地图
        initializeMap();
        
        // 定时器用于定期发布地图
        map_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / this->get_parameter("map_update_rate").as_double()),
            std::bind(&FastLivoMapConverter::publishMap, this));
        
        RCLCPP_INFO(this->get_logger(), "FAST-LIVO2 Map Converter节点已启动");
    }

private:
    void initializeMap()
    {
        map_.header.frame_id = "map";
        map_.info.resolution = this->get_parameter("map_resolution").as_double();
        map_.info.width = this->get_parameter("map_width").as_int();
        map_.info.height = this->get_parameter("map_height").as_int();
        map_.info.origin.position.x = this->get_parameter("origin_x").as_double();
        map_.info.origin.position.y = this->get_parameter("origin_y").as_double();
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.x = 0.0;
        map_.info.origin.orientation.y = 0.0;
        map_.info.origin.orientation.z = 0.0;
        map_.info.origin.orientation.w = 1.0;
        
        map_.data.resize(map_.info.width * map_.info.height, -1); // -1表示未知
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            return;
        }
        
        // 点云降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.05, 0.05, 0.05);
        voxel_filter.filter(*filtered_cloud);
        
        // 处理点云，更新2D地图
        updateMapFromPointCloud(filtered_cloud);
        
        // 保存点云用于后续更新
        last_pointcloud_ = filtered_cloud;
    }

    void mapPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // FAST-LIVO2可能发布单独的地图点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (!cloud->empty()) {
            updateMapFromPointCloud(cloud);
            map_points_ = cloud;
        }
    }

    void updateMapFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // 参数
        double z_min = this->get_parameter("z_min").as_double();
        double z_max = this->get_parameter("z_max").as_double();
        
        // 临时地图用于处理障碍物
        std::vector<int8_t> temp_map(map_.info.width * map_.info.height, -1);
        
        // 统计每个栅格的占据情况
        std::vector<int> cell_counts(map_.info.width * map_.info.height, 0);
        
        for (const auto& point : cloud->points) {
            // 过滤Z轴范围（地面到障碍物高度）
            if (point.z < z_min || point.z > z_max) {
                continue;
            }
            
            // 转换到地图坐标系
            int map_x = static_cast<int>((point.x - map_.info.origin.position.x) / map_.info.resolution);
            int map_y = static_cast<int>((point.y - map_.info.origin.position.y) / map_.info.resolution);
            
            // 检查边界
            if (map_x >= 0 && map_x < map_.info.width && map_y >= 0 && map_y < map_.info.height) {
                int index = map_y * map_.info.width + map_x;
                cell_counts[index]++;
            }
        }
        
        // 更新地图，根据点数判断占据情况
        for (size_t i = 0; i < cell_counts.size(); ++i) {
            if (cell_counts[i] > 0) {
                map_.data[i] = 100; // 占据
            } else if (map_.data[i] == 100) {
                // 保持之前的占据状态，不清除
            }
        }
        
        // 膨胀障碍物（可选）
        inflateObstacles(1); // 膨胀1个栅格
    }

    void inflateObstacles(int inflation_radius)
    {
        if (inflation_radius <= 0) return;
        
        std::vector<int8_t> inflated_map = map_.data;
        
        for (int y = 0; y < map_.info.height; ++y) {
            for (int x = 0; x < map_.info.width; ++x) {
                int index = y * map_.info.width + x;
                
                if (map_.data[index] == 100) { // 如果是障碍物
                    // 膨胀周围栅格
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            if (nx >= 0 && nx < map_.info.width && ny >= 0 && ny < map_.info.height) {
                                int nidx = ny * map_.info.width + nx;
                                if (inflated_map[nidx] != 100) {
                                    inflated_map[nidx] = 50; // 膨胀区域设为半占据
                                }
                            }
                        }
                    }
                }
            }
        }
        
        map_.data = inflated_map;
    }

    void publishMap()
    {
        map_.header.stamp = this->now();
        map_pub_->publish(map_);
        
        // 首次发布时设置初始位姿
        static bool first_publish = true;
        if (first_publish) {
            setInitialPose();
            first_publish = false;
        }
    }

    void setInitialPose()
    {
        // 尝试获取机器人在map中的初始位姿
        try {
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            
            geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
            initial_pose.header.stamp = this->now();
            initial_pose.header.frame_id = "map";
            initial_pose.pose.pose.position.x = transform.transform.translation.x;
            initial_pose.pose.pose.position.y = transform.transform.translation.y;
            initial_pose.pose.pose.position.z = transform.transform.translation.z;
            initial_pose.pose.pose.orientation = transform.transform.rotation;
            
            // 设置协方差
            initial_pose.pose.covariance[0] = 0.25;  // x
            initial_pose.pose.covariance[7] = 0.25;  // y
            initial_pose.pose.covariance[35] = 0.06853891945200942;  // yaw
            
            initialpose_pub_->publish(initial_pose);
            RCLCPP_INFO(this->get_logger(), "已发布初始位姿: (%.2f, %.2f)", 
                       initial_pose.pose.pose.position.x, 
                       initial_pose.pose.pose.position.y);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "无法获取初始位姿: %s", ex.what());
        }
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    nav_msgs::msg::OccupancyGrid map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLivoMapConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}