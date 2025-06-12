// PCL 头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <vector>
#include <math.h>

void perspectivePointcloud(const sensor_msgs::msg::PointCloud2 &map_pointcloud /*, const geometry_msgs::msg::Pose &pose*/, pcl::PointCloud<pcl::PointXYZ> &output_cloud)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(map_pointcloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    constexpr double level_min_angle = -M_PI;
    constexpr double level_max_angle = M_PI;
    constexpr double angle_increment = 0.001745; // 0.1 度
    const size_t level_angle_bin_size = ((level_max_angle - level_min_angle) / angle_increment) + size_t(1 /*margin*/);

    constexpr double vertical_min_angle = -M_PI / 2;
    constexpr double vertical_max_angle = M_PI / 2;
    const size_t vertical_angle_bin_size = ((vertical_max_angle - vertical_min_angle) / angle_increment) + size_t(1 /*margin*/);

    // Create angle bins
    struct BinInfo
    {
        BinInfo() = default;
        BinInfo(const double _range, const double _wx, const double _wy, const double _wz)
            : range(_range), wx(_wx), wy(_wy), wz(_wz)
        {
        }
        double range;
        double wx;
        double wy;
        double wz;
    };

    // 使用num_bins初始化外层向量，并在内层向量中为每个分割初始化大小
    std::vector<std::vector<BinInfo>> map_pointcloud_angle_bins(level_angle_bin_size, std::vector<BinInfo>(vertical_angle_bin_size));

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(map_pointcloud, "x"),
         iter_y(obstacle_pointcloud, "y"),
         iter_z(obstacle_pointcloud, "z"),
         iter_x != iter_x.end(),
         ++iter_x, ++iter_y, ++iter_z)
    {
        const double level_angle = atan2(*iter_y, *iter_x);
        const double distance = std::sqrt(*iter_x, *iter_y, *iter_z);
        const double vertical_angle = sin(*iter_z, distance);
        int vertical_angle_bin_index = (vertical_angle - vertical_min_angle) / angle_increment;
        int level_angle_bin_index = (level_angle - level_min_angle) / angle_increment;
        if (distance < map_pointcloud_angle_bins[level_angle_bin_index][vertical_angle_bin_index].range)
        {
            BinInfo point(distance, *iter_x, *iter_y, *iter_z);
            map_pointcloud_angle_bins[level_angle_bin_index][vertical_angle_bin_index].push_back(point);
        }
    }

    // 遍历并输出元素
    for (size_t i = 0; i < level_angle_bin_size; ++i)
    {
        for (size_t j = 0; j < vertical_angle_bin_size; ++j)
        {
            pcl::PointXYZ closest_point;
            closest_point.x = map_pointcloud_angle_bins[i][j].wx;
            closest_point.y = map_pointcloud_angle_bins[i][j].wy;
            closest_point.z = map_pointcloud_angle_bins[i][j].wz;
            output_cloud.pushback(point);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("perspective_pointcloud");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Ready.");

    // 读取PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, cloud) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file.");
        return;
    }

    // 转换为ROS 2消息
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "map";    // 设置适当的坐标系
    point_cloud_msg.header.stamp = this->now(); // 设置时间戳

    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    perspectivePointcloud(point_cloud_msg, output_cloud);

    sensor_msgs::msg::PointCloud2 current_view_pointcloud;
    pcl_conversions::fromPCL(output_cloud, current_view_pointcloud);

    rclcpp::Publisher < pcl::PointCloud<pcl::PointXYZ>::SharedPtr pub = node->create_publisher < pcl::PointCloud<pcl::PointXYZ>("topic", 10);
    pub->publish(current_view_pointcloud);

    rclcpp::spin(node);
    //===  spinOnce ===//
    // rclcpp::spin_some()
    //===  end ===//
    //===  MultiThread spin ===//
    //    rclcpp::executors::MultiThreadedExecutor executor;
    //    executor.add_node(node);
    //    executor.spin(); // Uses multiple threads to process callbacks
    //====   end  =====//
    rclcpp::shutdown();
    return 0;
}