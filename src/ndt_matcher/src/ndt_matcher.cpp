/*/**
 * @file      ndt_matcher.cpp
 * @brief     接受imu_wheel的pose，作为初始值做ndt匹配
 * @author    y.f.duan@outlook.com
 * @date      2024/08/20
 * @version
 * @par Copyright (c):
 *         yufanteck
 * @par History:
 */

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

// Eigen 库头文件
#include <Eigen/Geometry>

// PCL 头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS 2 相关头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// tf2 相关头文件
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

// 项目特定头文件
#include "multigrid_pclomp/multi_voxel_grid_covariance_omp.h"
#include "multigrid_pclomp/multigrid_ndt_omp.h"
#include "util_func.hpp"
#include "pose_array_interpolator.hpp"
#include "ndt_msgs/msg/point_cloud_map_cell_with_id.hpp"

using std::placeholders::_1;

const std::string kMapPath = "/home/dyf/rosbag_0827_imu_wheel_vanjee_南风楼/map.pcd";

using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

/**     @class
 * @brief ndt matcher
 * @author y.f.duan@outlook.com
 * @date      2024/08/29
 */
class NdtMatcher : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_; // 接收imu和wheel融合的pose
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pc_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_; // 发布ndt match pose
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_pc_pub_;               // 发布匹配后坐标转换的点云
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose_in);
    void pcCallback(const sensor_msgs::msg::PointCloud2::UniquePtr pc_ros_in);
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<NormalDistributionsTransform> ndt_ptr_;
    std::mutex *ndt_ptr_mutex_;
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> imu_wheel_pose_msg_ptr_queue_;

public:
    NdtMatcher() : Node("ndt_matcher"), ndt_ptr_(new pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()), ndt_ptr_mutex_(new std::mutex())
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_pose_with_covariance", 5, std::bind(&NdtMatcher::poseCallback, this, _1));
        raw_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_points", 5, std::bind(&NdtMatcher::pcCallback, this, _1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose", 10);
        ndt_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points_tf", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        /**** set ndt params ****/
        pclomp::NdtParams ndt_params;
        ndt_params.trans_epsilon = 0.05;
        ndt_params.step_size = 0.2;
        ndt_params.resolution = 3.0;
        ndt_params.max_iterations = 30;
        ndt_params.num_threads = 8;
        ndt_params.regularization_scale_factor = 0.01;
        ndt_ptr_->setParams(ndt_params);

        /**** map load ****/
        this->mapLoad(kMapPath);
    }

    /**** map update ****/
    void update_map(
        const std::vector<ndt_msgs::msg::PointCloudMapCellWithID> &maps_to_add,
        const std::vector<std::string> &map_ids_to_remove);

    int mapLoad(const std::string &map_path);
    void publish_pose(const rclcpp::Time &sensor_ros_time, const geometry_msgs::msg::Pose &result_pose_msg);
};

/**
 * @brief 接收imu融合wheel的pose
 *
 * @author y.f.duan@outlook.com
 * @date 2024/08/28
 */
void NdtMatcher::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose_in)
{
    imu_wheel_pose_msg_ptr_queue_.push_back(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*pose_in));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "initial pose update");
}

/**
 * @brief 接收点云数据,在imu和wheel估计的pose 队列中找到对齐的pose，以此作为初始pose做ndt match。
 *
 * @param[in] pc_ros_in
 * @param[in] imu_wheel_pose_queue_
 * @param[out] ndt_pose
 *
 * @todo imu_wheel state queue 做插值 , 2024/08/26 插值有点问题，暂时先不用，当插入的值位姿偏差过大会assert fail，并且不确定点云的时间戳在imu时间序列内
 *
 * @author y.f.duan@outlook.com
 * @date 2024/08/20
 */
void NdtMatcher::pcCallback(const sensor_msgs::msg::PointCloud2::UniquePtr pc_ros_in)
{
    /**** 点云下采样并输入到NDT ****/
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 将 ROS 消息转换为 PCL 点云
    pcl::fromROSMsg(*pc_ros_in, *point_cloud);
    // 移除 NaN 点
    pcl::PointCloud<pcl::PointXYZ> filtered_point_cloud;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*point_cloud, filtered_point_cloud, indices);
    // 创建并配置体素网格滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(filtered_point_cloud.makeShared());
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // 设置体素的尺寸 (5cm 的分辨率)
    // 执行滤波
    pcl::PointCloud<pcl::PointXYZ> downsampled_point_cloud;
    voxel_filter.filter(downsampled_point_cloud);
    // 设置 NDT 输入源
    ndt_ptr_->setInputSource(downsampled_point_cloud.makeShared());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "pointcloud input done");
    /**** 点云下采样并输入到NDT ****/

    /**** 队列中找到对齐的pose，以此作为初始pose做ndt match ****/
    // TODO 2024/08/26 插值有点问题，暂时先不用，当插入的值位姿偏差过大会assert fail，并且不确定点云的时间戳在imu时间序列内
    // if (pc_ros_in)
    //     PoseArrayInterpolator interpolator(
    //         this, pc_ros_in->header.stamp, imu_wheel_pose_msg_ptr_queue_, 1.0,
    //         10);
    // if (!interpolator.is_success())
    //     return;
    // pop_old_pose(initial_pose_msg_ptr_array_, pc_ros_in->header.stamp);
    // if (initial_pose_msg_ptr_array_.empty())
    // {
    //     std::cout << "initial_pose_msg_ptr_array_ is empty " << std::endl;
    // }
    static Eigen::Matrix4f previou_ndt_pose;
    Eigen::Matrix4f current_pose;
    auto current_pc_time = rclcpp::Time(pc_ros_in->header.stamp).nanoseconds();
    auto latest_imu_wheel_pose_time = rclcpp::Time(imu_wheel_pose_msg_ptr_queue_.back()->header.stamp).nanoseconds();
    auto oldest_imu_wheel_pose_time = rclcpp::Time(imu_wheel_pose_msg_ptr_queue_.front()->header.stamp).nanoseconds();
    if (!imu_wheel_pose_msg_ptr_queue_.size())
    {
        current_pose = previou_ndt_pose;
        std::cout << "imu_wheel_pose_queue is empty,use previou ndt pose " << std::endl;
    }
    else
    {
        if (current_pc_time > latest_imu_wheel_pose_time)
        {
            current_pose = pose_to_matrix4f(imu_wheel_pose_msg_ptr_queue_.back()->pose.pose);
            imu_wheel_pose_msg_ptr_queue_.clear();
        }
        else if (current_pc_time < oldest_imu_wheel_pose_time)
        {
            current_pose = previou_ndt_pose;
        }
        else
        {
            for (const auto &imu_wheel_pose : imu_wheel_pose_msg_ptr_queue_)
            {
                if (current_pc_time > rclcpp::Time(imu_wheel_pose->header.stamp).nanoseconds())
                {
                    imu_wheel_pose_msg_ptr_queue_.pop_front();
                }
                else
                {
                    current_pose = pose_to_matrix4f(imu_wheel_pose_msg_ptr_queue_.front()->pose.pose);
                }
            }
        }
    }
    /**** 队列中找到对齐的pose，以此作为初始pose做ndt match ****/

    /**** 执行NDT ****/
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ndt_ptr_->align(*output_cloud, pose_to_matrix4f(imu_wheel_pose_msg_ptr_queue_.back()->pose.pose));
    const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();
    previou_ndt_pose = ndt_result.pose;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "ndt done");
    /**** 执行NDT ****/

    /**** publish pose  ****/
    const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
    publish_pose(pc_ros_in->header.stamp, result_pose_msg);

    /**** publish pointcloud 速度有点慢****/
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*output_cloud, ros_cloud);
    ros_cloud.header.frame_id = "odom";               // 设置帧 ID
    ros_cloud.header.stamp = pc_ros_in->header.stamp; // 设置时间戳
    ndt_pc_pub_->publish(ros_cloud);
    /**** publish pointclout result ****/
}

void NdtMatcher::publish_pose(
    const rclcpp::Time &sensor_ros_time, const geometry_msgs::msg::Pose &result_pose_msg)
{
    geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
    result_pose_with_cov_msg.header.stamp = sensor_ros_time;
    result_pose_with_cov_msg.header.frame_id = "odom";
    result_pose_with_cov_msg.pose.pose = result_pose_msg;
    result_pose_with_cov_msg.pose.covariance = {0};

    pose_pub_->publish(result_pose_with_cov_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NdtMatcher>());
    rclcpp::shutdown();
    return 0;
}

/**
 * @brief update map
 * @param[in]
 * @param[out]
 * @return
 * @author y.f.duan@outlook.com
 * @date      2024/08/29
 */
void NdtMatcher::update_map(
    const std::vector<ndt_msgs::msg::PointCloudMapCellWithID> &maps_to_add,
    const std::vector<std::string> &map_ids_to_remove)
{
    RCLCPP_INFO(
        this->get_logger(), "Update map (Add: %lu, Remove: %lu)", maps_to_add.size(), map_ids_to_remove.size());
    if (maps_to_add.empty() && map_ids_to_remove.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Skip map update");
        return;
    }
    const auto exe_start_time = std::chrono::system_clock::now();

    NormalDistributionsTransform backup_ndt = *ndt_ptr_;

    // Add pcd
    for (const auto &map_to_add : maps_to_add)
    {
        pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(map_to_add.pointcloud, *map_points_ptr);
        backup_ndt.addTarget(map_points_ptr, map_to_add.cell_id);
    }

    // Remove pcd
    for (const std::string &map_id_to_remove : map_ids_to_remove)
    {
        backup_ndt.removeTarget(map_id_to_remove);
    }

    backup_ndt.createVoxelKdtree();

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time =
        std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
        1000.0;
    RCLCPP_INFO(this->get_logger(), "Time duration for creating new ndt_ptr: %lf [ms]", exe_time);

    // swap
    (*ndt_ptr_mutex_).lock();
    // ToDo (kminoda): Here negligible NDT copy occurs during the new map loading phase, which should
    // ideally be avoided. But I will leave this for now since I cannot come up with a solution other
    // than using pointer of pointer.
    *ndt_ptr_ = backup_ndt;
    (*ndt_ptr_mutex_).unlock();

    // publish_partial_pcd_map();
}

/**
 * @brief map load
 * @param[in] const std::string &map_path
 * @return
 * @author y.f.duan@outlook.com
 * @date      2024/08/29
 */
int NdtMatcher::mapLoad(const std::string &map_path)
{
    NormalDistributionsTransform new_ndt;
    new_ndt.setParams(ndt_ptr_->getParams());

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file.\n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(map_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f); // 设置体素的尺寸 (这里是 1cm 的分辨率)
    sor.filter(*cloud_filtered);

    new_ndt.setInputTarget(cloud_filtered);
    // swap
    ndt_ptr_mutex_->lock();
    *ndt_ptr_ = new_ndt;
    ndt_ptr_mutex_->unlock();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "map load done");
    return 0;
}
