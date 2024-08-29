/*/**
 * @file      my_kalman_node.cpp
 * @brief     融合IMU和wheel的数据，得到融合后的pose和twist，并通过接收ndt的pose来更新pose
 * @author    y.f.duan@outlook.com
 * @date      2024/08/16
 * @version
 * @par Copyright (c):
 *         yufanteck
 * @par History:
 */
#define DEBUG // 打印调试信息
#include <deque>
#include <map>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

const double kGyroNoise = 1e-6; // 单位m/s
const double kAccNoise = 1e-6;  // 单位m/s^2
const double kWheelNoise = 0.01;

const double kEpsilon = 1e-6; // 无穷小值
const double kG = 9.81;       // 单位是 m/s^2，表示重力加速度

const double kDeltaT = 0.01; // 单位 s

/**     @class
 * @brief imu 融合 wheel
 * @author y.f.duan@outlook.com
 * @date      2024/08/29
 */
class ImuKalmanNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_baselink_tf_broadcaster_;

    void imuSubCallback(const sensor_msgs::msg::Imu::UniquePtr imu_in);
    void wheelSubCallback(const nav_msgs::msg::Odometry::UniquePtr wheel_in);
    void poseSubCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose_in);
    void publishPose();
    void rpyStatePredict(const Eigen::Vector3d &gyro_in);
    void rpyStateUpdate(const Eigen::Vector3d &acc_in);
    void positionStatePredict(const Eigen::Vector2d &wheel_in);

    /**********************(rpy EKF)*********************/
    // Roll, Pitch, Yaw (RPY) state vector
    Eigen::Vector3d rpy_state_{0, 0, 0};

    // rpy State transition matrix (identity matrix)
    Eigen::Matrix3d rpy_state_transition_matrix_{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}};

    // rpy state input matrix
    Eigen::Matrix3d rpy_state_input_matrix_{{kDeltaT * 5, 0, 0},
                                            {0, kDeltaT * 5, 0},
                                            {0, 0, kDeltaT * 5}};

    // rpy State covariance matrix (initially small)
    Eigen::Matrix3d rpy_state_covariance_matrix_{{kEpsilon, 0, 0},
                                                 {0, kEpsilon, 0},
                                                 {0, 0, kEpsilon}};

    // gyro Input noise covariance matrix
    Eigen::Matrix3d gyro_input_noise_covariance_matrix_{
        {kGyroNoise * kGyroNoise, 0, 0},
        {0, kGyroNoise *kGyroNoise, 0},
        {0, 0, kGyroNoise *kGyroNoise}};

    // acc Observation noise covariance matrix
    Eigen::Matrix3d acc_observation_noise_covariance_matrix_{
        {kAccNoise * kAccNoise, 0, 0},
        {0, kAccNoise *kAccNoise, 0},
        {0, 0, kAccNoise *kAccNoise}};

    // 观测矩阵线性化后的雅可比矩阵
    // Jacobian matrix of the rpy state observation model
    Eigen::Matrix3d rpy_state_observation_jacobian_matrix_;
    /*********************(rpy EKF)*********************/

    /**********************(position predict)*********************/
    // X,Y,Z state vector
    Eigen::Vector3d position_state_{0, 0, 0}; // xyz

    // position State covariance matrix (initially small)
    Eigen::Matrix3d position_state_covariance_matrix_{{kEpsilon, 0, 0},
                                                      {0, kEpsilon, 0},
                                                      {0, 0, kEpsilon}};
    // positon State transition matrix (identity matrix)
    Eigen::Matrix3d position_state_transition_matrix_{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}};

    // Position input matrix
    Eigen::Matrix<double, 3, 2> position_state_input_matrix_;

    // Wheel input noise covariance matrix
    Eigen::Matrix3d wheel_input_noise_covariance_matrix_{
        {kWheelNoise * kWheelNoise, 0, 0},
        {0, kWheelNoise *kWheelNoise, 0},
        {0, 0, 0}};

    /**********************(position predict)********************/

    // state timestamp queue 通过这个队列来保存历史状态
    std::deque<double> state_timestamp_queue_;

    // rpy history State vector map with timestamps ,key is timestamp valve is state
    std::map<double, Eigen::Vector3d> rpy_state_map_;

    // position State vector history map with timestamps
    std::map<double, Eigen::Vector3d> position_state_map_;

    // vehicle twist data queue
    std::deque<geometry_msgs::msg::TwistWithCovariance> vehicle_twist_queue_;

    // imu data queue
    std::deque<sensor_msgs::msg::Imu> imu_queue_;

public:
    ImuKalmanNode() : Node("my_kalman_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/Imu_data", 10, std::bind(&ImuKalmanNode::imuSubCallback, this, _1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose", 10, std::bind(&ImuKalmanNode::poseSubCallback, this, _1));
        twist_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ImuKalmanNode::wheelSubCallback, this, _1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_pose_with_covariance", 10);
        odom_baselink_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    void performEkfEstimation(const Eigen::Vector3d &gyro_in, const Eigen::Vector3d &acc_in);

    geometry_msgs::msg::TwistWithCovarianceStamped concatGyroAndOdometer(
        const std::deque<geometry_msgs::msg::TwistWithCovariance> &vehicle_twist_queue,
        const std::deque<sensor_msgs::msg::Imu> &gyro_queue);
};

Eigen::Quaterniond rpyToQuaternion(double roll_rad, double pitch_rad, double yaw_rad)
{
    // Create rotation matrices for each axis
    Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());

    // Combine the rotations
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}

/**
 * @brief 处理IMU数据的回调函数, 融合imu 和 轮速计 得到pose 和twist
 * 1. 传入的IMU数据，将其存入队列
 * 2. 处理imu_queue和vehicle_twsit_queue 的数据，通过concatGyroAndOdometer函数得到较为准确的twist数据
 * 3. 使用twist的角速度和imu加速度执行EKF得到准确的rpy_state
 * 4. 使用twist的线速度来预测position
 * 5. 将rpy position_state_ pushback到rpy_state_map_ 和position_state_map_
 * 5. 发布pose和tf
 *
 * @todo 只发布了pose没有发布twist
 * @param[in] imu_in
 * @param[in] wheel_in
 * @param[out] pose_with_cov_stamp
 * @param[out] twist_with_cov_stamp
 *
 * @author y.f.duan@outlook.com
 * @date 2024/08/28
 */
void ImuKalmanNode::imuSubCallback(const sensor_msgs::msg::Imu::UniquePtr imu_in)
{
    // 将接收到的 IMU 数据推入队列
    imu_queue_.push_back(*imu_in);
    // 当队列中积累了 5 条 IMU 数据时进行处理
    if (imu_queue_.size() >= 5 && vehicle_twist_queue_.size() > 0)
    {
        Eigen::Vector3d acc_sum(0, 0, 0); // 单位为m/s^2
        Eigen::Vector3d gyro(0, 0, 0);    // 单位为m/s^2

        // 累加加速度数据,均值滤波
        for (auto &imu : imu_queue_)
        {
            acc_sum[0] += imu.linear_acceleration.x;
            acc_sum[1] += imu.linear_acceleration.y;
            acc_sum[2] += imu.linear_acceleration.z;
        }
        // 将 IMU 数据和轮速计数据结合，得到包含协方差的 twist 数据
        auto twist_with_cov_ = concatGyroAndOdometer(vehicle_twist_queue_, imu_queue_);

        gyro[0] = twist_with_cov_.twist.twist.angular.x;
        gyro[1] = twist_with_cov_.twist.twist.angular.y;
        gyro[2] = twist_with_cov_.twist.twist.angular.z;
        // 通过角速度和加速度执行EKF

        rpy_state_input_matrix_ << kDeltaT * imu_queue_.size(), 0, 0,
            0, kDeltaT * imu_queue_.size(), 0,
            0, 0, kDeltaT * imu_queue_.size();

        performEkfEstimation(gyro, acc_sum / imu_queue_.size());

        Eigen::Vector2d velocity_and_angular(twist_with_cov_.twist.twist.linear.x, twist_with_cov_.twist.twist.angular.z);
        // 通过线速度和角速度到到position

        position_state_input_matrix_ << kDeltaT * imu_queue_.size() * cos(rpy_state_[2]), 0,
            kDeltaT * imu_queue_.size() * sin(rpy_state_[2]), 0,
            0, 0;
        positionStatePredict(velocity_and_angular);

        /**** 清除imu和轮速计队列 ****/
        imu_queue_.clear();
        vehicle_twist_queue_.clear();

        /**** 将rpy state和position state 添加到map中 ****/
        rpy_state_map_[rclcpp::Time(imu_in->header.stamp).nanoseconds()] = rpy_state_;
        state_timestamp_queue_.push_back(rclcpp::Time(imu_in->header.stamp).nanoseconds());
        position_state_map_[rclcpp::Time(imu_in->header.stamp).nanoseconds()] = position_state_;

        /***************************** 发布pose ******************************/
        geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_stamp;
        pose_with_cov_stamp.header.stamp = imu_in->header.stamp;
        pose_with_cov_stamp.header.frame_id = "odom";
        Eigen::Quaterniond q = rpyToQuaternion(rpy_state_[0], rpy_state_[1], rpy_state_[2]);

        pose_with_cov_stamp.pose.pose.orientation.x = q.x();
        pose_with_cov_stamp.pose.pose.orientation.y = q.y();
        pose_with_cov_stamp.pose.pose.orientation.z = q.z();
        pose_with_cov_stamp.pose.pose.orientation.w = q.w();
        pose_with_cov_stamp.pose.pose.position.x = position_state_[0];
        pose_with_cov_stamp.pose.pose.position.y = position_state_[1];
        pose_with_cov_stamp.pose.pose.position.z = position_state_[2];
        // 将位移协方差矩阵赋值给 pose.covariance 的前 3x3 部分
        pose_with_cov_stamp.pose.covariance[0] = position_state_covariance_matrix_(0, 0);
        pose_with_cov_stamp.pose.covariance[1] = position_state_covariance_matrix_(0, 1);
        pose_with_cov_stamp.pose.covariance[2] = position_state_covariance_matrix_(0, 2);
        pose_with_cov_stamp.pose.covariance[6] = position_state_covariance_matrix_(1, 0);
        pose_with_cov_stamp.pose.covariance[7] = position_state_covariance_matrix_(1, 1);
        pose_with_cov_stamp.pose.covariance[8] = position_state_covariance_matrix_(1, 2);
        pose_with_cov_stamp.pose.covariance[12] = position_state_covariance_matrix_(2, 0);
        pose_with_cov_stamp.pose.covariance[13] = position_state_covariance_matrix_(2, 1);
        pose_with_cov_stamp.pose.covariance[14] = position_state_covariance_matrix_(2, 2);
        // 将姿态协方差矩阵赋值给 pose.covariance 的后 3x3 部分
        pose_with_cov_stamp.pose.covariance[21] = rpy_state_covariance_matrix_(0, 0);
        pose_with_cov_stamp.pose.covariance[22] = rpy_state_covariance_matrix_(0, 1);
        pose_with_cov_stamp.pose.covariance[23] = rpy_state_covariance_matrix_(0, 2);
        pose_with_cov_stamp.pose.covariance[27] = rpy_state_covariance_matrix_(1, 0);
        pose_with_cov_stamp.pose.covariance[28] = rpy_state_covariance_matrix_(1, 1);
        pose_with_cov_stamp.pose.covariance[29] = rpy_state_covariance_matrix_(1, 2);
        pose_with_cov_stamp.pose.covariance[33] = rpy_state_covariance_matrix_(2, 0);
        pose_with_cov_stamp.pose.covariance[34] = rpy_state_covariance_matrix_(2, 1);
        pose_with_cov_stamp.pose.covariance[35] = rpy_state_covariance_matrix_(2, 2);
        pose_pub_->publish(pose_with_cov_stamp);
        /***************************** 发布pose ******************************/

        /************************ 发布 odom2baselink tf *********************/
        geometry_msgs::msg::TransformStamped t; // 四元数+平移

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = position_state_[0];
        t.transform.translation.y = position_state_[1];
        t.transform.translation.z = position_state_[2];
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        odom_baselink_tf_broadcaster_->sendTransform(t);
        /****************************** 发布tf ****************************/
    }
}

void ImuKalmanNode::wheelSubCallback(const nav_msgs::msg::Odometry::UniquePtr wheel_in)
{
    vehicle_twist_queue_.push_back(wheel_in->twist);
}

/**
 * @brief 接收ndt位姿消息并对齐到imu和wheel估计的pose 队列中，并更新队列的pose。
 *
 * @param[in] pose_in
 * @param[out] rpy_state_
 * @param[out] position_state_
 * @param[out] rpy_state_map_
 * @param[out] position_state_map_
 *
 * @bug euler angle 存在奇异性问题，只要保证roll和pitch在[-π/2, π/2] 范围内就能保证euler angle的唯一性
 * @bug ndt 的pose time按理来说应该在imu time queue之中，但会出现在之外的情况
 *
 * @warning ndt pose time > imu latest time 会造成pose无法更新，从而累计误差
 *
 * @todo Replace Euler angle representation with quaternions to avoid singularity issues.
 * @todo imu_wheel state queue 做插值进行更新.
 *
 * @author y.f.duan@outlook.com
 * @date 2024/08/29
 */
void ImuKalmanNode::poseSubCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose_in)
{
    /*********** pose2eular *************/
    Eigen::Quaterniond quaternion(pose_in->pose.pose.orientation.w, pose_in->pose.pose.orientation.x, pose_in->pose.pose.orientation.y, pose_in->pose.pose.orientation.z); // 四元数 (w, x, y, z)
    // 将四元数转换为 3x3 旋转矩阵
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
    // 使用 Eigen 提供的函数将旋转矩阵转换为欧拉角（RPY）
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0, 1, 2); //  Roll (X),Pitch (Y),Yaw (Z)
    /*********** pose2eular *************/

    /**** pose转化为eular角时存在奇异性，将roll和pitch限制在 [-π/2, π/2] 范围内****/
    // TODO 2024/08/26 欧拉角存在奇异性导致同样的旋转存在多个欧拉角，后面替换为四元数
    // 确保 eular_angles 在 [-π, π] 范围内
    euler_angles[0] = std::fmod(euler_angles[0] + M_PI, 2 * M_PI) - M_PI;
    euler_angles[1] = std::fmod(euler_angles[1] + M_PI, 2 * M_PI) - M_PI;
    euler_angles[2] = std::fmod(euler_angles[2] + M_PI, 2 * M_PI) - M_PI;
    // 将 roll 和 pitch 限制在 [-π/2, π/2] 范围内
    if (euler_angles[1] >= M_PI_2)
    {
        euler_angles[1] = M_PI - euler_angles[1];
        euler_angles[0] += M_PI;
        euler_angles[2] += M_PI;
    }
    else if (euler_angles[1] < -M_PI_2)
    {
        euler_angles[1] = -M_PI - euler_angles[1];
        euler_angles[0] += M_PI;
        euler_angles[2] += M_PI;
    }
    // 将 roll 和 yaw 角度标准化到 [-π, π] 范围
    euler_angles[0] = std::fmod(euler_angles[0] + M_PI, 2 * M_PI) - M_PI;
    euler_angles[2] = std::fmod(euler_angles[2] + M_PI, 2 * M_PI) - M_PI;
    if (fabs(euler_angles[0]) > 1 || fabs(euler_angles[1]) > 1)
    {
        std::cout << "ndt error ,skip the pose" << std::endl;
        return;
    }
    /**** pose转化为eular角时存在奇异性，将roll和pitch限制在 [-π/2, π/2] 范围内****/

    /*********** ndt pose time aligned to imu queue time  ***********/
    double latest_imu_time = state_timestamp_queue_.back();
    double oldest_imu_time = state_timestamp_queue_.front();
    double aligned_to_imu_time;
    double ndt_pose_time = rclcpp::Time(pose_in->header.stamp).nanoseconds();
    if (ndt_pose_time > latest_imu_time)
    {
        aligned_to_imu_time = latest_imu_time;
        std::cout << "ndt pose timestamp > imu latest timestamp" << std::endl;

        /**** 清空之前的状态 ****/
        rpy_state_map_.clear();
        position_state_map_.clear();
        std::deque<double> empty;
        std::swap(state_timestamp_queue_, empty);
        return;
    }
    else if (ndt_pose_time < oldest_imu_time)
    {
        std::cout << "ndt pose timestamp < imu oldest timestamp" << std::endl;
        ndt_pose_time = oldest_imu_time;
    }
    else
    {
        for (const auto &state_time : state_timestamp_queue_)
        {
            /**** clear old state ****/
            if (ndt_pose_time >= state_time)
            {
                state_timestamp_queue_.pop_front();
                rpy_state_map_.erase(state_time);
                position_state_map_.erase(state_time);
            }
            else
            {
                aligned_to_imu_time = state_time;
                break;
            }
        }
    }
    /*********** ndt pose time aligned to imu pose queue time  ***********/

    /**** update current state ****/
    Eigen::Vector3d delta_rpy_state = rpy_state_map_[latest_imu_time] - rpy_state_map_[aligned_to_imu_time];
    Eigen::Vector3d delta_position_state = position_state_map_[latest_imu_time] - position_state_map_[aligned_to_imu_time];
    rpy_state_ = euler_angles;
    rpy_state_ += delta_rpy_state;
    position_state_[0] = pose_in->pose.pose.position.x;
    position_state_[1] = pose_in->pose.pose.position.y;
    position_state_[2] = pose_in->pose.pose.position.z;
    auto current_position_state = position_state_;
    position_state_ += delta_position_state;
    auto aligned_position_state = position_state_map_[aligned_to_imu_time]; // save current state , to update state queue
    auto aligned_rpy_state = rpy_state_map_[aligned_to_imu_time];

    /**** update state queue ****/
    for (const auto &state_time : state_timestamp_queue_)
    {
        delta_rpy_state = rpy_state_map_[state_time] - aligned_rpy_state;
        delta_position_state = position_state_map_[state_time] - aligned_position_state;
        rpy_state_map_[state_time] = euler_angles + delta_rpy_state;
        position_state_map_[state_time] = current_position_state + delta_position_state;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "initial pose update");

#ifdef DEBUG
    std::cout << "rpy_state_: " << rpy_state_ << std::endl;
    std::cout << "position state_: " << position_state_ << std::endl;
#endif
}

/**
 * @brief rpy EKF predict
 *
 * @param[in] gyro_in
 * @param[out] rpy_state_
 *
 * @author y.f.duan@outlook.com
 * @date 2024/08/29
 */
void ImuKalmanNode::rpyStatePredict(const Eigen::Vector3d &gyro_in) // u_k =gyro.xyz
{
    /**** 状态预测 ****/
    rpy_state_ = rpy_state_transition_matrix_ * rpy_state_ + rpy_state_input_matrix_ * gyro_in;
    /**** 状态协方差预测 ****/
    rpy_state_covariance_matrix_ = rpy_state_transition_matrix_ * rpy_state_covariance_matrix_ * rpy_state_transition_matrix_.transpose() + gyro_input_noise_covariance_matrix_;

#ifdef DEBUG
    /**********************(Debug cout)*********************/
    std::cout << "predict end" << std::endl;
    std::cout << "rpy_state_= " << rpy_state_ << std::endl;
    /**********************(Debug END)********************/
#endif
}

/**
 * @brief rpy EKF update
 *
 * @param[in] acc_in
 * @param[out] rpy_state_
 *
 * @author y.f.duan@outlook.com
 * @date 2024/08/29
 */
void ImuKalmanNode::rpyStateUpdate(const Eigen::Vector3d &acc_in) // z_k =acc.xyz
{
    Eigen::Matrix3d E{
        {exp(fabs(acc_in.norm() - kG) / 100000) - 1, 0, 0},
        {0, exp(fabs(acc_in.norm() - kG / 100000)) - 1, 0},
        {0, 0, exp(fabs(acc_in.norm() - kG / 100000)) - 1}};

    rpy_state_observation_jacobian_matrix_ << 0, -1 * kG * cos(rpy_state_[1]), 0,
        kG * cos(rpy_state_[1]) * cos(rpy_state_[0]), -1 * kG * sin(rpy_state_[1]) * sin(rpy_state_[0]), 0,
        -1 * kG * cos(rpy_state_[1]) * sin(rpy_state_[0]), -1 * kG * sin(rpy_state_[1]) * cos(rpy_state_[0]), 0; // 观测方程线性化的雅可比矩阵
    Eigen::Matrix3d kalman_gain;
    kalman_gain = rpy_state_covariance_matrix_ * rpy_state_observation_jacobian_matrix_.transpose() * (rpy_state_observation_jacobian_matrix_ * rpy_state_covariance_matrix_ * rpy_state_observation_jacobian_matrix_.transpose() + acc_observation_noise_covariance_matrix_ + E).inverse(); // 求卡尔曼增益，eigen的求逆会根据矩阵类型自动选择求逆方法
    /**** 状态更新 ****/
    rpy_state_ = rpy_state_ + kalman_gain * (acc_in - Eigen::Vector3d(-1 * kG * cos(rpy_state_[0]) * sin(rpy_state_[1]), kG * sin(rpy_state_[0]), kG * cos(rpy_state_[0]) * cos(rpy_state_[1])));
    /**** 协方差更新 ****/

    rpy_state_covariance_matrix_ = (Eigen::Matrix3d::Identity() - kalman_gain * rpy_state_observation_jacobian_matrix_) * rpy_state_covariance_matrix_;

#ifdef DEBUG
    std::cout << "update end" << std::endl;
    std::cout << "rpy_state_= " << rpy_state_ << std::endl;
#endif
}

/**
 * @brief rpy EKF preform
 *
 * @param[in] acc_in
 * @param[in] gyro_in
 * @param[out] rpy_state_
 *
 * @author y.f.duan@outlook.com
 * @date 2024/08/29
 */
void ImuKalmanNode::performEkfEstimation(const Eigen::Vector3d &gyro_in, const Eigen::Vector3d &acc_in) // gyro.xyz
{
    rpyStatePredict(gyro_in);
    rpyStateUpdate(acc_in);
    // 将 roll 和 yaw 角度标准化到 [-π, π] 范围
    rpy_state_[0] = std::fmod(rpy_state_[0] + M_PI, 2 * M_PI) - M_PI;
    rpy_state_[1] = std::fmod(rpy_state_[1] + M_PI, 2 * M_PI) - M_PI;
    rpy_state_[2] = std::fmod(rpy_state_[2] + M_PI, 2 * M_PI) - M_PI;
}

void ImuKalmanNode::positionStatePredict(const Eigen::Vector2d &wheel_in)
{
    // position_state_input_matrix_ = Eigen::Matrix<double, 3, 2>{
    //     {kDeltaT * 5 * cos(rpy_state_[2]), 0},
    //     {kDeltaT * 5 * sin(rpy_state_[2]), 0},
    //     {0, 0}};
    /**** 状态预测 ****/
    position_state_ = position_state_transition_matrix_ * position_state_ + position_state_input_matrix_ * wheel_in;
    /**** 状态协方差预测 ****/
    position_state_covariance_matrix_ = position_state_transition_matrix_ * position_state_covariance_matrix_ * position_state_transition_matrix_.transpose() + wheel_input_noise_covariance_matrix_;

#ifdef DEBUG
    std::cout << "position predict" << position_state_ << std::endl;
#endif
}

/**
 * @brief 融合车辆里程计和 IMU（gyro）的数据生成更准确的twist，gyro得到角速度，wheel得到线速度,通过加权平均
 * @param[in] vehicle_twist_queue
 * @param[in] gyro_queue
 * @param[out] twist_with_cov
 *
 * @todo 由于轮速计没有协方差，暂时使用固定值
 * @todo 只用了imu的时间戳
 *
 * @author y.f.duan@outlook.com
 * @date      2024/08/26
 */
geometry_msgs::msg::TwistWithCovarianceStamped ImuKalmanNode::concatGyroAndOdometer(
    const std::deque<geometry_msgs::msg::TwistWithCovariance> &vehicle_twist_queue,
    const std::deque<sensor_msgs::msg::Imu> &gyro_queue)
{
    // using COV_IDX_XYZ = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;
    // using COV_IDX_XYZRPY = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

    double vx_mean = 0;
    geometry_msgs::msg::Vector3 w_mean{};
    double vx_covariance_original = 0;
    geometry_msgs::msg::Vector3 w_covariance_original{};
    for (const auto &vehicle_twist : vehicle_twist_queue)
    {
        vx_mean += vehicle_twist.twist.linear.x;
        // vx_covariance_original += vehicle_twist.twist.covariance[0 * 6 + 0];
        // TODO 2024/08/26 由于轮速计没有协方差，暂时使用固定值
        vx_covariance_original += 1e-6 * 1e-6;
        w_mean.z += vehicle_twist.twist.angular.z; // TODO 2024/08/26 权重暂时设置与imu一样
        w_covariance_original.z += 1e-6 * 1e-6;
    }
    vx_mean /= vehicle_twist_queue.size();
    vx_covariance_original /= vehicle_twist_queue.size();

    for (const auto &gyro : gyro_queue)
    {
        w_mean.x += gyro.angular_velocity.x;
        w_mean.y += gyro.angular_velocity.y;
        w_mean.z += gyro.angular_velocity.z;

        // gyro_covariance_original.x += gyro.angular_velocity_covariance[COV_IDX_XYZ::X_X];
        // gyro_covariance_original.y += gyro.angular_velocity_covariance[COV_IDX_XYZ::Y_Y];
        // gyro_covariance_original.z += gyro.angular_velocity_covariance[COV_IDX_XYZ::Z_Z];
        // TODO 2024/08/26 imu数据 没有协方差
        w_covariance_original.x += kGyroNoise * kGyroNoise;
        w_covariance_original.y += kGyroNoise * kGyroNoise;
        w_covariance_original.z += kGyroNoise * kGyroNoise;
    }
    w_mean.x /= gyro_queue.size();
    w_mean.y /= gyro_queue.size();
    w_mean.z /= gyro_queue.size() + vehicle_twist_queue.size();
    w_covariance_original.x /= gyro_queue.size();
    w_covariance_original.y /= gyro_queue.size();
    w_covariance_original.z /= gyro_queue.size() + vehicle_twist_queue.size();

    geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
    const auto latest_imu_stamp = rclcpp::Time(gyro_queue.back().header.stamp);
    // const auto latest_vehicle_twist_stamp = rclcpp::Time(vehicle_twist_queue.back().header.stamp);
    // if (latest_vehicle_twist_stamp < latest_imu_stamp)
    // {
    //     twist_with_cov.header.stamp = latest_imu_stamp;
    // }
    // else
    // {
    //     twist_with_cov.header.stamp = latest_vehicle_twist_stamp;
    // }
    //     // TODO 2024/08/26 只用了imu的时间戳
    twist_with_cov.header.stamp = latest_imu_stamp;
    twist_with_cov.header.frame_id = gyro_queue.front().header.frame_id;
    twist_with_cov.twist.twist.linear.x = vx_mean;
    twist_with_cov.twist.twist.angular = w_mean;

    // From a statistical point of view, here we reduce the covariances according to the number of
    // observed data
    twist_with_cov.twist.covariance[0] =
        vx_covariance_original;
    twist_with_cov.twist.covariance[7] = 100000.0;
    twist_with_cov.twist.covariance[14] = 100000.0;
    twist_with_cov.twist.covariance[21] =
        w_covariance_original.x;
    twist_with_cov.twist.covariance[28] =
        w_covariance_original.y;
    twist_with_cov.twist.covariance[35] =
        w_covariance_original.z;

    return twist_with_cov;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Ready.");
    rclcpp::spin(std::make_shared<ImuKalmanNode>());
    rclcpp::shutdown();
    return 0;
}
