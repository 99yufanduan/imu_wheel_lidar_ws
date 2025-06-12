#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <deque>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/filters/passthrough.h>
#include "passthrough_uint16.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

#include <pcl_ros/transforms.hpp>

struct CropBoxParam
{
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    bool negative{false};
};

Eigen::Vector3d getArbitraryOrthogonalVector(const Eigen::Vector3d &input)
{
    const double x = input.x();
    const double y = input.y();
    const double z = input.z();
    const double x2 = std::pow(x, 2);
    const double y2 = std::pow(y, 2);
    const double z2 = std::pow(z, 2);

    Eigen::Vector3d unit_vec{0, 0, 0};
    if (x2 <= y2 && x2 <= z2)
    {
        unit_vec.x() = 0;
        unit_vec.y() = z;
        unit_vec.z() = -y;
        unit_vec = unit_vec / std::sqrt(y2 + z2);
    }
    else if (y2 <= x2 && y2 <= z2)
    {
        unit_vec.x() = -z;
        unit_vec.y() = 0;
        unit_vec.z() = x;
        unit_vec = unit_vec / std::sqrt(z2 + x2);
    }
    else if (z2 <= x2 && z2 <= y2)
    {
        unit_vec.x() = y;
        unit_vec.y() = -x;
        unit_vec.z() = 0;
        unit_vec = unit_vec / std::sqrt(x2 + y2);
    }
    return unit_vec;
}

struct PlaneBasis
{
    Eigen::Vector3d e_x;
    Eigen::Vector3d e_y;
    Eigen::Vector3d e_z;
};

PlaneBasis getPlaneBasis(const Eigen::Vector3d &plane_normal)
{
    PlaneBasis basis;
    basis.e_z = plane_normal;
    basis.e_x = getArbitraryOrthogonalVector(plane_normal);
    basis.e_y = basis.e_x.cross(basis.e_z);
    return basis;
}

Eigen::Affine3d getPlaneAffine(
    const pcl::PointCloud<pcl::PointXYZ> segment_ground_cloud, const Eigen::Vector3d &plane_normal)
{
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (const auto p : segment_ground_cloud.points)
    {
        centroid.add(p);
    }
    pcl::PointXYZ centroid_point;
    centroid.get(centroid_point);
    Eigen::Translation<double, 3> trans(centroid_point.x, centroid_point.y, centroid_point.z);
    const PlaneBasis basis = getPlaneBasis(plane_normal);
    Eigen::Matrix3d rot;
    rot << basis.e_x.x(), basis.e_y.x(), basis.e_z.x(), basis.e_x.y(), basis.e_y.y(), basis.e_z.y(),
        basis.e_x.z(), basis.e_y.z(), basis.e_z.z();
    return trans * rot;
}

void extractPointsIndices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, const pcl::PointIndices &in_indices,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_only_indices_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_removed_indices_cloud_ptr)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(in_cloud_ptr);
    extract_ground.setIndices(pcl::make_shared<pcl::PointIndices>(in_indices));

    extract_ground.setNegative(false); // true removes the indices, false leaves only the indices
    extract_ground.filter(*out_only_indices_cloud_ptr);

    extract_ground.setNegative(true); // true removes the indices, false leaves only the indices
    extract_ground.filter(*out_removed_indices_cloud_ptr);
}

void applyRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointIndices::Ptr &output_inliers,
    pcl::ModelCoefficients::Ptr &output_coefficients)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setRadiusLimits(0.3, std::numeric_limits<double>::max());
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(input);
    seg.setMaxIterations(1000);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.segment(*output_inliers, *output_coefficients);
}

class PointcloudPreprocessor : public rclcpp::Node
{
public:
    PointcloudPreprocessor()
        : Node("pointcloud_subscriber")
    {
        // 创建一个订阅者，订阅 /pointcloud 话题
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 2,
            std::bind(&PointcloudPreprocessor::pointcloud_callback, this, std::placeholders::_1));

        crop_box_polygon_pub_ =
            this->create_publisher<geometry_msgs::msg::PolygonStamped>("/crop_box_polygon", 2);

        crop_box_point_cloud_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/crop_box_point_cloud", 2);

        ring_passthrough_point_cloud_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/ring_passthrough_point_cloud", 2);

        no_ground_points_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/no_ground_pointcloud", 2);

        crop_outer_param_.negative = false;
        crop_outer_param_.min_x = -15;
        crop_outer_param_.min_y = -15;
        crop_outer_param_.min_z = -2;
        crop_outer_param_.max_x = 15;
        crop_outer_param_.max_y = 15;
        crop_outer_param_.max_z = 0;

        crop_inner_param_.negative = true;
        crop_inner_param_.min_x = -0.5;
        crop_inner_param_.min_y = -0.5;
        crop_inner_param_.min_z = -0.5;
        crop_inner_param_.max_x = 0.5;
        crop_inner_param_.max_y = 0.5;
        crop_inner_param_.max_z = 0;

        // set initial parameters
        int filter_min = 32;
        int filter_max = 64;
        impl_.setFilterLimits(filter_min, filter_max);

        impl_.setFilterFieldName("ring");
        impl_.setKeepOrganized(false);
        impl_.setFilterLimitsNegative(false);
    }

    void ground_filter(
        const sensor_msgs::msg::PointCloud2 &input, [[maybe_unused]] const pcl::IndicesPtr &indices,
        sensor_msgs::msg::PointCloud2 &output)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(input, *current_sensor_cloud_ptr);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(current_sensor_cloud_ptr);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.47, -1.07); // 雷达离地面约1.27m
        pass.filter(*cloud_filtered);

        // downsample pointcloud to reduce ransac calculation cost
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        downsampled_cloud->points.reserve(current_sensor_cloud_ptr->points.size());
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud_filtered);
        filter.setLeafSize(0.02, 0.02, 0.02);
        filter.filter(*downsampled_cloud);

        // apply ransac
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        applyRANSAC(downsampled_cloud, inliers, coefficients);

        if (coefficients->values.empty())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
                "failed to find a plane");
            output = input;
            return;
        }

        // filter too tilt plane to avoid mis-fitting (e.g. fitting to wall plane)
        Eigen::Vector3d plane_normal(
            coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        {
            const auto plane_slope = std::abs(
                std::acos(plane_normal.dot(unit_vec_) / (plane_normal.norm() * unit_vec_.norm())) * 180 /
                M_PI);
            if (plane_slope > 10.0)
            {
                output = input;
                return;
            }
        }

        // extract pointcloud from indices
        pcl::PointCloud<pcl::PointXYZ>::Ptr segment_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr segment_no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        extractPointsIndices(
            downsampled_cloud, *inliers, segment_ground_cloud_ptr, segment_no_ground_cloud_ptr);
        const Eigen::Affine3d plane_affine = getPlaneAffine(*segment_ground_cloud_ptr, plane_normal);
        pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        // use not downsampled pointcloud for extract pointcloud that higher than height threshold
        for (const auto &p : current_sensor_cloud_ptr->points)
        {
            const Eigen::Vector3d transformed_point =
                plane_affine.inverse() * Eigen::Vector3d(p.x, p.y, p.z);
            if (std::abs(transformed_point.z()) > 0.08) // height_threshold
            {
                no_ground_cloud_ptr->points.push_back(p);
            }
        }

        sensor_msgs::msg::PointCloud2::SharedPtr no_ground_cloud_msg_ptr(
            new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*no_ground_cloud_ptr, *no_ground_cloud_msg_ptr);
        no_ground_cloud_msg_ptr->header = input.header;
        output = *no_ground_cloud_msg_ptr;
        no_ground_points_pub_->publish(output);
    }

    void passthrough_filter(
        const sensor_msgs::msg::PointCloud2 &input, const pcl::IndicesPtr &indices, sensor_msgs::msg::PointCloud2 &output)
    {
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(input, *(pcl_input));
        impl_.setInputCloud(pcl_input);
        impl_.setIndices(indices);
        pcl::PCLPointCloud2 pcl_output;
        impl_.filter(pcl_output);
        pcl_conversions::moveFromPCL(pcl_output, output);
        output.header = input.header;
        // ring_passthrough_point_cloud_pub_->publish(output);
    }

    void crop_box_filter(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input, [[maybe_unused]] const pcl::IndicesPtr &indices,
        sensor_msgs::msg::PointCloud2 &output, CropBoxParam param)
    {
        output.data.resize(input->data.size());
        Eigen::Vector3f pt(Eigen::Vector3f::Zero());
        size_t j = 0;
        const auto data_size = input->data.size();
        const auto point_step = input->point_step;
        // If inside the cropbox
        if (!param.negative)
        {
            for (size_t i = 0; i + point_step < data_size; i += point_step)
            {
                memcpy(pt.data(), &input->data[i], sizeof(float) * 3);
                if (
                    param.min_z < pt.z() && pt.z() < param.max_z && param.min_y < pt.y() &&
                    pt.y() < param.max_y && param.min_x < pt.x() && pt.x() < param.max_x)
                {
                    memcpy(&output.data[j], &input->data[i], point_step);
                    j += point_step;
                }
            }
            // If outside the cropbox
        }
        else
        {
            for (size_t i = 0; i + point_step < data_size; i += point_step)
            {
                memcpy(pt.data(), &input->data[i], sizeof(float) * 3);
                if (
                    param.min_z > pt.z() || pt.z() > param.max_z || param.min_y > pt.y() ||
                    pt.y() > param.max_y || param.min_x > pt.x() || pt.x() > param.max_x)
                {
                    memcpy(&output.data[j], &input->data[i], point_step);
                    j += point_step;
                }
            }
        }

        output.data.resize(j);
        output.header = input->header;
        output.height = 1;
        output.fields = input->fields;
        output.is_bigendian = input->is_bigendian;
        output.point_step = input->point_step;
        output.is_dense = input->is_dense;
        output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
        output.row_step = static_cast<uint32_t>(output.data.size() / output.height);
        // crop_box_point_cloud_pub_->publish(output);
        // publishCropBoxPolygon(param);
    }

    void publishCropBoxPolygon(CropBoxParam param)
    {
        auto generatePoint = [](double x, double y, double z)
        {
            geometry_msgs::msg::Point32 point;
            point.x = x;
            point.y = y;
            point.z = z;
            return point;
        };

        const double x1 = param.max_x;
        const double x2 = param.min_x;
        const double x3 = param.min_x;
        const double x4 = param.max_x;

        const double y1 = param.max_y;
        const double y2 = param.max_y;
        const double y3 = param.min_y;
        const double y4 = param.min_y;

        const double z1 = param.min_z;
        const double z2 = param.max_z;

        geometry_msgs::msg::PolygonStamped polygon_msg;
        polygon_msg.header.frame_id = "vanjee_lidar";
        polygon_msg.header.stamp = get_clock()->now();
        polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
        polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
        polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
        polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
        polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

        polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

        polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
        polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
        polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

        polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
        polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
        polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

        polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
        polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
        polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

        polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

        crop_box_polygon_pub_->publish(polygon_msg);
    }

private:
    int pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points)
    {
        pcl::IndicesPtr indice;
        sensor_msgs::msg::PointCloud2::Ptr output(new sensor_msgs::msg::PointCloud2);
        crop_box_filter(points, indice, *output, crop_outer_param_);

        sensor_msgs::msg::PointCloud2 output1;
        crop_box_filter(output, indice, output1, crop_inner_param_);

        publishCropBoxPolygon(crop_inner_param_);

        sensor_msgs::msg::PointCloud2 output2;
        passthrough_filter(output1, indice, output2);

        sensor_msgs::msg::PointCloud2 output3;
        ground_filter(output2, indice, output3);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr crop_box_polygon_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_box_point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ring_passthrough_point_cloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_points_pub_;
    pcl::PassThroughUInt16<pcl::PCLPointCloud2> impl_;
    Eigen::Vector3d unit_vec_ = Eigen::Vector3d::UnitZ();
    CropBoxParam crop_outer_param_, crop_inner_param_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PointcloudPreprocessor> node = std::make_shared<PointcloudPreprocessor>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // Uses multiple threads to process callbacks
    rclcpp::shutdown();
    return 0;
}
