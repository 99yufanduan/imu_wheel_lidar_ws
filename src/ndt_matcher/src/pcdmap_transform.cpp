#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
    // 创建一个点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取输入的 PCD 文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/dyf/rosbag_0816_imu_wheel_vanjee/map.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from input.pcd" << std::endl;

    // 定义一个 4x4 的变换矩阵，初始化为单位矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // 定义平移：例如，沿着X轴平移2.5单位，Y轴平移3.0单位，Z轴平移1.0单位
    transform(0, 3) = 0; // X轴
    transform(1, 3) = 0; // Y轴
    transform(2, 3) = 0; // Z轴

    // 定义旋转：例如，绕Z轴旋转90度（注意，角度需要转换为弧度）
    float theta = -M_PI / 2; // 90度的弧度
    transform(0, 0) = cos(theta);
    transform(0, 1) = -sin(theta);
    transform(1, 0) = sin(theta);
    transform(1, 1) = cos(theta);

    // 输出变换矩阵
    std::cout << "Transformation Matrix:\n"
              << transform << std::endl;

    // 应用变换到点云上
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // 保存变换后的点云数据到一个新的 PCD 文件
    pcl::io::savePCDFileASCII("/home/dyf/rosbag_0816_imu_wheel_vanjee/map_tf.pcd", *transformed_cloud);
    std::cout << "Saved transformed point cloud to output.pcd" << std::endl;

    return 0;
}
