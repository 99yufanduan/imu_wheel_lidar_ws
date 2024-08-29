# base on autoware localization frame

```mermaid
graph TB

1[ndt_matcher]
2[gyro-odom]
3[pose_initializer]
4[ekf_localization]
5[ndt_matcher_pose_init]
6[gyro-acc_EKF]
7[gyro-wheel_predict]

subgraph input
a[\gnss/user\]
b[\lidar\]
c[\imu_gyro\]
d[\imu_acc\]
e[\wheel\]
end

subgraph output
A[/one_shot_init_pose/]
B[/ekf_pose/]
C[/ekf_twist/]
end


e--twist-->2
c--gyro-->2
2--twist-->4
b-->1--pose-->4
4-->B
4-->C
a--pose-->3--pose-->5--montecarlo-->A
A-->4
B-->1
c-->6
d-->6
e-->7
c-->7
6--init_rpy-->1
7--init_position-->1
1-->6
1-->7

```


# raw autoware localization frame

```mermaid
graph TB

1[ndt_matcher]
2[gyro-odom]
3[pose_initializer]
4[ekf_localization]
5[ndt_matcher_pose_init]

subgraph input
a[\gnss/user\]
b[\lidar\]
c[\imu\]
d[\wheel\]
end

subgraph output
A[/one_shot_init_pose/]
B[/ekf_pose/]
C[/ekf_twist/]
end

d--twist-->2
c--gyro-->2
2--twist-->4
b-->1--pose-->4
4-->B
4-->C
a--pose-->3--pose-->5--montecarlo-->A
A-->4
B-->1

```

# use

## set pcd map path
const std::string kMapPath = "/home/dyf/rosbag_0827_imu_wheel_vanjee_南风楼/map.pcd";
- [ndt_matcher](/src/ndt_matcher/src/ndt_matcher.cpp)

## set map pub path
pub once only for watch point cloud map
- [ndt_matcher](/src/ndt_matcher/src/pcd_publisher_node.cpp)

## launch
``` bash
ros2 launch ndt_matcher ndt_launch.py
```
## pub map
``` bash
ros2 run ndt_matcher pcd_publisher_node
```

# rosbag 
``` bash
ros2 bag play rosbag_0827_imu_wheel_vanjee_南风楼/rosbag2_2024_08_27-19_42_33/
```
