# lidar_accumulator
## Brief
Point cloud (sensor_msgs::PointCloud2) accumulation tool using any odometry (nav_msgs::Odometry, position and orientation) input.

## Requirement
- ROS

## Install
```
cd {$YOUR_ROS_WS}/src
git clone https://github.com/JiwonSeokOOO/lidar_accumulator.git
cd ..
catkin_make
```

## Run
```
source devel/setup.bash
roslaunch lidar_accumulator lidar_accumulator.launch
```

## Parameters
    - s_input_point_topic_name_: input point cloud topic name (sensor_msgs::PointCloud2)
    - s_input-odom_topic_name_: input odometry topic name (nav_msgs::Odometry)
    - s_output_point_topic_name_: output point_cloud tpic name (sensor_msgs::PointCloud2)

    - cfg_i_accum_num_lidar_scan_: the number of lidar scan for accumulation
    - cfg_d_accum_key_threshold_m_: to prevent accumulation of too close point cloud. Use keyframe(only consider translation).

    - cfg_d_roi_min_m_: point cloud ROI. Only cut the scan point cloud
    - cfg_d_roi_max_m_: point cloud ROI. Only cut the scan point cloud

    - cfg_b_filter_use_voxel_: use voxel filtering.
    - cfg_d_filter_voxel_leaf_: voxel filter size [m]

    - cfg_b_msg_debug_: show debug message
    - cfg_b_msg_comp_time_: show computation time (filtering, accumulation ...)

# ToDo
    - Apply Calilbration
    - Supply many lidar point cloud type
        - Velodyne
        - Ouster
        - Livox
    - Apply output point cloud frame_id configuration