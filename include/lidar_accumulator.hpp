#ifndef __LIDAR_ACCUMULATOR_HPP__
#define __LIDAR_ACCUMULATOR_HPP__

// ROS header
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include <pcl_conversions/pcl_conversions.h>

// Utility header
// Point Cloud Library
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

// TSL
#include <tsl/robin_map.h>

// Standard
#include <unordered_map>
#include <deque>

// template<typename pointT=pcl::PointXYZI>
using PointType = pcl::PointXYZI;
using Voxel = Eigen::Vector3i;
struct VoxelHash
{
    size_t operator()(const Voxel &voxel) const
    {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
class LidarAccumulator
{
public:
    LidarAccumulator();
    ~LidarAccumulator();

private:
    bool b_lidar_callback_flag_ = false;
    bool b_init_pose_flag_ = false;
    bool b_lidar_scan_exist_ = false;

    const double D_RAD_2_DEG = 180 / M_PI;
    const double D_DEG_2_RAD = M_PI / 180;

    // ROS variable
private:
    ros::Subscriber rossub_lidar_;
    ros::Subscriber rossub_odometry_;

    ros::Publisher rospub_input_lidar_;
    ros::Publisher rospub_accumulated_lidar_points_;
    ros::Publisher rospub_filtered_points;
    ros::Publisher rospub_odom_path_;

    geometry_msgs::Pose odom_poses;

    std_msgs::Header h_lidar_header_;

    std::string s_input_point_topic_name_;
    std::string s_input_odom_topic_name_;

    std::string s_output_point_topic_name_;

    std::string s_lidar_frmae_name_;

    bool publish_flag;

private:
    pcl::PointCloud<PointType>::Ptr p_accum_lidar_points_;
    std::deque<pcl::PointCloud<PointType>::Ptr> deque_p_input_point_clouds_;
    std::deque<std::pair<uint64_t, Eigen::Affine3d>> deque_time_affine_pose_; // [time_us, Pose]
    std::deque<std::pair<uint64_t, Eigen::Affine3d>> deque_time_affine_point_cloud_synced_global_pose_; // [time_us, Pose]
    std::deque<std::pair<Eigen::Affine3d, pcl::PointCloud<PointType>::Ptr>> deque_synced_pose_point_cloud; // [Pose, PointCloud]

    double last_publish_time_us_;

private:
    // Configurations
    bool cfg_b_pub_odom_tf_;
    double cfg_d_accum_time_window_;
    int cfg_i_accum_num_lidar_scan_;
    double cfg_d_accum_key_threshold_m_;

    std::vector<double> cfg_odom_to_lidar_calib_param_;

    double cfg_d_roi_min_m_;
    double cfg_d_roi_max_m_;

    bool cfg_b_filter_use_voxel_;
    double cfg_d_filter_voxel_leaf_;

    bool cfg_b_msg_debug_;
    bool cfg_b_msg_comp_time_;

public:
private:
    bool Downsampling(pcl::PointCloud<PointType>::Ptr input_point_cloud, pcl::PointCloud<PointType>::Ptr output_point_cloud, double leaf_size);
    Eigen::Affine3d ConvertOdometryToAffine3d(const nav_msgs::Odometry& odom);
    Eigen::Affine3d InterpolateAffine3d(const Eigen::Affine3d& start_transform, const Eigen::Affine3d& end_transform, double ratio);

    bool Run();

    void CallbackPoints(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void CallbackOdom(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // __LIDAR_ACCUMULATOR_HPP__