/*
 * @copyright Automotive Intelligence Lab, Konkuk University
 * @author pauljiown96@gmail.com
 * @file lidar_accumulator.cpp
 * @brief
 * @version 1.0
 * @date 2023-08-10
 */

#include "lidar_accumulator.hpp"

LidarAccumulator::LidarAccumulator() : publish_flag(false),
                                       p_accum_lidar_points_(NULL),
                                       deque_p_input_point_clouds_(std::deque<pcl::PointCloud<PointType>::Ptr>()),
                                       deque_time_affine_pose_(std::deque<std::pair<uint64_t, Eigen::Affine3d>>()),
                                       deque_time_affine_point_cloud_synced_global_pose_(std::deque<std::pair<uint64_t, Eigen::Affine3d>>()),
                                       last_publish_time_us_(0.0),
                                       cfg_b_pub_odom_tf_(false),
                                       cfg_d_accum_time_window_(0.0),
                                       cfg_i_accum_num_lidar_scan_(0),
                                       cfg_d_accum_key_threshold_m_(0.0),
                                       cfg_odom_to_lidar_calib_param_(std::vector<double>(6)),
                                       cfg_d_roi_min_m_(0.0),
                                       cfg_d_roi_max_m_(0.0),
                                       cfg_b_filter_use_voxel_(false),
                                       cfg_d_filter_voxel_leaf_(0.0),
                                       cfg_b_msg_debug_(false),
                                       cfg_b_msg_comp_time_(0.0)
{
    ros::NodeHandle nh;
    nh.getParam("/lidar_accumulator/s_input_point_topic_name_", s_input_point_topic_name_);
    nh.getParam("/lidar_accumulator/s_input_odom_topic_name_", s_input_odom_topic_name_);

    nh.getParam("/lidar_accumulator/s_output_point_topic_name_", s_output_point_topic_name_);

    nh.getParam("/lidar_accumulator/cfg_i_accum_num_lidar_scan_", cfg_i_accum_num_lidar_scan_);

    nh.getParam("/lidar_accumulator/cfg_d_accum_key_threshold_m_", cfg_d_accum_key_threshold_m_);

    nh.getParam("/lidar_accumulator/cfg_d_roi_min_m_", cfg_d_roi_min_m_);
    nh.getParam("/lidar_accumulator/cfg_d_roi_max_m_", cfg_d_roi_max_m_);

    nh.getParam("/lidar_accumulator/cfg_b_filter_use_voxel_", cfg_b_filter_use_voxel_);
    nh.getParam("/lidar_accumulator/cfg_d_filter_voxel_leaf_", cfg_d_filter_voxel_leaf_);

    nh.getParam("/lidar_accumulator/cfg_b_msg_debug_", cfg_b_msg_debug_);
    nh.getParam("/lidar_accumulator/cfg_b_msg_comp_time_", cfg_b_msg_comp_time_);

    ROS_INFO_STREAM("---------------- LIDAR ACCUMULATOR 2 -------------------");
    ROS_INFO_STREAM("Input Point Topic              : " << s_input_point_topic_name_);
    ROS_INFO_STREAM("Input Odom Topic               : " << s_input_odom_topic_name_);
    ROS_INFO_STREAM("Output Point Topic             : " << s_output_point_topic_name_);
    ROS_INFO_STREAM("Num of Accum Scan              : " << cfg_i_accum_num_lidar_scan_);
    ROS_INFO_STREAM("ROI Min                        : " << cfg_d_roi_min_m_);
    ROS_INFO_STREAM("ROI Max                        : " << cfg_d_roi_max_m_);
    ROS_INFO_STREAM("Use Voxel Filter               : " << cfg_b_filter_use_voxel_);
    ROS_INFO_STREAM("Voxel Size                     : " << cfg_d_filter_voxel_leaf_);
    ROS_INFO_STREAM("---------- NODE INITIALIZATION COMPLETE ----------");

    rossub_lidar_ = nh.subscribe(s_input_point_topic_name_, 1, &LidarAccumulator::CallbackPoints, this);
    rossub_odometry_ = nh.subscribe(s_input_odom_topic_name_, 10, &LidarAccumulator::CallbackOdom, this);

    rospub_input_lidar_ = nh.advertise<sensor_msgs::PointCloud2>("/point_accumulator/input_point", 10);
    rospub_accumulated_lidar_points_ = nh.advertise<sensor_msgs::PointCloud2>(s_output_point_topic_name_, 10);
    rospub_filtered_points = nh.advertise<sensor_msgs::PointCloud2>("/point_accumulator/filtered_points", 10);
    // rospub_odom_path_ = nh.advertise<nav_msgs::PoseStamped>("/lidar_odom_pose_stamped", 10);

    p_accum_lidar_points_.reset(new pcl::PointCloud<PointType>());
}

LidarAccumulator::~LidarAccumulator()
{
}

bool LidarAccumulator::Downsampling(pcl::PointCloud<PointType>::Ptr input_point_cloud, pcl::PointCloud<PointType>::Ptr output_point_cloud, double leaf_size)
{
    auto start_time = std::chrono::steady_clock::now();
    if (input_point_cloud == NULL)
        return false;

    // Hashing
    tsl::robin_map<Voxel, PointType, VoxelHash> grid;
    Voxel voxel({int(input_point_cloud->points[0].x / leaf_size), int(input_point_cloud->points[0].y / leaf_size), int(input_point_cloud->points[0].z / leaf_size)});
    grid.reserve(input_point_cloud->points.size());
    for (auto &point : input_point_cloud->points)
    {
        // ROI Filtering
        double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (range > cfg_d_roi_max_m_ || range < cfg_d_roi_min_m_)
            continue;
        // Voxelizing
        const auto voxel = Voxel({int(point.x / leaf_size), int(point.y / leaf_size), int(point.z / leaf_size)});
        if (grid.contains(voxel))
            continue;
        grid.insert({voxel, point});
    }

    // Accumulation
    output_point_cloud->reserve(grid.size());
    output_point_cloud->header = input_point_cloud->header;
    for (const auto &[voxel, point] : grid)
    {
        (void)voxel;
        output_point_cloud->push_back(point);
    }
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration_time = end_time - start_time;
    double processing_time = duration_time.count() * 1000;
    if (cfg_b_msg_comp_time_ == true)
    {
        std::cout << "Downsample time: " << processing_time << " [ms]" << std::endl;
    }
    return true;
}

Eigen::Affine3d LidarAccumulator::ConvertOdometryToAffine3d(const nav_msgs::Odometry &odom)
{
    Eigen::Translation3d translation(odom.pose.pose.position.x,
                                     odom.pose.pose.position.y,
                                     odom.pose.pose.position.z);

    Eigen::Quaterniond rotation(odom.pose.pose.orientation.w,
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z);

    Eigen::Affine3d transform = translation * rotation;

    return transform;
}

Eigen::Affine3d LidarAccumulator::InterpolateAffine3d(const Eigen::Affine3d &start_transform, const Eigen::Affine3d &end_transform, double ratio)
{
    // Extract translation and rotation from the start and end transformations
    Eigen::Vector3d start_translation = start_transform.translation();
    Eigen::Quaterniond start_rotation(start_transform.rotation());

    Eigen::Vector3d end_translation = end_transform.translation();
    Eigen::Quaterniond end_rotation(end_transform.rotation());

    // Linearly interpolate the translation
    Eigen::Vector3d interpolated_translation = (1.0 - ratio) * start_translation + ratio * end_translation;

    // Spherically interpolate the rotation
    Eigen::Quaterniond interpolated_rotation = start_rotation.slerp(ratio, end_rotation);

    // Combine to create the resulting Affine3d object
    Eigen::Affine3d interpolated_transform;
    interpolated_transform.fromPositionOrientationScale(interpolated_translation, interpolated_rotation, Eigen::Vector3d::Ones());

    return interpolated_transform;
}

bool LidarAccumulator::Run()
{
    static uint64_t last_publish_time_us = 0;

    // Check algorithm condition
    if (deque_time_affine_pose_.empty() == true)
        return true;
    if (deque_p_input_point_clouds_.empty() == true)
        return true;

    if (deque_p_input_point_clouds_.back()->header.stamp > deque_time_affine_pose_.front().first)
        return true;

    // Que clear
    while (deque_p_input_point_clouds_.back()->header.stamp <= deque_time_affine_pose_.back().first)
    {
        deque_p_input_point_clouds_.pop_back();
        if (deque_p_input_point_clouds_.empty() == true)
            return true;
    }
    if (deque_time_affine_pose_.size() > 2)
    {
        while (deque_time_affine_pose_[deque_time_affine_pose_.size() - 2].first <= deque_p_input_point_clouds_.back()->header.stamp)
        {
            deque_time_affine_pose_.pop_back();
            if (deque_time_affine_pose_.size() < 2)
                break;
        }
    }

    // Syncronise data
    if (cfg_b_msg_debug_ == true)
    {
        std::cout << "Run Syncronization" << std::endl;
    }
    auto start_time_sync = std::chrono::steady_clock::now();
    std::deque<pcl::PointCloud<PointType>::Ptr> tmp_deque_p_not_synced_point_clouds;
    for (auto cloud_iter = deque_p_input_point_clouds_.rbegin(); cloud_iter != deque_p_input_point_clouds_.rend(); cloud_iter++)
    {
        auto cloud_ptr = *cloud_iter;
        uint64_t cloud_time_us = cloud_ptr->header.stamp;
        bool synced = false;
        for (size_t i = 0; i < deque_time_affine_pose_.size() - 1; ++i)
        {
            uint64_t start_time = deque_time_affine_pose_[i].first;
            uint64_t end_time = deque_time_affine_pose_[i + 1].first;

            if (start_time >= cloud_time_us && cloud_time_us > end_time)
            {
                // Calculate interpolated (synced to point cloud) pose
                double ratio = double((cloud_time_us - start_time) / (end_time - start_time));
                auto point_synced_pose = InterpolateAffine3d(deque_time_affine_pose_[i].second, deque_time_affine_pose_[i + 1].second, ratio);

                if (deque_synced_pose_point_cloud.empty() == true)
                    deque_synced_pose_point_cloud.push_front(std::make_pair(point_synced_pose, cloud_ptr));
                else
                {
                    // Check key frame
                    Eigen::Affine3d prev_pose = deque_synced_pose_point_cloud.front().first;
                    Eigen::Affine3d delta_pose = prev_pose.inverse() * point_synced_pose;
                    auto translation = delta_pose.translation();
                    double dist_m = sqrt(translation[0] * translation[0] + translation[1] * translation[1] + translation[2] * translation[2]);
                    if (dist_m > cfg_d_accum_key_threshold_m_)
                        deque_synced_pose_point_cloud.push_front(std::make_pair(point_synced_pose, cloud_ptr));
                }
                synced = true;
                break;
            }
        }
        if (synced == false)
        {
            tmp_deque_p_not_synced_point_clouds.push_front(cloud_ptr);
        }
    }
    deque_p_input_point_clouds_.swap(tmp_deque_p_not_synced_point_clouds);

    auto end_time_sync = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration_time_sync = end_time_sync - start_time_sync;
    double processing_time_sync = duration_time_sync.count() * 1000;
    if (cfg_b_msg_comp_time_ == true)
    {
        std::cout << "Syncrnization time: " << processing_time_sync << " [ms]" << std::endl;
    }

    // Accmulation
    auto start_time = std::chrono::steady_clock::now();
    if (deque_synced_pose_point_cloud.empty() == true)
        return true;
    // Remove overflowed points
    while (deque_synced_pose_point_cloud.size() > cfg_i_accum_num_lidar_scan_)
        deque_synced_pose_point_cloud.pop_back();
    // Check pub
    if (last_publish_time_us == deque_synced_pose_point_cloud.front().second->header.stamp)
        return true;

    if (cfg_b_msg_debug_ == true)
    {
        std::cout << "Run Accumulation" << std::endl;
    }
    p_accum_lidar_points_->clear();
    p_accum_lidar_points_->reserve(int(deque_synced_pose_point_cloud[0].second->size() * cfg_i_accum_num_lidar_scan_ * 1.5)); // 1.5 is masic number for
    p_accum_lidar_points_->header = deque_synced_pose_point_cloud[0].second->header;
    *p_accum_lidar_points_ += *deque_synced_pose_point_cloud[0].second;
    auto point_synced_pose = deque_synced_pose_point_cloud[0].first;
    for (int cloud_idx = 1; cloud_idx < deque_synced_pose_point_cloud.size(); cloud_idx++)
    {
        // Get relative pose current(current to prev)
        Eigen::Affine3d prev_pose = deque_synced_pose_point_cloud[cloud_idx].first;
        Eigen::Affine3d rel_pose = point_synced_pose.inverse() * prev_pose;

        // Transform point cloud to current pose
        pcl::PointCloud<PointType>::Ptr p_transformed_point_cloud(new pcl::PointCloud<PointType>());
        p_transformed_point_cloud->reserve(deque_synced_pose_point_cloud[cloud_idx].second->size());

        pcl::transformPointCloud(*deque_synced_pose_point_cloud[cloud_idx].second, *p_transformed_point_cloud, rel_pose.cast<float>());
        *p_accum_lidar_points_ += *p_transformed_point_cloud;
    }
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration_time = end_time - start_time;
    double processing_time = duration_time.count() * 1000;
    if (cfg_b_msg_comp_time_ == true)
    {
        std::cout << "Accululation time: " << processing_time << " [ms]" << std::endl;
    }

    // Publish Accumed Point Cloud
    sensor_msgs::PointCloud2 accum_points_msg;
    pcl::toROSMsg(*p_accum_lidar_points_, accum_points_msg);
    accum_points_msg.header.frame_id = s_lidar_frmae_name_;
    rospub_accumulated_lidar_points_.publish(accum_points_msg);
    last_publish_time_us_ = deque_synced_pose_point_cloud.front().second->header.stamp;
    publish_flag = false;
    return true;
}
void LidarAccumulator::CallbackPoints(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<PointType>::Ptr pcl_input_points(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *pcl_input_points);
    s_lidar_frmae_name_ = msg->header.frame_id;

    // Downsample points
    pcl::PointCloud<PointType>::Ptr downsampled_points(new pcl::PointCloud<PointType>);
    if (cfg_b_filter_use_voxel_ == true)
    {
        if (cfg_b_msg_debug_ == true)
        {
            std::cout << "Run Downsample" << std::endl;
        }
        Downsampling(pcl_input_points, downsampled_points, cfg_d_filter_voxel_leaf_);
        if (cfg_b_msg_debug_ == true)
        {
            std::cout << "Input size: " << pcl_input_points->size() << std::endl;
            std::cout << "Downsample size: " << downsampled_points->size() << std::endl;
        }
    }
    else
    {
        downsampled_points = pcl_input_points;
    }
    if (downsampled_points == NULL)
        return;

    // Push to que
    while (deque_p_input_point_clouds_.size() > cfg_i_accum_num_lidar_scan_)
    {
        deque_p_input_point_clouds_.pop_back();
    }
    deque_p_input_point_clouds_.push_front(downsampled_points);

    if (cfg_b_msg_debug_ == true)
    {
        std::cout << "Publish" << std::endl;
    }
    sensor_msgs::PointCloud2 sample_points_msg;
    pcl::toROSMsg(*downsampled_points, sample_points_msg);
    sample_points_msg.header = msg->header;
    rospub_filtered_points.publish(sample_points_msg);
    publish_flag = true;
}

void LidarAccumulator::CallbackOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    auto start_time = std::chrono::steady_clock::now();

    Eigen::Affine3d affine_odom_pose = ConvertOdometryToAffine3d(*msg);
    uint64_t time_us = msg->header.stamp.toNSec() / 1e3;

    if (cfg_b_msg_debug_ == true)
    {
        std::cout << "Run Odome callback" << std::endl;
    }

    // Push to global pose deque
    deque_time_affine_pose_.push_front(std::make_pair(time_us, affine_odom_pose));

    Run();
    return;
}

int main(int argc, char **argv)
{
    std::string str_nodename = "lidar_accumulation";
    ros::init(argc, argv, str_nodename);
    ros::NodeHandle nh;

    LidarAccumulator lidar_accumulator;

    ros::spin();

    return 0;
}