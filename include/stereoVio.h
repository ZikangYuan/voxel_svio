#pragma once

// c++ include
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_set>

// lib include
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// function include
#include "utility.h"
#include "parameters.h"
#include "sensorData.h"
#include "state.h"
#include "stateHelper.h"
#include "msckf.h"
#include "feature.h"
#include "featureTracker.h"
#include "mapPoint.h"
#include "mapManagement.h"
#include "initializer.h"
#include "frame.h"

struct Measurements
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<imuData> imu_measurements;
    cameraData image_measurements;
};

class voxelStereoVio
{
private:

    void processImu(imuData &imu_data);

    void processImage(cameraData &image_measurements_const);

    void featureUpdate(cameraData &image_measurements);

    void triangulateActiveTracks(cameraData &image_measurements);

    void getRecentVoxel(double timestamp, pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit);

    bool tryToInitialize(cameraData &image_measurements);

    std::queue<cameraData> camera_buffer;

    std::queue<imuData> imu_buffer;

    std::map<int, double> camera_last_timestamp;

    odometryOptions odometry_options;

    std::shared_ptr<trackKLT> featureTracker;

    std::shared_ptr<inertialInitializer> initializer_ptr;

    double startup_time;

    double current_time;

    double time_newest_imu;

    std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_meas;
    std::vector<imuState> imu_states;

    std::shared_ptr<state> state_ptr;

    imuData last_imu_data;

    std::shared_ptr<propagator> propagator_ptr;

    std::shared_ptr<updaterMsckf> updaterMsckf_ptr;

    std::shared_ptr<updaterSlam> updaterSlam_ptr;

    std::shared_ptr<gammaPixel> gammaPixel_ptr;

    std::shared_ptr<frame> newest_fh;

    std::vector<double> camera_queue_init;

    std::vector<std::shared_ptr<frame>> frame_queue_init;

    voxelHashMap voxel_map;

    int frame_count;

    MarginalizeStatus marginalize_status;

    double last_time_image;

    std::vector<Eigen::Vector3d> good_features_msckf;

    double active_tracks_time = -1;
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_pos_world; // active_tracks_posinG
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_pos_world_new;
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
    cv::Mat active_image;
    std::map<size_t, Eigen::Matrix3d> active_feat_linsys_A;
    std::map<size_t, Eigen::Vector3d> active_feat_linsys_b;
    std::map<size_t, int> active_feat_linsys_count;

    std::vector<Eigen::Matrix<short, 3, 1>> recent_voxels;

    std::ofstream of_statistics;
    boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

    double timelastupdate = -1;
    double distance = 0;

    // time test
    double sum_time_1 = 0.0;
    double sum_time_2 = 0.0;
    double sum_time_3 = 0.0;
    double sum_time_4 = 0.0;
    double sum_time_5 = 0.0;
    double sum_time_6 = 0.0;
    double sum_time_7 = 0.0;

    double sum_time_sum = 0.0;

    int n_time_1 = 0;
    int n_time_2 = 0;
    int n_time_3 = 0;
    int n_time_4 = 0;
    int n_time_5 = 0;
    int n_time_6 = 0;
    int n_time_7 = 0;

    int n_time_sum = 0;
    // time test

public:

    voxelStereoVio();

    void readParameters();

    void allocateMemory();

    void initialValue();

    void imuHandler(const sensor_msgs::Imu::ConstPtr &msg);

    void stereoImageHandler(const sensor_msgs::ImageConstPtr &msg_0, const sensor_msgs::ImageConstPtr &msg_1, int cam_0, int cam_1);

    void run();

    ros::NodeHandle nh;

    std::string image_left_topic;
    std::string image_right_topic;
    std::string imu_topic;

    ros::Subscriber sub_imu;
    std::vector<ros::Subscriber> subs_cam;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncStereoImage;
    std::vector<std::shared_ptr<message_filters::Synchronizer<SyncStereoImage>>> sync_cam;
    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>> sync_subs_cam;

    // display
    void pubFeatImage(cv::Mat &stereo_image, double &timestamp);
    void pubOdometry(std::shared_ptr<state> state_ptr, double &timestamp);
    void setPoseStamp(geometry_msgs::PoseStamped &body_pose_out, std::shared_ptr<state> state_ptr);
    void pubPath(std::shared_ptr<state> state_ptr, double &timestamp);
    void pubHistoryPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_history, double &timestamp);
    void pubWindowPoints(std::shared_ptr<state> state_ptr, double &timestamp);
    void pubHistoryVoxels(pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history, double &timestamp);
    void pubVisitVoxels(pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit, double &timestamp);

    image_transport::ImageTransport it;
    image_transport::Publisher pub_feat_image;

    ros::Publisher pub_odom;
    ros::Publisher pub_path;
    ros::Publisher pub_points_history;
    ros::Publisher pub_points_window;
    ros::Publisher pub_voxels_history;
    ros::Publisher pub_voxels_visit;

    geometry_msgs::PoseStamped msg_body_pose;
    nav_msgs::Odometry odom;
    nav_msgs::Path path;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_history;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_window;
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history;
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit;
    // display
};
