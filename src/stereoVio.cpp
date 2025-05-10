#include "stereoVio.h"

voxelStereoVio::voxelStereoVio() : it(nh)
{
    readParameters();

    initialValue();

    allocateMemory();

    sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, &voxelStereoVio::imuHandler, this);

    auto sub_img_left = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, image_left_topic, 5);
    auto sub_img_right = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, image_right_topic, 5);
    auto sync = std::make_shared<message_filters::Synchronizer<SyncStereoImage>>(SyncStereoImage(10), *sub_img_left, *sub_img_right);
    sync->registerCallback(boost::bind(&voxelStereoVio::stereoImageHandler, this, _1, _2, 0, 1));

    sync_cam.push_back(sync);
    sync_subs_cam.push_back(sub_img_left);
    sync_subs_cam.push_back(sub_img_right);

    // display
    pub_feat_image = it.advertise("camera/stereo_feat_image", 5);
    pub_odom = nh.advertise<nav_msgs::Odometry>("vio/odom", 5);
    pub_path = nh.advertise<nav_msgs::Path>("vio/path", 5);
    pub_points_history = nh.advertise<sensor_msgs::PointCloud2>("vio/history_map_points", 2);
    pub_points_window = nh.advertise<sensor_msgs::PointCloud2>("vio/window_map_points", 2);
    pub_voxels_history = nh.advertise<sensor_msgs::PointCloud2>("vio/history_voxels", 2);
    pub_voxels_visit = nh.advertise<sensor_msgs::PointCloud2>("vio/visit_voxels", 2);

    points_history.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    points_window.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxels_history.reset(new pcl::PointCloud<pcl::PointXYZI>());
    voxels_visit.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // display

    // odometry_options.recordParameters();
}

void voxelStereoVio::readParameters()
{	
    int para_int;
    double para_double;
    bool para_bool;
    std::string str_temp;

    nh.param<std::string>("common/image_left_topic", image_left_topic, "/cam0/image_raw");
    nh.param<std::string>("common/image_right_topic", image_right_topic, "/cam1/image_raw");
    nh.param<std::string>("common/imu_topic", imu_topic, "/imu0");
    nh.param<std::string>("output_path", output_path, "");

    nh.param<bool>("state_parameter/use_fej", para_bool, false); odometry_options.state_options.do_fej = para_bool;
    nh.param<bool>("state_parameter/calib_cam_extrinsics", para_bool, false); odometry_options.state_options.do_calib_camera_pose = para_bool;
    nh.param<bool>("state_parameter/calib_cam_intrinsics", para_bool, false); odometry_options.state_options.do_calib_camera_intrinsics = para_bool;
    nh.param<bool>("state_parameter/calib_cam_timeoffset", para_bool, false); odometry_options.state_options.do_calib_camera_timeoffset = para_bool;
    nh.param<bool>("state_parameter/calib_imu_intrinsics", para_bool, false); odometry_options.state_options.do_calib_imu_intrinsics = para_bool;
    nh.param<bool>("state_parameter/calib_imu_g_sensitivity", para_bool, false); odometry_options.state_options.do_calib_imu_g_sensitivity = para_bool;

    nh.param<std::string>("state_parameter/imu_intrinsics_model", str_temp, "kalibr");
    if (str_temp == "kalibr" || str_temp == "calibrated") odometry_options.state_options.imu_model = ImuModel::KALIBR;
    else if (str_temp == "rpng") odometry_options.state_options.imu_model = ImuModel::RPNG;
    else {
        std::cout << "Invalid IMU model: " << str_temp << std::endl;
        std::cout << "Please select a valid model: kalibr, rpng." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (str_temp == "calibrated")
    {
        odometry_options.state_options.do_calib_imu_intrinsics = false;
        odometry_options.state_options.do_calib_imu_g_sensitivity = false;
    }

    nh.param<int>("state_parameter/max_clones", para_int, 11); odometry_options.state_options.max_clone_size = para_int;
    nh.param<int>("state_parameter/max_slam", para_int, 25); odometry_options.state_options.max_slam_features = para_int;
    nh.param<int>("state_parameter/max_slam_in_update", para_int, 1000); odometry_options.state_options.max_slam_in_update = para_int;
    nh.param<int>("state_parameter/max_msckf_in_update", para_int, 1000); odometry_options.state_options.max_msckf_in_update = para_int;

    nh.param<double>("initializer_parameter/init_window_time", para_double, 1.0); odometry_options.init_options.init_window_time = para_double;
    nh.param<double>("initializer_parameter/init_imu_thresh", para_double, 1.0); odometry_options.init_options.init_imu_thresh = para_double;
    nh.param<double>("initializer_parameter/init_max_disparity", para_double, 1.0); odometry_options.init_options.init_max_disparity = para_double;
    nh.param<int>("initializer_parameter/init_max_features", para_int, 50); odometry_options.init_options.init_max_features = para_int;
    nh.param<bool>("initializer_parameter/init_dyn_use", para_bool, false); odometry_options.init_options.init_dyn_use = para_bool;
    nh.param<bool>("initializer_parameter/init_dyn_mle_opt_calib", para_bool, false); odometry_options.init_options.init_dyn_mle_opt_calib = para_bool;
    nh.param<int>("initializer_parameter/init_dyn_mle_max_iter", para_int, 20); odometry_options.init_options.init_dyn_mle_max_iter = para_int;
    nh.param<int>("initializer_parameter/init_dyn_mle_max_threads", para_int, 20); odometry_options.init_options.init_dyn_mle_max_threads = para_int;
    nh.param<double>("initializer_parameter/init_dyn_mle_max_time", para_double, 5.0); odometry_options.init_options.init_dyn_mle_max_time = para_double;
    nh.param<int>("initializer_parameter/init_dyn_num_pose", para_int, 5); odometry_options.init_options.init_dyn_num_pose = para_int;
    nh.param<double>("initializer_parameter/init_dyn_min_deg", para_double, 45.0); odometry_options.init_options.init_dyn_min_deg = para_double;
    nh.param<double>("initializer_parameter/init_dyn_inflation_ori", para_double, 10.0); odometry_options.init_options.init_dyn_inflation_orientation = para_double;
    nh.param<double>("initializer_parameter/init_dyn_inflation_vel", para_double, 10.0); odometry_options.init_options.init_dyn_inflation_velocity = para_double;
    nh.param<double>("initializer_parameter/init_dyn_inflation_bg", para_double, 100.0); odometry_options.init_options.init_dyn_inflation_bias_gyro = para_double;
    nh.param<double>("initializer_parameter/init_dyn_inflation_ba", para_double, 100.0); odometry_options.init_options.init_dyn_inflation_bias_accel = para_double;
    nh.param<double>("initializer_parameter/init_dyn_min_rec_cond", para_double, 1e-15); odometry_options.init_options.init_dyn_min_rec_cond = para_double;

    std::vector<double> v_bias_acc, v_bias_gyr;
    nh.param<std::vector<double>>("initializer_parameter/init_dyn_bias_g", v_bias_gyr, std::vector<double>());
    nh.param<std::vector<double>>("initializer_parameter/init_dyn_bias_a", v_bias_acc, std::vector<double>());
    odometry_options.init_options.init_dyn_bias_g << v_bias_gyr.at(0), v_bias_gyr.at(1), v_bias_gyr.at(2);
    odometry_options.init_options.init_dyn_bias_a << v_bias_acc.at(0), v_bias_acc.at(1), v_bias_acc.at(2);

    nh.param<double>("initializer_parameter/gravity_mag", para_double, 9.81); odometry_options.init_options.gravity_mag = para_double;
    nh.param<bool>("initializer_parameter/downsample_cameras", para_bool, false); odometry_options.init_options.downsample_cameras = para_bool;
 
    double calib_camimu_dt_left, calib_camimu_dt_right;
    nh.param<double>("camera_parameter/timeshift_cam_imu_left", calib_camimu_dt_left, 0.0);
    nh.param<double>("camera_parameter/timeshift_cam_imu_right", calib_camimu_dt_right, 0.0);
    odometry_options.calib_camimu_dt = calib_camimu_dt_left;

    std::string dist_model_left, dist_model_right;
    nh.param<std::string>("camera_parameter/distortion_model_left", dist_model_left, "radtan");
    nh.param<std::string>("camera_parameter/distortion_model_right", dist_model_right, "radtan");

    std::vector<double> cam_calib_1_left = {1, 1, 0, 0};
    std::vector<double> cam_calib_1_right = {1, 1, 0, 0};
    std::vector<double> cam_calib_2_left = {0, 0, 0, 0};
    std::vector<double> cam_calib_2_right = {0, 0, 0, 0};
    nh.param<std::vector<double>>("camera_parameter/intrinsics_left", cam_calib_1_left, std::vector<double>());
    nh.param<std::vector<double>>("camera_parameter/intrinsics_right", cam_calib_1_right, std::vector<double>());
    nh.param<std::vector<double>>("camera_parameter/distortion_coeffs_left", cam_calib_2_left, std::vector<double>());
    nh.param<std::vector<double>>("camera_parameter/distortion_coeffs_right", cam_calib_2_right, std::vector<double>());
    Eigen::VectorXd cam_calib_left = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd cam_calib_right = Eigen::VectorXd::Zero(8);
    cam_calib_left << cam_calib_1_left.at(0), cam_calib_1_left.at(1), cam_calib_1_left.at(2), cam_calib_1_left.at(3), 
                      cam_calib_2_left.at(0), cam_calib_2_left.at(1), cam_calib_2_left.at(2), cam_calib_2_left.at(3);
    cam_calib_right << cam_calib_1_right.at(0), cam_calib_1_right.at(1), cam_calib_1_right.at(2), cam_calib_1_right.at(3), 
                       cam_calib_2_right.at(0), cam_calib_2_right.at(1), cam_calib_2_right.at(2), cam_calib_2_right.at(3);

    cam_calib_left(0) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    cam_calib_left(1) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    cam_calib_left(2) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    cam_calib_left(3) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;

    cam_calib_right(0) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    cam_calib_right(1) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    cam_calib_right(2) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    cam_calib_right(3) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;

    std::vector<int> matrix_wh_left = {1, 1};
    nh.param<std::vector<int>>("camera_parameter/resolution_left", matrix_wh_left, std::vector<int>());
    matrix_wh_left.at(0) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    matrix_wh_left.at(1) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;

    std::vector<int> matrix_wh_right = {1, 1};
    nh.param<std::vector<int>>("camera_parameter/resolution_right", matrix_wh_right, std::vector<int>());
    matrix_wh_right.at(0) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;
    matrix_wh_right.at(1) /= (odometry_options.init_options.downsample_cameras) ? 2.0 : 1.0;

    assert(matrix_wh_left.at(0) == matrix_wh_right.at(0));
    assert(matrix_wh_left.at(1) == matrix_wh_right.at(1));

    wG[0] = matrix_wh_left.at(0);
    hG[0] = matrix_wh_left.at(1);

    std::vector<double> v_T_imu_cam_left;
    nh.param<std::vector<double>>("camera_parameter/T_imu_cam_left", v_T_imu_cam_left, std::vector<double>());
    Eigen::Matrix4d T_imu_cam_left = mat44FromArray(v_T_imu_cam_left);

    std::vector<double> v_T_imu_cam_right;
    nh.param<std::vector<double>>("camera_parameter/T_imu_cam_right", v_T_imu_cam_right, std::vector<double>());
    Eigen::Matrix4d T_imu_cam_right = mat44FromArray(v_T_imu_cam_right);

    Eigen::Matrix<double, 7, 1> cam_eigen_left;
    cam_eigen_left.block<4, 1>(0, 0) = quatType::rotToQuat(T_imu_cam_left.block<3, 3>(0, 0).transpose());
    cam_eigen_left.block<3, 1>(4, 0) = - T_imu_cam_left.block<3, 3>(0, 0).transpose() * T_imu_cam_left.block<3, 1>(0, 3);

    Eigen::Matrix<double, 7, 1> cam_eigen_right;
    cam_eigen_right.block<4, 1>(0, 0) = quatType::rotToQuat(T_imu_cam_right.block<3, 3>(0, 0).transpose());
    cam_eigen_right.block<3, 1>(4, 0) = - T_imu_cam_right.block<3, 3>(0, 0).transpose() * T_imu_cam_right.block<3, 1>(0, 3);

    if (dist_model_left == "equidistant")
    {
        odometry_options.init_options.camera_intrinsics.insert({0, std::make_shared<cameraEqui>(matrix_wh_left.at(0), matrix_wh_left.at(1))});
        odometry_options.init_options.camera_intrinsics.at(0)->setValue(cam_calib_left);

        odometry_options.camera_intrinsics.insert({0, std::make_shared<cameraEqui>(matrix_wh_left.at(0), matrix_wh_left.at(1))});
        odometry_options.camera_intrinsics.at(0)->setValue(cam_calib_left);
    }
    else
    {
        odometry_options.init_options.camera_intrinsics.insert({0, std::make_shared<cameraRadtan>(matrix_wh_left.at(0), matrix_wh_left.at(1))});
        odometry_options.init_options.camera_intrinsics.at(0)->setValue(cam_calib_left);

        odometry_options.camera_intrinsics.insert({0, std::make_shared<cameraRadtan>(matrix_wh_left.at(0), matrix_wh_left.at(1))});
        odometry_options.camera_intrinsics.at(0)->setValue(cam_calib_left);
    }
    odometry_options.init_options.camera_extrinsics.insert({0, cam_eigen_left});
    odometry_options.camera_extrinsics.insert({0, cam_eigen_left});

    if (dist_model_right == "equidistant")
    {
        odometry_options.init_options.camera_intrinsics.insert({1, std::make_shared<cameraEqui>(matrix_wh_right.at(0), matrix_wh_right.at(1))});
        odometry_options.init_options.camera_intrinsics.at(1)->setValue(cam_calib_right);

        odometry_options.camera_intrinsics.insert({1, std::make_shared<cameraEqui>(matrix_wh_right.at(0), matrix_wh_right.at(1))});
        odometry_options.camera_intrinsics.at(1)->setValue(cam_calib_right);
    }
    else
    {
        odometry_options.init_options.camera_intrinsics.insert({1, std::make_shared<cameraRadtan>(matrix_wh_right.at(0), matrix_wh_right.at(1))});
        odometry_options.init_options.camera_intrinsics.at(1)->setValue(cam_calib_right);

        odometry_options.camera_intrinsics.insert({1, std::make_shared<cameraRadtan>(matrix_wh_right.at(0), matrix_wh_right.at(1))});
        odometry_options.camera_intrinsics.at(1)->setValue(cam_calib_right);
    }
    odometry_options.init_options.camera_extrinsics.insert({1, cam_eigen_right});
    odometry_options.camera_extrinsics.insert({1, cam_eigen_right});

    nh.param<bool>("odometry_parameter/use_mask", para_bool, false); odometry_options.use_mask = para_bool;
    if (odometry_options.use_mask)
    {
        std::string mask_left_path, mask_right_path;
        nh.param<std::string>("camera_parameter/mask_left_path", str_temp, ""); mask_left_path = str_temp;
        nh.param<std::string>("camera_parameter/mask_right_path", str_temp, ""); mask_right_path = str_temp;

        if (!boost::filesystem::exists(mask_left_path))
        {
            std::cout << "Invalid mask path: " << mask_left_path << std::endl;
            std::exit(EXIT_FAILURE);
        }
        cv::Mat mask_left = cv::imread(mask_left_path, cv::IMREAD_GRAYSCALE);

        if (!boost::filesystem::exists(mask_right_path))
        {
            std::cout << "Invalid mask path: " << mask_right_path << std::endl;
            std::exit(EXIT_FAILURE);
        }
        cv::Mat mask_right = cv::imread(mask_right_path, cv::IMREAD_GRAYSCALE);

        if (mask_left.cols != odometry_options.camera_intrinsics.at(0)->w() || mask_left.rows != odometry_options.camera_intrinsics.at(0)->h())
        {
            std::cout << "Mask size does not match left camera!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        if (mask_right.cols != odometry_options.camera_intrinsics.at(1)->w() || mask_right.rows != odometry_options.camera_intrinsics.at(1)->h())
        {
            std::cout << "Mask size does not match right camera!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        odometry_options.masks.insert({0, mask_left});
        odometry_options.masks.insert({1, mask_right});
    }

    nh.param<double>("imu_parameter/gyroscope_noise_density", para_double, 1.6968e-04); odometry_options.init_options.sigma_w = para_double;
    nh.param<double>("imu_parameter/gyroscope_random_walk", para_double, 1.9393e-05); odometry_options.init_options.sigma_wb = para_double;
    nh.param<double>("imu_parameter/accelerometer_noise_density", para_double, 2.0000e-3); odometry_options.init_options.sigma_a = para_double;
    nh.param<double>("imu_parameter/accelerometer_random_walk", para_double, 3.0000e-03); odometry_options.init_options.sigma_ab = para_double;
    nh.param<double>("imu_parameter/sigma_pix", para_double, 1.0); odometry_options.init_options.sigma_pix = para_double;

    std::vector<double> v_Tw;
    nh.param<std::vector<double>>("imu_parameter/Tw", v_Tw, std::vector<double>());
    Eigen::Matrix3d Tw = mat33FromArray(v_Tw);
    std::vector<double> v_Ta;
    nh.param<std::vector<double>>("imu_parameter/Ta", v_Ta, std::vector<double>());
    Eigen::Matrix3d Ta = mat33FromArray(v_Ta);
    std::vector<double> v_R_acc_imu;
    nh.param<std::vector<double>>("imu_parameter/R_acc_imu", v_R_acc_imu, std::vector<double>());
    Eigen::Matrix3d R_acc_imu = mat33FromArray(v_R_acc_imu);
    std::vector<double> v_R_gyr_imu;
    nh.param<std::vector<double>>("imu_parameter/R_gyr_imu", v_R_gyr_imu, std::vector<double>());
    Eigen::Matrix3d R_gyr_imu = mat33FromArray(v_R_gyr_imu);
    std::vector<double> v_Tg;
    nh.param<std::vector<double>>("imu_parameter/Tg", v_Tg, std::vector<double>());
    Eigen::Matrix3d Tg = mat33FromArray(v_Tg);

    Eigen::Matrix3d Dw = Tw.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d Da = Ta.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d R_imu_acc = R_acc_imu.transpose();
    Eigen::Matrix3d R_imu_gyr = R_gyr_imu.transpose();

    if (std::isnan(Tw.norm()) || std::isnan(Dw.norm()))
    {
        std::cout << "gyroscope has bad intrinsic values!" << std::endl;
        std::cout << "Tw = " << Tw << std::endl;
        std::cout << "Dw = " << Dw << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (std::isnan(Ta.norm()) || std::isnan(Da.norm()))
    {
        std::cout << "accelerometer has bad intrinsic values!" << std::endl;
        std::cout << "Ta = " << Ta << std::endl;
        std::cout << "Da = " << Da << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (odometry_options.state_options.imu_model == ImuModel::KALIBR)
    {
        odometry_options.vec_dw << Dw.block<3, 1>(0, 0), Dw.block<2, 1>(1, 1), Dw(2, 2);
        odometry_options.vec_da << Da.block<3, 1>(0, 0), Da.block<2, 1>(1, 1), Da(2, 2);
    }
    else
    {
        odometry_options.vec_dw << Dw(0, 0), Dw.block<2, 1>(0, 1), Dw.block<3, 1>(0, 2);
        odometry_options.vec_da << Da(0, 0), Da.block<2, 1>(0, 1), Da.block<3, 1>(0, 2);
    }

    odometry_options.vec_tg << Tg.block<3, 1>(0, 0), Tg.block<3, 1>(0, 1), Tg.block<3, 1>(0, 2);
    odometry_options.q_imu_acc = quatType::rotToQuat(R_imu_acc);
    odometry_options.q_imu_gyr = quatType::rotToQuat(R_imu_gyr);

    nh.param<double>("odometry_parameter/dt_slam_delay", para_double, 2.0); odometry_options.dt_slam_delay = para_double;

    odometry_options.imu_noises.sigma_w = odometry_options.init_options.sigma_w;
    odometry_options.imu_noises.sigma_wb = odometry_options.init_options.sigma_wb;
    odometry_options.imu_noises.sigma_a = odometry_options.init_options.sigma_a;
    odometry_options.imu_noises.sigma_ab = odometry_options.init_options.sigma_ab;
    odometry_options.imu_noises.sigma_w_2 = std::pow(odometry_options.imu_noises.sigma_w, 2);
    odometry_options.imu_noises.sigma_wb_2 = std::pow(odometry_options.imu_noises.sigma_wb, 2);
    odometry_options.imu_noises.sigma_a_2 = std::pow(odometry_options.imu_noises.sigma_a, 2);
    odometry_options.imu_noises.sigma_ab_2 = std::pow(odometry_options.imu_noises.sigma_ab, 2);

    nh.param<double>("odometry_parameter/up_msckf_sigma_px", para_double, 1.0); odometry_options.msckf_options.sigma_pix = para_double;
    nh.param<double>("odometry_parameter/up_msckf_chi2_multipler", para_double, 5.0); odometry_options.msckf_options.chi2_multipler = para_double;
    nh.param<double>("odometry_parameter/up_slam_sigma_px", para_double, 1.0); odometry_options.slam_options.sigma_pix = para_double;
    nh.param<double>("odometry_parameter/up_slam_chi2_multipler", para_double, 5.0); odometry_options.slam_options.chi2_multipler = para_double;
    odometry_options.msckf_options.sigma_pix_sq = std::pow(odometry_options.msckf_options.sigma_pix, 2);
    odometry_options.slam_options.sigma_pix_sq = std::pow(odometry_options.slam_options.sigma_pix, 2);

    nh.param<bool>("odometry_parameter/downsample_cameras", para_bool, false); odometry_options.downsample_cameras = para_bool;
    nh.param<int>("odometry_parameter/num_pts", para_int, 150); odometry_options.num_pts = para_int;
    nh.param<int>("odometry_parameter/fast_threshold", para_int, 20); odometry_options.fast_threshold = para_int;
    nh.param<int>("odometry_parameter/patch_size_x", para_int, 5); odometry_options.patch_size_x = para_int;
    nh.param<int>("odometry_parameter/patch_size_y", para_int, 5); odometry_options.patch_size_y = para_int;
    nh.param<int>("odometry_parameter/min_px_dist", para_int, 10); odometry_options.min_px_dist = para_int;
    nh.param<std::string>("odometry_parameter/histogram_method", str_temp, "histogram");
    if (str_temp == "none") odometry_options.histogram_method = HistogramMethod::NONE;
    else if (str_temp == "histogram") odometry_options.histogram_method = HistogramMethod::HISTOGRAM;
    else if (str_temp == "clahe") odometry_options.histogram_method = HistogramMethod::CLAHE;
    else {
        std::cout << "Invalid feature histogram specified: " << str_temp << std::endl;
        std::cout << "Please select a valid histogram method: none, histogram, clahe." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    nh.param<double>("odometry_parameter/track_frequency", para_double, 20.0); odometry_options.track_frequency = para_double;

    nh.param<bool>("odometry_parameter/use_huber", para_bool, true); odometry_options.state_options.use_huber = para_bool;
    nh.param<bool>("odometry_parameter/use_keyframe", para_bool, true); odometry_options.use_keyframe = para_bool;

    nh.param<bool>("feature_parameter/refine_features", para_bool, true); odometry_options.featinit_options.refine_features = para_bool;
    nh.param<int>("feature_parameter/max_runs", para_int, 5); odometry_options.featinit_options.max_runs = para_int;
    nh.param<double>("feature_parameter/init_lamda", para_double, 1e-3); odometry_options.featinit_options.init_lamda = para_double;
    nh.param<double>("feature_parameter/max_lamda", para_double, 1e10); odometry_options.featinit_options.max_lamda = para_double;
    nh.param<double>("feature_parameter/min_dx", para_double, 1e-6); odometry_options.featinit_options.min_dx = para_double;
    nh.param<double>("feature_parameter/min_dcost", para_double, 1e-6); odometry_options.featinit_options.min_dcost = para_double;
    nh.param<double>("feature_parameter/lam_mult", para_double, 10.0); odometry_options.featinit_options.lam_mult = para_double;
    nh.param<double>("feature_parameter/min_dist", para_double, 0.10); odometry_options.featinit_options.min_dist = para_double;
    nh.param<double>("feature_parameter/max_dist", para_double, 60.0); odometry_options.featinit_options.max_dist = para_double;
    nh.param<double>("feature_parameter/max_baseline", para_double, 40.0); odometry_options.featinit_options.max_baseline = para_double;
    nh.param<double>("feature_parameter/max_cond_number", para_double, 10000.0); odometry_options.featinit_options.max_cond_number = para_double;

    nh.param<int>("feature_parameter/keyframe_parallax", para_int, 10); setting_min_parallax = para_int;
    setting_min_parallax = setting_min_parallax / cam_calib_1_left.at(0);

    nh.param<double>("voxel_parameter/voxel_size", para_double, 0.1); odometry_options.voxel_size = odometry_options.state_options.voxel_size = para_double;
    nh.param<int>("voxel_parameter/max_num_points_in_voxel", para_int, 5); odometry_options.max_num_points_in_voxel = odometry_options.state_options.max_num_points_in_voxel = para_int;
    nh.param<double>("voxel_parameter/min_distance_points", para_double, 0.03); odometry_options.min_distance_points = odometry_options.state_options.min_distance_points = para_double;
    nh.param<int>("voxel_parameter/nb_voxels_visited", para_int, 1); odometry_options.nb_voxels_visited = para_int;
    nh.param<bool>("voxel_parameter/use_all_points", para_bool, false); odometry_options.use_all_points = para_bool;
}

void voxelStereoVio::allocateMemory()
{
    state_ptr = std::make_shared<state>(odometry_options.state_options);

    state_ptr->calib_imu_dw->setValue(odometry_options.vec_dw);
    state_ptr->calib_imu_dw->setFej(odometry_options.vec_dw);
    state_ptr->calib_imu_da->setValue(odometry_options.vec_da);
    state_ptr->calib_imu_da->setFej(odometry_options.vec_da);
    state_ptr->calib_imu_tg->setValue(odometry_options.vec_tg);
    state_ptr->calib_imu_tg->setFej(odometry_options.vec_tg);
    state_ptr->calib_imu_gyr->setValue(odometry_options.q_imu_gyr);
    state_ptr->calib_imu_gyr->setFej(odometry_options.q_imu_gyr);
    state_ptr->calib_imu_acc->setValue(odometry_options.q_imu_acc);
    state_ptr->calib_imu_acc->setFej(odometry_options.q_imu_acc);

    Eigen::VectorXd temp_camimu_dt;
    temp_camimu_dt.resize(1);
    temp_camimu_dt(0) = odometry_options.calib_camimu_dt;
    state_ptr->calib_dt_imu_cam->setValue(temp_camimu_dt);
    state_ptr->calib_dt_imu_cam->setFej(temp_camimu_dt);

    state_ptr->cam_intrinsics_cameras = odometry_options.camera_intrinsics;

    for (int i = 0; i < 2; i++)
    {
        state_ptr->cam_intrinsics.at(i)->setValue(odometry_options.camera_intrinsics.at(i)->getValue());
        state_ptr->cam_intrinsics.at(i)->setFej(odometry_options.camera_intrinsics.at(i)->getValue());
        state_ptr->calib_cam_imu.at(i)->setValue(odometry_options.camera_extrinsics.at(i));
        state_ptr->calib_cam_imu.at(i)->setFej(odometry_options.camera_extrinsics.at(i));
    }

    int init_max_features = std::floor((double)odometry_options.init_options.init_max_features / (double)2.0);

    featureTracker = std::shared_ptr<trackKLT>(new trackKLT(state_ptr->cam_intrinsics_cameras, init_max_features, odometry_options.histogram_method, 
        odometry_options.fast_threshold, odometry_options.patch_size_x, odometry_options.patch_size_y, odometry_options.min_px_dist));

    propagator_ptr = std::make_shared<propagator>(odometry_options.imu_noises, odometry_options.gravity_mag);

    initializer_ptr = std::make_shared<inertialInitializer>(odometry_options.init_options, featureTracker->getFeatureDatabase());

    updaterMsckf_ptr = std::make_shared<updaterMsckf>(odometry_options.msckf_options, odometry_options.featinit_options);

    updaterSlam_ptr = std::make_shared<updaterSlam>(odometry_options.slam_options, odometry_options.featinit_options, featureTracker->getFeatureDatabase());

    gammaPixel_ptr = std::make_shared<gammaPixel>();

    int wlvl = wG[0], hlvl = hG[0];

    while(wlvl % 2 == 0 && hlvl % 2 == 0 && wlvl * hlvl > 5000 && pyr_levels_used < PYR_LEVELS)
    {
        wlvl /=2;
        hlvl /=2;
        pyr_levels_used++;
    }

    for (int level = 1; level < pyr_levels_used; ++ level)
    {
        wG[level] = wG[0] >> level;
        hG[level] = hG[0] >> level;
    }

    frame_count = 0;
}

void voxelStereoVio::initialValue()
{
    last_time_image = -1;

    last_imu_data.timestamp = -1;
    last_imu_data.gyr.setZero();
    last_imu_data.acc.setZero();

    startup_time = -1;
    current_time = -1;

    time_newest_imu = -1;
}

void voxelStereoVio::imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    imuData imu_data;
    imu_data.timestamp = msg->header.stamp.toSec();
    imu_data.gyr << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu_data.acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    processImu(imu_data);

    time_newest_imu = imu_data.timestamp;
}

void voxelStereoVio::stereoImageHandler(const sensor_msgs::ImageConstPtr &msg_0, const sensor_msgs::ImageConstPtr &msg_1, int cam_0, int cam_1)
{
    double timestamp = msg_0->header.stamp.toSec();
    double time_delta = 1.0 / odometry_options.track_frequency;

    if (camera_last_timestamp.find(cam_0) != camera_last_timestamp.end() && timestamp < camera_last_timestamp.at(cam_0) + time_delta) return;

    camera_last_timestamp[cam_0] = timestamp;

    cv_bridge::CvImageConstPtr cv_ptr_0;
    try
    {
        cv_ptr_0 = cv_bridge::toCvShare(msg_0, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr_1;
    try
    {
        cv_ptr_1 = cv_bridge::toCvShare(msg_1, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    cameraData camera_data;
    camera_data.timestamp = cv_ptr_0->header.stamp.toSec();
    camera_data.camera_ids.push_back(cam_0);
    camera_data.camera_ids.push_back(cam_1);
    camera_data.images.push_back(cv_ptr_0->image.clone());
    camera_data.images.push_back(cv_ptr_1->image.clone());

    if (odometry_options.use_mask)
    {
        camera_data.masks.push_back(odometry_options.masks.at(cam_0));
        camera_data.masks.push_back(odometry_options.masks.at(cam_1));
    }
    else
    {
        camera_data.masks.push_back(cv::Mat::zeros(cv_ptr_0->image.rows, cv_ptr_0->image.cols, CV_8UC1));
        camera_data.masks.push_back(cv::Mat::zeros(cv_ptr_1->image.rows, cv_ptr_1->image.cols, CV_8UC1));
    }

    camera_buffer.push(camera_data);
}

void voxelStereoVio::processImu(imuData &imu_data)
{
    double oldest_time = state_ptr->margtimestep();

    if (oldest_time > state_ptr->timestamp)
        oldest_time = -1;

    if (!initial_flag)
        oldest_time = imu_data.timestamp - odometry_options.init_options.init_window_time + state_ptr->calib_dt_imu_cam->value()(0) - 0.10;

    propagator_ptr->feedImu(imu_data, oldest_time);

    if (!initial_flag)
        initializer_ptr->feedImu(imu_data, oldest_time);
}

bool voxelStereoVio::tryToInitialize(cameraData &image_measurements)
{
    camera_queue_init.push_back(image_measurements.timestamp);

    assert(newest_fh->timestamp == image_measurements.timestamp);
    frame_queue_init.push_back(newest_fh);

    double timestamp;
    Eigen::MatrixXd covariance;
    std::vector<std::shared_ptr<baseType>> order;
    auto init_rT1 = boost::posix_time::microsec_clock::local_time();

    bool success = initializer_ptr->initialize(timestamp, covariance, order, state_ptr->imu_ptr, true);

    if (success)
    {
        stateHelper::setInitialCovariance(state_ptr, covariance, order);

        state_ptr->timestamp = timestamp;
        startup_time = timestamp;

        featureTracker->getFeatureDatabase()->cleanUpOldmeasurements(state_ptr->timestamp);
        featureTracker->setNumFeatures(std::floor((double)odometry_options.num_pts / (double)2.0));

        auto init_rT2 = boost::posix_time::microsec_clock::local_time();
        std::cout << std::fixed << "[initialize]: Successful initialization in " << (init_rT2 - init_rT1).total_microseconds() * 1e-6 << " seconds" << std::endl;
        std::cout << std::fixed << "[initialize]: position = " << state_ptr->imu_ptr->getPos()(0) << ", " << state_ptr->imu_ptr->getPos()(1) << ", "
            << state_ptr->imu_ptr->getPos()(2) << std::endl;
        std::cout << std::fixed << "[initialize]: orientation = " << state_ptr->imu_ptr->getQuat()(0) << ", " << state_ptr->imu_ptr->getQuat()(1) << ", " << state_ptr->imu_ptr->getQuat()(2) 
            << ", " << state_ptr->imu_ptr->getQuat()(3) << std::endl;
        std::cout << std::fixed << "[initialize]: velocity = " << state_ptr->imu_ptr->getVel()(0) << ", " << state_ptr->imu_ptr->getVel()(1) << ", "
            << state_ptr->imu_ptr->getVel()(2) << std::endl;
        std::cout << std::fixed << "[initialize]: bias accel = " << state_ptr->imu_ptr->getBa()(0) << ", " << state_ptr->imu_ptr->getBa()(1) << ", "
            << state_ptr->imu_ptr->getBa()(2) << std::endl;
        std::cout << std::fixed << "[initialize]: bias gyro = " << state_ptr->imu_ptr->getBg()(0) << ", " << state_ptr->imu_ptr->getBg()(1) << ", " 
            << state_ptr->imu_ptr->getBg()(2) << std::endl;

        std::vector<double> camera_timestamps_to_init;
        for (size_t i = 0; i < camera_queue_init.size(); i++)
        {
            if (camera_queue_init.at(i) > timestamp)
                camera_timestamps_to_init.push_back(camera_queue_init.at(i));
        }

        std::vector<std::shared_ptr<frame>> camera_frames_to_init;
        for (size_t i = 0; i < frame_queue_init.size(); i++)
        {
            if (frame_queue_init.at(i)->timestamp > timestamp)
                camera_frames_to_init.push_back(frame_queue_init.at(i));
        }

        size_t clone_rate = (size_t)((double)camera_timestamps_to_init.size() / (double)odometry_options.state_options.max_clone_size) + 1;
      
        for (size_t i = 0; i < camera_timestamps_to_init.size(); i += clone_rate)
        {
            propagator_ptr->propagateAndClone(state_ptr, camera_timestamps_to_init.at(i));

            if (state_ptr->clones_frame.find(camera_frames_to_init.at(i)->timestamp) == state_ptr->clones_frame.end())
                state_ptr->clones_frame[camera_frames_to_init.at(i)->timestamp] = camera_frames_to_init.at(i);

            stateHelper::marginalizeOldClone(state_ptr);
        }

        std::cout << "[initialize]: Moved the state forward " << state_ptr->timestamp - timestamp << " seconds" << std::endl;
        camera_queue_init.clear();

        frame_queue_init.clear();

        return true;
    }
    else
    {
        auto init_rT2 = boost::posix_time::microsec_clock::local_time();
        std::cout << std::fixed << "[initialize]: Failed initialization in " << (init_rT2 - init_rT1).total_microseconds() * 1e-6 << " seconds" << std::endl;

        double oldest_time = image_measurements.timestamp - odometry_options.init_options.init_window_time - 0.10;

        auto it0 = camera_queue_init.begin();
    
        while (it0 != camera_queue_init.end())
        {
            if (*it0 < oldest_time)
                it0 = camera_queue_init.erase(it0);
            else
                it0++;
        }

        auto it1 = frame_queue_init.begin();
    
        while (it1 != frame_queue_init.end())
        {
            if ((*it1)->timestamp < oldest_time)
            {
                assert((*it1)->v_feat_ptr[0].size() == 0);
                assert((*it1)->v_feat_ptr[1].size() == 0);
                (*it1)->release();
                it1 = frame_queue_init.erase(it1);
            }
            else
                it1++;
        }

        return false;
    }
}

void voxelStereoVio::triangulateActiveTracks(cameraData &image_measurements)
{
    boost::posix_time::ptime retri_rT1, retri_rT2, retri_rT3;
    retri_rT1 = boost::posix_time::microsec_clock::local_time();

    assert(state_ptr->clones_imu.find(image_measurements.timestamp) != state_ptr->clones_imu.end());
    active_tracks_time = image_measurements.timestamp;

    // draw
    /*
    active_image = cv::Mat();
    featureTracker->displayActive(active_image, 255, 255, 255, 255, 255, 255, " ");
    if (!active_image.empty())
    {
        active_image = active_image(cv::Rect(0, 0, image_measurements.images.at(0).cols, image_measurements.images.at(0).rows));
    }
    */
    // draw

    active_tracks_pos_world.clear();
    active_tracks_uvd.clear();

    auto last_obs = featureTracker->getLastObs();
    auto last_ids = featureTracker->getLastIds();

    std::map<size_t, Eigen::Matrix3d> active_feat_linsys_A_new;
    std::map<size_t, Eigen::Vector3d> active_feat_linsys_b_new;
    std::map<size_t, int> active_feat_linsys_count_new;
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_pos_world_new;

    std::map<size_t, cv::Point2f> feat_uvs_in_cam_0;

    // display
    assert(state_ptr->clones_frame.find(active_tracks_time) != state_ptr->clones_frame.end());

    cv::Mat img_left = image_measurements.images.at(0);
    cv::Mat img_right = image_measurements.images.at(1);

    cv::Mat color_left = convertCvImage(img_left);
    cv::Mat color_right = convertCvImage(img_right);

    cv::Vec3b color_feat = cv::Vec3b(0, 255, 0);
    // display

    for (auto const &cam_id : image_measurements.camera_ids)
    {
        Eigen::Matrix3d R_GtoI = state_ptr->clones_imu.at(active_tracks_time)->getRot();
        Eigen::Vector3d p_IinG = state_ptr->clones_imu.at(active_tracks_time)->getPos();

        Eigen::Matrix3d R_ItoC = state_ptr->calib_cam_imu.at(cam_id)->getRot();
        Eigen::Vector3d p_IinC = state_ptr->calib_cam_imu.at(cam_id)->getPos();

        Eigen::Matrix3d R_GtoCi = R_ItoC * R_GtoI;
        Eigen::Vector3d p_CiinG = p_IinG - R_GtoCi.transpose() * p_IinC;

        assert(last_obs.find(cam_id) != last_obs.end());
        assert(last_ids.find(cam_id) != last_ids.end());

        for (size_t i = 0; i < last_obs.at(cam_id).size(); i++)
        {
            size_t feature_id = last_ids.at(cam_id).at(i);
            cv::Point2f pt_d = last_obs.at(cam_id).at(i).pt;

            // display
            if (pt_d.x > 5 && pt_d.x < wG[0] - 5 && pt_d.y > 5 && pt_d.y < hG[0] - 5)
            {   
                if (cam_id == 0)
                    drawPointL(color_left, pt_d.x, pt_d.y, color_feat);
                else
                    drawPointL(color_right, pt_d.x, pt_d.y, color_feat);
            }
            // display

            if (cam_id == 0) feat_uvs_in_cam_0[feature_id] = pt_d;

            if (state_ptr->map_points.find(feature_id) != state_ptr->map_points.end()) continue;

            cv::Point2f pt_n = state_ptr->cam_intrinsics_cameras.at(cam_id)->undistortCV(pt_d);
            Eigen::Matrix<double, 3, 1> b_i;
            b_i << pt_n.x, pt_n.y, 1;
            b_i = R_GtoCi.transpose() * b_i;
            b_i = b_i / b_i.norm();
            Eigen::Matrix3d B_perp = quatType::skewSymmetric(b_i);

            Eigen::Matrix3d Ai = B_perp.transpose() * B_perp;
            Eigen::Vector3d bi = Ai * p_CiinG;

            if (active_feat_linsys_A.find(feature_id) == active_feat_linsys_A.end())
            {
                active_feat_linsys_A_new.insert({feature_id, Ai});
                active_feat_linsys_b_new.insert({feature_id, bi});
                active_feat_linsys_count_new.insert({feature_id, 1});
            }
            else
            {
                active_feat_linsys_A_new[feature_id] = Ai + active_feat_linsys_A[feature_id];
                active_feat_linsys_b_new[feature_id] = bi + active_feat_linsys_b[feature_id];
                active_feat_linsys_count_new[feature_id] = 1 + active_feat_linsys_count[feature_id];
            }

            if (active_feat_linsys_count_new.at(feature_id) > 3)
            {
                Eigen::Matrix3d A = active_feat_linsys_A_new[feature_id];
                Eigen::Vector3d b = active_feat_linsys_b_new[feature_id];
                Eigen::MatrixXd p_FinG = A.colPivHouseholderQr().solve(b);
                Eigen::MatrixXd p_FinCi = R_GtoCi * (p_FinG - p_CiinG);

                Eigen::JacobiSVD<Eigen::Matrix3d> svd(A);
                Eigen::MatrixXd singularValues;
                singularValues.resize(svd.singularValues().rows(), 1);
                singularValues = svd.singularValues();
                double cond_A = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);

                if (std::abs(cond_A) <= odometry_options.featinit_options.max_cond_number && p_FinCi(2, 0) >= odometry_options.featinit_options.min_dist &&
                    p_FinCi(2, 0) <= odometry_options.featinit_options.max_dist && !std::isnan(p_FinCi.norm()))
                {
                    active_tracks_pos_world_new[feature_id] = p_FinG;
                }
            }
        }
    }

    // display
    cv::Mat stereo_image;
    cv::vconcat(color_left, color_right, stereo_image);

    /*
    cv::imshow("Stereo Image", stereo_image);
    cv::waitKey(1);
    */
    pubFeatImage(stereo_image, active_tracks_time);

    img_left.release();
    img_right.release();

    color_left.release();
    color_right.release();

    stereo_image.release();
    // display

    size_t total_triangulated = active_tracks_pos_world.size();

    active_feat_linsys_A = active_feat_linsys_A_new;
    active_feat_linsys_b = active_feat_linsys_b_new;
    active_feat_linsys_count = active_feat_linsys_count_new;
    active_tracks_pos_world = active_tracks_pos_world_new;
    retri_rT2 = boost::posix_time::microsec_clock::local_time();

    if (active_tracks_pos_world.empty() && state_ptr->map_points.empty()) return;

    for (const auto &feat : state_ptr->map_points)
    {
        Eigen::Vector3d p_FinG = feat.second->getPointXYZ(false);
        active_tracks_pos_world[feat.second->feature_id] = p_FinG;
    }

    std::shared_ptr<vec> distortion = state_ptr->cam_intrinsics.at(0);
    std::shared_ptr<poseJpl> calibration = state_ptr->calib_cam_imu.at(0);
    Eigen::Matrix<double, 3, 3> R_ItoC = calibration->getRot();
    Eigen::Matrix<double, 3, 1> p_IinC = calibration->getPos();

    std::shared_ptr<poseJpl> clone_Ii = state_ptr->clones_imu.at(active_tracks_time);
    Eigen::Matrix3d R_GtoIi = clone_Ii->getRot();
    Eigen::Vector3d p_IiinG = clone_Ii->getPos();

    for (const auto &feat : active_tracks_pos_world)
    {
        if (feat_uvs_in_cam_0.find(feat.first) == feat_uvs_in_cam_0.end()) continue;

        Eigen::Vector3d p_FinIi = R_GtoIi * (feat.second - p_IiinG);
        Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;

        double depth = p_FinCi(2);
        Eigen::Vector2d uv_dist;
    
        if (feat_uvs_in_cam_0.find(feat.first) != feat_uvs_in_cam_0.end())
        {
            uv_dist << (double)feat_uvs_in_cam_0.at(feat.first).x, (double)feat_uvs_in_cam_0.at(feat.first).y;
        }
        else
        {
            Eigen::Vector2d uv_norm;
            uv_norm << p_FinCi(0) / depth, p_FinCi(1) / depth;
            uv_dist = state_ptr->cam_intrinsics_cameras.at(0)->distortD(uv_norm);
        }

        if (depth < 0.1) continue;

        int width = state_ptr->cam_intrinsics_cameras.at(0)->w();
        int height = state_ptr->cam_intrinsics_cameras.at(0)->h();

        if (uv_dist(0) < 0 || (int)uv_dist(0) >= width || uv_dist(1) < 0 || (int)uv_dist(1) >= height)
        {
            continue;
        }

        Eigen::Vector3d uvd;
        uvd << uv_dist, depth;
        active_tracks_uvd.insert({feat.first, uvd});
    }
    retri_rT3 = boost::posix_time::microsec_clock::local_time();

    // Time test
    /*
    std::cout << std::fixed << "[triangulateActiveTracks]: " << (retri_rT2 - retri_rT1).total_microseconds() * 1e-6 << " seconds for triangulation (" 
        << total_triangulated << " tri of " << active_feat_linsys_A.size() << " active)" << std::endl;

    std::cout << std::fixed << "[triangulateActiveTracks]: " << (retri_rT3 - retri_rT2).total_microseconds() * 1e-6 << " seconds for re-projection into current" << std::endl;

    std::cout << std::fixed << "[triangulateActiveTracks]: " << (retri_rT3 - retri_rT1).total_microseconds() * 1e-6 << " seconds total" << std::endl;
    */
    // Time test
}

void voxelStereoVio::getRecentVoxel(double timestamp, pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit)
{
    std::vector<Eigen::Matrix<short, 3, 1>>().swap(recent_voxels);

    // display
    voxels_visit->clear();
    // display

    for (const auto &point : active_tracks_pos_world)
    {
        short kx = static_cast<short>(point.second[0] / odometry_options.voxel_size);
        short ky = static_cast<short>(point.second[1] / odometry_options.voxel_size);
        short kz = static_cast<short>(point.second[2] / odometry_options.voxel_size);

        for (short kxx = kx - odometry_options.nb_voxels_visited; kxx < kx + odometry_options.nb_voxels_visited + 1; kxx++)
        {
            for (short kyy = ky - odometry_options.nb_voxels_visited; kyy < ky + odometry_options.nb_voxels_visited + 1; kyy++)
            {
                for (short kzz = kz - odometry_options.nb_voxels_visited; kzz < kz + odometry_options.nb_voxels_visited + 1; kzz++)
                {
                    voxelHashMap::iterator search = voxel_map.find(voxel(kxx, kyy, kzz));

                    if(search != voxel_map.end())
                    {
                        auto &voxel_block = (search.value());

                        if (voxel_block.last_visit_time < timestamp && voxel_block.NumPoints() > 0)
                        {
                            Eigen::Matrix<short, 3, 1> voxel_temp;

                            voxel_temp[0] = kxx;
                            voxel_temp[1] = kyy;
                            voxel_temp[2] = kzz;

                            recent_voxels.push_back(voxel_temp);
                            voxel_block.last_visit_time = timestamp;

                            // display
                            pcl::PointXYZI point_temp;
    
                            point_temp.x = kxx >= 0 ? (kxx + 0.5) * odometry_options.voxel_size : (kxx - 0.5) * odometry_options.voxel_size;
                            point_temp.y = kyy >= 0 ? (kyy + 0.5) * odometry_options.voxel_size : (kyy - 0.5) * odometry_options.voxel_size;
                            point_temp.z = kzz >= 0 ? (kyy + 0.5) * odometry_options.voxel_size : (kzz - 0.5) * odometry_options.voxel_size;
                            point_temp.intensity = 1;

                            voxels_visit->points.push_back(point_temp);
                            // display
                        }
                    }
                }
            }
        }
    }
}

void voxelStereoVio::featureUpdate(cameraData &image_measurements)
{
    if (state_ptr->timestamp > image_measurements.timestamp)
    {
        std::cout << std::fixed << "Image received out of order, unable to do anything (prop dt = " << image_measurements.timestamp - state_ptr->timestamp << ")" << std::endl;
        return;
    }

    if (state_ptr->timestamp != image_measurements.timestamp)
    {
        propagator_ptr->propagateAndClone(state_ptr, image_measurements.timestamp);

        if (state_ptr->clones_frame.find(image_measurements.timestamp) == state_ptr->clones_frame.end())
            state_ptr->clones_frame[image_measurements.timestamp] = newest_fh;

        assert(state_ptr->clones_imu.size() == state_ptr->clones_frame.size());
    }

    rT3 = boost::posix_time::microsec_clock::local_time();

    if ((int)state_ptr->clones_imu.size() < std::min(state_ptr->options.max_clone_size, 5))
    {
        std::cout << "Waiting for enough clone states (" << (int)state_ptr->clones_imu.size() << " of " 
            << std::min(state_ptr->options.max_clone_size, 5) << ")" << std::endl;
        return;
    }

    if (state_ptr->timestamp > image_measurements.timestamp)
    {
        std::cout << "Msckf unable to propagate the state forward in time" << std::endl;
        std::cout << std::fixed << "It has been " << image_measurements.timestamp - state_ptr->timestamp << " since last time we propagated" << std::endl;
        return;
    }

    std::vector<std::shared_ptr<feature>> features_lost, features_marg, features_slam;
    features_lost = featureTracker->getFeatureDatabase()->getOldFeatures(state_ptr->timestamp, false, true);

    if ((int)state_ptr->clones_imu.size() > state_ptr->options.max_clone_size || (int)state_ptr->clones_imu.size() > 5)
    {
        features_marg = featureTracker->getFeatureDatabase()->getFeatures(state_ptr->margtimestep(), false, true);
    }

    auto it_1 = features_lost.begin();
    while (it_1 != features_lost.end())
    {
        bool found_current_cam_id = false;
        for (const auto &cam_uv_pair : (*it_1)->uvs)
        {
            if (std::find(image_measurements.camera_ids.begin(), image_measurements.camera_ids.end(), cam_uv_pair.first) 
                != image_measurements.camera_ids.end())
            {
                found_current_cam_id = true;
                break;
            }
        }

        if (found_current_cam_id)
        {
            it_1++;
        }
        else 
        {
            it_1 = features_lost.erase(it_1);
        }
    }

    it_1 = features_lost.begin();
    while (it_1 != features_lost.end())
    {
        if (std::find(features_marg.begin(), features_marg.end(), (*it_1)) != features_marg.end())
            it_1 = features_lost.erase(it_1);
        else
            it_1++;
    }

    std::vector<std::shared_ptr<feature>> features_maxtracks;
    auto it_2 = features_marg.begin();

    while (it_2 != features_marg.end())
    {
        bool reached_max = false;
    
        for (const auto &cams : (*it_2)->timestamps)
        {
            if ((int)cams.second.size() > state_ptr->options.max_clone_size)
            {
                reached_max = true;
                break;
            }
        }
    
        if (reached_max)
        {
            features_maxtracks.push_back(*it_2);
            it_2 = features_marg.erase(it_2);
        }
        else
        {
            it_2++;
        }
    }

    if (state_ptr->options.max_slam_features > 0 && image_measurements.timestamp - startup_time >= odometry_options.dt_slam_delay &&
        (int)state_ptr->map_points.size() < state_ptr->options.max_slam_features)
    {
        int amount_to_add = (state_ptr->options.max_slam_features) - (int)state_ptr->map_points.size();
        int valid_amount = (amount_to_add > (int)features_maxtracks.size()) ? (int)features_maxtracks.size() : amount_to_add;

        if (valid_amount > 0)
        {
            features_slam.insert(features_slam.end(), features_maxtracks.end() - valid_amount, features_maxtracks.end());
            features_maxtracks.erase(features_maxtracks.end() - valid_amount, features_maxtracks.end());
        }
    }

    for (int i = 0; i < recent_voxels.size(); i++)
    {
        short kx = recent_voxels[i][0];
        short ky = recent_voxels[i][1];
        short kz = recent_voxels[i][2];

        voxelHashMap::iterator search = voxel_map.find(voxel(kx, ky, kz));
        auto &voxel_block = (search.value());

        if (odometry_options.use_all_points)
        {
            for (int j = 0; j < voxel_block.points.size(); j++)
            {
                std::shared_ptr<mapPoint> map_point_ptr = voxel_block.points[j];

                std::shared_ptr<feature> feature = featureTracker->getFeatureDatabase()->getFeature(map_point_ptr->feature_id);
                if (feature != nullptr)
                    features_slam.push_back(feature);

                assert(map_point_ptr->unique_camera_id != -1);

                bool current_host_cam = std::find(image_measurements.camera_ids.begin(), image_measurements.camera_ids.end(), map_point_ptr->unique_camera_id) != image_measurements.camera_ids.end();

                if (feature == nullptr && current_host_cam)
                    map_point_ptr->should_marg = true;

                if (map_point_ptr->update_fail_count > 1)
                    map_point_ptr->should_marg = true;
            }
        }
        else
        {
            std::shared_ptr<mapPoint> map_point_ptr = voxel_block.points.front();

            std::shared_ptr<feature> feature = featureTracker->getFeatureDatabase()->getFeature(map_point_ptr->feature_id);
            if (feature != nullptr)
                features_slam.push_back(feature);

            assert(map_point_ptr->unique_camera_id != -1);

            bool current_host_cam = std::find(image_measurements.camera_ids.begin(), image_measurements.camera_ids.end(), map_point_ptr->unique_camera_id) != image_measurements.camera_ids.end();

            if (feature == nullptr && current_host_cam)
                map_point_ptr->should_marg = true;

            if (map_point_ptr->update_fail_count > 1)
                map_point_ptr->should_marg = true;
        }
    }

    // time test
    boost::posix_time::ptime rT_begin6 = boost::posix_time::microsec_clock::local_time();
    // time test

    stateHelper::marginalizeSlam(state_ptr, voxel_map, points_history);

    // time test
    boost::posix_time::ptime rT_end6 = boost::posix_time::microsec_clock::local_time();
    double time_cost = (rT_end6 - rT_begin6).total_microseconds() * 1e-6;
    sum_time_6 = sum_time_6 * n_time_6 + time_cost;
    n_time_6++;
    sum_time_6 = sum_time_6 / n_time_6;
    std::cout << std::fixed << std::setprecision(6) << "[marginalizeSlam] time cost = " << sum_time_6 << std::endl;
    // time test

    std::vector<std::shared_ptr<feature>> features_slam_delayed, features_slam_update;

    for (size_t i = 0; i < features_slam.size(); i++)
    {
        if (state_ptr->map_points.find(features_slam.at(i)->feature_id) != state_ptr->map_points.end())
        {
            features_slam_update.push_back(features_slam.at(i));
        }
        else
        {
            features_slam_delayed.push_back(features_slam.at(i));
        }
    }

    std::vector<std::shared_ptr<feature>> features_up_msckf = features_lost;
    features_up_msckf.insert(features_up_msckf.end(), features_marg.begin(), features_marg.end());
    features_up_msckf.insert(features_up_msckf.end(), features_maxtracks.begin(), features_maxtracks.end());

    auto compare_feature = [](const std::shared_ptr<feature> &a, const std::shared_ptr<feature> &b) -> bool {
        size_t size_a = 0;
        size_t size_b = 0;
    
        for (const auto &pair : a->timestamps)
            size_a += pair.second.size();
        
        for (const auto &pair : b->timestamps)
            size_b += pair.second.size();

        return size_a < size_b;
    };
    std::sort(features_up_msckf.begin(), features_up_msckf.end(), compare_feature);

    if ((int)features_up_msckf.size() > state_ptr->options.max_msckf_in_update)
        features_up_msckf.erase(features_up_msckf.begin(), features_up_msckf.end() - state_ptr->options.max_msckf_in_update);

    // time test
    boost::posix_time::ptime rT_begin2 = boost::posix_time::microsec_clock::local_time();
    // time test
  
    updaterMsckf_ptr->update(state_ptr, features_up_msckf);

    // time test
    boost::posix_time::ptime rT_end2 = boost::posix_time::microsec_clock::local_time();
    time_cost = (rT_end2 - rT_begin2).total_microseconds() * 1e-6;
    sum_time_2 = sum_time_2 * n_time_2 + time_cost;
    n_time_2++;
    sum_time_2 = sum_time_2 / n_time_2;
    std::cout << std::fixed << std::setprecision(6) << "[state update without map points] time cost = " << sum_time_2 << std::endl;
    // time test

    propagator_ptr->invalidateCache();

    rT4 = boost::posix_time::microsec_clock::local_time();

    std::vector<std::shared_ptr<feature>> features_slam_update_temp;

    while (!features_slam_update.empty())
    {
        std::vector<std::shared_ptr<feature>> features_up_temp;
        
        features_up_temp.insert(features_up_temp.begin(), features_slam_update.begin(), 
            features_slam_update.begin() + std::min(state_ptr->options.max_slam_in_update, (int)features_slam_update.size()));
        
        features_slam_update.erase(features_slam_update.begin(), 
            features_slam_update.begin() + std::min(state_ptr->options.max_slam_in_update, (int)features_slam_update.size()));

        // time test
        boost::posix_time::ptime rT_begin3 = boost::posix_time::microsec_clock::local_time();
        // time test

        updaterSlam_ptr->update(state_ptr, voxel_map, features_up_temp);

        // time test
        boost::posix_time::ptime rT_end3 = boost::posix_time::microsec_clock::local_time();
        time_cost = (rT_end3 - rT_begin3).total_microseconds() * 1e-6;
        sum_time_3 = sum_time_3 * n_time_3 + time_cost;
        n_time_3++;
        sum_time_3 = sum_time_3 / n_time_3;
        std::cout << std::fixed << std::setprecision(6) << "[state update without map points] time cost = " << sum_time_3 << std::endl;
        // time test

        auto it = state_ptr->map_points.begin();

        while (it != state_ptr->map_points.end())
        {
            mapManagement::changeHostVoxel(voxel_map, (*it).second, odometry_options.voxel_size, odometry_options.max_num_points_in_voxel, odometry_options.min_distance_points, voxels_history);
            it++;
        }

        features_slam_update_temp.insert(features_slam_update_temp.end(), features_up_temp.begin(), features_up_temp.end());
        propagator_ptr->invalidateCache();
    }

    features_slam_update = features_slam_update_temp;
    rT5 = boost::posix_time::microsec_clock::local_time();

    // time test
    boost::posix_time::ptime rT_begin4 = boost::posix_time::microsec_clock::local_time();
    // time test

    updaterSlam_ptr->delayedInit(state_ptr, voxel_map, features_slam_delayed, voxels_history);

    // time test
    boost::posix_time::ptime rT_end4 = boost::posix_time::microsec_clock::local_time();
    time_cost = (rT_end4 - rT_begin4).total_microseconds() * 1e-6;
    sum_time_4 = sum_time_4 * n_time_4 + time_cost;
    n_time_4++;
    sum_time_4 = sum_time_4 / n_time_4;
    std::cout << std::fixed << std::setprecision(6) << "[map point registration] time cost = " << sum_time_4 << std::endl;
    // time test

    rT6 = boost::posix_time::microsec_clock::local_time();

    if (image_measurements.camera_ids.at(0) == 0)
    {
        // time test
        boost::posix_time::ptime rT_begin5 = boost::posix_time::microsec_clock::local_time();
        // time test

        triangulateActiveTracks(image_measurements);

        getRecentVoxel(image_measurements.timestamp, voxels_visit);

        // time test
        boost::posix_time::ptime rT_end5 = boost::posix_time::microsec_clock::local_time();
        time_cost = (rT_end5 - rT_begin5).total_microseconds() * 1e-6;
        sum_time_5 = sum_time_5 * n_time_5 + time_cost;
        n_time_5++;
        sum_time_5 = sum_time_5 / n_time_5;
        std::cout << std::fixed << std::setprecision(6) << "[suitable map point selection] time cost = " << sum_time_5 << std::endl;
        // time test

        good_features_msckf.clear();
    }

    for (auto const &feat : features_up_msckf)
    {
        good_features_msckf.push_back(feat->position_global);
        feat->to_delete = true;
    }

    featureTracker->getFeatureDatabase()->cleanUp();

    if ((int)state_ptr->clones_imu.size() > state_ptr->options.max_clone_size)
    {
        featureTracker->getFeatureDatabase()->cleanUpOldmeasurements(state_ptr->margtimestep());
    }

    if (marginalize_status == MarginalizeStatus::OLD)
    {
        stateHelper::marginalizeOldClone(state_ptr);
    }
    else
    {
        stateHelper::marginalizeNewClone(state_ptr);
    }

    rT7 = boost::posix_time::microsec_clock::local_time();

    double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
    double time_prop = (rT3 - rT2).total_microseconds() * 1e-6;
    double time_msckf = (rT4 - rT3).total_microseconds() * 1e-6;
    double time_slam_update = (rT5 - rT4).total_microseconds() * 1e-6;
    double time_slam_delay = (rT6 - rT5).total_microseconds() * 1e-6;
    double time_marg = (rT7 - rT6).total_microseconds() * 1e-6;
    double time_total = (rT7 - rT1).total_microseconds() * 1e-6;

    // Time test
    /*
    std::cout << std::fixed << "[processImage]: " << time_track << " seconds for tracking" << std::endl;
    std::cout << std::fixed << "[featureUpdate]: " << time_prop << " seconds for propagation" << std::endl;
    std::cout << std::fixed << "[featureUpdate]: " << time_msckf << " seconds for MSCKF update (" << (int)features_up_msckf.size() << " feats)" << std::endl;
    
    if (state_ptr->options.max_slam_features > 0)
    {
        std::cout << std::fixed << "[featureUpdate]: " << time_slam_update << " seconds for SLAM update (" << (int)state_ptr->map_points.size() << " feats)" << std::endl;
        std::cout << std::fixed << "[featureUpdate]: " << time_slam_delay << " seconds for SLAM delayed init (" << (int)features_slam_delayed.size() << " feats)" << std::endl;
    }

    std::cout << std::fixed << "[featureUpdate]: " << time_marg << " seconds for re-tri & marg (" << (int)state_ptr->clones_imu.size() << " clones in state)" << std::endl;

    std::stringstream ss;
    ss << "[featureUpdate]: " << std::setprecision(4) << time_total << " seconds for total (camera";

    for (const auto &id : image_measurements.camera_ids) ss << " " << id;

    ss << ")" << std::endl;

    std::cout << ss.str();
    */
    // Time test

    if (timelastupdate != -1 && state_ptr->clones_imu.find(timelastupdate) != state_ptr->clones_imu.end())
    {
        Eigen::Matrix<double, 3, 1> dx = state_ptr->imu_ptr->getPos() - state_ptr->clones_imu.at(timelastupdate)->getPos();
        distance += dx.norm();
    }
    timelastupdate = image_measurements.timestamp;

    // display
    pubOdometry(state_ptr, timelastupdate);
    pubPath(state_ptr, timelastupdate);
    pubHistoryPoints(points_history, timelastupdate);
    pubWindowPoints(state_ptr, timelastupdate);
    pubHistoryVoxels(voxels_history, timelastupdate);
    pubVisitVoxels(voxels_visit, timelastupdate);
    // display

    std::cout << std::fixed << std::setprecision(3) << "q_GtoI = " << state_ptr->imu_ptr->getQuat()(0) << " " << state_ptr->imu_ptr->getQuat()(1) << " "
        << state_ptr->imu_ptr->getQuat()(2) << " " << state_ptr->imu_ptr->getQuat()(3) << std::endl;

    std::cout << std::fixed << std::setprecision(3) << "p_IinG = " << state_ptr->imu_ptr->getPos()(0) << " " << state_ptr->imu_ptr->getPos()(1) << " "
        << state_ptr->imu_ptr->getPos()(2) << std::endl;

    std::cout << std::fixed << std::setprecision(2) << "dist = " << distance << " meters" << std::endl;

    std::cout << std::fixed << std::setprecision(4) << "bg = " << state_ptr->imu_ptr->getBg()(0) << " " << state_ptr->imu_ptr->getBg()(1) << " "
        << state_ptr->imu_ptr->getBg()(2) << std::endl;

    std::cout << std::fixed << std::setprecision(4) << "ba = " << state_ptr->imu_ptr->getBa()(0) << " " << state_ptr->imu_ptr->getBa()(1) << " "
        << state_ptr->imu_ptr->getBa()(2) << std::endl;

    std::ofstream foutC(std::string(output_path + "/pose.txt"), std::ios::app);
    foutC.setf(std::ios::scientific, std::ios::floatfield);
    foutC.precision(6);
    foutC << std::fixed << image_measurements.timestamp + state_ptr->calib_dt_imu_cam->value()(0) << " " << state_ptr->imu_ptr->getPos()(0) << " " << state_ptr->imu_ptr->getPos()(1) << " " << state_ptr->imu_ptr->getPos()(2) << " " 
        << state_ptr->imu_ptr->getQuat()(0) << " " << state_ptr->imu_ptr->getQuat()(1) << " " << state_ptr->imu_ptr->getQuat()(2) << " " << state_ptr->imu_ptr->getQuat()(3) << std::endl;
    foutC.close();

    if (state_ptr->options.do_calib_camera_timeoffset)
        std::cout << std::fixed << std::setprecision(5) << "camera-imu timeoffset = " << state_ptr->calib_dt_imu_cam->value()(0) << std::endl;

    if (state_ptr->options.do_calib_camera_intrinsics)
    {
        for (int i = 0; i < 2; i++)
        {
            std::shared_ptr<vec> calib = state_ptr->cam_intrinsics.at(i);

            std::cout << std::fixed << std::setprecision(3) << "cam" << (int)i << " intrinsics = " << calib->value()(0) << " " << calib->value()(1) << " "
                << calib->value()(2) << " " << calib->value()(3) << " " << calib->value()(4) << " " << calib->value()(5) << " "
                << calib->value()(6) << " " << calib->value()(7) << std::endl;

        }
    }

    if (state_ptr->options.do_calib_camera_pose)
    {
        for (int i = 0; i < 2; i++)
        {
            std::shared_ptr<poseJpl> calib = state_ptr->calib_cam_imu.at(i);

            std::cout << std::fixed << std::setprecision(3) << "cam" << (int)i << " extrinsics = " << calib->getQuat()(0) << " " << calib->getQuat()(1) << " "
                << calib->getQuat()(2) << " " << calib->getQuat()(3) << " " << calib->getPos()(0) << " " << calib->getPos()(1) << " "
                << calib->getPos()(2) << std::endl;
        }
    }

    if (state_ptr->options.do_calib_imu_intrinsics && state_ptr->options.imu_model == ImuModel::KALIBR)
    {
        std::cout << std::fixed << std::setprecision(3) << "q_GYROtoI = " << state_ptr->calib_imu_gyr->value()(0) << " " << state_ptr->calib_imu_gyr->value()(1) << " "
            << state_ptr->calib_imu_gyr->value()(2) << " " << state_ptr->calib_imu_gyr->value()(3) << std::endl;
    }

    if (state_ptr->options.do_calib_imu_intrinsics && state_ptr->options.imu_model == ImuModel::RPNG)
    {
        std::cout << std::fixed << std::setprecision(3) << "q_ACCtoI = " << state_ptr->calib_imu_acc->value()(0) << " " << state_ptr->calib_imu_acc->value()(1) << " "
            << state_ptr->calib_imu_acc->value()(2) << " " << state_ptr->calib_imu_acc->value()(3) << std::endl;
    }

    if (state_ptr->options.do_calib_imu_intrinsics && state_ptr->options.imu_model == ImuModel::KALIBR)
    {
        std::cout << std::fixed << std::setprecision(4) << "Dw = | " << state_ptr->calib_imu_dw->value()(0) << ", " << state_ptr->calib_imu_dw->value()(1) << ", "
            << state_ptr->calib_imu_dw->value()(2) << " | " << state_ptr->calib_imu_dw->value()(3) << ", " << state_ptr->calib_imu_dw->value()(4)
            << " | " << state_ptr->calib_imu_dw->value()(5) << " |" << std::endl;

        std::cout << std::fixed << std::setprecision(4) << "Da = | " << state_ptr->calib_imu_da->value()(0) << ", " << state_ptr->calib_imu_da->value()(1) << ", "
            << state_ptr->calib_imu_da->value()(2) << " | " << state_ptr->calib_imu_da->value()(3) << ", " << state_ptr->calib_imu_da->value()(4)
            << " | " << state_ptr->calib_imu_da->value()(5) << " |" << std::endl;
    }

    if (state_ptr->options.do_calib_imu_intrinsics && state_ptr->options.imu_model == ImuModel::RPNG)
    {
        std::cout << std::fixed << std::setprecision(4) << "Dw = | " << state_ptr->calib_imu_dw->value()(0) << " | " << state_ptr->calib_imu_dw->value()(1) << ", "
            << state_ptr->calib_imu_dw->value()(2) << " | " << state_ptr->calib_imu_dw->value()(3) << ", " << state_ptr->calib_imu_dw->value()(4)
            << ", " << state_ptr->calib_imu_dw->value()(5) << " |" << std::endl;

        std::cout << std::fixed << std::setprecision(4) << "Da = | " << state_ptr->calib_imu_da->value()(0) << " | " << state_ptr->calib_imu_da->value()(1) << ", "
            << state_ptr->calib_imu_da->value()(2) << " | " << state_ptr->calib_imu_da->value()(3) << ", " << state_ptr->calib_imu_da->value()(4)
            << ", " << state_ptr->calib_imu_da->value()(5) << " |" << std::endl;
    }

    if (state_ptr->options.do_calib_imu_intrinsics && state_ptr->options.do_calib_imu_g_sensitivity)
    {
        std::cout << std::fixed << std::setprecision(4) << "Tg = | " << state_ptr->calib_imu_tg->value()(0) << ", " << state_ptr->calib_imu_tg->value()(1) << ", "
            << state_ptr->calib_imu_tg->value()(2) << " | " << state_ptr->calib_imu_tg->value()(3) << ", " << state_ptr->calib_imu_tg->value()(4) << ", "
            << state_ptr->calib_imu_tg->value()(5) << " | " << state_ptr->calib_imu_tg->value()(6) << ", " << state_ptr->calib_imu_tg->value()(7) << ", "
            << state_ptr->calib_imu_tg->value()(8) << " |" << std::endl;
    }

    std::cout << std::endl;
}

// display
void voxelStereoVio::pubFeatImage(cv::Mat &stereo_image, double &timestamp)
{
    if (!stereo_image.empty() && pub_feat_image.getNumSubscribers() > 0)
    {
        try {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", stereo_image).toImageMsg();
            msg->header.stamp = ros::Time().fromSec(timestamp);
            msg->header.frame_id = "camera_init";
            pub_feat_image.publish(msg);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}

void voxelStereoVio::pubOdometry(std::shared_ptr<state> state_ptr, double &timestamp)
{
    if (pub_odom.getNumSubscribers() > 0)
    {
        odom.header.frame_id = "camera_init";
        odom.child_frame_id = "body";
        odom.header.stamp = ros::Time().fromSec(timestamp);
        odom.pose.pose.orientation.x = state_ptr->imu_ptr->getQuat()(0);
        odom.pose.pose.orientation.y = state_ptr->imu_ptr->getQuat()(1);
        odom.pose.pose.orientation.z = state_ptr->imu_ptr->getQuat()(2);
        odom.pose.pose.orientation.w = state_ptr->imu_ptr->getQuat()(3);
        odom.pose.pose.position.x = state_ptr->imu_ptr->getPos()(0);
        odom.pose.pose.position.y = state_ptr->imu_ptr->getPos()(1);
        odom.pose.pose.position.z = state_ptr->imu_ptr->getPos()(2);
        pub_odom.publish(odom);
    }
}

void voxelStereoVio::setPoseStamp(geometry_msgs::PoseStamped &body_pose_out, std::shared_ptr<state> state_ptr)
{
    body_pose_out.pose.position.x = state_ptr->imu_ptr->getPos()(0);
    body_pose_out.pose.position.y = state_ptr->imu_ptr->getPos()(1);
    body_pose_out.pose.position.z = state_ptr->imu_ptr->getPos()(2);
    
    body_pose_out.pose.orientation.x = state_ptr->imu_ptr->getQuat()(0);
    body_pose_out.pose.orientation.y = state_ptr->imu_ptr->getQuat()(1);
    body_pose_out.pose.orientation.z = state_ptr->imu_ptr->getQuat()(2);
    body_pose_out.pose.orientation.w = state_ptr->imu_ptr->getQuat()(3);
}

void voxelStereoVio::pubPath(std::shared_ptr<state> state_ptr, double &timestamp)
{
    if (pub_path.getNumSubscribers() > 0)
    {
        setPoseStamp(msg_body_pose, state_ptr);
        msg_body_pose.header.stamp = ros::Time().fromSec(timestamp);
        msg_body_pose.header.frame_id = "camera_init";

        static int i = 0;
        i++;
        if (i % 10 == 0) 
        {
            path.poses.push_back(msg_body_pose);
            path.header.stamp = ros::Time().fromSec(timestamp);
            path.header.frame_id ="camera_init";
            pub_path.publish(path);
        }
    }
}

void voxelStereoVio::pubHistoryPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_history, double &timestamp)
{
    if (pub_points_history.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*points_history, point_cloud_msg);
        point_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        point_cloud_msg.header.frame_id = "camera_init";
        pub_points_history.publish(point_cloud_msg);
    }
}

void voxelStereoVio::pubWindowPoints(std::shared_ptr<state> state_ptr, double &timestamp)
{
    points_window->clear();

    if (pub_points_window.getNumSubscribers() > 0)
    {
        auto it0 = state_ptr->map_points.begin();
        while (it0 != state_ptr->map_points.end())
        {
            pcl::PointXYZRGB point_temp;
            point_temp.x = (*it0).second->getPointXYZ(false)[0];
            point_temp.y = (*it0).second->getPointXYZ(false)[1];
            point_temp.z = (*it0).second->getPointXYZ(false)[2];
            point_temp.r = (*it0).second->color;
            point_temp.g = (*it0).second->color;
            point_temp.b = (*it0).second->color;
            points_window->points.push_back(point_temp);

            it0++;
        }

        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*points_window, point_cloud_msg);
        point_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        point_cloud_msg.header.frame_id = "camera_init";
        pub_points_window.publish(point_cloud_msg);
    }
}

void voxelStereoVio::pubHistoryVoxels(pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history, double &timestamp)
{
    if (pub_voxels_history.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*voxels_history, point_cloud_msg);
        point_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        point_cloud_msg.header.frame_id = "camera_init";
        pub_voxels_history.publish(point_cloud_msg);
    }

    voxels_history->clear();
}

void voxelStereoVio::pubVisitVoxels(pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit, double &timestamp)
{
    if (pub_voxels_visit.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*voxels_visit, point_cloud_msg);
        point_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        point_cloud_msg.header.frame_id = "camera_init";
        pub_voxels_visit.publish(point_cloud_msg);
    }
}
// display

void voxelStereoVio::processImage(cameraData &image_measurements_const)
{
    rT1 = boost::posix_time::microsec_clock::local_time();

    assert(!image_measurements_const.camera_ids.empty());
    assert(image_measurements_const.camera_ids.size() == image_measurements_const.images.size());
  
    for (size_t i = 0; i < image_measurements_const.camera_ids.size() - 1; i++)
        assert(image_measurements_const.camera_ids.at(i) != image_measurements_const.camera_ids.at(i + 1));

    // time test
    boost::posix_time::ptime rT_begin7 = boost::posix_time::microsec_clock::local_time();
    // time test

    cameraData image_measurements = image_measurements_const;

    for (size_t i = 0; i < image_measurements.camera_ids.size() && odometry_options.downsample_cameras; i++)
    {
        cv::Mat img = image_measurements.images.at(i);
        cv::Mat mask = image_measurements.masks.at(i);
        cv::Mat img_temp, mask_temp;
        cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
        image_measurements.images.at(i) = img_temp;
        cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
        image_measurements.masks.at(i) = mask_temp;
    }

    std::shared_ptr<frame> fh = std::make_shared<frame>();
    fh->makeImageLeft(image_measurements.images.at(0).data, image_measurements.masks.at(0).data, gammaPixel_ptr, image_measurements.timestamp);
    fh->makeImageRight(image_measurements.images.at(1).data, image_measurements.masks.at(1).data, gammaPixel_ptr, image_measurements.timestamp);
    fh->frame_id = frame_count;

    // time test
    boost::posix_time::ptime rT_end7 = boost::posix_time::microsec_clock::local_time();
    double time_cost = (rT_end7 - rT_begin7).total_microseconds() * 1e-6;
    sum_time_7 = sum_time_7 * n_time_7 + time_cost;
    n_time_7++;
    sum_time_7 = sum_time_7 / n_time_7;
    std::cout << std::fixed << std::setprecision(6) << "[image processing] time cost = " << sum_time_7 << std::endl;
    // time test

    newest_fh = fh;

    // time test
    boost::posix_time::ptime rT_begin1 = boost::posix_time::microsec_clock::local_time();
    // time test

    featureTracker->feedNewImage(image_measurements, fh);

    // time test
    boost::posix_time::ptime rT_end1 = boost::posix_time::microsec_clock::local_time();
    time_cost = (rT_end1 - rT_begin1).total_microseconds() * 1e-6;
    sum_time_1 = sum_time_1 * n_time_1 + time_cost;
    n_time_1++;
    sum_time_1 = sum_time_1 / n_time_1;
    std::cout << std::fixed << std::setprecision(6) << "[feature extraction and matching] time cost = " << sum_time_1 << std::endl;
    // time test

    rT2 = boost::posix_time::microsec_clock::local_time();

    if (!initial_flag)
    {
        initial_flag = tryToInitialize(image_measurements);

        if (!initial_flag)
        {
            double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
            std::cout << std::fixed << "[processImage]: " << time_track << " seconds for tracking" << std::endl;
            return;
        }
    }

    if (featureTracker->getFeatureDatabase()->featureCheckParallax(frame_count))
        marginalize_status = MarginalizeStatus::OLD;
    else
    {
        if (odometry_options.use_keyframe)
            marginalize_status = MarginalizeStatus::NEW;
        else
            marginalize_status = MarginalizeStatus::OLD;
    }

    std::cout << "marginalize_status = " << marginalize_status << std::endl;

    featureUpdate(image_measurements);

    frame_count++;
}

void voxelStereoVio::run()
{
    double timestamp_imu_in_camera = time_newest_imu - state_ptr->calib_dt_imu_cam->value()(0);

    while (!camera_buffer.empty() && camera_buffer.front().timestamp < timestamp_imu_in_camera)
    {
        // time test
        boost::posix_time::ptime rT_begin_sum = boost::posix_time::microsec_clock::local_time();
        // time test

        processImage(camera_buffer.front());

        // time test
        boost::posix_time::ptime rT_end_sum = boost::posix_time::microsec_clock::local_time();
        double time_cost = (rT_end_sum - rT_begin_sum).total_microseconds() * 1e-6;
        sum_time_sum = sum_time_sum * n_time_sum + time_cost;
        n_time_sum++;
        sum_time_sum = sum_time_sum / n_time_sum;
        std::cout << std::fixed << std::setprecision(6) << "[total] time cost = " << sum_time_sum << std::endl;
        // time test

        camera_buffer.pop();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vio_node");
    ros::Time::init();
    
    voxelStereoVio VIO;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        VIO.run();

        rate.sleep();
    }

    return 0;
}
