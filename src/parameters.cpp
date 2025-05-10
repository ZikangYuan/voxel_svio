#include "parameters.h"

void stateOptions::recordParameters()
{
	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	str_temp = do_fej ? "true" : "false";
	foutC << "do_fej: " << str_temp << std::endl;

	str_temp = do_calib_camera_pose ? "true" : "false";
	foutC << "do_calib_camera_pose: " << str_temp << std::endl;

	str_temp = do_calib_camera_intrinsics ? "true" : "false";
	foutC << "do_calib_camera_intrinsics: " << str_temp << std::endl;

	str_temp = do_calib_camera_timeoffset ? "true" : "false";
	foutC << "do_calib_camera_timeoffset: " << str_temp << std::endl;

	str_temp = do_calib_imu_intrinsics ? "true" : "false";
	foutC << "do_calib_imu_intrinsics: " << str_temp << std::endl;

	str_temp = do_calib_imu_g_sensitivity ? "true" : "false";
	foutC << "do_calib_imu_g_sensitivity: " << str_temp << std::endl;

	switch (imu_model)
	{
	case 0:
		str_temp = "kalibr";
		break;
	case 1:
		str_temp = "rpng";
		break;
	default:
		break;
	}
	foutC << "imu_model: " << str_temp << std::endl;

	foutC << "max_clone_size: " << max_clone_size << std::endl;

	foutC << "max_slam_features: " << max_slam_features << std::endl;

	foutC << "max_slam_in_update: " << max_slam_in_update << std::endl;

	foutC << "max_msckf_in_update: " << max_msckf_in_update << std::endl;

	foutC.close();
}

void inertialInitializerOptions::recordParameters()
{
	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << std::fixed << "init_window_time: " << init_window_time << std::endl;

	foutC << std::fixed << "init_imu_thresh: " << init_imu_thresh << std::endl;

	foutC << std::fixed << "init_max_disparity: " << init_max_disparity << std::endl;

	foutC << "init_max_features: " << init_max_features << std::endl;

	str_temp = init_dyn_use ? "true" : "false";
	foutC << "init_dyn_use: " << str_temp << std::endl;

	str_temp = init_dyn_mle_opt_calib ? "true" : "false";
	foutC << "init_dyn_mle_opt_calib: " << str_temp << std::endl;

	foutC << "init_dyn_mle_max_iter: " << init_dyn_mle_max_iter << std::endl;

	foutC << "init_dyn_mle_max_threads: " << init_dyn_mle_max_threads << std::endl;

	foutC << std::fixed << "init_dyn_mle_max_time: " << init_dyn_mle_max_time << std::endl;

	foutC << "init_dyn_num_pose: " << init_dyn_num_pose << std::endl;

	foutC << std::fixed << "init_dyn_min_deg: " << init_dyn_min_deg << std::endl;

	foutC << std::fixed << "init_dyn_inflation_orientation: " << init_dyn_inflation_orientation << std::endl;

	foutC << std::fixed << "init_dyn_inflation_velocity: " << init_dyn_inflation_velocity << std::endl;

	foutC << std::fixed << "init_dyn_inflation_bias_gyro: " << init_dyn_inflation_bias_gyro << std::endl;

	foutC << std::fixed << "init_dyn_inflation_bias_accel: " << init_dyn_inflation_bias_accel << std::endl;

	foutC << std::fixed << "init_dyn_min_rec_cond: " << init_dyn_min_rec_cond << std::endl;

	foutC << std::fixed << "init_dyn_bias_g: " << init_dyn_bias_g.transpose() << std::endl;

	foutC << std::fixed << "init_dyn_bias_a: " << init_dyn_bias_a.transpose() << std::endl;

	foutC << std::fixed << "sigma_w: " << sigma_w << std::endl;

	foutC << std::fixed << "sigma_wb: " << sigma_wb << std::endl;

	foutC << std::fixed << "sigma_a: " << sigma_a << std::endl;

	foutC << std::fixed << "sigma_ab: " << sigma_ab << std::endl;

	foutC << std::fixed << "sigma_pix: " << sigma_pix << std::endl;

	foutC << std::fixed << "gravity_mag: " << gravity_mag << std::endl;

	str_temp = downsample_cameras ? "true" : "false";
	foutC << "downsample_cameras: " << str_temp << std::endl;

	foutC << std::fixed << "calib_camimu_dt: " << calib_camimu_dt << std::endl;

	for (int i = 0; i < 2; i++)
	{
		if (i == 0) foutC << "left camera parameters: " << std::endl;
		else foutC << "right camera parameters: " << std::endl;

		foutC << std::fixed << "camera intrinic: " << camera_intrinsics.at(i)->getValue().transpose() << std::endl;
		foutC << std::fixed << "camera_k_OPENCV: " 
			  << camera_intrinsics.at(i)->getK()(0, 0) << " " << camera_intrinsics.at(i)->getK()(0, 1) << " " << camera_intrinsics.at(i)->getK()(0, 2) << std::endl
			  << camera_intrinsics.at(i)->getK()(1, 0) << " " << camera_intrinsics.at(i)->getK()(1, 1) << " " << camera_intrinsics.at(i)->getK()(1, 2) << std::endl
			  << camera_intrinsics.at(i)->getK()(2, 0) << " " << camera_intrinsics.at(i)->getK()(2, 1) << " " << camera_intrinsics.at(i)->getK()(2, 2) << std::endl;

		foutC << std::fixed << "camera_d_OPENCV: "
			  << camera_intrinsics.at(i)->getD()(0) << " " << camera_intrinsics.at(i)->getD()(1) << " " 
			  << camera_intrinsics.at(i)->getD()(2) << " " << camera_intrinsics.at(i)->getD()(3) << std::endl;

		foutC << "image width: " << camera_intrinsics.at(i)->w() << std::endl;
		foutC << "image height: " << camera_intrinsics.at(i)->h() << std::endl;

		foutC << std::fixed << "camera_extrinsics: " << camera_extrinsics.at(i).transpose() << std::endl;
	}

	foutC.close();
}

void featureInitializerOptions::recordParameters()
{
	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	str_temp = refine_features ? "true" : "false";
	foutC << std::fixed << "refine_features: " << str_temp << std::endl;

	foutC << "max_runs: " << max_runs << std::endl;

	foutC << std::fixed << "init_lamda: " << init_lamda << std::endl;

	foutC << std::fixed << "max_lamda: " << max_lamda << std::endl;

	foutC << std::fixed << "min_dx: " << min_dx << std::endl;

	foutC << std::fixed << "min_dcost: " << min_dcost << std::endl;

	foutC << std::fixed << "lam_mult: " << lam_mult << std::endl;

	foutC << std::fixed << "min_dist: " << min_dist << std::endl;

	foutC << std::fixed << "max_dist: " << max_dist << std::endl;

	foutC << std::fixed << "max_baseline: " << max_baseline << std::endl;

	foutC << std::fixed << "max_cond_number: " << max_cond_number << std::endl;

	foutC.close();
}

void noiseManager::recordParameters()
{
	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << std::fixed << "sigma_w: " << sigma_w << std::endl;

	foutC << std::fixed << "sigma_w_2: " << sigma_w_2 << std::endl;

	foutC << std::fixed << "sigma_wb: " << sigma_wb << std::endl;

	foutC << std::fixed << "sigma_wb_2: " << sigma_wb_2 << std::endl;

	foutC << std::fixed << "sigma_a: " << sigma_a << std::endl;

	foutC << std::fixed << "sigma_a_2: " << sigma_a_2 << std::endl;

	foutC << std::fixed << "sigma_ab: " << sigma_ab << std::endl;

	foutC << std::fixed << "sigma_ab_2: " << sigma_ab_2 << std::endl;

	foutC.close();
}

void updaterOptions::recordParameters()
{
	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << std::fixed << "chi2_multipler: " << chi2_multipler << std::endl;

	foutC << std::fixed << "sigma_pix: " << sigma_pix << std::endl;

	foutC << std::fixed << "sigma_pix_sq: " << sigma_pix_sq << std::endl;

	foutC.close();
}


void odometryOptions::recordParameters()
{
	state_options.recordParameters();

	init_options.recordParameters();

	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << std::fixed << "dt_slam_delay: " << dt_slam_delay << std::endl;

	imu_noises.recordParameters();

	msckf_options.recordParameters();

	slam_options.recordParameters();

	foutC << std::fixed << "gravity_mag: " << gravity_mag << std::endl;

	foutC << std::fixed << "vec_dw: " << vec_dw.transpose() << std::endl;

	foutC << std::fixed << "vec_da: " << vec_da.transpose() << std::endl;

	foutC << std::fixed << "vec_tg: " << vec_tg.transpose() << std::endl;

	foutC << std::fixed << "q_imu_acc: " << q_imu_acc.w() << " " << q_imu_acc.x() << " " << q_imu_acc.y() << " " << q_imu_acc.z() << std::endl;

	foutC << std::fixed << "q_imu_gyr: " << q_imu_gyr.w() << " " << q_imu_gyr.x() << " " << q_imu_gyr.y() << " " << q_imu_gyr.z() << std::endl;

	foutC << std::fixed << "calib_camimu_dt: " << calib_camimu_dt << std::endl;

	for (int i = 0; i < 2; i++)
	{
		if (i == 0) foutC << "left camera parameters: " << std::endl;
		else foutC << "right camera parameters: " << std::endl;

		foutC << std::fixed << "camera intrinic: " << camera_intrinsics.at(i)->getValue().transpose() << std::endl;
		foutC << std::fixed << "camera_k_OPENCV: " 
			  << camera_intrinsics.at(i)->getK()(0, 0) << " " << camera_intrinsics.at(i)->getK()(0, 1) << " " << camera_intrinsics.at(i)->getK()(0, 2) << std::endl
			  << camera_intrinsics.at(i)->getK()(1, 0) << " " << camera_intrinsics.at(i)->getK()(1, 1) << " " << camera_intrinsics.at(i)->getK()(1, 2) << std::endl
			  << camera_intrinsics.at(i)->getK()(2, 0) << " " << camera_intrinsics.at(i)->getK()(2, 1) << " " << camera_intrinsics.at(i)->getK()(2, 2) << std::endl;

		foutC << std::fixed << "camera_d_OPENCV: "
			  << camera_intrinsics.at(i)->getD()(0) << " " << camera_intrinsics.at(i)->getD()(1) << " " 
			  << camera_intrinsics.at(i)->getD()(2) << " " << camera_intrinsics.at(i)->getD()(3) << std::endl;

		foutC << "image width: " << camera_intrinsics.at(i)->w() << std::endl;
		foutC << "image height: " << camera_intrinsics.at(i)->h() << std::endl;

		foutC << std::fixed << "camera_extrinsics: " << camera_extrinsics.at(i).transpose() << std::endl;
	}

	str_temp = use_mask ? "true" : "false";
	foutC << "use_mask: " << str_temp << std::endl;

	str_temp = downsample_cameras ? "true" : "false";
	foutC << "downsample_cameras: " << str_temp << std::endl;

	foutC << "num_pts: " << num_pts << std::endl;

	foutC << "fast_threshold: " << fast_threshold << std::endl;

	foutC << "patch_size_x: " << patch_size_x << std::endl;

	foutC << "patch_size_y: " << patch_size_y << std::endl;

	foutC << "min_px_dist: " << min_px_dist << std::endl;

	switch (histogram_method)
	{
	case 0:
		str_temp = "none";
		break;
	case 1:
		str_temp = "histogram";
		break;
	case 2:
		str_temp = "clahe";
		break;
	default:
		break;
	}
	foutC << "histogram_method: " << str_temp << std::endl;

	foutC << std::fixed << "track_frequency: " << track_frequency << std::endl;

	featinit_options.recordParameters();

	foutC.close();
}