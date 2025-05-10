#include "featureTracker.h"
#include "camera.h"
#include "feature.h"
#include "featureHelper.h"
#include "state.h"

trackKLT::trackKLT(std::unordered_map<size_t, std::shared_ptr<cameraBase>> camera_calib_, int num_features_, 
  HistogramMethod histogram_method_, int fast_threshold_, int patch_size_x_, int patch_size_y_, int min_px_dist_) 
  : camera_calib(camera_calib_), database(new featureDatabase()), num_features(num_features_), histogram_method(histogram_method_), 
  threshold(fast_threshold_), patch_size_x(patch_size_x_), patch_size_y(patch_size_y_), min_px_dist(min_px_dist_)
{
  current_id = 1;

  pixelSelector_ptr = std::make_shared<pixelSelector>(wG[0], hG[0]);
}

std::shared_ptr<featureDatabase> trackKLT::getFeatureDatabase()
{
  return database;
}

std::unordered_map<size_t, std::vector<cv::KeyPoint>> trackKLT::getLastObs()
{
  return pts_last;
}

std::unordered_map<size_t, std::vector<size_t>> trackKLT::getLastIds()
{
  return ids_last;
}

void trackKLT::setNumFeatures(int num_features_)
{
  num_features = num_features_;
}

void trackKLT::feedNewImage(const cameraData &image_measurements, std::shared_ptr<frame> fh)
{
  if (image_measurements.camera_ids.empty() || image_measurements.camera_ids.size() != image_measurements.images.size() 
    || image_measurements.images.size() != image_measurements.masks.size())
  {
    std::cout << "[ERROR]: image_measurements data sizes do not match or empty." << std::endl;
    std::cout << "[ERROR]: - image_measurements.camera_ids.size() = " << image_measurements.camera_ids.size() << std::endl;
    std::cout << "[ERROR]: - image_measurements.images.size() = " << image_measurements.images.size() << std::endl;
    std::cout << "[ERROR]: - image_measurements.masks.size() = " << image_measurements.masks.size() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  rT1 = boost::posix_time::microsec_clock::local_time();
  size_t num_images = image_measurements.images.size();

  for (size_t image_id = 0; image_id < num_images; image_id++)
  {
    size_t cam_id = image_measurements.camera_ids.at(image_id);

    cv::Mat image;

    if (histogram_method == HistogramMethod::HISTOGRAM)
    {
      cv::equalizeHist(image_measurements.images.at(image_id), image);
    }
    else if (histogram_method == HistogramMethod::CLAHE)
    {
      double eq_clip_limit = 10.0;
      cv::Size eq_win_size = cv::Size(8, 8);
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
      clahe->apply(image_measurements.images.at(image_id), image);
    }
    else
    {
      image = image_measurements.images.at(image_id);
    }

    std::vector<cv::Mat> img_pyr;
    cv::buildOpticalFlowPyramid(image, img_pyr, win_size, pyr_levels);

    img_curr[cam_id] = image;
    img_pyramid_curr[cam_id] = img_pyr;
  }

  feedStereo(image_measurements, fh, 0, 1);
}

void trackKLT::feedStereo(const cameraData &image_measurements, std::shared_ptr<frame> fh, size_t image_id_left, size_t image_id_right)
{
  size_t cam_id_left = image_measurements.camera_ids.at(image_id_left);
  size_t cam_id_right = image_measurements.camera_ids.at(image_id_right);

  cv::Mat img_left = img_curr.at(cam_id_left);
  cv::Mat img_right = img_curr.at(cam_id_right);
  std::vector<cv::Mat> img_pyr_left = img_pyramid_curr.at(cam_id_left);
  std::vector<cv::Mat> img_pyr_right = img_pyramid_curr.at(cam_id_right);
  cv::Mat mask_left = image_measurements.masks.at(image_id_left);
  cv::Mat mask_right = image_measurements.masks.at(image_id_right);
  rT2 = boost::posix_time::microsec_clock::local_time();

  if (pts_last[cam_id_left].empty() && pts_last[cam_id_right].empty())
  {
    std::vector<cv::KeyPoint> good_left, good_right;
    std::vector<size_t> good_ids_left, good_ids_right;
    performDetectionStereo(fh, img_pyr_left, img_pyr_right, mask_left, mask_right, cam_id_left, cam_id_right, good_left, good_right, good_ids_left, good_ids_right);
    
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_pyramid_last[cam_id_left] = img_pyr_left;
    img_pyramid_last[cam_id_right] = img_pyr_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
    
    return;
  }

  int pts_before_detect = (int)pts_last[cam_id_left].size();
  auto pts_left_old = pts_last[cam_id_left];
  auto pts_right_old = pts_last[cam_id_right];
  auto ids_left_old = ids_last[cam_id_left];
  auto ids_right_old = ids_last[cam_id_right];
  performDetectionStereo(fh, img_pyramid_last[cam_id_left], img_pyramid_last[cam_id_right], img_mask_last[cam_id_left],
    img_mask_last[cam_id_right], cam_id_left, cam_id_right, pts_left_old, pts_right_old, ids_left_old, ids_right_old);

  rT3 = boost::posix_time::microsec_clock::local_time();

  std::vector<uchar> mask_ll, mask_rr;
  std::vector<cv::KeyPoint> pts_left_new = pts_left_old;
  std::vector<cv::KeyPoint> pts_right_new = pts_right_old;

  parallel_for_(cv::Range(0, 2), lambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    performMatching(img_pyramid_last[is_left ? cam_id_left : cam_id_right], is_left ? img_pyr_left : img_pyr_right,
                                    is_left ? pts_left_old : pts_right_old, is_left ? pts_left_new : pts_right_new,
                                    is_left ? cam_id_left : cam_id_right, is_left ? cam_id_left : cam_id_right,
                                    is_left ? mask_ll : mask_rr);
                  }
                }));
  rT4 = boost::posix_time::microsec_clock::local_time();


  rT5 = boost::posix_time::microsec_clock::local_time();

  if (mask_ll.empty() && mask_rr.empty())
  {
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_pyramid_last[cam_id_left] = img_pyr_left;
    img_pyramid_last[cam_id_right] = img_pyr_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left].clear();
    pts_last[cam_id_right].clear();
    ids_last[cam_id_left].clear();
    ids_last[cam_id_right].clear();
    std::cout << "[trackKLT::feedStereo]: Not enough points for RANSAC, resetting." << std::endl;
    
    return;
  }
  
  std::vector<cv::KeyPoint> good_left, good_right;
  std::vector<size_t> good_ids_left, good_ids_right;

  for (size_t i = 0; i < pts_left_new.size(); i++)
  {
    if (pts_left_new.at(i).pt.x < 0 || pts_left_new.at(i).pt.y < 0 || (int)pts_left_new.at(i).pt.x > img_left.cols ||
      (int)pts_left_new.at(i).pt.y > img_left.rows)
      continue;
    
    bool found_right = false;
    size_t index_right = 0;
    for (size_t n = 0; n < ids_right_old.size(); n++)
    {
      if (ids_left_old.at(i) == ids_right_old.at(n))
      {
        found_right = true;
        index_right = n;
        break;
      }
    }
    
    if (mask_ll[i] && found_right && mask_rr[index_right])
    {
      if (pts_right_new.at(index_right).pt.x < 0 || pts_right_new.at(index_right).pt.y < 0 || 
        (int)pts_right_new.at(index_right).pt.x >= img_right.cols || (int)pts_right_new.at(index_right).pt.y >= img_right.rows)
        continue;

      good_left.push_back(pts_left_new.at(i));
      good_right.push_back(pts_right_new.at(index_right));
      good_ids_left.push_back(ids_left_old.at(i));
      good_ids_right.push_back(ids_right_old.at(index_right));
    }
    else if (mask_ll[i])
    {
      good_left.push_back(pts_left_new.at(i));
      good_ids_left.push_back(ids_left_old.at(i));
    }
  }

  for (size_t i = 0; i < pts_right_new.size(); i++)
  {
    if (pts_right_new.at(i).pt.x < 0 || pts_right_new.at(i).pt.y < 0 || (int)pts_right_new.at(i).pt.x >= img_right.cols ||
      (int)pts_right_new.at(i).pt.y >= img_right.rows)
      continue;
    
    bool added_already = (std::find(good_ids_right.begin(), good_ids_right.end(), ids_right_old.at(i)) != good_ids_right.end());

    if (mask_rr[i] && !added_already)
    {
      good_right.push_back(pts_right_new.at(i));
      good_ids_right.push_back(ids_right_old.at(i));
    }
  }

  for (size_t i = 0; i < good_left.size(); i++)
  {
    cv::Point2f npt_l = camera_calib.at(cam_id_left)->undistortCV(good_left.at(i).pt);
    database->updateFeature(fh, good_ids_left.at(i), image_measurements.timestamp, cam_id_left, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x, npt_l.y);
  }
  for (size_t i = 0; i < good_right.size(); i++)
  {
    cv::Point2f npt_r = camera_calib.at(cam_id_right)->undistortCV(good_right.at(i).pt);
    database->updateFeature(fh, good_ids_right.at(i), image_measurements.timestamp, cam_id_right, good_right.at(i).pt.x, good_right.at(i).pt.y, npt_r.x, npt_r.y);
  }

  {
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_pyramid_last[cam_id_left] = img_pyr_left;
    img_pyramid_last[cam_id_right] = img_pyr_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
  }
  rT6 = boost::posix_time::microsec_clock::local_time();

  // Time test
  /*
  std::cout << std::fixed << "[trackKLT]: " << (rT2 - rT1).total_microseconds() * 1e-6 << " seconds for pyramid." << std::endl;
  std::cout << std::fixed << "[trackKLT]: " << (rT3 - rT2).total_microseconds() * 1e-6 << " seconds for detection (" << (int)pts_last[cam_id_left].size() - pts_before_detect << " detected)." << std::endl;
  std::cout << std::fixed << "[trackKLT]: " << (rT4 - rT3).total_microseconds() * 1e-6 << " seconds for temporal klt." << std::endl;
  std::cout << std::fixed << "[trackKLT]: " << (rT5 - rT4).total_microseconds() * 1e-6 << " seconds for stereo klt." << std::endl;
  std::cout << std::fixed << "[trackKLT]: " << (rT6 - rT5).total_microseconds() * 1e-6 << " seconds for feature DB update (" << (int)good_left.size() << " features)." << std::endl;
  std::cout << std::fixed << "[trackKLT]: " << (rT6 - rT1).total_microseconds() * 1e-6 << " seconds for total." << std::endl;
  */
  // Time test
}

void trackKLT::performDetectionStereo(std::shared_ptr<frame> fh, const std::vector<cv::Mat> &img_0_pyr, const std::vector<cv::Mat> &img_1_pyr, 
  const cv::Mat &mask_0, const cv::Mat &mask_1, size_t cam_id_left, size_t cam_id_right, std::vector<cv::KeyPoint> &pts_0, std::vector<cv::KeyPoint> &pts_1, 
  std::vector<size_t> &ids_0, std::vector<size_t> &ids_1)
{
  cv::Size size_close_0((int)((float)img_0_pyr.at(0).cols / (float)min_px_dist),
                        (int)((float)img_0_pyr.at(0).rows / (float)min_px_dist));
  cv::Mat grid_2d_close_0 = cv::Mat::zeros(size_close_0, CV_8UC1);
  float size_x_0 = (float)img_0_pyr.at(0).cols / (float)patch_size_x;
  float size_y_0 = (float)img_0_pyr.at(0).rows / (float)patch_size_y;

  cv::Size size_grid_0(patch_size_x, patch_size_y);
  cv::Mat grid_2d_grid_0 = cv::Mat::zeros(size_grid_0, CV_8UC1);
  cv::Mat mask_0_updated = mask_0.clone();

  auto it_0 = pts_0.begin();
  auto it_1 = ids_0.begin();

  while (it_0 != pts_0.end())
  {
    cv::KeyPoint kpt = *it_0;
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;

    int edge = 10;
    if (x < edge || x >= img_0_pyr.at(0).cols - edge || y < edge || y >= img_0_pyr.at(0).rows - edge)
    {
      it_0 = pts_0.erase(it_0);
      it_1 = ids_0.erase(it_1);
      continue;
    }

    int x_close = (int)(kpt.pt.x / (float)min_px_dist);
    int y_close = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_close < 0 || x_close >= size_close_0.width || y_close < 0 || y_close >= size_close_0.height)
    {
      it_0 = pts_0.erase(it_0);
      it_1 = ids_0.erase(it_1);
      continue;
    }

    int x_grid = std::floor(kpt.pt.x / size_x_0);
    int y_grid = std::floor(kpt.pt.y / size_y_0);

    if (x_grid < 0 || x_grid >= size_grid_0.width || y_grid < 0 || y_grid >= size_grid_0.height)
    {
      it_0 = pts_0.erase(it_0);
      it_1 = ids_0.erase(it_1);
      continue;
    }

    if (grid_2d_close_0.at<uint8_t>(y_close, x_close) > 127)
    {
      it_0 = pts_0.erase(it_0);
      it_1 = ids_0.erase(it_1);
      continue;
    }
    
    if (mask_0.at<uint8_t>(y, x) > 127)
    {
      it_0 = pts_0.erase(it_0);
      it_1 = ids_0.erase(it_1);
      continue;
    }
    
    grid_2d_close_0.at<uint8_t>(y_close, x_close) = 255;
    if (grid_2d_grid_0.at<uint8_t>(y_grid, x_grid) < 255)
    {
      grid_2d_grid_0.at<uint8_t>(y_grid, x_grid) += 1;
    }
    
    if (x - min_px_dist >= 0 && x + min_px_dist < img_0_pyr.at(0).cols && y - min_px_dist >= 0 && y + min_px_dist < img_0_pyr.at(0).rows)
    {
      cv::Point pt_1(x - min_px_dist, y - min_px_dist);
      cv::Point pt_2(x + min_px_dist, y + min_px_dist);
      cv::rectangle(mask_0_updated, pt_1, pt_2, cv::Scalar(255), -1);

      int begin_idx = x - min_px_dist + wG[0] * (y - min_px_dist);
      int end_idx = x + min_px_dist + wG[0] * (y + min_px_dist);

      for(int i = begin_idx; i <= end_idx; i++) fh->mask_left[i] = true;
    }

    it_0++;
    it_1++;
  }

  double min_feat_percent = 0.50;
  int num_featsneeded_0 = num_features - (int)pts_0.size();

  if (num_featsneeded_0 > std::min(20, (int)(min_feat_percent * num_features)))
  {
    pixelSelector_ptr->pixelSelectionLeft(fh, num_featsneeded_0);

    std::vector<cv::KeyPoint> kpts_0_new;
    std::vector<cv::Point2f> pts_0_new;

    for (int y = 0; y < hG[0]; y++)
    {
      for (int x = 0; x < wG[0]; x++)
      {
        int idx = x + y * wG[0];

        if (fh->selected_pixels_left[idx] == 0) continue;

        cv::KeyPoint kpt;
        kpt.pt.x = x;
        kpt.pt.y = y;

        kpts_0_new.push_back(kpt);
        pts_0_new.push_back(kpt.pt);
      }
    }

    std::vector<cv::KeyPoint> kpts_1_new;
    std::vector<cv::Point2f> pts_1_new;
    kpts_1_new = kpts_0_new;
    pts_1_new = pts_0_new;

    if (!pts_0_new.empty())
    {
      std::vector<uchar> mask;
      std::vector<float> error;
      cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
      cv::calcOpticalFlowPyrLK(img_0_pyr, img_1_pyr, pts_0_new, pts_1_new, mask, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);

      for (size_t i = 0; i < pts_0_new.size(); i++)
      {
        bool oob_left = ((int)pts_0_new.at(i).x < 0 || (int)pts_0_new.at(i).x >= img_0_pyr.at(0).cols || (int)pts_0_new.at(i).y < 0 ||
                         (int)pts_0_new.at(i).y >= img_0_pyr.at(0).rows);
        bool oob_right = ((int)pts_1_new.at(i).x < 0 || (int)pts_1_new.at(i).x >= img_1_pyr.at(0).cols || (int)pts_1_new.at(i).y < 0 ||
                          (int)pts_1_new.at(i).y >= img_1_pyr.at(0).rows);

        if (!oob_left && !oob_right && mask[i] == 1)
        {
          kpts_0_new.at(i).pt = pts_0_new.at(i);
          kpts_1_new.at(i).pt = pts_1_new.at(i);
          
          pts_0.push_back(kpts_0_new.at(i));
          pts_1.push_back(kpts_1_new.at(i));
          
          size_t temp = ++current_id;
          ids_0.push_back(temp);
          ids_1.push_back(temp);
        }
        else if (!oob_left)
        {
          kpts_0_new.at(i).pt = pts_0_new.at(i);
          pts_0.push_back(kpts_0_new.at(i));
          
          size_t temp = ++current_id;
          ids_0.push_back(temp);
        }
      }
    }
  }

  cv::Size size_close_1((int)((float)img_1_pyr.at(0).cols / (float)min_px_dist), (int)((float)img_1_pyr.at(0).rows / (float)min_px_dist));
  cv::Mat grid_2d_close_1 = cv::Mat::zeros(size_close_1, CV_8UC1);
  float size_x_1 = (float)img_1_pyr.at(0).cols / (float)patch_size_x;
  float size_y_1 = (float)img_1_pyr.at(0).rows / (float)patch_size_y;
  
  cv::Size size_grid_1(patch_size_x, patch_size_y);
  cv::Mat grid_2d_grid_1 = cv::Mat::zeros(size_grid_1, CV_8UC1);
  cv::Mat mask_1_updated = mask_0.clone();

  it_0 = pts_1.begin();
  it_1 = ids_1.begin();

  while (it_0 != pts_1.end())
  {
    cv::KeyPoint kpt = *it_0;
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;
    int edge = 10;
    if (x < edge || x >= img_1_pyr.at(0).cols - edge || y < edge || y >= img_1_pyr.at(0).rows - edge)
    {
      it_0 = pts_1.erase(it_0);
      it_1 = ids_1.erase(it_1);
      continue;
    }
    
    int x_close = (int)(kpt.pt.x / (float)min_px_dist);
    int y_close = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_close < 0 || x_close >= size_close_1.width || y_close < 0 || y_close >= size_close_1.height)
    {
      it_0 = pts_1.erase(it_0);
      it_1 = ids_1.erase(it_1);
      continue;
    }

    int x_grid = std::floor(kpt.pt.x / size_x_1);
    int y_grid = std::floor(kpt.pt.y / size_y_1);
    if (x_grid < 0 || x_grid >= size_grid_1.width || y_grid < 0 || y_grid >= size_grid_1.height)
    {
      it_0 = pts_1.erase(it_0);
      it_1 = ids_1.erase(it_1);
      continue;
    }
    
    bool is_stereo = (std::find(ids_0.begin(), ids_0.end(), *it_1) != ids_0.end());

    if (grid_2d_close_1.at<uint8_t>(y_close, x_close) > 127 && !is_stereo)
    {
      it_0 = pts_1.erase(it_0);
      it_1 = ids_1.erase(it_1);
      continue;
    }

    if (mask_1.at<uint8_t>(y, x) > 127)
    {
      it_0 = pts_1.erase(it_0);
      it_1 = ids_1.erase(it_1);
      continue;
    }

    grid_2d_close_1.at<uint8_t>(y_close, x_close) = 255;

    if (grid_2d_grid_1.at<uint8_t>(y_grid, x_grid) < 255)
    {
      grid_2d_grid_1.at<uint8_t>(y_grid, x_grid) += 1;
    }

    if (x - min_px_dist >= 0 && x + min_px_dist < img_1_pyr.at(0).cols && y - min_px_dist >= 0 && y + min_px_dist < img_1_pyr.at(0).rows)
    {
      cv::Point pt_1(x - min_px_dist, y - min_px_dist);
      cv::Point pt_2(x + min_px_dist, y + min_px_dist);
      cv::rectangle(mask_1_updated, pt_1, pt_2, cv::Scalar(255), -1);

      int begin_idx = x - min_px_dist + wG[0] * (y - min_px_dist);
      int end_idx = x + min_px_dist + wG[0] * (y + min_px_dist);

      for(int i = begin_idx; i <= end_idx; i++) fh->mask_right[i] = true;
    }

    it_0++;
    it_1++;
  }

  int num_featsneeded_1 = num_features - (int)pts_1.size();
  if (num_featsneeded_1 > std::min(20, (int)(min_feat_percent * num_features)))
  {
    pixelSelector_ptr->pixelSelectionRight(fh, num_featsneeded_1);

    for (int y = 0; y < hG[0]; y++)
    {
      for (int x = 0; x < wG[0]; x++)
      {
        int idx = x + y * wG[0];

        if (fh->selected_pixels_right[idx] == 0) continue;

        cv::KeyPoint kpt;
        kpt.pt.x = x;
        kpt.pt.y = y;

        pts_1.push_back(kpt);
        size_t temp = ++current_id;
        ids_1.push_back(temp);
      }
    }
  }
}

void trackKLT::performMatching(const std::vector<cv::Mat> &img_0_pyr, const std::vector<cv::Mat> &img_1_pyr, 
  std::vector<cv::KeyPoint> &kpts_0, std::vector<cv::KeyPoint> &kpts_1, size_t id_0, size_t id_1, std::vector<uchar> &mask_out)
{
  assert(kpts_0.size() == kpts_1.size());

  if (kpts_0.empty() || kpts_1.empty())
    return;

  std::vector<cv::Point2f> pts_0, pts_1;
  for (size_t i = 0; i < kpts_0.size(); i++)
  {
    pts_0.push_back(kpts_0.at(i).pt);
    pts_1.push_back(kpts_1.at(i).pt);
  }

  if (pts_0.size() < 10)
  {
    for (size_t i = 0; i < pts_0.size(); i++)
      mask_out.push_back((uchar)0);

    return;
  }

  std::vector<uchar> mask_klt;
  std::vector<float> error;
  cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
  cv::calcOpticalFlowPyrLK(img_0_pyr, img_1_pyr, pts_0, pts_1, mask_klt, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);

  std::vector<cv::Point2f> pts_0_n, pts_1_n;
  for (size_t i = 0; i < pts_0.size(); i++)
  {
    pts_0_n.push_back(camera_calib.at(id_0)->undistortCV(pts_0.at(i)));
    pts_1_n.push_back(camera_calib.at(id_1)->undistortCV(pts_1.at(i)));
  }

  std::vector<uchar> mask_rsc;
  double max_focallength_img_0 = std::max(camera_calib.at(id_0)->getK()(0, 0), camera_calib.at(id_0)->getK()(1, 1));
  double max_focallength_img_1 = std::max(camera_calib.at(id_1)->getK()(0, 0), camera_calib.at(id_1)->getK()(1, 1));
  double max_focallength = std::max(max_focallength_img_0, max_focallength_img_1);
  cv::findFundamentalMat(pts_0_n, pts_1_n, cv::FM_RANSAC, 2.0 / max_focallength, 0.999, mask_rsc);

  for (size_t i = 0; i < mask_klt.size(); i++)
  {
    auto mask = (uchar)((i < mask_klt.size() && mask_klt[i] && i < mask_rsc.size() && mask_rsc[i]) ? 1 : 0);
    mask_out.push_back(mask);
  }

  for (size_t i = 0; i < pts_0.size(); i++)
  {
    kpts_0.at(i).pt = pts_0.at(i);
    kpts_1.at(i).pt = pts_1.at(i);
  }
}

void trackKLT::displayActive(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay)
{
  std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
  std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last_cache;
  {
    img_last_cache = img_last;
    img_mask_last_cache = img_mask_last;
    pts_last_cache = pts_last;
  }

  int max_width = -1;
  int max_height = -1;

  for (auto const &pair : img_last_cache)
  {
    if (max_width < pair.second.cols)
      max_width = pair.second.cols;
    if (max_height < pair.second.rows)
      max_height = pair.second.rows;
  }

  if (img_last_cache.empty() || max_width == -1 || max_height == -1)
    return;

  bool is_small = (std::min(max_width, max_height) < 400);

  bool image_new = ((int)img_last_cache.size() * max_width != img_out.cols || max_height != img_out.rows);

  if (image_new)
    img_out = cv::Mat(max_height, (int)img_last_cache.size() * max_width, CV_8UC3, cv::Scalar(0, 0, 0));

  int index_cam = 0;
  for (auto const &pair : img_last_cache)
  {
    cv::Mat img_temp;

    if (image_new)
      cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
    else
      img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));

    for (size_t i = 0; i < pts_last_cache[pair.first].size(); i++)
    {
      cv::Point2f pt_l = pts_last_cache[pair.first].at(i).pt;
      cv::circle(img_temp, pt_l, (is_small) ? 1 : 2, cv::Scalar(r1, g1, b1), cv::FILLED);

      cv::Point2f pt_l_top = cv::Point2f(pt_l.x - 3, pt_l.y - 3);
      cv::Point2f pt_l_bot = cv::Point2f(pt_l.x + 3, pt_l.y + 3);
      cv::rectangle(img_temp, pt_l_top, pt_l_bot, cv::Scalar(r2, g2, b2), 1);
    }

    auto txtpt = (is_small) ? cv::Point(10, 30) : cv::Point(30, 60);
    
    if (overlay == "")
    {
      cv::putText(img_temp, "CAM:" + std::to_string((int)pair.first), txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0, cv::Scalar(0, 255, 0), 3);
    }
    else
    {
      cv::putText(img_temp, overlay, txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0, cv::Scalar(0, 0, 255), 3);
    }

    cv::Mat mask = cv::Mat::zeros(img_mask_last_cache[pair.first].rows, img_mask_last_cache[pair.first].cols, CV_8UC3);
    mask.setTo(cv::Scalar(0, 0, 255), img_mask_last_cache[pair.first]);
    cv::addWeighted(mask, 0.1, img_temp, 1.0, 0.0, img_temp);

    img_temp.copyTo(img_out(cv::Rect(max_width * index_cam, 0, img_last_cache[pair.first].cols, img_last_cache[pair.first].rows)));
    index_cam++;
  }
}